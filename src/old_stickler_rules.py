#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from std_msgs.msg import Bool
import ConsoleFormatter
import time
import random
import threading
import sys
import rospy
import math
import os
import numpy as np
from std_srvs.srv import SetBool

from navigation_msgs.srv import constant_spin_srv, get_absolute_position_srv, get_absolute_position_srvRequest
from navigation_msgs.msg import simple_feedback_msg
from perception_msgs.msg import get_labels_msg
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srv
from robot_toolkit_msgs.msg import animation_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class STICKLER_RULES(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "stickler_for_the_rules"
        states = ['INIT', 'LOOK4PERSON', 'LOOK4SHOES', 'LOOK4DRINK', 'ASK4SHOES', 'ASK2LEAVE', 'ASK4DRINK', 'GO2KITCHEN', 'GO2LIVING', 'GO2ROOM']
        self.tm = tm(perception = True,speech=True, manipulation=False, navigation=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'STICKLER_RULES', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'LOOK4PERSON'},
            {'trigger': 'person_found', 'source': 'LOOK4PERSON', 'dest': 'LOOK4SHOES'},
            {'trigger': 'shoes_not_found', 'source': 'LOOK4SHOES', 'dest': 'ASK4SHOES'},
            {'trigger': 'shoe_asked', 'source': 'ASK4SHOES', 'dest': 'LOOK4SHOES'},
            {'trigger': 'shoes_found', 'source': 'LOOK4SHOES', 'dest': 'LOOK4DRINK'},
            {'trigger': 'drink_not_found', 'source': 'LOOK4DRINK', 'dest': 'ASK4DRINK'},
            {'trigger': 'drink_asked_not_kitchen', 'source': 'ASK4DRINK', 'dest': 'GO2KITCHEN'},
            {'trigger': 'arrive_kitchen', 'source': 'GO2KITCHEN', 'dest': 'ASK4DRINK'},
            {'trigger': 'drink_asked_kitchen', 'source': 'ASK4DRINK', 'dest': 'LOOK4DRINK'},
            {'trigger': 'drink_found_not_kitchen', 'source': 'LOOK4DRINK', 'dest': 'LOOK4PERSON'},
            {'trigger': 'drink_found_kitchen', 'source': 'LOOK4DRINK', 'dest': 'GO2LIVING'},
            {'trigger': 'arrive_living', 'source': 'GO2LIVING', 'dest': 'LOOK4PERSON'},
            {'trigger': 'full_turn_living', 'source': 'LOOK4PERSON', 'dest':'GO2ROOM'},
            {'trigger': 'arrive_room', 'source': 'GO2ROOM', 'dest':'LOOK4PERSON'},
            {'trigger': 'person_room', 'source': 'LOOK4PERSON', 'dest': 'ASK2LEAVE'},
            {'trigger': 'person_asked', 'source': 'ASK2LEAVE', 'dest': 'LOOK4PERSON'},
            {'trigger': 'full_turn_room', 'source': 'LOOK4PERSON', 'dest':'GO2LIVING'},
        ]
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='STICKLER_RULES')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        # ROS Callbacks
        print(self.consoleFormatter.format("Waiting for /amcl_pose", "WARNING"))
        subscriber_odom = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_spin_until)

        print(self.consoleFormatter.format("Waiting for /look_for_object_publisher", "WARNING"))
        self.subscriber_look_for_object = rospy.Subscriber("/perception_utilities/look_for_object_publisher", Bool, self.callback_look_for_object)

        # ROS Services (PyToolkit)
        print(self.consoleFormatter.format("Waiting for pytoolkit/awareness...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        self.awareness_srv = rospy.ServiceProxy("/pytoolkit/ALBasicAwareness/set_awareness_srv",SetBool)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/move_head...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        self.move_head_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/move_head_srv",move_head_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
        self.show_topic_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv",tablet_service_srv)

        # ROS Services (Navigation)
        rospy.wait_for_service('/navigation_utilities/get_absolute_position_srv')
        self.get_position_proxy = rospy.ServiceProxy('/navigation_utilities/get_absolute_position_srv', get_absolute_position_srv)

        # ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        #Subscriber:
        self.get_labels_publisher = rospy.Subscriber("/perception_utilities/get_labels_publisher", get_labels_msg, self.callback_get_labels)
        
        ##################### ROS CALLBACK VARIABLES #####################
        self.angle = 0
        self.recognize_person_counter = 0
        self.angle_stop_looking_person = 359
        self.stop_rotation = False
        self.stop_rotation = False
        ##################### GLOBAL VARIABLES #####################

        self.at_living = True
        self.at_room = False
        self.at_kitchen = False
        self.ids_seen = []
        self.person_seen = False
        self.wall_distance = 1.8
        self.times_spin = 0
        self.labels = []
        self.widths = []
        self.heights = []
        self.ids = []
        self.objects = {}
        self.person_ids = []
        self.sizes = []
        self.reference_width, self.reference_distance = 93, 2 # Average width = 42.5cm (About 97px at 2m)
        # 320x240

    def callback_spin_until(self,data):
        self.angle = int(np.degrees(euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2] % (2 * math.pi)))
        return self.angle

    def callback_get_labels(self,data):
        self.labels = data.labels
        self.ids = data.ids
        self.widths = data.widths
        self.heights = data.heights
        for i in range(0, len(self.heights)):
            size_dict = {"label": self.labels[i], "width": self.widths[i], "height": self.heights[i]}
            if self.labels[i] == 'person':
                self.person_ids.append(self.ids[i])
            self.objects[self.ids[i]] = size_dict

    def callback_look_for_object(self,data):
        print(data.data)
        if data.data:
            for id in self.person_ids:
                if id not in self.ids_seen:
                    self.person_seen = self.object_distance(id, self.wall_distance)
                    print(f"Person seen: {self.person_seen}")
            self.shoes = True
        return data
    
    def object_distance(self, object_id, wall_distance=2, offset=0.4):
        """
        Function to know if aa person is inside or not of a room calculating 
        the distance 
        Args:
            object_id (float): Key in the dictionary to know the info about the person.
            wall_distance (float, optional): Distance from pepper to the wall. Defaults to 200.
            offset (float, optional): An offset to calibrate the function. Defaults to 20.
        Returns:
            bool: If the person is inside or not
        """
        width = self.objects[object_id]['width']
        estimated_distance = (self.reference_width * self.reference_distance) / width
        print(f"Width: {width}")
        if estimated_distance <= (wall_distance + offset):
            return True
        else:
            return False

    def on_enter_INIT(self):
        self.tm.talk("I am going to do the "+self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",1,15) 
        self.tm.turn_camera("bottom_camera","custom",1,15)
        self.tm.go_to_place("living_room")
        self.beggining()

    # ============================== LOOK4 STATES ==============================
    def on_enter_LOOK4PERSON(self):
        self.labels = []
        self.ids = []
        print(self.consoleFormatter.format("LOOK4PERSON", "HEADER"))
        self.awareness_srv(True)
        time.sleep(0.5)
        self.animations_publisher.publish("animations","Gestures/Maybe_1")
        time.sleep(0.5)
        self.awareness_srv(False)
        self.tm.start_recognition("front_camera")
        rospy.sleep(1)
        self.move_head_srv("default")
        #self.show_topic_srv("/perception_utilities/yolo_publisher")
        self.tm.look_for_object("person", False)
        self.stop_rotation=False
        while not self.times_spin == 7 and not self.person_seen:
            print(f"person_seen {self.person_seen}")
            self.times_spin += 1
            print(f"Times spin: {self.times_spin}")
            self.tm.spin_srv(45)
            self.ids_seen.extend(self.person_ids)
            time.sleep(1)
        print(self.consoleFormatter.format("ROBOT STOP", "WARNING"))
        self.tm.robot_stop_srv()
        self.tm.start_recognition("")
        self.tm.look_for_object("",True)
        self.stop_rotation = False
        if self.person_seen:
            self.person_seen = False
            if self.at_living:
                self.tm.talk("Please come closer","English")
                self.person_found()
            self.person_room()
        elif self.times_spin == 7:
            self.ids_seen = []
            self.times_spin = 0
            if self.at_living:
                self.full_turn_living()
            self.full_turn_room()

    def on_enter_LOOK4SHOES(self):
        # TODO: Mirar para abajo
        print(self.consoleFormatter.format("LOOK4SHOES", "HEADER"))
        self.awareness_srv(False)
        time.sleep(0.5)
        self.move_head_srv("down")
        self.tm.start_recognition("")
        self.tm.start_recognition("front_camera")
        #self.show_topic_srv("/perception_utilities/yolo_publisher")
        #change cell phone for shoes
        self.tm.look_for_object("cell phone", True)
        rospy.sleep(1)
        object_found = self.tm.wait_for_object(1)
        if not object_found:
            self.tm.start_recognition("")
            self.tm.look_for_object("",True)
            self.move_head_srv("default")
            self.shoes_found()
        else:
            self.shoes_not_found()

    def on_enter_LOOK4DRINK(self):
        print(self.consoleFormatter.format("LOOK4DRINK", "HEADER"))
        self.awareness_srv(False)
        time.sleep(0.5)
        self.tm.start_recognition("")
        self.tm.start_recognition("front_camera")
        #self.show_topic_srv("/perception_utilities/yolo_publisher")
        self.tm.look_for_object("bottle", True)
        rospy.sleep(1)
        object_found = self.tm.wait_for_object(1)
        if object_found:
            self.tm.start_recognition("")
            self.tm.look_for_object("",True)
            if self.at_kitchen:
                self.tm.talk("Right! I'm gonna go the living room again.","English", False)
                self.drink_found_kitchen()
            else:
                self.drink_found_not_kitchen()
        else:
            self.drink_not_found()

    # ============================== ASK4 STATES ===============================
    def on_enter_ASK4SHOES(self):
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        self.tm.talk("You must not wear shoes in this place.","English", False)
        time.sleep(5)
        self.tm.talk("Please hurry up!","English")
        time.sleep(5)
        self.shoe_asked()

    def on_enter_ASK4DRINK(self):
        print(self.consoleFormatter.format("ASK4DRINK", "HEADER"))
        self.tm.talk("You must take a Drink","English", False)
        time.sleep(5)
        if self.at_kitchen:
            self.drink_asked_kitchen()
        else:
            self.drink_asked_not_kitchen()

    def on_enter_ASK2LEAVE(self):
        print(self.consoleFormatter.format("ASK2LEAVE", "HEADER"))
        self.times_spin = 0
        self.tm.talk("You must leave this room please","English", False)
        time.sleep(5)
        self.tm.talk("Please hurry up!, you really must not be here!","English")
        time.sleep(5)
        self.person_asked()

    # =============================== GO2 STATES ===============================
    def on_enter_GO2KITCHEN(self):
        print(self.consoleFormatter.format("GO2KITCHEN", "HEADER"))
        self.tm.talk("Please follow me to the kitchen","English")
        self.tm.go_to_place("door") # Definir cocina
        self.tm.wait_go_to_place() 
        self.at_kitchen = True
        self.at_living, self.at_room = False, False
        self.arrive_kitchen()

    def on_enter_GO2LIVING(self):
        print(self.consoleFormatter.format("GO2LIVING", "HEADER"))
        self.tm.go_to_place("living_room")
        self.tm.wait_go_to_place()
        self.at_living = True
        self.at_kitchen, self.at_room = False, False
        self.arrive_living()

    def on_enter_GO2ROOM(self):                                                                                     
        print(self.consoleFormatter.format("GO2ROOM", "HEADER"))
        self.tm.talk("I'm gonna check the forbidden room.","English")
        self.tm.go_to_place("door") # Definir cuarto
        self.tm.wait_go_to_place()
        self.at_room = True
        self.at_kitchen, self.at_living = False, False
        self.arrive_room()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        self.start()

# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = STICKLER_RULES()
    sm.run()
    rospy.spin()

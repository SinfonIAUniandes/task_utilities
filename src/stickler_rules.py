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
from robot_toolkit_msgs.srv import tablet_service_srv
from robot_toolkit_msgs.msg import animation_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class STICKLER_RULES(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "stickler for the rules"
        states = ['INIT', 'LOOK4PERSON', 'LOOK4SHOES', 'ASK4SHOES', 'LOOK4DRINK', 'ASK4DRINK', 'GO2KITCHEN', 'GO2LIVING', 'GO2ROOM', 'ASK2LEAVE']
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
            {'trigger': 'person_at_room', 'source': 'LOOK4PERSON', 'dest': 'ASK2LEAVE'},
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


    def callback_spin_until(self,data):
        self.angle = int(np.degrees(euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2] % (2 * math.pi)))
        return self.angle
    
    def callback_look_for_object(self,data):
        self.stop_rotation=data.data
        if self.stop_rotation:
            for i in self.ids:
                if i not in self.ids_seen:
                    self.person_seen=True
            self.shoes = True
        return data

    def callback_get_labels(self,data):
        self.labels = data.labels
        self.ids = data.ids

    def check_angle(self,desired,angle,offset)->bool:
        if desired+offset>360:
            return 0<=angle<=(desired+offset)%360 or desired-offset<=angle<=360
        elif desired-offset<0:
            return 0<=angle<=desired+offset or (desired-offset)%360<=angle<=360
        else:
            return desired-offset<=angle<=desired+offset

    def on_enter_INIT(self):
        self.tm.talk("I am going to do the "+self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",1,15) 
        self.tm.turn_camera("bottom_camera","custom",1,15)
        self.tm.go_to_place("living_room")
        self.beggining()

    # ============================== LOOK4 STATES ==============================
    def on_enter_LOOK4PERSON(self):
        print(self.consoleFormatter.format("LOOK4PERSON", "HEADER"))
        self.awareness_srv(True)
        time.sleep(0.5)
        self.animations_publisher.publish("animations","Gestures/Maybe_1")
        time.sleep(0.5)
        self.awareness_srv(False)
        self.tm.start_recognition("front_camera")
        rospy.sleep(0.5)
        self.tm.look_for_object("person",True)
        self.stop_rotation=False
        x, y, current_angle = self.get_position_proxy()
        while current_angle <= 360 and not self.person_seen:
            tm.spin_srv(45)
            self.ids_seen.extend(self.ids)
            x, y, current_angle = self.get_position_proxy()
        print("Angle: ",self.angle)
        print("El angulo es de: "+str(self.angle)+" y stop rotation es: "+str(self.stop_rotation))
        print(self.consoleFormatter.format("ROBOT STOP", "WARNING"))
        self.tm.robot_stop_srv()
        self.tm.start_recognition("")
        self.tm.look_for_object("",True)
        self.stop_rotation = False
        self.person_seen = False
        if current_angle >= 360:
            if self.at_living:
                self.full_turn_living()
            elif self.at_room:
                self.full_turn_room()
        else:
            if self.at_room:
                self.person_at_room()
            else:
                self.person_found()

    def on_enter_LOOK4SHOES(self):
        # TODO: Mirar para abajo
        print(self.consoleFormatter.format("LOOK4SHOES", "HEADER"))
        self.tm.start_recognition("")
        self.tm.start_recognition("bottom_camera")
        rospy.sleep(0.5)
        #change cellphone for shoes
        self.tm.look_for_object("cellphone",True)
        object_found = self.tm.wait_for_object(0.5)
        self.tm.start_recognition("")
        self.tm.look_for_object("",True)
        if object_found:
            self.shoes_found()
        else:
            self.shoes_not_found()

    def on_enter_LOOK4DRINK(self):
        print(self.consoleFormatter.format("LOOK4DRINK", "HEADER"))
        self.tm.start_recognition("")
        self.tm.start_recognition("front_camera")
        rospy.sleep(0.5)
        #change cellphone for cup or bottle
        self.tm.look_for_object("cellphone",True)
        object_found = self.tm.wait_for_object(0.5)
        self.tm.start_recognition("")
        self.tm.look_for_object("",True)
        if object_found:
            if self.at_kitchen:
                self.drink_found_kitchen()
            else:
                self.drink_found_not_kitchen()
        else:
            self.drink_not_found()

    # ============================== ASK4 STATES ===============================
    def on_enter_ASK4SHOES(self):
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        self.tm.talk("You must wear shoes in this place.","English")
        time.sleep(5)
        self.tm.talk("Please hurry up!","English")
        time.sleep(15)
        self.shoe_asked()

    def on_enter_ASK4DRINK(self):
        print(self.consoleFormatter.format("ASK4DRINK", "HEADER"))
        self.tm.talk("You must take a Drink","English")
        time.sleep(5)
        if self.at_kitchen:
            self.drink_asked_kitchen()
        else:
            self.drink_asked_not_kitchen()

    def on_enter_ASK2LEAVE(self):
        print(self.consoleFormatter.format("ASK2LEAVE", "HEADER"))
        self.tm.talk("You must leave this room please","English")
        time.sleep(5)
        self.tm.talk("Please hurry up!, you really must not be here!","English")
        time.sleep(10)
        self.person_asked()

    # =============================== GO2 STATES ===============================
    def on_enter_GO2KITCHEN(self):
        print(self.consoleFormatter.format("GO2KITCHEN", "HEADER"))
        self.tm.talk("Please follow me to the kitchen","English")
        self.tm.go_to_place("pantry") # Definir cocina
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
        self.tm.go_to_place("pantry") # Definir cuarto
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
        while not rospy.is_shutdown():
            self.start()

# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = STICKLER_RULES()
    sm.run()
    rospy.spin()

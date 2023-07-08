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
from perception_msgs.srv import start_recognition_srv, start_recognition_srvRequest
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srv
from robot_toolkit_msgs.msg import animation_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class STICKLER_RULES(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles 
        self.task_name = "stickler_for_the_rules"
        states = ['INIT', 'SFTR', 'GO_TO_PLACE','CHECK_RULE','SOLVE']
        self.tm = tm(perception = True , speech=True, manipulation=False, navigation=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'SFTR', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO_TO_PLACE'},
            {'trigger': 'check', 'source': 'GO_TO_PLACE', 'dest': 'CHECK_RULE'},
            {'trigger': 'solution', 'source': 'CHECK_RULE', 'dest': 'SOLVE'},
            {'trigger': 'continue_checking', 'source': 'SOLVE', 'dest': 'CHECK_RULE'},
            {'trigger': 'next_location', 'source': 'CHECK_RULE', 'dest': 'GO_TO_PLACE'},
            {'trigger': 'good_person', 'source': 'CHECK_RULE', 'dest': 'CHECK_RULE'},
        ]
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='SFTR')

        # Inicializa el thread SFTR
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        # ROS Callback
        print(self.consoleFormatter.format("Waiting for /look_for_object_publisher", "WARNING"))
        self.subscriber_look_for_object = rospy.Subscriber("/perception_utilities/look_for_object_publisher", Bool, self.callback_look_for_object_default)

        #Navigation angle
        print(self.consoleFormatter.format("Waiting for /amcl_pose", "WARNING"))
        self.subscriber_odom = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_spin_until)

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

        # ROS Services (Perception - shoes)
        rospy.wait_for_service('/perception_utilities_shoes/start_recognition_srv')
        self.start_recognition_shoes_proxy = rospy.ServiceProxy('/perception_utilities_shoes/start_recognition_srv', start_recognition_srv)

        # ROS Services (Perception - garbage)
        rospy.wait_for_service('/perception_utilities_garbage/start_recognition_srv')
        self.start_recognition_garbage_proxy = rospy.ServiceProxy('/perception_utilities_garbage/start_recognition_srv', start_recognition_srv)

        # ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        #Subscriber:
        self.get_labels_publisher = rospy.Subscriber("/perception_utilities/get_labels_publisher", get_labels_msg, self.callback_get_labels_default)
        self.get_labels_shoes_publisher = rospy.Subscriber("/perception_utilities_shoes/get_labels_publisher", get_labels_msg, self.callback_get_labels_shoes)
        self.get_labels_garbage_publisher = rospy.Subscriber("/perception_utilities_garbage/get_labels_publisher", get_labels_msg, self.callback_get_labels_garbage)
        
        ##################### ROS CALLBACK VARIABLES #####################
        
        self.angle = 0
        self.recognize_person_counter = 0
        self.angle_stop_looking_person = 359
        self.stop_rotation = False
        self.broken_rule = None
        self.time_to_rotate = 0

        # Callbacks for the perception
        self.default_labels = []
        self.shoes_labels = []
        self.garbage_labels = []
        
        self.look_for_object_default = False
        self.look_for_object_shoes = False
        self.look_for_object_garbage = False
        
        ##################### GLOBAL VARIABLES #####################

        self.places_available = ["kitchen", "bedroom", "living_room", "study"]

        self.places_i = 0

        self.places_instructions = {
            "living_room":{"angle": 360, "width": 0, "is_forbidden": False},
            "kitchen":{"angle": 360, "width": 0, "is_forbidden": False},
            "bedroom":{"angle": 180, "width": 0, "is_forbidden": True},
            "study":{"angle": 180, "width": 0, "is_forbidden": False}
        }

        self.rules = {
            "drink":{"action": "", "suggestion": ""},
            "shoes":{"action": "", "suggestion": ""},
            "garbage":{"action": "", "suggestion": ""},
            "is_forbidden":{"action": "", "suggestion": ""}
        }

    def on_enter_INIT(self):
        self.tm.talk("I am going to do the "+self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",1,15) 
        self.tm.turn_camera("bottom_camera","custom",1,15)
        #self.tm.go_to_place("living_room")
        self.beggining()

    # ============================== LOOK4 STATES ==============================
    
    def on_enter_GO_TO_PLACE(self):
        self.tm.go_to_place(self.places_available[self.places_i])
        self.check()

    def on_enter_CHECK_RULE(self):
        self.tm.talk("Looking for people breaking rules")
        self.tm.start_recognition_srv("front_camera")
        self.start_recognition_shoes_proxy("bottom_camera")
        self.start_recognition_garbage_proxy("bottom_camera")
        # Extraer el angulo que se necesita girar
        rotating_angle = self.places_instructions[self.places_available[self.places_i]]["angle"]
        angular_speed = 15
        self.tm.constant_spin_srv(15)
        # Si es la primera vez sacar cuanto tiempo se debe girar
        if self.time_to_rotate is None: #PIPE NO ENTIENDE ESTA
            self.time_to_rotate = rotating_angle/angular_speed
        t1 = time.time()
        # Wait to see person or all the angle
        self.tm.look_for_object("person")

        while(time.time()-t1 < self.time_to_rotate):
        # Check person
            if self.look_for_object_default:
                break
        self.tm.robot_stop_srv()
        # All angled covered
        # snow-julian-juanjo(el marica) TODO: SE ASUME QUE SOLO HAY UNA PERSONA EN CADA ROOM? - DEBERÍA SER >?
        if time.time()-t1 < self.time_to_rotate:
            self.time_to_rotate = None
            self.places_i += 1
            self.next_location()
            
        self.time_to_rotate = self.time_to_rotate - (time.time()-t1)
        

        if(self.places_instructions[self.places_i]["is_forbidden"] == False):
            t2 = time.time()
            drink_broken = True
            shoes_broken = False
            gargabe_broken = False
            while(time.time()-t2 < 5):
                for label in self.default_labels:
                    if((label["label"] == "bottle") and (320*0.2<label["x"]<320*0.8) or (label["label"] == "cup") and (320*0.2<label["x"]<320*0.8)):
                        print("drink found")
                        drink_broken = False
                        break
                
                for label in self.shoes_labels:
                    if( (label["label"] == "shoes") and (320*0.2<label["x"]<320*0.8) or (label["label"] == "socks") and (320*0.2<label["x"]<320*0.8)):
                        print("shoes found")
                        shoes_broken = True
                        break

                for label in self.garbage_labels:
                    if(320*0.2<label["x"]<320*0.8):
                        print("garbage found")
                        gargabe_broken = True
                        break
        else:
            self.broken_rule = "is_forbidden"
            self.solution()


        # No drink found, rule broken
        if drink_broken:
            self.broken_rule = "drink"
            self.solution()

        # Shoes not found
        elif shoes_broken:
            self.broken_rule = "shoes"
            self.solution()

        # garbage on the floor
        elif gargabe_broken:
            self.broken_rule = "garbage"
            self.solution()
        
        # TODO: DEBERIA TENER EN CUENTA EL ID DE LA PERSONA PARA NO VOLVER A VERLA
        
        # Buena people
        self.tm.talk("Guest, thank you for following the rules of the house")
        self.good_person()
                
    def on_enter_SOLVE(self):
        # Check drink 
        if(self.broken_rule == list(self.rules.keys())[0]):
            self.tm.talk(self.rules[0]["suggestion"])
            self.tm.talk(self.rules[0]["action"])

            while not self.isTouched:
                rospy.sleep(0.1)

            # TODO: Check drink again
            # Tiempo que nos damos para verificar
            isAccomplish = False
            t0 = time.time()
            while(time.time()-t0 < 5):
                for label in self.default_labels:
                    if((label["label"] == "bottle") and (320*0.2<label["x"]<320*0.8) or (label["label"] == "cup") and (320*0.2<label["x"]<320*0.8)):
                        print("drink found")
                        isAccomplish = True
                        break
        
            # TODO: if isAccomplish: Thank you gest - else: hdp
            
        # Check shoes
        elif(self.broken_rule == list(self.rules.keys())[1]):
            self.tm.talk(self.rules[0]["suggestion"])
            self.tm.talk(self.rules[0]["action"])
            
            while not self.isTouched:
                rospy.sleep(0.1)

            # TODO: Check shoes again
            
        # Check garbage
        elif(self.broken_rule == list(self.rules.keys())[2]):
            self.tm.talk(self.rules[0]["suggestion"])
            self.tm.talk(self.rules[0]["action"])
            
            while not self.isTouched:
                rospy.sleep(0.1)

            # TODO Check garbage again

        # Is forbidden
        elif(self.broken_rule == list(self.rules.keys())[3]):
            self.tm.talk(self.rules[0]["suggestion"])
            self.tm.talk(self.rules[0]["action"])
            
            while not self.isTouched:
                rospy.sleep(0.1)

        self.broken_rule = None    
        self.continue_checking()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        self.start()

    def callback_spin_until(self,data):
        self.angle = int(np.degrees(euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2] % (2 * math.pi)))
        return self.angle

    def callback_get_labels_default(self,data):
        labels = data.labels
        ids = data.ids
        widths = data.widths
        heights = data.heights
        x=data.x_coordinates
        y=data.y_coordinates
        self.default_labels =[]
        for i in range(0, len(self.heights)):
            label = {"label": self.labels[i], "id": self.ids[i], "width": self.widths[i], "height": self.heights[i], "x": self.x[i], "y": self.y[i]}
            self.default_labels.append(label)

    #TODO incluir la subsccripcion al topico como tal
    def callback_get_labels_shoes(self,data):
        self.labels = data.labels
        self.ids = data.ids
        self.widths = data.widths
        self.heights = data.heights
        self.default_labels =[]
        for i in range(0, len(self.heights)):
            label = {"label": self.labels[i], "id": self.ids[i], "width": self.widths[i], "height": self.heights[i]}
            self.shoes_labels.append(label)

    #TODO incluir la subiscrcion al topico como tal
    def callback_get_labels_garbage(self,data):
        self.labels = data.labels
        self.ids = data.ids
        self.widths = data.widths
        self.heights = data.heights
        self.default_labels =[]
        for i in range(0, len(self.heights)):
            label = {"label": self.labels[i], "id": self.ids[i], "width": self.widths[i], "height": self.heights[i]}
            self.garbage_labels.append(label)

    def callback_look_for_object_default(self,data):
        self.look_for_object_default = data.data

    def callback_head_sensor_subscriber(self, msg):
        if msg.name == "head_rear":
            self.sensorRear = msg.state
        elif msg.state == "head_middle":
            self.sensorMiddle = msg.state
        elif msg.state == "head_front":
            self.sensorFront = msg.state
        if self.sensorFront or self.sensorMiddle or self.sensorRear:
            self.isTouched=True
        else:
            self.isTouched=False

# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = STICKLER_RULES()
    sm.run()
    rospy.spin()

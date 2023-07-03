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
from std_msgs.msg import String, touch_msg

from navigation_msgs.srv import constant_spin_srv
from perception_msgs.msg import get_labels_msg
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srv
from robot_toolkit_msgs.msg import animation_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class CML(object):
    def __init__(self):
        
        # TODO
        """
        - Revisar si se meten los servicios de interface o del pytoolkit directamente (ojala nodo de interface)
        - Falta meter todos los servicios del pytoolkit al modulo para que se puedan llamar facilmente desde la maquina de estados.
        - Falta crear behaviours como el spin_until_object que es usado varias veces en varios tasks.
        - Falta revisar todos los angulos de navegacion al hacer look_4_something y la velocidad de giro 
        """

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "carry_my_luggage"
        states = ['INIT', 'CHOOSE_BAG', 'GRAB_BAG', 'FOLLOW']   
        self.tm = tm(perception = True,speech=True,manipulation=True, navigation=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'CML', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'CHOOSE_BAG'},
            {'trigger': 'bag_not_chosen', 'source': 'CHOOSE_BAG', 'dest': 'CHOOSE_BAG'},
            {'trigger': 'bag_chosen', 'source': 'CHOOSE_BAG', 'dest': 'GRAB_BAG'},
            {'trigger': 'bag_grabbed', 'source': 'GRAB_BAG', 'dest': 'FOLLOW'},
            
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='CML')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        # ROS Callbacks

        # ROS Services (PyToolkit)
        print(self.consoleFormatter.format("Waiting for pytoolkit/awareness...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        self.awareness_srv = rospy.ServiceProxy("/pytoolkit/ALBasicAwareness/set_awareness_srv",SetBool)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/move_head...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        self.move_head_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/move_head_srv",move_head_srv)

        print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/show_image_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.show_image_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_image_srv",tablet_service_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
        self.show_topic_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv",tablet_service_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/autononumusLife...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALAutonomousLife/set_state_srv")
        self.autonomous_life_srv = rospy.ServiceProxy("/pytoolkit/ALAutonomousLife/set_state_srv",SetBool)
        
        # ROS subscribers (perception)
        print(self.consoleFormatter.format("Waiting for /perception_utilities/get_labels_publisher", "WARNING"))
        self.get_labels_publisher = rospy.Subscriber("/perception_utilities/get_labels_publisher", get_labels_msg, self.callback_get_labels)

        print(self.consoleFormatter.format("Waiting for /perception_utilities/pose_publisher", "WARNING"))
        self.pose_publisher = rospy.Subscriber("/perception_utilities/pose_publisher", String, self.callback_pose)

        # ROS subscribers (head)
        print(self.consoleFormatter.format("Waiting for /touch", "WARNING"))
        subscriber_touch= rospy.Subscriber("/touch", touch_msg, self.callback_touch)

        # ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        ##################### ROS CALLBACK VARIABLES #####################
        self.pose = ""
        self.touch = False
        ##################### GLOBAL VARIABLES #####################
        self.initial_place="init"

    def callback_pose(self,data):
        self.pose = data

    def callback_touch(self,data):
        if("head" in data.name and data.state == True):
            self.touch = True
        else:
            self.touch = False

    def on_enter_INIT(self):
        self.tm.set_current_place(self.initial_place)
        self.autonomous_life_srv(False)
        self.tm.talk("I am going to do the carry my luggage task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",1,15) 
        self.awareness_srv(False)
        #TODO
        self.tm.go_to_place("choose_bag")
        self.beggining()
    
    def on_enter_CHOOSE_BAG(self):
        self.tm.talk("Please, choose a bag, signal the bag you want me to grab","English")
        possible_options = ["Pointing to the left","Pointing to the right"]
        t1 = time.time()
        while self.pose not in possible_options and time.time()-t1<5:
            time.sleep(0.05)
        if self.pose == "Pointing to the left":
            self.tm.go_to_place("left_bag")
            self.bag_chosen()
        elif self.pose == "Pointing to the right":
            self.tm.go_to_place("right_bag")   
            self.bag_chosen() 
        else:
            self.bag_not_chosen()

    def on_enter_GRAB_BAG(self):
        self.tm.talk("I am going to grab the bag","English")
        self.tm.go_to_pose("small_object_right_hand")
        self.tm.talk("Please place the bag in my hand, when you are ready touch my head","English")
        while not self.touch:
            time.sleep(0.05)
        self.tm.talk("Thank you","English")
        self.bag_grabbed()

    def on_enter_FOLLOW(self):
        self.tm.talk("I am going to follow you","English")
        #TODO
        self.tm.go_to_place("outside")
        self.tm.talk("I am leaving your bag here for you","English")
        self.tm.execute_trayectory("place_right_arm")
            

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
    sm = CML()
    sm.run()
    rospy.spin()

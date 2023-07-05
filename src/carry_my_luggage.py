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
from std_msgs.msg import String

from navigation_msgs.srv import constant_spin_srv
from perception_msgs.msg import get_labels_msg
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srv, misc_tools_srv
from robot_toolkit_msgs.msg import animation_msg,touch_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class CML(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "carry_my_luggage"
        states = ['INIT', 'CHOOSE_BAG', 'GRAB_BAG', 'FOLLOW','RETURN_TO_HOUSE']   
        self.tm = tm(perception = True,speech=True,manipulation=True, navigation=False, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'CML', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'CHOOSE_BAG'},
            {'trigger': 'bag_not_chosen', 'source': 'CHOOSE_BAG', 'dest': 'CHOOSE_BAG'},
            {'trigger': 'bag_chosen', 'source': 'CHOOSE_BAG', 'dest': 'GRAB_BAG'},
            {'trigger': 'bag_grabbed', 'source': 'GRAB_BAG', 'dest': 'FOLLOW'},
            {'trigger': 'return_home', 'source': 'FOLLOW', 'dest': 'RETURN_TO_HOUSE'},
            
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
        
        print(self.consoleFormatter.format("Waiting for robot_toolkit/misc_tools_srv...", "WARNING"))
        rospy.wait_for_service("/robot_toolkit/misc_tools_srv")
        self.misc_tools_srv = rospy.ServiceProxy("/robot_toolkit/misc_tools_srv",misc_tools_srv)

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
        self.place_counter=0

    def callback_pose(self,data):
        self.pose = data.data

    def callback_touch(self,data):
        if("head" in data.name and data.state == True):
            self.touch = True
        else:
            self.touch = False

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.show_image("sinfonia")
        self.tm.add_place("place"+str(self.place_counter))
        self.place_counter+=1
        self.tm.talk("I am going to do the carry my luggage task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",1,15) 
        self.tm.start_recognition("front_camera")
        self.tm.pose_srv("front_camera")
        self.awareness_srv(False)
        #TODO
        self.tm.go_to_place("choose_bag")
        self.beggining()
    
    def on_enter_CHOOSE_BAG(self):
        print(self.consoleFormatter.format("CHOOSE_BAG", "HEADER"))
        self.tm.show_topic("/perception_utilities/pose_image_publisher")
        self.tm.talk("Please, choose a bag, signal the bag you want me to grab","English")
        self.tm.talk("Make sure your arms appear in my tablet","English")
        possible_options = ["Pointing to the left","Pointing to the right"]
        t1 = time.time()
        while self.pose not in possible_options and time.time()-t1<8:
            time.sleep(0.05)
        if self.pose == "Pointing to the left":
            self.tm.talk("I am going to grab the bag on my left","English")
            self.tm.go_to_place("left_bag")
            self.bag_chosen()
        elif self.pose == "Pointing to the right":
            self.tm.talk("I am going to grab the bag on my right","English")
            self.tm.go_to_place("right_bag")   
            self.bag_chosen() 
        else:
            self.bag_not_chosen()

    def on_enter_GRAB_BAG(self):
        print(self.consoleFormatter.format("GRAB_BAG", "HEADER"))
        self.tm.talk("I am going to grab the bag","English")
        self.tm.go_to_pose("small_object_right_hand")
        self.tm.talk("Please place the bag in my hand, when you are ready touch my head","English")
        while not self.touch:
            time.sleep(0.05)
        self.tm.go_to_pose("close_right_hand")
        self.tm.talk("Thank you","English")
        self.bag_grabbed()

    def on_enter_FOLLOW(self):
        print(self.consoleFormatter.format("FOLLOW", "HEADER"))
        self.tm.talk("I am going to follow you","English")
        #TODO 
        self.tm.go_to_place("outside")
        self.tm.talk("Could you pick up your bag?","English")
        self.tm.go_to_pose("open_right_hand")
        self.tm.talk("Touch my head when when you had picked up your bag","English")
        while not self.touch:
            time.sleep(0.05)
        self.tm.talk("Thank you","English")
        self.tm.execute_trayectory("place_right_arm")
        self.return_home()

    def on_enter_RETURN_TO_HOUSE(self):
        print(self.consoleFormatter.format("RETURN_TO_HOUSE", "HEADER"))
        self.tm.talk("Returning home","English")
        self.tm.go_to_place("door")
        print(self.consoleFormatter.format("TASK FINISHED", "OKGREEN"))
        os._exit(os.EX_OK)

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

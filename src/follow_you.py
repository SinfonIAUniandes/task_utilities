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
from code_generation import ls_generate as gen
from code_generation import generate_utils 
from code_generation.database.models import Model

from robot_toolkit_msgs.msg import touch_msg

from navigation_msgs.srv import constant_spin_srv
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.msg import animation_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class FOLLOW_YOU(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "TEST"
        self.isTouched = False
        states = ['TEST', 'INIT', 'FOLLOW_YOU', 'FINISH']
        self.tm = tm(perception = True, speech=False, manipulation=False, navigation=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'TEST', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'FOLLOW_YOU'},
            {'trigger': 'stop', 'source': 'FOLLOW_YOU', 'dest': 'FINISH'}
        ]

        self.handSensorSubscriber = rospy.Subscriber(
            "/touch", touch_msg, self.callback_hand_sensor_subscriber
        )
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='TEST')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ############################# STATES #############################

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.initialize_pepper()
        print("I am going to test follow you")
        #self.tm.talk("I am going to test follow you","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.beggining()

    def on_enter_FOLLOW_YOU(self):
        print(self.consoleFormatter.format("FOLLOW_YOU", "HEADER"))
        self.tm.follow_you(True)
        print("Follow you activated!")
        while self.isTouched == False:
            time.sleep(0.1)
        self.stop()
            
    def on_enter_FINISH (self):
        self.tm.follow_you(False)
        print(self.consoleFormatter.format("FINISH", "HEADER"))
        self.tm.talk("I have finished the "+self.task_name+" task","English")
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
    
    def callback_hand_sensor_subscriber(self, msg: touch_msg):
        if "hand" in msg.name:
            self.isTouched = msg.state
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = FOLLOW_YOU()
    sm.run()
    rospy.spin()

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



from navigation_msgs.srv import constant_spin_srv
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.msg import animation_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class MANIPULATION_TEST(object):
    def __init__(self):
        self.gen = gen.LongStringGenerator()
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "MANIPULATION_TEST"
        states = ['INIT', 'GO2POSE', 'GRASPOBJ', 'PLAYACTION', 'MOVEHEAD', 'FINISH']
        self.tm = tm(perception = False,speech=False,manipulation=False, navigation=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'MANIPULATION_TEST', 'dest': 'INIT'},
            {'trigger': 'test_go2pose', 'source': 'INIT', 'dest': 'GO2POSE'},
            {'trigger': 'test_grasp_object', 'source': 'GO2POSE', 'dest': 'GRASPOBJ'},
            {'trigger': 'test_play_action', 'source': 'GRASPOBJ', 'dest': 'PLAYACTION'},
            {'trigger': 'test_move_head', 'source': 'PLAYACTION', 'dest': 'MOVEHEAD'},
            {'trigger': 'finish', 'source': 'MOVEHEAD', 'dest': 'FINISH'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='GPSR')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ############################# GLOBAL VARIABLES #############################

    def on_enter_INIT(self):
        print("entra aqui")
        
        self.test_go2pose()

    def on_enter_GO2POSE(self):
        print("entra aqui")
        
        self.test_grasp_object()

    def on_enter_GRASPOBJ(self):
        print("entra aqui")
        
        self.test_play_action()

    def on_enter_PLAYACTION(self):
        print("entra aqui")
        
        self.test_move_head()
        
    def on_enter_MOVEHEAD(self):
        print("entra aqui")
        
        self.finish()
        
    def on_enter_FINISH(self):
        print("entra aqui")
        
        self.person_arrived()

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
    sm = MANIPULATION_TEST()
    sm.run()
    rospy.spin()

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

from navigation_msgs.srv import constant_spin_srv
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.srv import tablet_service_srv
from robot_toolkit_msgs.msg import animation_msg, touch_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class ROBOT_INSPECTION(object):
    def __init__(self):
        
        # TODO
        """
        - Revisar si se meten los servicios de interface o del pytoolkit directamente (ojala nodo de interface)
        - Falta meter todos los servicios del pytoolkit al modulo para que se puedan llamar facilmente desde la maquina de estados.
        - Falta crear behaviours como el spin_until_object que es usado varias veces en varios tasks.
        - Falta revisar todos los angulos de navegacion al hacer look_4_something y la velocidad de giro 
        - Poner una redundancia para cuando el robot asigna un nuevo a id a una misma persona para que no la presente (lista con los nombres de los que ya presento, incluyendo el actual)
        """

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "robot inspection"
        states = ['INIT', 'GO2TABLE', 'TOUCHING', 'GO2DOOR']
        self.tm = tm(perception = False, speech=True,manipulation=False, navigation=True)
        self.tm.initialize_node("robot_inspection")
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'ROBOT_INSPECTION', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO2TABLE'},
            {'trigger': 'waiting', 'source': 'GO2TABLE', 'dest': 'TOUCHING'},
            {'trigger': 'endings', 'source': 'TOUCHING', 'dest': 'GO2DOOR'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='ROBOT_INSPECTION')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        # ROS Callbacks
        print(self.consoleFormatter.format("Waiting for /touch", "WARNING"))
        subscriber_touch= rospy.Subscriber("/touch", touch_msg, self.callback_touch)

        ##################### ROS CALLBACK VARIABLES #####################
        self.touch = 0

        ##################### GLOBAL VARIABLES #####################

    def callback_touch(self,data):
        if(data.name == "head_front" and data.state == True):
            self.touch = 1
        else:
            self.touch = 0
                
    def on_enter_INIT(self):
        self.tm.talk("I am going to do the  "+ self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.beggining()
                
    def on_enter_GO2TABLE(self):
        print(self.consoleFormatter.format("GO2TABLE", "HEADER"))
        self.tm.talk("I am going to the table position","English",wait=False)
        self.tm.go_to_place("table")
        self.tm.talk("I'm in the table position","English",wait=False)
        self.waiting()
    
    def on_enter_TOUCHING(self):
        self.tm.talk("Touch my head please","English",wait=False)
        while self.touch < 1:
            print("Head not touched")

        self.tm.talk("My head has been touched","English",wait=False)
        self.endings()

    def on_enter_GO2DOOR(self):
        print(self.consoleFormatter.format("GO2DOOR", "HEADER"))
        self.tm.talk("I am going to the door position","English",wait=False)
        self.tm.go_to_place("door")
        self.tm.talk("Robot inspection completed","English", wait=False)
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
    sm = ROBOT_INSPECTION()
    sm.run()
    rospy.spin()
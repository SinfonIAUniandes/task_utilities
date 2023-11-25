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

class GPSR(object):
    def __init__(self):
        self.gen = gen.LongStringGenerator()
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
        self.task_name = "ANSWERTEST"
        states = ['INIT', 'MAL', 'GPSR', 'BIEN', "ANSWER"]
        self.tm = tm(perception = True,speech=True,manipulation=False, navigation=False, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'GPSR', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'ANSWER'},
            {'trigger': 'bien', 'source': 'ANSWER', 'dest': 'BIEN'},
            {'trigger': 'mal', 'source': 'ANSWER', 'dest': 'MAL'},
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='GPSR')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ############################# GLOBAL VARIABLES #############################
        self.location = "receptionist"

    def on_enter_INIT(self):
        self.tm.talk("I am going to do the  "+self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.beggining()
    
    def on_enter_ANSWER(self):
        if self.tm.answer():
            self.bien()
        else:
            self.mal()

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
    sm = GPSR()
    sm.run()
    rospy.spin()

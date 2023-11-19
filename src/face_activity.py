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
from code_generation import generate as gen

from navigation_msgs.srv import constant_spin_srv
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.msg import animation_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class GPSR(object):
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
        self.task_name = "GPSR"
        states = ['INIT', 'WAIT4GUEST', 'GPSR', 'GO2GPSR']
        self.tm = tm(perception = True,speech=True,manipulation=True, navigation=False, pytoolkit=False)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'GPSR', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO2GPSR'},
            {'trigger': 'go_to_gpsr', 'source': 'GO2GPSR', 'dest': 'WAIT4GUEST'},
            {'trigger': 'person_arrived', 'source': 'WAIT4GUEST', 'dest': 'GPSR'},
            {'trigger': 'GPSR_done', 'source': 'GPSR', 'dest': 'GO2GPSR'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='GPSR')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ############################# GLOBAL VARIABLES #############################
        self.location = "bookshelf"

    def on_enter_INIT(self):
        self.tm.go_to_pose("up_head")
        self.tm.turn_camera("front_camera","custom",1,15) 
        self.tm.start_recognition("front_camera")
        while True:
            self.tm.talk("Waiting for guests")
            self.tm.look_for_object("person",False)
            self.tm.wait_for_object(-1)        
            self.tm.look_for_object("",True)
            time.sleep(2)
            person_name = "Unknown"
            self.tm.talk("Recognizing person")
            person_name = self.tm.recognize_face(3)
            if person_name in ["Unknown","NO_PERSON_DETECTED","ERROR"]:
                self.tm.talk("I don't recognize you. ")
                person_name = self.tm.q_a_speech("name")
                self.tm.talk(f"{person_name}, will take some pictures of your face to recognize you in future occasions")
                succed = self.tm.save_face(person_name,5)
            else: 
                self.tm.talk(f'Hello {person_name}, welcome to the event')
            rospy.sleep(2)
        self.beggining()

    def on_enter_GPSR(self):
        print(self.consoleFormatter.format("GPSR", "HEADER"))
        self.tm.look_for_object("")
        self.tm.talk("Hello guest, please tell me what you want me to do, I will try to execute the task you give me. Please talk loud and say the task once. Talk to me now: ","English")
        task = self.tm.speech2text_srv("gpsr",10,True)
        print("task",task)
        code = gen.generate_code(task)
        print(code)

        self.tm.talk("I will: "+task,"English")
        try:
            exec(code)
        except:
            self.tm.talk("I cannot do this task: "+task,"English")
        self.GPSR_done()

    def on_enter_GO2GPSR(self):
        print(self.consoleFormatter.format("GO2GPSR", "HEADER"))
        self.tm.talk("I am going to the GPSR location","English")
        self.tm.go_to_place(self.location)
        self.tm.go_to_defined_angle_srv(0)
        self.go_to_gpsr()

    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.talk("Waiting for guests","English")
        self.tm.look_for_object("person")
        self.tm.wait_for_object(-1)
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
    sm = GPSR()
    sm.run()
    rospy.spin()

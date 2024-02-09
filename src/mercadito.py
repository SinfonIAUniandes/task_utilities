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

class MERCADITO(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "MERCADITO"
        self.is_done = False
        self.hey_pepper=False
        self.isTouched = False
        states = ['MERCADITO','INIT', 'FOLLOW_YOU','FINISH','MERCADITO_DONE']
        self.tm = tm(perception = True,speech=True,manipulation=True, navigation=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'MERCADITO', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'FOLLOW_YOU'},
            {'trigger': 'market_ready', 'source': 'FOLLOW_YOU', 'dest': 'FINISH'},
            {'trigger': 'finish', 'source': 'FINISH', 'dest': 'MERCADITO_DONE'},
        ]

        self.handSensorSubscriber = rospy.Subscriber(
            "/touch", touch_msg, self.callback_hand_sensor_subscriber
        )
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='MERCADITO')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ############################# STATES #############################

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.initialize_pepper()
        self.tm.talk("I am going to do the shopping task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.talk("Hello I will help you with your shopping today, when you are ready put your basket in my hands","English")
        self.tm.go_to_pose("basket", 0.1)
        self.tm.go_to_pose("open_both_hands", 0.1)
        if self.isTouched == True:
            self.beggining()

    def on_enter_FOLLOW_YOU(self):
        print(self.consoleFormatter.format("FOLLOW_YOU", "HEADER"))
        self.tm.follow_you()
        stop_thread = threading.Thread(target=self.stop_thread)
        stop_thread.start()

        while not self.is_done:
            if self.hey_pepper:
                self.hey_pepper_function()
                self.hey_pepper=False
            time.sleep(0.1)

        self.market_ready()

    def on_enter_FININSH(self):
        print(self.consoleFormatter.format("FINISH", "HEADER"))
        self.tm.follow_you()
        while not self.is_done:
            time.sleep(0.1)
        self.finish()

    def on_enter_MERCADITO_DONE(self):
        print(self.consoleFormatter.format("MERCADITO_DONE", "HEADER"))
        self.tm.talk("I have finished the "+self.task_name+" task","English")
        os._exit(os.EX_OK)

    def hey_pepper_function(self):
        self.tm.get_labels(True)
        text = self.tm.speech2text(seconds = 5)
        labels=self.tm.get_labels(False)
        request = f"""The person asked: {text}. You can see the labels: {", ".join(labels)}"""
        answer=self.tm.answer_question(request)
        self.tm.talk(answer,"English")
    # def callback_word_recognition(req):
    #     if req.word == "stop":
    #         self.tm.follow_you(False)
    #         self.is_done = True
    #     elif req.word == "hi_pepeper":
    #         self.hey_pepper = True
    
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
    sm = MERCADITO()
    sm.run()
    rospy.spin()

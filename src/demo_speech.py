#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from std_msgs.msg import Bool, String
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

class DEMO(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.is_done = False
        self.hey_pepper = False
        # Definir los estados posibles del semáforo
        self.task_name = "DEMO_SPEECH"
        states = ['DEMO_SPEECH','INIT', 'WAITING4WAKEWORD','ANSWERQUESTION']
        self.tm = tm(speech=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'MERCADITO', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'WAITING4WAKEWORD'},
            {'trigger': 'word_listened', 'source': 'WAITING4WAKEWORD', 'dest': 'ANSWERQUESTION'},
            {'trigger': 'question_answered', 'source': 'ANSWERQUESTION', 'dest': 'WAITING4WAKEWORD'},
            {'trigger': 'stop_listened', 'source': 'WAITING4WAKEWORD', 'dest': 'DONE'},
        ]

        self.handSensorSubscriber = rospy.Subscriber(
            "/touch", touch_msg, self.callback_hand_sensor_subscriber
        )
        self.hotwordSubscriber = rospy.Subscriber("/speech_utilities/hotword",String,self.callback_hot_word)
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='MERCADITO')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ############################# STATES #############################

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.initialize_pepper()
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.beggining()

    def on_enter_WAITING4WAKEWORD(self):
        print(self.consoleFormatter.format("WAITING4WAKEWORD", "HEADER"))
        self.tm.hot_word(["hello","stop"])
        while not self.is_done:
            if self.hey_pepper:
                self.word_listened()
                self.tm.hot_word([])
                self.hey_pepper=False
            time.sleep(0.1)
        self.tm.hot_word([])
        self.stop_listened()

    def on_enter_ANSWERQUESTION(self):
        print(self.consoleFormatter.format("ANSWERQUESTION", "HEADER"))
        self.tm.talk("Please tell me your question", "English")
        question = self.tm.speech2text_srv()
        answer = self.tm.answer_question(question)
        self.tm.talk(answer,"English")
        self.question_answered()

    def on_enter_DONE(self):
        print(self.consoleFormatter.format("DONE", "HEADER"))
        self.tm.talk("I have finished the "+self.task_name+" task","English")
        os._exit(os.EX_OK)

    def callback_hot_word(self,data):
        word = data.data
        if word == "stop":
            self.is_done = True
        elif word == "hello":
            self.hey_pepper = True
    
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
    sm = DEMO()
    sm.run()
    rospy.spin()

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

from robot_toolkit_msgs.msg import touch_msg,speech_recognition_status_msg

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
        self.language = "Spanish"
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

        # self.handSensorSubscriber = rospy.Subscriber(
        #     "/touch", touch_msg, self.callback_hand_sensor_subscriber
        # )
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='MERCADITO')
        subscriber = rospy.Subscriber("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,self.callback_hot_word)
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ############################# STATES #############################

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.initialize_pepper()
        if self.language =="Spanish":
        
            self.tm.talk("Voy a hacer la tarea de la compra", self.language)
        else:
            self.tm.talk("I am going to do the shopping task","English")
            
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.go_to_pose("basket", 0.1)
        self.tm.go_to_pose("open_both_hands", 0.1)
        if self.language == "Spanish":
            self.tm.talk("Hola soy nova, te voy a ayudar con tus compras hoy, cuando estes listo pon la canasta en mis manos", self.language)
        else:
            self.tm.talk("Hello I will help you with your shopping today, when you are ready put your basket in my hands","English")
        self.beggining()

    def on_enter_FOLLOW_YOU(self):
        print(self.consoleFormatter.format("FOLLOW_YOU", "HEADER"))
        if self.language == "Spanish":
 
            self.tm.talk("Cuando tengas una pregunta sobre la comida por favor di Hey nova y espera a que yo responda. Si quieres que pare y te entregue la canasta por favor di detente nova","Spanish", wait=False)
        else:
            self.tm.talk("When you have a question regarding your food please say Hey nova. If you want me to stop and hand you the basket say Stop","English", wait=False)
            
        time.sleep(1)
        self.tm.hot_word(["hey nova","oye nova" ,"stop", "detente nova"], threshold= [0.3, 0.3, 0.7, 0.3 ])
        print("true")
        self.tm.get_labels(True)
        self.tm.follow_you(True) 
        while not self.is_done:
            if self.hey_pepper:
                print(self.consoleFormatter.format("Hey Pepper detected ", "WARNING"))
                self.hey_pepper_function()
                self.hey_pepper=False
            time.sleep(0.1)
        print(self.consoleFormatter.format("Stop detected ", "WARNING"))
        self.market_ready()

    def on_enter_FININSH(self):
        print(self.consoleFormatter.format("FINISH", "HEADER"))
        print("False")
        self.tm.follow_you(False)
        self.finish()

    def on_enter_MERCADITO_DONE(self):
        print(self.consoleFormatter.format("MERCADITO_DONE", "HEADER"))
        if self.language == "Spanish":
            self.tm.talk("He terminado la tarea de mercado","Spanish")
        else:
            self.tm.talk("I have finished the "+self.task_name+" task","English")
        os._exit(os.EX_OK)

    def hey_pepper_function(self):
        self.tm.get_labels(True)
        if self.language == "Spanish":
            self.tm.talk("Cual es tu pregunta?","Spanish",wait=False)
        else:
            self.tm.talk("What is your question?","English",wait=False)
            
        time.sleep(1)
        text = self.tm.speech2text_srv() 
        # labels=self.tm.get_labels(False)
        if self.language =="Spanish":
            gpt_vision_prompt = "Que objetos tiene la persona, responde con un texto en formato de lista de python donde cada item de la lista es el nombre cada objeto sostenido por la persona"
            labels = self.tm.img_description(gpt_vision_prompt)["message"]
            print(labels)
            request = f"""La persona pregunto: {text}. Mientras la persona habló, tu viste los siguientes objetos: {labels}. Por favor contesta en español."""
        else:
            request = f"""The person asked: {text}.While the person spoke, you saw the next objects: {labels}"""
        answer=self.tm.answer_question(request)
        self.tm.talk(answer,self.language)
        self.tm.get_labels(True)
        self.tm.follow_you(True)

    def callback_hot_word(self,data):
        word = data.status
        if word == "stop pepper" or word=="detente nova":
            print("False")
            self.tm.follow_you(False)
            self.is_done = True
        elif word == "hey nova" or word =="oye nova": 
            print("False")
            self.tm.follow_you(False)
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
    
    # def callback_hand_sensor_subscriber(self, msg: touch_msg):
    #     if "hand" in msg.name:
    #         self.hey_pepper = True
    #     if "head" in msg.name:
    #         print("head_touched")
    #         self.tm.follow_you(False)
    #         self.is_done = True
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = MERCADITO()
    sm.run()
    rospy.spin()
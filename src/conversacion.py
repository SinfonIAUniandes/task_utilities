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
       self.task_name = "CONVERSACION"
       self.is_done = False
       self.hey_pepper=False
       self.isTouched = False
       states = ['CONVERSACION','INIT', 'HABLAR','FINISH','CONVERSACION_DONE']
       self.tm = tm(speech=True, pytoolkit=True)
       self.tm.initialize_node(self.task_name)
       # Definir las transiciones permitidas entre los estados
       transitions = [
           {'trigger': 'start', 'source': 'CONVERSACION', 'dest': 'INIT'},
           {'trigger': 'beggining', 'source': 'INIT', 'dest': 'HABLAR'},
           {'trigger': 'hablar_ready', 'source': 'HABLAR', 'dest': 'FINISH'},
           {'trigger': 'finish', 'source': 'FINISH', 'dest': 'CONVERSACION_DONE'},
       ]

       # Crear la máquina de estados
       self.machine = Machine(model=self, states=states, transitions=transitions, initial='CONVERSACION')
       rospy_check = threading.Thread(target=self.check_rospy)
       rospy_check.start()

       ############################# STATES #############################

   def on_enter_INIT(self):
       print(self.consoleFormatter.format("INIT", "HEADER"))
       self.tm.initialize_pepper()
       self.tm.talk("I am going to do the conversation task","English")
       print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
       subscriber = rospy.Subscriber("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,self.callback_hot_word)
       self.beggining()

   def on_enter_HABLAR(self):
       print(self.consoleFormatter.format("HABLAR", "HEADER"))
       self.tm.talk("Hello my name is Nova, say hey nova to talk with me. Please talk slow, clear and loud.","English")
       self.tm.hot_word(["hey nova", "stop"], thresholds=[0.4, 0.5])
       while not self.is_done:
            if self.hey_pepper:
                self.hey_pepper_function()
                self.hey_pepper=False
            time.sleep(0.1)
       self.tm.talk("have a great day!")
       self.tm.talk("I have finished the "+self.task_name+" task","English")
       self.hablar_ready()

   def on_enter_FININSH(self):
       print(self.consoleFormatter.format("FINISH", "HEADER"))
       
       self.finish()

   def on_enter_CONVERSACION_DONE(self):
       print(self.consoleFormatter.format("CONVERSACION_DONE", "HEADER"))
       
       os._exit(os.EX_OK)

   def hey_pepper_function(self):
       self.tm.talk("What is your question?","English")
       text = self.tm.speech2text_srv() 
       request = f"""The person asked: {text}."""
       answer=self.tm.answer_question(request) 
       self.tm.talk(answer,"English")

   def callback_hot_word(self,data):
       word = data.status
       print(word, " listened")
       if word == "stop":
            self.is_done = True
       elif word == "hey nova":
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
   
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
   sm = MERCADITO()
   sm.run()
   rospy.spin()

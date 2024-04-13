#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os

from robot_toolkit_msgs.msg import speech_recognition_status_msg
from robot_toolkit_msgs.srv import set_output_volume_srv,battery_service_srv
from perception_msgs.msg import get_labels_msg

class MERCADITO(object):
   def __init__(self):

       self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
       # Definir los estados posibles del semáforo
       self.task_name = "CONVERSACION"
       self.is_done = False
       self.hey_pepper=False
       self.bye = False
       self.exploring = False
       self.person_frames = 0
       states = ['CONVERSACION','INIT', 'HABLAR', 'EXPLORE','FINISH','CONVERSACION_DONE']
       self.tm = tm(perception = True,speech=True, pytoolkit=True)
       self.tm.initialize_node(self.task_name)
       # Definir las transiciones permitidas entre los estados
       transitions = [
           {'trigger': 'start', 'source': 'CONVERSACION', 'dest': 'INIT'},
           {'trigger': 'beggining', 'source': 'INIT', 'dest': 'EXPLORE'},
           {'trigger': 'hablar_ready', 'source': 'EXPLORE', 'dest': 'HABLAR'},
           {'trigger': 'repetir', 'source': 'HABLAR', 'dest': 'EXPLORE'},
           {'trigger': 'repetir2', 'source': 'EXPLORE', 'dest': 'EXPLORE'},
           {'trigger': 'acabar', 'source': 'HABLAR', 'dest': 'FINISH'},
           {'trigger': 'finish', 'source': 'FINISH', 'dest': 'CONVERSACION_DONE'},
       ]

       # Crear la máquina de estados
       self.machine = Machine(model=self, states=states, transitions=transitions, initial='CONVERSACION')
       
       rospy_check = threading.Thread(target=self.check_rospy)
       rospy_check.start()
       self.get_labels_publisher = rospy.Subscriber('/perception_utilities/get_labels_publisher', get_labels_msg, self.tm.callback_get_labels_subscriber)
       hot_word_subscriber = rospy.Subscriber("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,self.callback_hot_word)
       rospy.wait_for_service("/pytoolkit/ALNavigation/start_exploring_srv")
       self.start_exploring_srv_proxy = rospy.ServiceProxy("/pytoolkit/ALNavigation/start_exploring_srv", set_output_volume_srv)
       print(self.consoleFormatter.format("Waiting for pytoolkit/ALNavigation/start_exploring_srv...", "WARNING"))
       rospy.wait_for_service("/pytoolkit/ALNavigation/stop_exploring_srv")
       self.stop_exploring_srv_proxy = rospy.ServiceProxy("/pytoolkit/ALNavigation/stop_exploring_srv", battery_service_srv)
       print(self.consoleFormatter.format("Waiting for pytoolkit/ALNavigation/stop_exploring_srv...", "WARNING"))
       person_thread = threading.Thread(target=self.tm.get_closer_person)
       person_thread.start()
       head_thread = threading.Thread(target=self.tm.head_srv_thread)
       head_thread.start()

       ############################# STATES #############################

   def on_enter_INIT(self):
       print(self.consoleFormatter.format("INIT", "HEADER"))
       self.tm.initialize_pepper()
       self.tm.talk("I am going to do the meet and greet task","English")
       self.tm.hot_word(["hey nova", "stop","bye"], thresholds=[0.4, 0.5,0.45])
       print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
       self.beggining()

   def on_enter_EXPLORE(self):
       print(self.consoleFormatter.format("EXPLORAR", "HEADER"))
       self.bye = False
       explore_thread = threading.Thread(target=self.explore_thread)
       explore_thread.start()
       while self.exploring:
           if self.person_frames>30:
               self.exploring = False
           elif "person" in self.tm.labels:
               self.person_frames+=1
           else:
               self.person_frames=0
       self.person_frames=0
       self.tm.stop_moving()
       self.stop_exploring_srv_proxy()
       self.stop_exploring_srv_proxy()
       self.stop_exploring_srv_proxy()
       rospy.sleep(0.2)
       # Stop rotating but keeping moving forward
       self.tm.stop_moving()
       followed_person = self.tm.closest_person
       person_width = followed_person[3]
       print("person found")
       t1 = time.time()
       self.tm.start_moving(0.3, 0, 0)
       while person_width<140:
           followed_person = self.tm.closest_person
           person_width = followed_person[3]
           if time.time()-t1>30:
                print("Couldn't approach person")
                self.tm.stop_moving()
                self.repetir2()
       self.tm.stop_moving()
       self.hablar_ready()

   def on_enter_HABLAR(self):
       print(self.consoleFormatter.format("HABLAR", "HEADER"))
       self.tm.talk("Hello my name is Nova, say hey nova to talk with me. Please talk slow, clear and loud.","English")
       t1 = time.time()
       while not self.is_done:
            if self.hey_pepper:
                self.hey_pepper_function()
                self.hey_pepper=False
            if self.bye:
                self.tm.talk("Bye, it was a pleasure talking with you","English")
                self.repetir()
            elif time.time()-t1>10:
                    print("Person did not respond")
                    self.repetir()
            time.sleep(0.1)
       self.tm.talk("have a great day!")
       self.tm.talk("I have finished the "+self.task_name+" task","English")
       self.acabar()

   def on_enter_FININSH(self):
       print(self.consoleFormatter.format("FINISH", "HEADER"))
       self.finish()

   def on_enter_CONVERSACION_DONE(self):
       print(self.consoleFormatter.format("CONVERSACION_DONE", "HEADER"))
       
       os._exit(os.EX_OK)
       
   def explore_thread(self):
       self.exploring = True
       while self.exploring:
           self.start_exploring_srv_proxy(3)

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
       elif word == "bye":
            self.bye = True
   
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

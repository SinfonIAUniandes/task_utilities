#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os

from robot_toolkit_msgs.srv import set_open_close_hand_srv, set_open_close_hand_srvRequest

class MERCADITO(object):
   def __init__(self):

       self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
       # Definir los estados posibles del semáforo
       self.task_name = "PREGUNTAS_Y_RESPUESTAS"
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
       self.toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
       request = set_open_close_hand_srvRequest()
       request.hand = "All"
       request.state = "True"
       self.toggle_breathing_proxy(request)
       print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
       self.beggining()

   def on_enter_HABLAR(self):
       print(self.consoleFormatter.format("HABLAR", "HEADER"))
       while True:
            self.hey_pepper_function()
            time.sleep(0.1)
       self.hablar_ready()

   def on_enter_FININSH(self):
       print(self.consoleFormatter.format("FINISH", "HEADER"))
       
       self.finish()

   def on_enter_CONVERSACION_DONE(self):
       print(self.consoleFormatter.format("CONVERSACION_DONE", "HEADER"))
       
       os._exit(os.EX_OK)

   def hey_pepper_function(self):
       self.tm.talk("¿Cual es tu pregunta?","Spanish",animated=True)
       text = input("Escribe la pregunta de redes sociales: ")
       request = f"""La persona pregunto: {text}."""
       answer=self.tm.answer_question(request) 
       self.tm.talk(answer,"Spanish",animated=True)
   
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

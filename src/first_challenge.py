#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os

class FIRST_CHALLENGE(object):
    def __init__(self):
        
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "first challenge"
        states = ['INIT', 'ASK4NAME', 'RETURN', 'SHARE']
        self.tm = tm(perception = False, speech=True,manipulation=False, navigation=True)
        self.tm.initialize_node("first_challenge")
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'FIRST_CHALLENGE', 'dest': 'INIT'},
            {'trigger': 'ask', 'source': 'INIT', 'dest': 'ASK4NAME'},
            {'trigger': 'ret', 'source': 'ASK4NAME', 'dest': 'RETURN'},
            {'trigger': 'tell', 'source': 'RETURN', 'dest': 'SHARE'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='FIRST_CHALLENGE')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ##################### ROS CALLBACK VARIABLES #####################


        ##################### GLOBAL VARIABLES #####################
        self.name = ""
        
    
    def on_enter_INIT(self):
        self.tm.talk("Voy a realizar el primer reto","Spanish")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.talk("Mi misión es preguntar el nombre a la persona del fondo y compartirlo con ustedes","Spanish")
        self.tm.go_to_relative_point(2,0,0)
        self.ask()   

    def on_enter_ASK4NAME(self):
        self.tm.talk("Hola, ¿podrías decirme tu nombre? Por favor","Spanish")
        self.name=self.tm.q_a_speech("name")
        self.tm.talk("Muchas gracias","Spanish")
        self.ret()  

    def on_enter_RETURN(self):
        self.tm.talk("Ahora dare la vuelta para poder compartir tu nombre","Spanish")
        self.tm.go_to_relative_point(0,0,180)
        time.sleep(1)
        self.tm.go_to_relative_point(2,0,0)
        self.tell()  

    def on_enter_SHARE(self):
        self.tm.talk("El nombre de la persona del fondo es " + self.name,"Spanish")
        time.sleep(1)
        self.tm.talk("Si necesitas otra cosa no dudes en decirme","Spanish")
        
        
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
    sm = FIRST_CHALLENGE()
    sm.run()
    rospy.spin()

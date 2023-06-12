#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import time
import random
import threading
import sys
import rospy

class SM(object):
    def __init__(self):
        # Definir los estados posibles del semáforo
        states = ['INIT', 'WAIT4GUEST', 'TALK', 'Finish']
        self.tm = tm(perception = True,speech=False,manipulation=False, navigation=False)
        
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'change', 'source': 'SM', 'dest': 'INIT'},
            {'trigger': 'change', 'source': 'INIT', 'dest': 'WAIT4GUEST'},
            {'trigger': 'change', 'source': 'WAIT4GUEST', 'dest': 'TALK'},
            {'trigger': 'change', 'source': 'TALK', 'dest': 'WAIT4GUEST'},
            {'trigger': 'finish', 'source': '*', 'dest': 'Finish'},
            {'trigger': 'restart', 'source': 'Finish', 'dest': 'INIT'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='SM')
    
    def on_enter_INIT(self):
        print("INIT")
        self.tm.initialize_node()
        # self.tm.talk_speech("Spanish","Voy a hacer el task de sisandes",2)
        print("Voy a hacer el task de sisandes")
        self.tm.turn_camera("front_camera","enable",0,0)
        self.tm.start_recognition("front_camera")
    
    def on_enter_WAIT4GUEST(self):
        print("WAIT4GUEST")
        self.tm.look_for_object("person")
        self.tm.wait_for_object(10)
        self.tm.look_for_object("")


    def on_enter_TALK(self):
        print("CHECK")   
        # self.tm.talk_speech("Spanish","¡Saludos! Soy un robot diseñado para compartir información sobre ciudades inteligentes. Hoy, vamos a explorar el fascinante mundo de las ciudades inteligentes y cómo están transformando nuestro entorno urbano. ¿Están listos? ¡Comencemos!",2)
        # self.tm.talk_speech("Spanish","Las ciudades inteligentes son aquellas que utilizan la tecnología y la innovación para mejorar la calidad de vida de sus habitantes. Estas ciudades están impulsadas por la recopilación de datos, la conectividad y la automatización para optimizar los servicios urbanos y promover la sostenibilidad. Una de las principales áreas en las que se enfocan es la infraestructura y la gestión de servicios urbanos.",2)
        print("¡Saludos! Soy un robot diseñado para compartir información sobre ciudades inteligentes. Hoy, vamos a explorar el fascinante mundo de las ciudades inteligentes y cómo están transformando nuestro entorno urbano. ¿Están listos? ¡Comencemos!")

    def on_enter_Finish(self):
        print("Finish")
        sys.exit()

    def check_global_variable(self):
        while True:
            if rospy.is_shutdown():
                self.finish()
                break
            time.sleep(1)

    def run(self):
        # Crear un hilo para realizar el chequeo continuo
        check_thread = threading.Thread(target=self.check_global_variable)
        check_thread.start()

        while not rospy.is_shutdown():
            self.change()
            # if self.state == 'Finish':
            #     sys.exit()

    
# Crear una instancia del semáforo
sm = SM()
sm.run()
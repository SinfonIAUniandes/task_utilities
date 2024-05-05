#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import random
import threading
import sys
import rospy
import math
import os
import numpy as np

from navigation_msgs.srv import get_absolute_position_srv
from robot_toolkit_msgs.srv import move_head_srv

class STICKLER_RULES(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "stickler_for_the_rules"
        states = ['INIT', 'LOOK4PERSON', 'LOOK4GARBAGE', 'ASK4SHOES', 'ASK4DRINK', 'GO2NEXT']
        self.tm = tm(perception = True,speech=True, navigation=True, pytoolkit=True, manipulation=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'STICKLER_RULES', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'LOOK4PERSON'},
            {'trigger': 'rules_checked', 'source': 'LOOK4PERSON', 'dest': 'GO2NEXT'},
            {'trigger': 'arrive_next', 'source': 'GO2NEXT', 'dest': 'LOOK4PERSON'},
        ]
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='STICKLER_RULES')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        # ROS Services (PyToolkit)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/move_head...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        self.move_head_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/move_head_srv",move_head_srv)

        print(self.consoleFormatter.format("Waiting for navigation_utilities/get_absolute_position_srv...", "WARNING"))
        rospy.wait_for_service("navigation_utilities/get_absolute_position_srv")
        self.get_absolute_position_srv = rospy.ServiceProxy("navigation_utilities/get_absolute_position_srv",get_absolute_position_srv)

        self.last_place = "living_room"
        #TODO poner numero de guests totales que hay
        self.number_guests = 5
        #TODO poner numero de personas rompiendo las reglas totales
        self.total_rule_breakers = 4
        #numero de personas encontradas rompiendo las reglas
        self.breakers_found = 0
        #TODO Poner el cuarto que sea forbidden
        #self.list_places = ["bedroom","kitchen","office","living_room","bathroom", "forbidden"] # Ya hay una lista de lugares y forbidden se puede usar como una variable
        self.list_places = ["house","living","kitchen","room","dining"]
        self.forbidden = "room"
        self.checked_places = []

    def on_enter_INIT(self):
        self.tm.talk("I am going to do the "+self.task_name+" task","English", wait=False)
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        self.tm.show_topic("/perception_utilities/yolo_publisher")
        #self.tm.go_to_place("living_room") # El cuarto principal se llama house
        #self.tm.go_to_place("house")
        self.beggining()

    # ============================== LOOK4 STATES ==============================
    def on_enter_LOOK4PERSON(self):
        print(self.consoleFormatter.format("ROBOT STOP", "WARNING"))
        self.tm.talk("I am going to check if the guests are breaking the rules!","English", wait=False)
        tiempo_inicial = time.time()
        grados_seg = 15
        # A 15 grados/seg toma 24 segundos dar 1 vuelta
        tiempo_una_vuelta = 25
        angulos_personas = []
        self.tm.go_to_pose("default_head")
        tiempo_transcurrido = 0
        while (tiempo_transcurrido < tiempo_una_vuelta) or self.breakers_found<self.total_rule_breakers:
            self.tm.look_for_object("person")
            self.tm.constant_spin_srv(grados_seg)
            self.tm.wait_for_object(tiempo_una_vuelta-tiempo_transcurrido)
            angulo_persona = get_absolute_position_srv().theta
            for angulo in angulos_personas:
                #60 grados es un poco mas del campo de vision del robot, si el angulo de la persona actual es similar a algun otro por 60 grados puede ser la misma persona 
                if abs(angulo_persona-angulo)>=60:
                    angulos_personas.append(angulo_persona)
                    self.tm.robot_stop_srv()
                    if self.check_shoes() or self.check_drink() or self.check_forbidden():
                        self.breakers_found += 1
            tiempo_transcurrido = time.time()-tiempo_inicial
        self.rules_checked()

    def check_shoes(self):
        print(self.consoleFormatter.format("LOOK4SHOES", "HEADER"))
        self.tm.talk("I am going to see if you have shoes on!","English", wait=False)
        self.move_head_srv("down")
        rospy.sleep(1)
        gpt_vision_prompt = f"Is there a person with shoes in this picture? Answer only with True or False"
        answer = self.tm.img_description(gpt_vision_prompt)["message"]
        print("GPT ANSWER:"+answer)
        self.move_head_srv("default")
        if "True" in answer:
            self.ASK4SHOES()
            return True
        else:
            self.tm.talk("You passed the shoes check!","English", wait=False)
        return False

    def check_drink(self):
        print(self.consoleFormatter.format("LOOK4DRINK", "HEADER"))
        self.tm.talk("I am going to see if you have a drink!","English", wait=False)
        gpt_vision_prompt = f"Is there a person without a drink in their hand in this picture? Answer only with True or False"
        answer = self.tm.img_description(gpt_vision_prompt)["message"]
        if "True" in answer:
            self.ASK4DRINK()
            return True
        else:
            self.tm.talk("You passed the drink check!","English", wait=False)
        return False

    def check_forbidden(self):
        #if self.last_place == "forbidden": # Manejar el forbidden room como variable
        if self.last_place == self.forbidden:
            self.ASK2LEAVE()
            return True
        return False

    # ============================== ASK4 STATES ===============================
    def ASK4SHOES(self):
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        self.tm.talk("You must not wear shoes in this place!. Go to the entrance and take them off!","English", wait=False)

    def ASK4DRINK(self):
        print(self.consoleFormatter.format("ASK4DRINK", "HEADER"))
        self.tm.talk("You must have a Drink in your hand. Go to the kitchen and grab a drink, you must keep it in your hand!","English", wait=False)

    def ASK2LEAVE(self):
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        self.tm.talk("This room is forbidden. You must leave this room please","English", wait=False)

    # =============================== GO2 STATES ===============================
    def on_enter_GO2NEXT(self):
        print("Current Place: " + self.last_place)
        self.tm.talk("I'm going to check another room!","English", wait=False)
        print(self.consoleFormatter.format("GO2NEXT", "HEADER"))
        for place in self.list_places:
            if not place in self.checked_places:
                self.tm.talk("I'm gonna check " + place,"English", wait=False)
                #self.tm.go_to_place(place)
                print("Next Place: " + place)
                self.checked_places.append(place)
                self.last_place = place
                self.arrive_next()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        self.start()

# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = STICKLER_RULES()
    sm.run()
    rospy.spin()

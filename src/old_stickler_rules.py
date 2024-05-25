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
from perception_msgs.msg import get_labels_msg
from robot_toolkit_msgs.srv import move_head_srv, set_security_distance_srv

class STICKLER_RULES(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "stickler_for_the_rules"
        states = ['INIT', 'LOOK4PERSON', 'LOOK4GARBAGE', 'ASK4SHOES', 'ASK4DRINK', 'GO2NEXT']
        self.tm = tm(perception = True,speech=True, navigation=True, pytoolkit=True, manipulation=True)
        print("Inicializando")
        self.tm.initialize_node(self.task_name)
        print("Inicializado")
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




        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_orthogonal_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv")
        self.set_orthogonal_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv",set_security_distance_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_tangential_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_tangential_security_distance_srv")
        self.set_tangential_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_tangential_security_distance_srv",set_security_distance_srv)

        print(self.consoleFormatter.format("Waiting for navigation_utilities/get_absolute_position_srv...", "WARNING"))
        rospy.wait_for_service("navigation_utilities/get_absolute_position_srv")
        self.get_absolute_position_proxy = rospy.ServiceProxy("navigation_utilities/get_absolute_position_srv",get_absolute_position_srv)
        self.tm.follow_you_active = True
        # Parametro de la task de si se quiere que el robot confirme que los invitados corrigieron la regla
        self.confirm_comppliance = False
        # Donde se encuentran ubicados los otros invitados, es para forbidden room
        self.party_place = "living"
        self.last_place = "living"
        #TODO poner numero de guests totales que hay
        self.number_guests = 5
        #TODO poner numero de personas rompiendo las reglas totales
        self.total_rule_breakers = 4
        #numero de personas encontradas rompiendo las reglas
        self.breakers_found = 0
        #TODO Poner el cuarto que sea forbidden
        #self.list_places = ["bedroom","kitchen","office","living_room","bathroom", "forbidden"] # Ya hay una lista de lugares y forbidden se puede usar como una variable
        self.list_places = ["dining","house","living","kitchen","room"]
        self.forbidden = "room"
        self.checked_places = []
        
        #Varibales globales para los chequeos con threading.
        # 2 Es que no se han hecho los drinks, 1 es que si tiene, 0 es que no tiene
        self.has_shoes =  2
        self.has_drink =  2

    def on_enter_INIT(self):
        self.tm.talk("I am going to do the "+self.task_name+" task","English", wait=False)
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        self.tm.turn_camera("bottom_camera","custom",1,15)
        self.tm.show_topic("/perception_utilities/yolo_publisher")
        self.set_orthogonal_security_srv(0.3)
        self.set_tangential_security_srv(0.05)
        self.get_labels_publisher = rospy.Subscriber('/perception_utilities/get_labels_publisher', get_labels_msg, self.tm.callback_get_labels_subscriber)
        person_thread = threading.Thread(target=self.tm.get_closer_person)
        person_thread.start()
        # Ir directo al forbidden room
        self.tm.talk("I'm gonna go check the forbidden room: " + self.forbidden ,"English", wait=False)
        print("Current Place: " + self.last_place)
        print("Next Place: " + self.forbidden)
        self.checked_places.append(self.forbidden)
        self.last_place = self.forbidden
        self.tm.go_to_place(self.forbidden)
        self.beggining()

    # ============================== LOOK4 STATES ==============================
    def on_enter_LOOK4PERSON(self):
        print(self.consoleFormatter.format("LOOK4PERSON", "WARNING"))
        self.tm.talk("I am going to check if the guests are breaking the rules!","English", wait=False)
        self.tm.setRPosture_srv("stand")
        #Estando en la puerta 
        self.tm.spin_srv(-60)
        grados_seg = 8
        # A 15 grados/seg toma 24 segundos dar 1 vuelta. Asumiendo que se esta en la puerta, toma 8 segundos chequear si el rango de vision es 60 grados y no se quiere ver gente fuera de la arena
        tiempo_una_vuelta = 12
        angulos_personas = []
        self.move_head_srv("default")
        breakers_current_room = 0
        tiempo_transcurrido = 0
        # While para dar una vuelta entera y encontrar a TODAS las personas en una habitacion
        while (tiempo_transcurrido < tiempo_una_vuelta): # and self.breakers_found<self.total_rule_breakers: (Chunk opcional si se quiere detener la task cuando se haya encontrado a todos los rule breakers)
            tiempo_inicio_rotacion = time.time()
            self.tm.look_for_object("person")
            self.tm.constant_spin_srv(grados_seg)
            person_found = self.tm.wait_for_object(tiempo_una_vuelta-tiempo_transcurrido)
            tiempo_rotando = time.time()-tiempo_inicio_rotacion
            tiempo_transcurrido += tiempo_rotando
            if person_found:
                #Encontro una persona o se acabo el tiempo
                angulo_persona = self.get_absolute_position_proxy().theta
                angulo_nuevo = True
                for angulo in angulos_personas:
                    #60 grados es un poco mas del campo de vision del robot, si el angulo de la persona actual es similar a algun otro por 60 grados puede ser la misma persona 
                    if abs(angulo_persona-angulo)<30:
                        angulo_nuevo = False
                found_person = self.tm.closest_person
                person_x = found_person[1]
                person_width = found_person[3]
                centered_point = (315 / 2) - (person_x + person_width/2)
                if angulo_nuevo and centered_point<=15 and "person" in self.tm.labels:
                    print(self.consoleFormatter.format("ROBOT STOP", "WARNING"))
                    while ( -15 >= centered_point or centered_point>= 15) and "person" in self.tm.labels:
                        found_person = self.tm.closest_person
                        person_x = found_person[1]
                        person_width = found_person[3]
                        centered_point = (315 / 2) - (person_x + person_width/2)
                    self.tm.robot_stop_srv()
                    angulos_personas.append(angulo_persona)
                    is_in_forbidden = self.check_forbidden(angulo_persona)
                    if not is_in_forbidden:
                        shoes_check_thread = threading.Thread(target=self.check_shoes)
                        shoes_check_thread.start()
                        drink_check_thread = threading.Thread(target=self.check_drink)
                        drink_check_thread.start()
                        while self.has_drink==2 and self.has_shoes==2:
                            rospy.sleep(0.1)
                    self.tm.robot_stop_srv()
                    if self.has_drink==0:
                        self.ASK4DRINK(angulo_persona)
                    if self.has_shoes==1:
                        self.ASK4SHOES(angulo_persona)
                    if is_in_forbidden or self.has_drink==0 or self.has_shoes==1:
                        self.breakers_found += 1
                        breakers_current_room +=1
                    else:
                        self.tm.talk("Congratulations! You're not breaking any rule","English", wait=False)
                    self.has_drink = 2
                    self.has_shoes = 2
            else:
                print(self.consoleFormatter.format("ROBOT STOP", "WARNING"))
                self.tm.robot_stop_srv()
        self.tm.talk("I'm done checkin this room'!","English", wait=False)
        self.tm.robot_stop_srv()
        print(self.consoleFormatter.format("ROBOT STOP", "WARNING"))
        self.rules_checked()

    def check_shoes(self):
        print(self.consoleFormatter.format("LOOK4SHOES", "HEADER"))
        self.tm.robot_stop_srv()
        gpt_vision_prompt = f"Is the closest person in the picture barefooted or in socks? Answer only with True or False"
        answer = self.tm.img_description(prompt=gpt_vision_prompt,camera_name="both")["message"]
        print("GPT ANSWER:"+answer)
        if "True" in answer:
            self.has_shoes = 0
        else:
            self.has_shoes = 1

    def check_drink(self):
        print(self.consoleFormatter.format("LOOK4DRINK", "HEADER"))
        self.tm.robot_stop_srv()
        gpt_vision_prompt = f"Is the closest person in the picture holding a bottle,a juice box, a cup, a can, or any kind of drink in their hand? Answer only with True or False"
        answer = self.tm.img_description(gpt_vision_prompt,camera_name="both")["message"]
        print("GPT ANSWER:"+answer)
        if "True" in answer:
            self.has_drink = 1
        else:
            self.has_drink = 0
        return False

    def check_forbidden(self, original_angle):
        self.tm.robot_stop_srv()
        if self.last_place == self.forbidden:
            self.ASK2LEAVE(original_angle)
            return True
        return False

    # ============================== ASK4 STATES ===============================
    def ASK4SHOES(self, original_angle):
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        if self.confirm_comppliance:
            if self.last_place == "init":
                self.tm.talk("You must not wear shoes in this place!","English", wait=False)
            else:
                self.tm.talk("You must not wear shoes in this place!. Follow me to the entrance and take them off!","English", wait=False)
                self.tm.go_to_place("init")
            self.tm.setRPosture_srv("stand")
            self.tm.talk("Now, take them off please.","English", wait=True)
            rospy.sleep(10)
            self.tm.talk("I am going to see if you have shoes on!","English", wait=False)
            gpt_vision_prompt = f"Is the closest person in the picture barefooted or in socks? Answer only with True or False"
            answer = self.tm.img_description(prompt=gpt_vision_prompt,camera_name="both")["message"]
            print("GPT ANSWER:"+answer)
            if "True" in answer:
                self.tm.talk("I see you did not comply, but i must continue checking the other guests","English", wait=False)
            else:
                self.tm.talk("Thank you for taking your shoes off!","English", wait=False)
            self.tm.go_to_place(self.last_place)
            self.tm.go_to_defined_angle_srv(original_angle)
            self.tm.setRPosture_srv("stand")
            self.move_head_srv("default")
        else:
            self.tm.talk("You must not wear shoes in this place!. Please, go to the entrance and take them off!","English", wait=False)

    def ASK4DRINK(self, original_angle):
        print(self.consoleFormatter.format("ASK4DRINK", "HEADER"))
        if self.confirm_comppliance:
            if self.last_place == "kitchen":
                self.tm.talk("You must have a Drink in your hand!","English", wait=False)
            else:
                self.tm.talk("You must have a Drink in your hand!. Follow me to the kitchen and grab a drink!","English", wait=False)
                self.tm.go_to_place("kitchen")
            self.tm.setRPosture_srv("stand")
            self.tm.talk("Now, please take a drink and show it to me","English", wait=True)
            rospy.sleep(5)
            self.tm.talk("I am going to see if you have a drink!","English", wait=False)
            gpt_vision_prompt = f"Is the closest person in the picture holding a bottle,a juice box, a cup, a can, or any kind of drink in their hand? Answer only with True or False"
            answer = self.tm.img_description(gpt_vision_prompt,camera_name="both")["message"]
            print("GPT ANSWER:"+answer)
            if "True" in answer:
                self.tm.talk("I see you did not comply, but i must continue checking the other guests","English", wait=False)
            else:
                self.tm.talk("Thank you for grabbing a drink!","English", wait=False)
            self.tm.go_to_place(self.last_place)
            self.tm.go_to_defined_angle_srv(original_angle)
            self.tm.setRPosture_srv("stand")
            self.move_head_srv("default")
        else:
            self.tm.talk("You must have a Drink in your hand. Please, go to the kitchen and grab a drink, you must keep it in your hand!","English", wait=False)

    def ASK2LEAVE(self, original_angle):
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        if self.confirm_comppliance:
            self.tm.talk("This room is forbidden. Follow me to the other party guests please","English", wait=False)
            self.tm.go_to_place(self.party_place)
            self.tm.setRPosture_srv("stand")
            self.tm.talk("Now, please stay here and do not come back to the forbidden room","English", wait=True)
            self.tm.go_to_place(self.last_place)
            self.tm.go_to_defined_angle_srv(original_angle)
            self.tm.setRPosture_srv("stand")
            self.move_head_srv("default")
        else:
            self.tm.talk("This room is forbidden. Please, leave this room","English", wait=False)

    # =============================== GO2 STATES ===============================
    def on_enter_GO2NEXT(self):
        print("Current Place: " + self.last_place)
        self.tm.talk("I'm going to check another room!","English", wait=False)
        self.tm.setRPosture_srv("stand")
        print(self.consoleFormatter.format("GO2NEXT", "HEADER"))
        for place in self.list_places:
            if not place in self.checked_places:
                self.tm.talk("I'm gonna check " + place,"English", wait=False)
                self.tm.go_to_place(place)
                print("Next Place: " + place)
                self.checked_places.append(place)
                self.last_place = place
                self.arrive_next()
        self.tm.talk("But i have checked all of the rooms! Yipee","English")
        os._exit(os.EX_OK)

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

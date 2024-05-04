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

from navigation_msgs.srv import constant_spin_srv, get_absolute_position_srv, get_absolute_position_srvRequest
from navigation_msgs.msg import simple_feedback_msg
from perception_msgs.msg import get_labels_msg
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srv
from robot_toolkit_msgs.msg import animation_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

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

        self.last_place = "living_room"
        #TODO poner numero de guests que hay
        self.number_guests = 5
        #numero de personas encontradas rompiendo las reglas
        self.breakers_found = 0
        #TODO Poner el cuarto que sea forbidden
        #self.list_places = ["bedroom","kitchen","office","living_room","bathroom", "forbidden"] # Ya hay una lista de lugares y forbidden se puede usar como una variable
        self.list_places = ["house","living","kitchen","room","dining"]
        self.forbidden = "room"
        self.checked_places = []

    def on_enter_INIT(self):
        self.tm.talk("I am going to do the "+self.task_name+" task","English")
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
        contador = 0
        while contador<self.number_guests-self.breakers_found:
            # Constant spin para esquivar a personas que ya vio
            self.tm.constant_spin_srv(15)
            t1 = time.time()    
            while time.time() - t1 < 2:
                rospy.sleep(0.1)
            self.tm.robot_stop_srv()
            person_found = self.tm.find_object("person", ignore_already_seen=True)
            if person_found:
                contador+=1
                self.check_shoes()
                self.check_drink()
                self.check_forbidden()
            else:
                self.rules_checked()
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
            self.breakers_found += 1
            self.ASK4SHOES()
        else:
            self.tm.talk("You passed the shoes check!","English", wait=False)

    def check_drink(self):
        print(self.consoleFormatter.format("LOOK4DRINK", "HEADER"))
        self.tm.talk("I am going to see if you have a drink!","English", wait=False)
        gpt_vision_prompt = f"Is there a person without a drink in their hand in this picture? Answer only with True or False"
        answer = self.tm.img_description(gpt_vision_prompt)["message"]
        if "True" in answer:
            self.breakers_found += 1
            self.ASK4DRINK()
        else:
            self.tm.talk("You passed the drink check!","English", wait=False)

    def check_forbidden(self):
        #if self.last_place == "forbidden": # Manejar el forbidden room como variable
        if self.last_place == self.forbidden:
            self.breakers_found += 1
            self.ASK2LEAVE()

    # ============================== ASK4 STATES ===============================
    def ASK4SHOES(self):
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        #self.tm.talk("You must not wear shoes in this place!. Go to the entrance and take them off!","English") # El robot tiene que ir a la entrada con el guest
        # ===========================================
        self.tm.talk("You must not wear shoes in this place!. Follow me to the entrance and take them off!","English")
        self.tm.go_to_place("house_door")
        rospy.sleep(10)
        self.tm.go_to_place(self.last_place)

    def ASK4DRINK(self):
        print(self.consoleFormatter.format("ASK4DRINK", "HEADER"))
        self.tm.talk("You must have a Drink in your hand","English")
        if self.last_place == "kitchen":
            self.tm.talk("Take a drink please!","English")
        else:
            self.tm.talk("Please follow me to the kitchen","English")
            #self.tm.go_to_place("kitchen") # Definir cocina
            self.tm.talk("Please take a drink and keep it in your hand! I will now return to the previous room","English")
            #self.tm.go_to_place(self.last_place)

    def ASK2LEAVE(self):
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        self.tm.talk("This room is forbidden. You must leave this room please","English")

    # =============================== GO2 STATES ===============================
    def on_enter_GO2NEXT(self):
        print("Current Place: " + self.last_place)
        self.tm.talk("I'm going to check another room!","English")
        print(self.consoleFormatter.format("GO2NEXT", "HEADER"))
        for place in self.list_places:
            if not place in self.checked_places:
                self.tm.talk("I'm gonna check " + place,"English")
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

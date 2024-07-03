#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import math
import os
import numpy as np

from navigation_msgs.srv import get_absolute_position_srv
from perception_msgs.msg import get_labels_msg
from robot_toolkit_msgs.srv import move_head_srv, set_security_distance_srv, set_angle_srv, set_angle_srvRequest

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
        self.get_absolute_position_proxy = rospy.ServiceProxy("navigation_utilities/get_absolute_position_srv",get_absolute_position_srv)
        self.tm.follow_you_active = True
        # Parametro de la task de si se quiere que el robot confirme que los invitados corrigieron la regla
        self.confirm_comppliance = False
        self.confirm_comppliance_forbidden = True
        # El robot empieza en 0 y luego se mueve hacia estos angulos
        self.angles_to_check = [0,-60,60]
        # Donde se encuentran ubicados los otros invitados, es para forbidden room
        self.party_place = "dining"
        self.initial_place = "bedroom"
        self.last_place = self.initial_place
        #TODO poner numero de guests totales que hay
        self.number_guests = 5
        #TODO poner numero de personas rompiendo las reglas totales
        self.total_rule_breakers = 4
        #numero de personas encontradas rompiendo las reglas
        self.breakers_found = 0
        #TODO Poner el cuarto que sea forbidden
        #self.list_places = ["bedroom","kitchen","office","living_room","bathroom", "forbidden"] # Ya hay una lista de lugares y forbidden se puede usar como una variable
        self.list_places = ["dining_stickler","main_room","kitchen","living_room"]
        self.places_names = ["dining","living_room","main_room","kitchen"]
        #TODO preguntar Luccas
        self.forbidden = "living_room"
        self.checked_places = []
        
        #Varibales globales para los chequeos con threading.
        # 2 Es que no se han hecho los drinks, 1 es que si tiene, 0 es que no tiene
        self.has_shoes =  2
        self.has_drink =  2

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        self.tm.turn_camera("bottom_camera","custom",1,15)
        self.tm.turn_camera("depth_camera","custom",1,15)
        self.tm.toggle_filter_by_distance(True,2,["person"])
        self.tm.show_topic("/perception_utilities/yolo_publisher")
        toggle_msg =  set_angle_srvRequest()
        toggle_msg.name = ["HeadYaw"]
        toggle_msg.angle = []
        toggle_msg.speed = 0
        self.tm.toggle_get_angles_topic_srv(toggle_msg)
        self.tm.set_current_place(self.initial_place)
        self.checked_places.append(self.initial_place)
        self.last_place = self.initial_place
        self.get_labels_publisher = rospy.Subscriber('/perception_utilities/get_labels_publisher', get_labels_msg, self.tm.callback_get_labels_subscriber)
        person_thread = threading.Thread(target=self.tm.get_closer_person)
        person_thread.start()
        self.tm.talk("I am going to do the "+self.task_name+" task","English", wait=False)
        self.beggining()

    # ============================== LOOK4 STATES ==============================
    def on_enter_LOOK4PERSON(self):
        print(self.consoleFormatter.format("LOOK4PERSON", "WARNING"))
        self.tm.talk("I am going to check if the guests are breaking the rules!","English", wait=False)
        self.tm.setRPosture_srv("stand")
        for angle in self.angles_to_check:
            print("angulo actual:",angle)
            self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
            if angle=="0":
                rospy.sleep(0)
            elif angle=="-60":
                rospy.sleep(3)
            elif angle=="60":
                        rospy.sleep(5)
            persons = self.tm.labels.get("person", [])
            for person in persons:
                print("centrando persona:",person)
                self.tm.center_head_with_label(person)
                print("checkeando reglas")
                self.check_rules()
                print("reglas checkeadas")
                self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
        self.tm.setRPosture_srv("stand")
        self.tm.talk("I'm done checkin this room'!","English", wait=False)
        self.rules_checked()

    def check_shoes(self):
        print(self.consoleFormatter.format("LOOK4SHOES", "HEADER"))
        self.tm.robot_stop_srv()
        gpt_vision_prompt = f"Is the person in the center of the picture barefooted or in socks? Answer only with True or False. If you can't see their feet answer only with 'None'. If the person you see is behind a wall answer only with 'Wall' and don't check the other rules. If you are not at least 80% sure that a person is wearing shoes answer True"
        answer = self.tm.img_description(prompt=gpt_vision_prompt,camera_name="both")["message"]
        print("SHOES GPT ANSWER:"+answer)
        if "True" in answer:
            # Esta descalso o con medias
            self.has_shoes = 0
        elif "None" in answer:
            # No se sabe
            self.has_shoes = 3
        elif "Wall" in answer:
            # Detras de una pared
            self.has_shoes = 4
        else:
            # Tiene zapatos
            self.has_shoes = 1

    def check_drink(self):
        print(self.consoleFormatter.format("LOOK4DRINK", "HEADER"))
        self.tm.robot_stop_srv()
        gpt_vision_prompt = f"Is the person in the center of the picture holding a bottle,a juice box, a cup, a can, or any kind of drink in their hand? Answer only with True or False. If the person you see is behind a wall answer only with 'Wall' and don't check the other rules"
        answer = self.tm.img_description(gpt_vision_prompt,camera_name="both")["message"]
        print("DRINK GPT ANSWER:"+answer)
        if "True" in answer:
            # Tiene una bebida
            self.has_drink = 1
        elif "Wall" in answer:
            # Detras de una pared
            self.has_drink = 4
        else:
            # No tiene una bebida
            self.has_drink = 0

    def check_forbidden(self):
        self.tm.robot_stop_srv()
        if self.confirm_comppliance_forbidden and self.last_place == self.forbidden:
            self.tm.talk("You should not be here, because this room is forbidden.","English", wait=True)
            self.move_head_srv("default")
            return True
        return False

    def calcular_tiempo_espera(self, angulo_actual, angulo_anterior):
        # Rango de ángulos permitidos
        angulo_minimo = -1.39  # radianes
        angulo_maximo = 1.39   # radiane.get("person", [])
        distancia = abs(angulo_actual - angulo_anterior)
        
        distancia_normalizada = distancia / (angulo_maximo - angulo_minimo)
        tiempo_minimo = 0 
        tiempo_maximo = 9 
        tiempo_espera = tiempo_minimo + distancia_normalizada * (tiempo_maximo - tiempo_minimo)
        
        return tiempo_espera

    # ============================== ASK4 STATES ===============================
    def ASK4SHOES(self):
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        self.tm.talk("You must not wear shoes in this place!. Please, go to the entrance and take them off!","English", wait=False)

    def ASK4DRINK(self):
        print(self.consoleFormatter.format("ASK4DRINK", "HEADER"))
        self.tm.talk("You must have a Drink in your hand. Please, go to the kitchen and grab a drink, you must keep it in your hand!","English", wait=False)

    def ASK2LEAVE(self):
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        if self.confirm_comppliance_forbidden:
            self.tm.talk("Follow me to the other party guests please","English", wait=False)
            self.tm.go_to_place(self.party_place)
            self.tm.setRPosture_srv("stand")
            self.move_head_srv("default")
            self.tm.talk("Now, please stay here and do not come back to the forbidden room","English", wait=False)
        else:
            self.tm.talk("This room is forbidden. Please, leave this room","English", wait=False)

    # =============================== GO2 STATES ===============================
    def on_enter_GO2NEXT(self):
        print("Current Place: " + self.last_place)
        self.tm.talk("I'm going to check another room!","English", wait=False)
        self.tm.setRPosture_srv("stand")
        print(self.consoleFormatter.format("GO2NEXT", "HEADER"))
        for place_num in range(len(self.list_places)):
            place = self.list_places[place_num]
            if not place in self.checked_places:
                self.tm.talk("I'm gonna check " + self.places_names[place_num],"English", wait=False)
                self.tm.go_to_place(place)
                print("Next Place: " + place)
                self.checked_places.append(place)
                self.last_place = place
                self.arrive_next()
        self.tm.talk("But i have checked all of the rooms! Yipee","English")
        os._exit(os.EX_OK)

    def check_rules(self):
        is_in_forbidden = self.check_forbidden()
        shoes_check_thread = threading.Thread(target=self.check_shoes)
        shoes_check_thread.start()
        drink_check_thread = threading.Thread(target=self.check_drink)
        drink_check_thread.start()
        while self.has_drink==2 and self.has_shoes==2:
            rospy.sleep(0.1)
        if self.has_drink==0:
            self.ASK4DRINK()
        if self.has_shoes==1:
            self.ASK4SHOES()
        elif self.has_shoes==3:
            self.tm.talk("I can't see your feet, but remember you can't use shoes in the house.","English", wait=False)
        elif self.has_drink==4:
            pass
        elif self.has_shoes==4:
            pass
        if is_in_forbidden or self.has_drink==0 or self.has_shoes==1:
            print("persona rompiendo una regla")
            self.breakers_found += 1
        else:
            if self.has_drink!=4 and self.has_shoes!=4:
                self.tm.talk("Congratulations! You're not breaking any rule","English", wait=False)
        self.has_drink = 2
        self.has_shoes = 2
        if self.last_place == self.forbidden:
            self.ASK2LEAVE()

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

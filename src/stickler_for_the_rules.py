#!/usr/bin/env python3

# --------------------------- GENERAL LIBRARIES IMPORTS ------------------------------
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import math
import os
import numpy as np

# --------------------------- MESSAGES AND SERVICES IMPORTS ------------------------------
from robot_toolkit_msgs.srv import move_head_srv, set_angle_srvRequest

# --------------------------- STICKLER FOR THE RULES TASK CLASS  ------------------------------
class STICKLER_RULES(object):
    
    # --------------------------- Class Constructor ------------------------------
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        # Task name
        self.task_name = "stickler_for_the_rules"
        
        # Possible states for the states machine
        states = ['INIT', 'LOOK4PERSON', 'LOOK4GARBAGE', 'ASK4SHOES', 'ASK4DRINK', 'GO2NEXT']
        
        # Initialization of the task module
        self.tm = tm(perception = True,speech=True, navigation=True, pytoolkit=True, manipulation=True)
        self.tm.initialize_node(self.task_name)
        
        # Definition of the permitted transitions between states
        transitions = [
            {'trigger': 'start', 'source': 'STICKLER_RULES', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO2NEXT'},
            {'trigger': 'rules_checked', 'source': 'LOOK4PERSON', 'dest': 'GO2NEXT'},
            {'trigger': 'arrive_next', 'source': 'GO2NEXT', 'dest': 'LOOK4PERSON'},
        ]
        
        # Creation of the states machine
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='STICKLER_RULES')

        # Thread to check if rospy is running
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        # --------------------------- ROS Services (PyToolkit) ------------------------------

        #TODO: Change this service for the TM one
        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/move_head...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        self.move_head_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/move_head_srv",move_head_srv)
 
 
        # --------------------------- Task Parameters ------------------------------
        
        # Task parameter if the robot wants to confirm that the guests corrected the rule
        self.confirm_comppliance = False
        self.confirm_comppliance_forbidden = True
        
        self.someone_in_forbidden = False
        
        # The robot starts at 0 and then moves to these angles
        self.angles_to_check = [0,-60,60]
        
        # Where the other guests are located, it is for the forbidden room
        self.party_place = "living_room"
        self.initial_place = "house_door"
        self.last_place = self.initial_place
        
        # TODO Set total guests
        self.number_guests = 5
        
        # TODO Set total rule breakers
        self.total_rule_breakers = 4
        
        # Number of people found breaking the rules
        self.breakers_found = 0
        
        # TODO Set the forbidden room
        self.forbidden = "kitchen"
        
        # List of places to check
        self.list_places = ["kitchen","living_room","intermed_door","dining","bedroom"]
        self.places_names = ["kitchen","living_room","main room","dining","bedroom"]
        
        # List of checked places
        self.checked_places = []
        
        # Global variables for the checks with threading
        # 2 Es que no se han hecho los drinks, 1 es que si tiene, 0 es que no tiene
        self.has_shoes =  2
        self.has_drink =  2
        
    
    # --------------------------- STATES FUNCTIONS ------------------------------
    
    # --------------------------- FIRST STATE: INIT ------------------------------
    def on_enter_INIT(self):
        
        
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
        
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        
        self.tm.talk("I am going to do the " + self.task_name + " task","English", wait=False)
        
        # Moving to the LOOK4PERSON state
        self.beggining()
        

    # --------------------------- SECOND STATE: LOOK FOR PERSON ------------------------------
    def on_enter_LOOK4PERSON(self):
        
        print(self.consoleFormatter.format("LOOK4PERSON", "WARNING"))
        
        self.tm.look_for_object("person", ignore_already_seen=True)
        
        self.tm.talk("I am going to check if the guests are breaking the rules!","English", wait=False)
        self.tm.setRPosture_srv("stand")
        
        for angle in self.angles_to_check:
            
            print("angulo actual:",angle)
            self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
            
            if angle==0:
                rospy.sleep(1)
                
            elif angle==-60:
                rospy.sleep(3)
                
            elif angle==60:
                rospy.sleep(5)
            rospy.sleep(2)
                
            persons = self.tm.labels.get("person", [])
            
            for person in persons:
                
                print("centrando persona:",person)
                self.tm.center_head_with_label(person)
                
                print("checkeando reglas")
                self.check_rules()
                
                print("reglas checkeadas")
                self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
            
        if self.confirm_comppliance_forbidden and self.someone_in_forbidden:
            if self.last_place == self.forbidden:
                # Moving to the ASK2FOLLOW state
                self.ASK2FOLLOW()
            
        self.tm.setRPosture_srv("stand")
        self.tm.talk("I'm done checkin this room'!","English", wait=False)
        
        # Moving to the GO2NEXT state
        self.rules_checked()
        
    # --------------------------- Complementary function 1: Check shoes ------------------------------
    def check_shoes(self):
        
        print(self.consoleFormatter.format("LOOK4SHOES", "HEADER"))
        
        self.tm.robot_stop_srv()
        
        gpt_vision_prompt = f"Is the person in the center of the picture barefooted or in socks? Answer only with True or False. If you can't see their feet answer only with 'None'. If the person you see is behind a wall answer only with 'Wall' and don't check the other rules. If you are not at least 70% sure that a person is wearing shoes answer True"
        
        answer = self.tm.img_description(prompt=gpt_vision_prompt,camera_name="both")["message"]
        
        print("Esta descalza:"+answer)
        
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

    # --------------------------- Complementary function 2: Check drink ------------------------------
    def check_drink(self):
        print(self.consoleFormatter.format("LOOK4DRINK", "HEADER"))
        self.tm.robot_stop_srv()
        gpt_vision_prompt = f"Is the person in the center of the picture holding a bottle,a juice box, a cup, a can, or any kind of drink in their hand? Answer only with True or False. If the person you see is behind a wall answer only with 'Wall' and don't check the other rules"
        answer = self.tm.img_description(gpt_vision_prompt,camera_name="both")["message"]
        print("Tiene bebida:"+answer)
        if "True" in answer:
            # Tiene una bebida
            self.has_drink = 1
        elif "Wall" in answer:
            # Detras de una pared
            self.has_drink = 4
        else:
            # No tiene una bebida
            self.has_drink = 0
            
    
    # --------------------- Complementary function 3: Check if a person is in the Forbideen room --------------------------
    def check_forbidden(self):
        
        self.tm.robot_stop_srv()
        
        if self.last_place == self.forbidden:
            
            self.tm.talk("You should not be here, because this room is forbidden.","English", wait=True)
            rospy.sleep(1)
            self.move_head_srv("default")
            self.someone_in_forbidden = True
            return True
        
        return False
    

    # --------------------- Complementary function 4: Calculate the waiting time --------------------------
    def calcular_tiempo_espera(self, angulo_actual, angulo_anterior):
        
        # Allowed angle range
        angulo_minimo = -1.39  # Given in radians
        angulo_maximo = 1.39   # Given in radians
        
        distancia = abs(angulo_actual - angulo_anterior)
        
        distancia_normalizada = distancia / (angulo_maximo - angulo_minimo)
        
        tiempo_minimo = 0 
        tiempo_maximo = 9 
        
        tiempo_espera = tiempo_minimo + distancia_normalizada * (tiempo_maximo - tiempo_minimo)
        
        return tiempo_espera
    

    # --------------------------- THIRD STATE: ASK FOR SHOES ------------------------------
    def ASK4SHOES(self):
        
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        
        self.tm.talk("You must not wear shoes in this place!. Please, go to the entrance and take them off!","English", wait=False)

    # --------------------------- FOURTH STATE: ASK FOR DRINK ------------------------------
    def ASK4DRINK(self):
        
        print(self.consoleFormatter.format("ASK4DRINK", "HEADER"))
        
        self.tm.talk("You must have a Drink in your hand. Please, go to the kitchen and grab a drink, you must keep it in your hand!","English", wait=False)

    # --------------------------- FIFTH STATE: ASK TO LEAVE ------------------------------
    def ASK2FOLLOW(self):
        
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        self.tm.setRPosture_srv("stand")
        self.tm.talk("Follow me to the other party guests please","English", wait=False)
        self.tm.go_to_place(self.party_place)
        self.tm.talk("Now, please stay here and do not come back to the forbidden room","English", wait=False)
            
            
    # --------------------------- SIXTH STATE: GO TO THE NEXT ROOM ------------------------------
    def on_enter_GO2NEXT(self):
        
        print("Current Place: " + self.last_place)
        
        self.tm.talk("I'm going to check another room!","English", wait=False)
        self.tm.setRPosture_srv("stand")
        
        print(self.consoleFormatter.format("GO2NEXT", "HEADER"))
        
        for place_num in range(len(self.list_places)):
            
            place = self.list_places[place_num]
            
            if place not in self.checked_places:
                
                self.tm.talk("I'm gonna check " + self.places_names[place_num],"English", wait=False)
                if self.tm.last_place != place:
                    self.tm.go_to_place(place)
                print("Next Place: " + place)
                self.checked_places.append(place)
                self.last_place = place
                self.arrive_next()
                
        self.tm.talk("But i have checked all of the rooms! Yipee","English")
        
        # Finishing the task
        os._exit(os.EX_OK)

    # --------------------- Complementary function 5: Check the rules --------------------------
    def check_rules(self):
        
        is_in_forbidden = self.check_forbidden()
        
        # Threads to check the rules related to wearing shoes
        shoes_check_thread = threading.Thread(target=self.check_shoes)
        shoes_check_thread.start()
        
        # Threads to check the rules related to having a drink
        drink_check_thread = threading.Thread(target=self.check_drink)
        drink_check_thread.start()
        
        while self.has_drink==2 or self.has_shoes==2:
            
            rospy.sleep(0.1)
            
        if self.has_drink == 0:
            self.ASK4DRINK()
            
        if self.has_shoes == 1:
            self.ASK4SHOES()
            
        elif self.has_shoes == 3:
            self.tm.talk("I can't see your feet, but remember you can't use shoes in the house.","English", wait=False)
        
        if is_in_forbidden or self.has_drink==0 or self.has_shoes==1:
            
            print("persona rompiendo una regla")
            self.breakers_found += 1
            
        else:
            if self.has_drink != 4 and self.has_shoes != 4 and (not is_in_forbidden):
                self.tm.talk("Congratulations! You're not breaking any rule","English", wait=False)
                        
        self.has_drink = 2
        self.has_shoes = 2
        

    # --------------------------- ROSPY CHECK FUNCTION ------------------------------
    def check_rospy(self):
        
        # Ends all processes if rospy is not running
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        
        os._exit(os.EX_OK)

    # --------------------------- RUN FUNCTION TO START THE TASK CLASS CONSTRUCTOR ------------------------------
    def run(self):
        self.start()

# --------------------------- MAIN FUNCTION OF THE STICKLER TASK CLASS ------------------------------
if __name__ == "__main__":
    sm = STICKLER_RULES()
    sm.run()
    rospy.spin()

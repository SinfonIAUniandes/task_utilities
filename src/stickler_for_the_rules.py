#!/usr/bin/env python3

# --------------------------- GENERAL LIBRARIES IMPORTS ------------------------------
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import threading
import rospy
import math
import os

# --------------------------- STICKLER FOR THE RULES TASK CLASS  ------------------------------
class STICKLER_RULES(object):
    
    # --------------------------- Class Constructor ------------------------------
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        # Task name
        self.task_name = "stickler_for_the_rules"
        
        # Possible states for the states machine
        states = ['INIT', 'LOOK4PERSON', 'ASK4SHOES', 'ASK4DRINK', 'GO2NEXT']
        
        # Initialization of the task module
        self.tm = tm(perception = True,speech=True, navigation=True, pytoolkit=True, manipulation=True)
        self.tm.initialize_node(self.task_name)
        self.tm.vision_model = "gpt-4o"
        
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

        # --------------------------- Task Parameters ------------------------------
        
        # Task parameter if the robot wants to confirm that the guests corrected the rule
        self.confirm_compliance = False
        self.confirm_compliance_forbidden = True
        
        self.someone_in_forbidden = False
        
        # The robot starts at 0 and then moves head to these angles
        self.angles_to_check = [0,-60,60]
        
        # Where the other guests are located, it is for the forbidden room
        self.party_place = ""
        # TODO create the initial place in NAVIGATION
        self.initial_place = "house_door"
        self.last_place = self.initial_place
        
        # TODO Set the forbidden room
        self.forbidden = "office"
        
        # List of places to check
        self.list_places = ["living_room", "hallway", "office", "kitchen"]
        self.places_names = ["living room","hallway",  "office", "kitchen"]
        
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
        
        self.tm.toggle_filter_by_distance(True,3,["person"])
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
        
        self.tm.talk("I am going to check if the guests are breaking the rules!","English", wait=False)
        self.tm.setRPosture_srv("stand")
        number_of_guests = 0
        for angle in self.angles_to_check:
            
            print("angulo actual:",angle)
            self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
            
            if angle==0:
                rospy.sleep(1)
                
            elif angle==-60:
                rospy.sleep(3)
                
            elif angle==60:
                rospy.sleep(5)
                
            self.tm.labels = dict()
            
            rospy.sleep(2)
                
            people = self.tm.labels.get("person", [])
            number_of_guests += len(people)
            
            for person in people:
                
                print("centrando persona:", person)
                self.tm.center_head_with_label(person)
                
                print("checkeando reglas")
                self.check_rules()
                
                print("reglas checkeadas")
                # go back to initial check position before moving to the next person
                self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
            
        if self.confirm_compliance_forbidden and self.someone_in_forbidden:
            if (self.last_place == self.forbidden) and (self.party_place != ""):
                # Moving to the ASK2FOLLOW state
                self.ASK2FOLLOW()
            
        if (number_of_guests > 0) and (self.last_place != self.forbidden):
            self.party_place = self.last_place
            
        self.tm.setRPosture_srv("stand")
        self.tm.talk("I'm done checking this room'!","English", wait=False)
        
        # Moving to the GO2NEXT state
        self.rules_checked()
        
    # --------------------------- Complementary function 1: Check shoes ------------------------------
    def check_shoes(self):
        
        print(self.consoleFormatter.format("LOOK4SHOES", "HEADER"))
        
        self.tm.robot_stop_srv()
        
        vision_prompt = f"Is the person in the center of the picture barefooted or in socks? Answer only with True or False. If you can't see their feet answer only with 'None'. If the person you see is behind a wall answer only with 'Wall' and don't check the other rules. If you are not at least 70% sure that a person is wearing shoes answer True"
        
        answer = self.tm.img_description(prompt=vision_prompt,camera_name="both")["message"]
        
        print("Esta descalza: "+answer)
        
        if "true" in answer.lower():
            # Esta descalso o con medias
            self.has_shoes = 0
            
        elif "none" in answer.lower():
            # No se sabe
            self.has_shoes = 3
            
        elif "wall" in answer.lower():
            # Detras de una pared
            self.has_shoes = 4
            
        else:
            # Tiene zapatos
            self.has_shoes = 1

    # --------------------------- Complementary function 2: Check drink ------------------------------
    def check_drink(self):
        print(self.consoleFormatter.format("LOOK4DRINK", "HEADER"))
        self.tm.robot_stop_srv()
        vision_prompt = f"Is the person in the center of the picture holding a bottle,a juice box, a cup, a can, or any kind of drink in their hand? Answer only with True or False. If the person you see is behind a wall answer only with 'Wall' and don't check the other rules"
        answer = self.tm.img_description(vision_prompt,camera_name="both")["message"]
        print("Tiene bebida: "+answer)
        if "true" in answer.lower():
            # Tiene una bebida
            self.has_drink = 1
        elif "wall" in answer.lower():
            # Detras de una pared
            self.has_drink = 4
        else:
            # No tiene una bebida
            self.has_drink = 0
            
    
    # --------------------- Complementary function 3: Check if a person is in the Forbidden room --------------------------
    def check_forbidden(self):
        
        self.tm.robot_stop_srv()
        
        if self.last_place == self.forbidden:
            
            self.tm.talk("You should not be here, because this room is forbidden.","English", wait=True)
            self.someone_in_forbidden = True
            return True
        
        return False

    # --------------------------- THIRD STATE: ASK FOR SHOES ------------------------------
    def ASK4SHOES(self):
        
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        
        self.tm.talk("You must not wear shoes in this place!. Please, go to the entrance and take them off!","English", wait=True)

    # --------------------------- FOURTH STATE: ASK FOR DRINK ------------------------------
    def ASK4DRINK(self):
        
        print(self.consoleFormatter.format("ASK4DRINK", "HEADER"))
        
        self.tm.talk("You must have a Drink in your hand. Please, go to the kitchen and grab a drink, you must keep it in your hand!","English", wait=True)

    # --------------------------- FIFTH STATE: ASK TO LEAVE ------------------------------
    def ASK2FOLLOW(self):
        
        print(self.consoleFormatter.format("ASK4SHOES", "HEADER"))
        self.tm.setRPosture_srv("stand")
        self.tm.talk("Follow me to the other party guests please","English", wait=False)
        self.tm.go_to_place(self.party_place)
        self.tm.go_to_relative_point(0,0,180)
        vision_prompt = f"Are the people in the picture close to camera (within 1 to 2 meters of the camera)? Answer only with True or False. If there is no one in the picture answer False"
        answer = self.tm.img_description(prompt=vision_prompt,camera_name="both")["message"]
        print("followed?:",answer)
        if "true" in answer.lower():
            self.tm.talk("Thank you for following me. Now, please stay here and do not come back to the forbidden room","English", wait=False)
        else:
            self.tm.talk("I see you did not comply but i must keep looking for the other guests. But please remember you must not be in the forbidden room","English", wait=False)    
            
    # --------------------------- SIXTH STATE: GO TO THE NEXT ROOM ------------------------------
    def on_enter_GO2NEXT(self):
        
        print("Current Place: " + self.last_place)
        
        
        print(self.consoleFormatter.format("GO2NEXT", "HEADER"))
        
        for place_num in range(len(self.list_places)):
            
            place = self.list_places[place_num]
            if place == "kitchen":
                self.tm.toggle_filter_by_distance(True,5,["person"])
            elif place == "lviing_room":
                self.tm.toggle_filter_by_distance(True,5,["person"])
            elif place == "office":
                self.tm.toggle_filter_by_distance(True,3,["person"])
            elif place == "hallway":
                self.tm.toggle_filter_by_distance(True,3,["person"])

            if place not in self.checked_places:
                self.tm.talk("I'm gonna check " + self.places_names[place_num],"English", wait=False)
                self.tm.setRPosture_srv("stand")
                self.tm.go_to_place(place)
                print("Next Place: " + place)
                self.checked_places.append(place)
                self.last_place = place
                self.arrive_next()
                
        self.tm.talk("I have checked all of the rooms! Yipee","English")
        
        # Finishing the task
        os._exit(os.EX_OK)

    # --------------------- Complementary function 5: Check the rules --------------------------
    def check_rules(self):
        
        
        # Threads to check the rules related to wearing shoes
        shoes_check_thread = threading.Thread(target=self.check_shoes)
        shoes_check_thread.start()
        
        # Threads to check the rules related to having a drink
        drink_check_thread = threading.Thread(target=self.check_drink)
        drink_check_thread.start()
        
        is_in_forbidden = self.check_forbidden()
        
        self.tm.talk("Please wait while i check if you're breaking more rules",wait=False)
        
        while self.has_drink==2 or self.has_shoes==2:
            
            rospy.sleep(0.1)
            
        if self.has_drink == 0:
            self.ASK4DRINK()
            
        if self.has_shoes == 1:
            self.ASK4SHOES()
            
        elif self.has_shoes == 3:
            self.tm.talk("I can't see your feet, but remember you can't use shoes in the house.","English", wait=True)
        
        if is_in_forbidden or self.has_drink==0 or self.has_shoes==1:
            print("persona rompiendo una regla")
            
        else:
            if self.has_drink != 4 and self.has_shoes != 4 and (not is_in_forbidden):
                self.tm.talk("Congratulations! You're not breaking any rule","English", wait=True)
                        
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

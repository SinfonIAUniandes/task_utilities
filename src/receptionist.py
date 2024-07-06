#!/usr/bin/env python3

# --------------------------- GENERAL LIBRARIES IMPORTS ------------------------------
from transitions import Machine
from task_module import Task_module as tm
from robot_toolkit_msgs.srv import set_angle_srv
import ConsoleFormatter
import threading
import rospy
import math
import os

# --------------------------- RECEPTIONIST TASK CLASS  ------------------------------
class RECEPTIONIST(object):
    
    # --------------------------- Class Constructor ------------------------------
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        # Task name
        self.task_name = "receptionist"
        
        # Possible states for the states machine
        states = ['INIT', 'GO2DOOR', 'WAIT4GUEST', 'SAVE_GUEST', 'GO2LIVING','INTRODUCE_GUEST']
        
        # Initialization of the task module
        self.tm = tm(perception = True,speech=True, navigation=True, pytoolkit=True, manipulation=True)
        self.tm.initialize_node(self.task_name)
        
        # Definition of the permitted transitions between states
        transitions = [
            {'trigger': 'start', 'source': 'RECEPTIONIST', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO2DOOR'},
            {'trigger': 'arrive_house_door', 'source': 'GO2DOOR', 'dest': 'WAIT4GUEST'},
            {'trigger': 'guest_found', 'source': 'WAIT4GUEST', 'dest': 'SAVE_GUEST'},
            {'trigger': 'guest_saved', 'source': 'SAVE_GUEST', 'dest': 'GO2LIVING'},
            {'trigger': 'arrive_living_room', 'source': 'GO2LIVING', 'dest': 'INTRODUCE_GUEST'},
            {'trigger': 'guest_introduced', 'source': 'INTRODUCE_GUEST', 'dest': 'GO2DOOR'},
        ]
        
        # Creation of the states machine
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='RECEPTIONIST')

        # Thread to check if rospy is running
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        # --------------------------- Pytoolkit Services ------------------------------

        rospy.wait_for_service("/pytoolkit/ALMotion/set_angle_srv")
        self.set_angle_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_angle_srv",set_angle_srv)
        

        # --------------------------- Task Parameters ------------------------------
        
        # The angles of the chairs the robot must check to introduce the people
        self.chair_angles = [-10,35,70]
        
        self.host = {
            "name": "charlie",
            "age": self.categorize_age(21),
            "drink": "Milk",
            "gender": "Man",
            "pronoun": "he",
        }
        
        self.first_guest = {
            "hair": "",
            "clothes": "",
            "age": "",
            "gender": "",
        }
        
        self.second_guest = {}
        
        self.first_guest_saved = False
        self.host_saved = False
        self.guests_count = 1
        # Where the robot must introduce the guests
        self.seating_place = "living_room"
        self.initial_place = "house_door"
        self.greeting_place = "house_door"
        self.last_place = self.initial_place
        
    
    # --------------------------- STATES FUNCTIONS ------------------------------
    
    # --------------------------- FIRST STATE: INIT ------------------------------
    def on_enter_INIT(self):
        
        
        self.tm.initialize_pepper()
        self.tm.turn_camera("depth_camera","custom",1,15)
        self.tm.toggle_filter_by_distance(True,2.5,["person"])
        self.tm.publish_filtered_image("face", "front_camera")
        self.tm.remove_faces_data()
        
        self.tm.set_current_place(self.initial_place)
        
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        
        self.tm.talk("I am going to do the " + self.task_name + " task","English", wait=False)
        
        # Moving to the LOOK4PERSON state
        self.beggining()
        
    # --------------------------- SECOND STATE: GO TO THE HOUSE DOOR ------------------------------
    def on_enter_GO2DOOR(self):
        
        self.tm.talk("I'm going to the door to greet the guests!","English", wait=False)
        
        print(self.consoleFormatter.format("GO2DOOR", "HEADER"))
        
        self.tm.setRPosture_srv("stand")
        #TODO EL robot navega de otro lugar a la puerta
        if self.tm.current_place != self.initial_place:
            self.tm.go_to_place(self.greeting_place)
        self.arrive_house_door()
        
    # --------------------------- THIRD STATE: WAIT FOR THE GUESTS ------------------------------
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.go_to_pose("default_head")
        self.tm.setRPosture_srv("stand")
        self.tm.setMoveHead_srv.call("up")
        self.tm.talk("Waiting for guests","English")
        self.tm.look_for_object("person")
        self.tm.wait_for_object(-1)
        self.guest_found()

    # --------------------------- FOURTH STATE: LOOK FOR PERSON ------------------------------
    def on_enter_SAVE_GUEST(self):
        
        print(self.consoleFormatter.format("LOOK4PERSON", "WARNING"))
        
        self.tm.set_angles_srv(["HeadYaw", "HeadPitch"],[0, -0.45],0.1,)
        
        print("guests:",self.guests_count)
        self.tm.show_topic("/perception_utilities/filtered_image")
        if self.guests_count == 1:
            self.tm.turn_camera("front_camera","custom",2,10)
            self.tm.talk("Hello guest, please center your face in the circle in my tablet so i can remember your face later!","English", wait=False)
            rospy.sleep(3)
            gpt_hair_thread = threading.Thread(target=self.gpt_hair_t)
            gpt_hair_thread.start()
            gpt_clothes_thread = threading.Thread(target=self.gpt_clothes_t)
            gpt_clothes_thread.start()
            save_face_thread = threading.Thread(target=self.save_face_t,args=[1])
            save_face_thread.start()
            age_gender_thread = threading.Thread(target=self.age_gender_t)
            age_gender_thread.start()
        
            rospy.sleep(4)
            self.tm.talk("I will ask you a couple of questions while i get a good look at your face! Please answer them when my eyes are blue","English", wait=True)
        else:
            self.tm.talk("I will ask you a couple of questions. Please answer them when my eyes are blue!","English", wait=True)
        name = self.tm.q_a("name").lower()
        drink = self.tm.q_a("drink")
        
        if self.guests_count == 1:
            self.first_guest["name"] = name
            self.first_guest["drink"] = drink
            while self.first_guest["age"]=="" or self.first_guest["gender"]=="" or not self.first_guest_saved:
                rospy.sleep(0.1)
        elif  self.guests_count == 2:
            self.second_guest["name"] = name
            self.second_guest["drink"] = drink
        self.tm.setRPosture_srv("stand")
        self.tm.talk("Thank you'!","English", wait=False)
        self.tm.turn_camera("front_camera","custom",1,15)
        
        self.guest_saved()
        
    # --------------------------- FIFTH STATE: GO TO THE LIVING ROOM ------------------------------
    def on_enter_GO2LIVING(self):
        
        self.tm.talk("Please follow me to the living room to introduce you!","English", wait=False)
        
        print(self.consoleFormatter.format("GO2LIVING", "HEADER"))
        
        self.tm.setRPosture_srv("stand")
        self.tm.show_topic("/perception_utilities/yolo_publisher")
        self.tm.go_to_place(self.seating_place)
        self.arrive_living_room()
        
    
    # --------------------------- SIXTH STATE: INTRODUCE GUEST ------------------------------
    def on_enter_INTRODUCE_GUEST(self):
        
        print(self.consoleFormatter.format("INTRODUCE_GUEST", "WARNING"))
        
        if self.guests_count == 1:
            self.tm.talk(f'Hello everyone, this is {self.first_guest["name"]}, {self.first_guest["pronoun"]} is a {self.first_guest["gender"]}.  {self.first_guest["pronoun"]} is {self.first_guest["age"]}, and {self.first_guest["pronoun"]} likes to drink {self.first_guest["drink"]}.',"English", wait=False)
        else:
            self.tm.talk(f'Hello everyone, this is {self.second_guest["name"]}, They like to drink {self.second_guest["drink"]}.',"English", wait=False)
            
        numero_personas = 0
        
        occupied_chairs = []
        
        for angle in self.chair_angles:
            
            print("angulo actual:",angle)
            self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
            
            if angle==-10:
                rospy.sleep(1)
                
            elif angle==35:
                rospy.sleep(2)
                
            elif angle==60:
                rospy.sleep(3)
                
            self.tm.labels = dict()
            
            rospy.sleep(1)
                
            persons = self.tm.labels.get("person", [])
            
            for person in persons:
                
                occupied_chairs.append(angle)
                numero_personas +=1
                
                self.tm.talk("Recognizing person!","English", wait=False)
                if self.guests_count == 1:
                    
                    print("centrando persona:",person)
                    self.tm.center_head_with_label(person, height=0,resolution=2)
                    print("persona centrada")
                    
                    self.tm.turn_camera("front_camera","custom",2,10)
                    rospy.sleep(2)
                    self.tm.show_topic("/perception_utilities/filtered_image")
                    self.tm.talk("Hi charlie, can you look at me and try to center your face in the circle in my tablet", "English", wait=False)
                    save_face_thread = threading.Thread(target=self.save_face_t,args=[2])
                    save_face_thread.start()
                    self.tm.talk(f"{self.first_guest['name']}, I introduce to you {self.host['name']}, {self.host['pronoun']} is a {self.host['gender']}. {self.host['pronoun']} is {self.host['age']}, and {self.host['pronoun']} likes to drink {self.host['drink']}.", "English", wait=False)
                    while not self.host_saved:
                        rospy.sleep(0.1)
                    print("host_saved")
                    self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
                else:
                    self.tm.turn_camera("front_camera","custom",2,10)
                    rospy.sleep(2)
                    guest_name = self.tm.recognize_face(3)
                    rospy.sleep(5)
                    print("guest:",guest_name)
                    if guest_name == "guest1":
                        self.tm.talk(f"{self.second_guest['name']}, I introduce to you {self.first_guest['name']} is a {self.first_guest['gender']}. {self.first_guest['pronoun']} is {self.first_guest['age']}, and {self.first_guest['pronoun']} likes to drink {self.first_guest['drink']}. {self.first_guest['pronoun']} is wearing {self.first_guest['clothes']}, and has {self.first_guest['hair']} hair.", "English", wait=False)
                    elif guest_name == self.host["name"]:
                        self.tm.talk(f"{self.second_guest['name']}, I introduce to you {self.host['name']} is a {self.host['gender']}. {self.host['pronoun']} is {self.host['age']}, and {self.host['pronoun']} likes to drink {self.host['drink']}.", "English", wait=False)
                    else:
                        self.tm.talk("I'm sorry I couldn't recognize you, could you tell me your name?", "English", wait=True)
                        name = self.tm.q_a("name").lower()
                        if name == self.first_guest["name"]:
                            self.tm.talk(f"{self.second_guest['name']}, I introduce to you {self.first_guest['name']} is a {self.first_guest['gender']}. {self.first_guest['pronoun']} is {self.first_guest['age']}, and {self.first_guest['pronoun']} likes to drink {self.first_guest['drink']}. {self.first_guest['pronoun']} is wearing {self.first_guest['clothes']}, and has {self.first_guest['hair']} hair.", "English", wait=True)
                        elif name == self.host["name"]:
                            self.tm.talk(f"{self.second_guest['name']}, I introduce to you {self.host['name']} is a {self.host['gender']}. {self.host['pronoun']} is {self.host['age']}, and {self.host['pronoun']} likes to drink {self.host['drink']}.", "English", wait=True)
                        else:
                            self.tm.talk(f"I'm sorry I couldn't introduce you to {self.second_guest['name']}, please introduce yourself", "English", wait=True)
                self.tm.turn_camera("front_camera","custom",1,15)
                
            if self.guests_count == 1 and self.host_saved:
                break
            if len(occupied_chairs) == 2:
                break

        self.tm.setRPosture_srv("stand")
        
        if numero_personas == 0:
            if self.guests_count == 1:
                self.tm.talk(f"I'm sorry i couldn't see anyone. Could everyone introduce themselves to {self.first_guest['name']} please?", "English", wait=True)
            else:
                self.tm.talk(f"I'm sorry i couldn't see anyone. Could everyone introduce themselves to {self.second_guest['name']} please?", "English", wait=True)
            
        for angle in self.chair_angles:
            if not angle in occupied_chairs:
                self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
                self.point_angle(angle)
                if angle==-10:
                    rospy.sleep(1)
                    
                elif angle==35:
                    rospy.sleep(3)
                    
                elif angle==60:
                    rospy.sleep(5)
                self.tm.talk("I see that chair is free. Please sit there.", "English", wait=True)
                self.tm.setRPosture_srv("stand")
                
                if self.guests_count == 2:
                    self.tm.talk("Receptionist task completed succesfully", "English", wait=True)
                    os._exit(os.EX_OK)
                self.guests_count +=1
                
                break
                
        # Moving to the GO2NEXT state
        self.guest_introduced()
        
    # --------------------------- Complementary thread 1: Save Face ------------------------------
    def save_face_t(self,who):
        
        print(self.consoleFormatter.format("SAVE_FACE THREAD", "HEADER"))
        if who == 1:
            attempts = 0
            while not self.first_guest_saved and attempts<3:
                self.first_guest_saved  = self.tm.save_face("guest1", 5)
        else:
            attempts = 0
            while not self.host_saved and attempts<3:
                self.host_saved = self.tm.save_face("charlie", 5)
        

    # --------------------------- Complementary thread 2: Get hair color with gptvision ------------------------------
    def gpt_hair_t(self):
        
        print(self.consoleFormatter.format("GET HAIR THREAD", "HEADER"))
        
        gpt_vision_prompt = "Please answer about the person centered in the image: What is the hair color of the person? Answer only with the color for example: Black"
        answer = self.tm.img_description(gpt_vision_prompt)["message"]
        self.first_guest["hair"] = answer
        

    # --------------------------- Complementary thread 3: Get clothes color with gptvision ------------------------------
    def gpt_clothes_t(self):
        
        print(self.consoleFormatter.format("GET CLOTHES THREAD", "HEADER"))
        
        gpt_vision_prompt = "Please answer about the person centered in the image: What is the main color of the clothes this person is wearing? Answer only with the color for example: Black"
        answer = self.tm.img_description(gpt_vision_prompt)["message"]
        self.first_guest["clothes"] = answer

    # --------------------------- Complementary thread 4: Get the persons age and gender with perception ------------------------------
    def age_gender_t(self):
        
        print(self.consoleFormatter.format("GET ATTRIBUTES THREAD", "HEADER"))
        
        person_attributes = self.tm.get_person_description()
        self.first_guest["age"] = self.categorize_age(person_attributes["age"])
        self.first_guest["gender"] = person_attributes["gender"]
        self.first_guest["pronoun"] = "he" if person_attributes["gender"] == "Man" else "she"
    
    
    # --------------------------- Complementary function 1: Calculates ranges of age ------------------------------
    def categorize_age(self, age):
        if age < 18:
            category = "a teenager"
        elif age < 25:
            category = "a young adult"
        elif age < 35:
            category = "an adult"
        elif age < 50:
            category = "a middle aged adult"
        elif age < 65:
            category = "a senior"
        else:
            category = "an elder"
        return category

    # --------------------------- Complementary function 2: Point to desired angle ------------------------------

    def point_angle(self, angle):
        self.tm.go_to_pose("open_both_hands")
        if angle < 0:
            self.tm.go_to_pose("point_there_left",0.1)
            joint = "LElbowRoll"
        elif angle > 0:
            self.tm.go_to_pose("point_there_right",0.1)
            joint = "RElbowRoll"
        rospy.sleep(2)
        self.tm.set_angles_srv([joint], [math.radians(angle)], 0.1)
            
        
    # --------------------------- ROSPY CHECK THREAD ------------------------------
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
    sm = RECEPTIONIST()
    sm.run()
    rospy.spin()

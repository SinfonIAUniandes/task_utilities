#!/usr/bin/env python3

# --------------------------- GENERAL LIBRARIES IMPORTS ------------------------------
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import threading
import rospy
import math
import ast
from code_generation import ls_generate as gen
from code_generation import generate_utils 
from code_generation.database.models import Model
class EGPSR(object):

    def __init__(self):
        self.gen = gen.LongStringGenerator()

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        # Task name
        self.task_name = "egpsr"
        
        # Possible states for the states machine
        states = ['INIT', 'LOOK4PERSON', 'EGPSR', 'GO2NEXT']
        
        # Initialization of the task module
        self.tm = tm(perception = True,speech=True, navigation=True, pytoolkit=True, manipulation=True)
        self.tm.initialize_node(self.task_name)
        
        # Definition of the permitted transitions between states
        transitions = [
            {'trigger': 'start', 'source': 'EGPSR', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'SEARCH4GUEST'},
            {'trigger': 'person_raised_hand', 'source': 'SEARCH4GUEST', 'dest': 'EGPSR'},
            {'trigger': 'again', 'source': 'SEARCH4GUEST', 'dest': 'SEARCH4GUEST'},
            {'trigger': 'GPSR_done', 'source': 'EGPSR', 'dest': 'SEARCH4GUEST'}
        ]
        
        # State machine creation
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='INIT')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        ############################# GLOBAL VARIABLES #############################
        self.init_place = "house_door"
        self.places_names = ["kitchen","living_room","dining","bedroom"] #TODO: Change if there are more places
        self.head_angles = [60,0,-60]
        self.task_counter = 0 
        
    
    # --------------------------- STATES FUNCTIONS ------------------------------
    
    # --------------------------- FIRST STATE: INIT ------------------------------
    def on_enter_INIT(self):
        
        
        self.tm.initialize_pepper()
        self.tm.turn_camera("bottom_camera","custom",1,15)
        # CAMBIAR EN LA COMPETENCIA, EL ROBOT NO INICIA EN GPSR_LOCATION DEBE NAVEGAR ALLA TODO
        self.tm.current_place = "house_door"
        self.tm.set_current_place(self.init_place)
        self.tm.turn_camera("depth_camera","custom",1,15)
        self.tm.toggle_filter_by_distance(True,2,["person"])
        
        self.tm.set_current_place(self.initial_place)
        self.checked_places.append(self.initial_place)
        self.last_place = self.initial_place
        
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.talk("I am going to do the  "+ self.task_name + " task","English")
        self.beggining()
        
    
    def on_enter_EGPSR(self):
        print(self.consoleFormatter.format("EGPSR", "HEADER"))
        self.tm.look_for_object("person")
        self.tm.talk("Hello guest, please tell me what you want me to do, I will try to execute the task you give me. Please talk loud and say the task once. You can talk to me when my eyes are blue: ","English")
        task = self.tm.speech2text_srv(0)
        self.generated_code = ""

        # Thread to call the method that generates code according to a task request
        code_gen_thread = threading.Thread(target=self.code_gen_t,args=[task])
        code_gen_thread.start()

        self.tm.talk(f"Your command is {task}","English", wait=True)
        self.tm.talk(f"If that is correct, please touch my head. If not, please wait until my eyes are blue again ","English", wait=False)
        
        correct = self.tm.wait_for_head_touch(message="", message_interval=100, timeout=13)
        
        while not correct:
            self.tm.talk("I am sorry, may you please repeat your command?","English", wait=True)
            task = self.tm.speech2text_srv(0)

            # Thread to call the method that generates code according to a task request after having failed in the first attempt
            code_gen_thread = threading.Thread(target=self.code_gen_t,args=[task])
            code_gen_thread.start()

            self.tm.talk(f"Your command is {task}. If that is correct, please touch my head","English", wait=True)
            correct = self.tm.wait_for_head_touch(message="", message_interval=13, timeout=13)
        
        self.tm.talk("I am processing your request")

        while self.generated_code == "":
            rospy.sleep(0.1)

        # Execution of the generated code if it is possible
        if self.generated_code != "impossible":
            exec(self.generated_code)

        print(f"Task: {task}")
        self.GPSR_done()
        

    def on_enter_SEARCH4GUEST(self):
        print(self.consoleFormatter.format("SEARCH4GUEST", "HEADER"))
        self.tm.go_to_pose("default_head")
        self.tm.go_to_pose("standard")
        self.current_place = self.places_names[0]
        self.tm.talk(f"I am going to search a guest in the {self.current_place}","English", False)
        self.tm.go_to_place(self.current_place)
            
        for angle in self.places_names:
            
            print("\nCurrent Angle:",angle,"\n")
            self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
            
            if angle==0:
                rospy.sleep(3)
                
            elif angle==-60:
                rospy.sleep(3)
                
            elif angle==60:
                rospy.sleep(5)
                
            self.tm.labels = dict()
            rospy.sleep(2)
            persons = self.tm.labels.get("person", [])
            
            for person in persons:
                
                # Centering the robot camera with the person
                self.tm.center_head_with_label(person)

                person_response = self.tm.img_description(
                        "Observe the person in front of you and respond only with True if the person is raising their hand. Otherwise, respond only with False. Here is an example output: False",
                        "front_camera")
                
                is_raisin = True if person_response["message"] == "True" else False

                self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
                
                if is_raisin:
                    # Going to the EGPSR state
                    self.person_arrived()
        
        self.places_names.pop(0)
        self.tm.talk("I have finished checking this room!","English", wait=False)

        # Returning to this same state
        self.again()

        
    # Thread to generate code
    def code_gen_t(self, task):
        
        print(self.consoleFormatter.format("GEN CODE THREAD", "HEADER"))
        
        contador = 0
        code = ""

        while contador<1:
            code = self.gen.generate_code(task, Model.GPT4).replace("`","").replace("python","")
            pattern = r'self\.tm\.go_to_place\((.*?)\)'
            code = re.sub(pattern, r'self.tm.go_to_place(\1, lower_arms=False)', code)
            print(code)

            if not "I am sorry but I cannot complete this task" in code:
                print("\nIt is possible to execute the request")

                if self.is_valid_syntax(code):
                    self.generated_code = code
                    self.task_counter += 1
                    contador = 5
            contador += 1

        if contador==1:
            self.generated_code = "impossible"
            self.tm.talk("I cannot the following task: " + task,"English")
        
        for place_num in range(len(self.list_places)):
            
            place = self.list_places[place_num]
            
            if place not in self.checked_places:
                
                self.tm.talk("I'm gonna check " + self.places_names[place_num],"English", wait=False)
                self.tm.setRPosture_srv("stand")
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
            self.breakers_found += 1
            
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

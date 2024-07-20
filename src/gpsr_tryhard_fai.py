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
import re
import ast
from code_generation import ls_generate as gen
from code_generation import generate_utils 
from code_generation.database.models import Model
from std_msgs.msg import String
from robot_toolkit_msgs.msg import speech_recognition_status_msg

class GPSR(object):
    def __init__(self):
        self.gen = gen.LongStringGenerator()

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "GPSR"
        states = ['INIT', 'WAIT4GUEST', 'GPSR', 'GO2GPSR']
        self.tm = tm(perception = True,speech=True,manipulation=True, navigation=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'GPSR', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO2GPSR'},
            {'trigger': 'go_to_gpsr', 'source': 'GO2GPSR', 'dest': 'WAIT4GUEST'},
            {'trigger': 'person_arrived', 'source': 'WAIT4GUEST', 'dest': 'GPSR'},
            {'trigger': 'GPSR_done', 'source': 'GPSR', 'dest': 'GO2GPSR'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='GPSR')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        ############################# GLOBAL VARIABLES #############################
        self.gpsr_location = "house_door"
        self.init_place = "house_door"
        self.task_counter = 0 
        self.generated_code = ""
        
        # List of possible request key words:

        self.synonyms = {
            'fetch': ['bring', 'get', 'fetch', 'grab', 'retrieve'],
            'count': ['count', 'number of', 'how many'],
            'search': ['find', 'search', 'locate', 'look for','what', 'heaviest', 'lightest', 'biggest', 'smallest', 'oldest', 'youngest', 'tallest', 'shortest', 'heavy', 'light', 'small', 'big', 'old', 'young', 'tall', 'short'],
            'follow': ['follow', 'come after', 'tail', 'track'],
            'introduce': ['introduce', 'presentation', 'show', 'present'],
            'describe': ['describe', 'tell about', 'detail', 'explain'],
            'identify': ['identify', 'who is', 'recognize', 'determine'],
            'navigate': ['go','go to', 'navigate', 'move to', 'head to'],
            'wait': ['wait', 'pause', 'hold on', 'delay'],
            'inform': ['tell me', 'inform', 'advise', 'notify'],
            'locate': ['where is', 'find location of', 'locate', 'position of','who'],
            'ask': ('ask', 'who', 'name'),
            'color': ('color', 'red', 'blue', 'yellow')
        }
        
        # Mapping related words to defined places
        self.place_mappings = {
            'living_room': ['sofa', 'tv', 'living', 'room', 'television', 'couch'],
            'hallway_cabinet': ['hallway_cabinet'],
            'desk': ['desk'],
            'shelf': ['shelf'],
            'coathanger': ['coathanger'],
            'exit': ['exit'],
            'tv_cable':['tv_cable'],
            'lounge_chair':['lounge_chair'],
            'lamp':['lamp'],
            'couch':['couch'],
            'coffee_table':['coffee_table'],
            'trashcan':['trashcan'],
            'kitchen_cabinet':['kitchen_cabinet'],
            'dinner_table':['dinner_table'],
            'dishwasher':['dishwasher'],
            'kitchen_counter':['kitchen_counter'],
            'hallway':['hallway'],
            'office':['office'],
            'kitchen': ['stove', 'oven', 'sink', 'kitchen'],
            'dining_room': ['table', 'dining', 'chairs', 'meal', 'eat'],
            'bedroom': ['bed', 'bedroom', 'dresser', 'pillow', 'blanket'],
            'entrance': ['entrance', 'door'],
            'refrigerator':['kitchen']
        }
        
        # Commonly ignored words that don't contribute to action determination
        self.irrelevant_words = set(['an', 'a', 'the', 'and', 'of', 'to', 'with', 'at', 'in', 'him', 'her', 'he', 'she', 'it', 'they','right','there', 'is', 'are', 'was', 'were', 'be', 'been', 'have', 'has', 'had', 'do', 'does', 'did', 'can', 'could', 'will', 'would', 'shall', 'should', 'may', 'might', 'must', 'about', 'above', 'across', 'after', 'against', 'along', 'among', 'around', 'as', 'before', 'behind', 'below', 'beneath', 'beside', 'between', 'beyond', 'but', 'by', 'despite', 'down', 'during', 'except', 'for', 'from', 'inside', 'into', 'like', 'near', 'off', 'on', 'onto', 'out', 'outside', 'over', 'past', 'since', 'through', 'throughout', 'till', 'to', 'toward', 'under', 'underneath', 'until', 'up', 'upon', 'without', 'within', 'yet'])
            
        self.action_keywords = {}
        for base_action, words in self.synonyms.items():
            for word in words:
                self.action_keywords[word] = base_action

    def determine_place(self, words):
        # Initialize scores for each defined place
        score = {place: 0 for place in self.place_mappings.keys()}
        
        # Score each word found in the input against the place mappings
        for word in words:
            for place, keywords in self.place_mappings.items():
                if word in keywords:
                    score[place] += 1

        # Determine the place with the highest score
        best_place = max(score, key=score.get)
        
        # Return the best place only if its score is greater than zero
        # This checks if any relevant words were actually found
        return best_place if score[best_place] > 0 else None

    def on_enter_INIT(self):
        generate_utils.load_code_gen_config() 
        self.tm.initialize_pepper()
        self.tm.turn_camera("bottom_camera","custom",1,15)
        # CAMBIAR EN LA COMPETENCIA, EL ROBOT NO INICIA EN GPSR_LOCATION DEBE NAVEGAR ALLA TODO
        self.tm.current_place = "house_door"
        self.tm.set_current_place(self.init_place)
        print("initial position: ",self.init_place)
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.talk("I am going to do the  "+ self.task_name + " task","English")
        self.beggining()

    def on_enter_GPSR(self):
        print(self.consoleFormatter.format("GPSR", "HEADER"))
        self.tm.look_for_object("")
        self.tm.talk("Hello guest, please tell me what you want me to do, I will try to execute the task you give me. Please talk loud and say the task once. You can talk to me when my eyes are blue: ","English")
        task = self.tm.speech2text_srv(0)
        self.generated_code = ""
        code_gen_thread = threading.Thread(target=self.code_gen_t,args=[task])
        code_gen_thread.start()
        self.tm.talk(f"Your command is {task}. If that is correct, please touch my head. If not, please wait until my eyes are blue again ","English", wait=False)
        print("antes de touch")
        correct = self.tm.wait_for_head_touch(message="", message_interval=100, timeout=13)
        print("despues de touch")
        
        while not correct:
            self.tm.talk("I am sorry, please repeat your command","English", wait=True)
            task = self.tm.speech2text_srv(0)
            code_gen_thread = threading.Thread(target=self.code_gen_t,args=[task])
            code_gen_thread.start()
            self.tm.talk(f"Your command is {task}. If that is correct, please touch my head","English", wait=True)
            correct = self.tm.wait_for_head_touch(message="", message_interval=13, timeout=13)
            
        print("processing your request")
        self.tm.talk("Processing your request")
        
        while self.generated_code == "":
            rospy.sleep(0.1)
        if self.generated_code != "impossible":
            exec(self.generated_code)
            
        print(f"Task: {task}")
        self.GPSR_done()

    def on_enter_GO2GPSR(self):
        
        if self.task_counter == 3:
            self.tm.talk("GPSR task completed succesfully", "English", wait=True)
            os._exit(os.EX_OK)

        print(self.consoleFormatter.format("GO2GPSR", "HEADER"))
        self.tm.talk("I am going to the GPSR location","English", wait=False)
        self.tm.go_to_place(self.gpsr_location)
        self.go_to_gpsr()

    def on_enter_WAIT4GUEST(self):
        
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        
        self.tm.go_to_pose("default_head")
        self.tm.setRPosture_srv("stand")
        self.tm.setMoveHead_srv.call("up")
        self.tm.talk("Waiting for guests","English")
        self.tm.look_for_object("person")
        self.tm.wait_for_object(-1)
        self.person_arrived()


    def code_gen_t(self, task):
        print(self.consoleFormatter.format("GEN CODE THREAD", "HEADER"))
        code = self.robot_code_generator_aux(task)
        if code != "impossible":
            if self.is_valid_syntax(code):
                self.generated_code = code
                self.task_counter += 1
                print("\nIt is possible to execute the request")
        else:
            self.generated_code = "impossible"
            self.tm.talk("I cannot do the task: " + task, "English")

            
    def robot_code_generator_aux(self, task):
        """
        Generates executable code for the robot based on the natural language task description.
        task: The requirement a person gave to the robot in Natural language
        """
        
        words = task.lower().split()
        filtered_words = [word for word in words if word not in self.irrelevant_words]  # Filter out irrelevant words
        actions = []
        
        print(filtered_words)

        # Queue actions based on keywords using the updated mapping with synonyms
        for word in filtered_words:
            action = self.action_keywords.get(word)
                
        print(actions)

        # Initialize the task execution
        code = "print('Starting task execution')\n"
        code += "self.tm.talk('Processing your request', 'English', wait=False)\n"
        
        if 'kitchen' or 'refrigerator' or 'fridge' or 'dish' or 'washer' or 'fruit' or 'food' or 'groceries' in filtered_words:
            code += "self.tm.talk('I will navigate to the kitchen.', 'English', wait=False)\n"
            code += "self.tm.go_to_place('kitchen', wait=True)\n"
            code += "self.tm.talk('I have arrived at the kitchen.', 'English', wait=False)\n"
            
        elif 'living_room' or 'sofa' or 'couch' or 'tv' in filtered_words:
            code += "self.tm.talk('I will navigate to the living room.', 'English', wait=False)\n"
            code += "self.tm.go_to_place('living_room', wait=True)\n"
            code += "self.tm.talk('I have arrived at the living room.', 'English', wait=False)\n"
        
        elif 'bedroom' or 'lamp' or 'bed' or 'shelf' or 'clothes' or 'tv' in filtered_words:
            code += "self.tm.talk('I will navigate to the bedroom.', 'English', wait=False)\n"
            code += "self.tm.go_to_place('bedroom', wait=True)\n"
            code += "self.tm.talk('I have arrived at the bedroom.', 'English', wait=False)\n"
            
        elif 'dinning room' or 'table' or 'chair' or 'chairs' in filtered_words:
            code += "self.tm.talk('I will navigate to the dinning room.', 'English', wait=False)\n"
            code += "self.tm.go_to_place('dining_room', wait=True)\n"
            code += "self.tm.talk('I have arrived at the dining room.', 'English', wait=False)\n"
        
        # Enhanced action processing with all functionalities
        for i, word in enumerate(filtered_words):
            action = self.action_keywords.get(word)
            
            if not action:
                continue

            next_index = i + 1 if i + 1 < len(filtered_words) else i
            next_word = filtered_words[next_index] if next_index < len(filtered_words) else ''


            print(action)
            if action == "fetch":
                item = next_word
                code += f"if self.tm.look_for_object('{item}'):\n"
                code += f"    self.tm.talk('I have found {item}. Now I will bring it to you.', 'English', wait=False)\n"
                
                best_place = self.determine_place(filtered_words)
                if best_place and "go_to_place" not in code:
                    code += f"self.tm.talk('I will now go to the {best_place}.', 'English', wait=False)\n"
                    go_to_place_request = f"self.tm.go_to_place('{best_place}', wait=True)\n"
                    
                    if go_to_place_request not in code: 
                        code += go_to_place_request
                        code += f"self.tm.talk('I have arrived at the {best_place}.', 'English', wait=False)\n"
                    
                code += f"self.tm.ask_for_object('{item}')\n"
                code += "self.tm.go_back()\n"
                code += f"self.tm.give_object('{item}')\n"
                code += f"self.tm.talk('Here is the object, thank you for using my services!', 'English', wait=False)\n"
                
            elif action == "count":
                item = next_word
                count = 0
                code += f"count = self.tm.count_objects('{item}')\n"
                code += "self.tm.go_back()\n"
                code += f"self.tm.talk('There are {count} {item}(s) present.', 'English', wait=False)\n"

            elif action == "find":
                code += f"if self.tm.look_for_object('person'):\n"
                code += f"    self.tm.talk('I have found you!', 'English', wait=False)\n"
                code += "else:\n"
                code += f"    self.tm.talk('I could not find the person.', 'English', wait=False)\n"
                code += "self.tm.go_back()\n"
            
            elif action == "follow":
                code += "self.tm.follow_you()\n"
                code += "self.tm.talk('Following you now. Please touch my head to stop.', 'English', wait=False)\n"
                code += "if self.tm.wait_for_head_touch():"
                code += "   self.tm.stop_following()\n"
                code += "   self.tm.talk('I have finished following you, I will now come back.', 'English', wait=False)\n"
                code += "self.tm.go_back()\n"

            elif action == "describe":
                code += "attributes = self.tm.get_person_description()\n"
                code += "self.tm.talk(f'The person is described as {attributes['gender']} and approximately {attributes['age']} years old.', 'English', wait=False)\n"

            elif action == "color":
                color = ''
                code += "color = self.tm.get_clothes_color()\n"
                code += "    self.tm.go_back()\n"
                code += f"self.tm.talk('The person is wearing {color} clothes.', 'English', wait=False)\n"

            elif action == "identify":
                gesture = ''
                code += "gesture = self.tm.get_person_gesture()"
                code += "    self.tm.go_back()\n"
                code += f"self.tm.talk('The person is {gesture}.', 'English', wait=False)\n"
                
            elif action == "wait_for_object":
                item = next_word
                code += f"if self.tm.wait_for_object(timeout=10):t\n"
                code += "     self.tm.go_back()\n"
                code += f"    self.tm.talk('I have spotted the {item}.', 'English', wait=False)\n"
                code += "else:\n"
                code += "     self.tm.go_back()\n"
                code += f"    self.tm.talk('I could not find the {item} after waiting.', 'English', wait=False)\n"

            # Handling object search with characteristics
            elif action == "search":
                
                if 'heaviest' in filtered_words:
                    characteristic = 'heaviest'
                
                elif 'lightest' in filtered_words:
                    characteristic = 'lightest'
                
                elif 'biggest' in filtered_words:
                    characteristic = 'biggest'
                    
                elif 'smallest' in filtered_words:
                    characteristic = 'smallest'
                    
                elif 'oldest' in filtered_words:
                    characteristic = 'oldest'
                
                elif 'youngest' in filtered_words:
                    characteristic = 'youngest'
                
                elif 'tallest' in filtered_words:
                    characteristic = 'tallest'
                    
                elif 'shortest' in filtered_words:
                    characteristic = 'shortest'
                
                elif 'heavy' in filtered_words:
                    characteristic = 'heavy'
                    
                elif 'light' in filtered_words:
                    characteristic = 'light'
                
                elif 'small' in filtered_words:
                    characteristic = 'small'
                
                elif 'big' in filtered_words:
                    characteristic = 'big'
                    
                elif 'red' in filtered_words:
                    characteristic = 'red'
                
                elif 'blue' in filtered_words:
                    characteristic = 'blue'
                
                elif 'green' in filtered_words:
                    characteristic = 'green'
                    
                elif 'yellow' in filtered_words:
                    characteristic = 'yellow'
                
                elif 'round' in filtered_words:
                    characteristic = 'round'
                
                elif 'square' in filtered_words:
                    characteristic = 'square'
                    
                elif 'dish' in filtered_words:
                    characteristic = 'dish'
                    
                elif 'fruit' in filtered_words:
                    characteristic = 'fruit'
                    
                elif 'food' in filtered_words:
                    characteristic = 'food'
            
                item_type = filtered_words[next_index + 1] if next_index + 1 < len(filtered_words) else ''
                code += f"item = self.tm.find_item_with_characteristic('{item_type}', '{characteristic}')\n"
                code += "self.tm.go_back()\n"
                code += f"self.tm.talk('I have found an item that matches the description of {characteristic}', 'English', wait=False)\n"

            # Handling navigation with dynamic re-routing
            elif action == "navigate":
                place = next_word
                code += f"self.tm.talk('I will now go to the {place}.', 'English', wait=False)\n"
                code += f"self.tm.go_to_place('{place}', wait=True)\n"
                code += f"self.tm.talk('I have arrived at the {place}', 'English', wait=False)\n"

            # Handling the retrieval and manipulation of an object
            elif action == "pick_object":
                item = next_word
                code += f"if self.tm.ask_for_object('{item}'):\n"
                code += f"    self.tm.talk('I have picked up the {item}.', 'English', wait=False)\n"
                code += "else:\n"
                code += f"    self.tm.talk('Unable to pick up the {item}.', 'English', wait=False)\n"
                code += "self.tm.go_back()\n"

            # Handling the introduction of the robot itself
            elif action == "introduce":
                code += "self.tm.talk('Hello, I am Pepper, your robotic assistant here to help.', 'English', wait=False)\n"

            # Handling commands to stop following a person
            elif action == "stop_following":
                code += "self.tm.stop_following()\n"
                code += "self.tm.talk('I have stopped following you as requested.', 'English', wait=False)\n"

            # Handling commands to dynamically adjust robot posture based on environmental feedback
            elif action == "adjust_posture":
                code += "self.tm.adjust_posture()\n"
                code += "self.tm.talk('Adjusting my posture for optimal performance.', 'English', wait=False)\n"
                
            elif action == "ask":
                request = next_word
                answer = ''
                code += f"answer = self.tm.q_a('Please tell me what is the {request}', 'English', wait=False)\n"
                code += "self.tm.go_back()\n"
                code += f"self.tm.talk('The name is {answer}', 'English', wait=False)\n"
                
            elif action == "go_back":
                code += "self.tm.go_back()\n"


        code += "print('Task execution completed')\n"
        return code

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
    def is_valid_syntax(self, code):
        try:
            ast.parse(code)
            return True
        except SyntaxError:
            return False
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = GPSR()
    sm.run()
    rospy.spin()

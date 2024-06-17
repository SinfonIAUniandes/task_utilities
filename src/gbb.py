#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os

from navigation_msgs.srv import get_absolute_position_srv
from perception_msgs.msg import get_labels_msg
from robot_toolkit_msgs.msg import speech_recognition_status_msg, touch_msg
from robot_toolkit_msgs.srv import move_head_srv, set_security_distance_srv

class GAY_BLIND_BALD(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()

        self.task_name = "GAY_BLIND_BALD"

        states = ['INIT', 'RECOGNIZE', 'GUIDE', 'SEARCH', 'READ', 'FINISH', 'END']

        self.tm = tm(perception = True,speech=True, navigation=True, pytoolkit=True, manipulation=True)

        self.tm.initialize_node(self.task_name)
        
        transitions = [
            {'trigger': 'start', 'source': 'GAY_BLIND_BALD', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'RECOGNIZE'},
            {'trigger': 'guide_t', 'source': 'RECOGNIZE', 'dest': 'GUIDE'},
            {'trigger': 'guide_recog', 'source': 'GUIDE', 'dest': 'RECOGNIZE'},
            {'trigger': 'search_t', 'source': 'RECOGNIZE', 'dest': 'SEARCH'},
            {'trigger': 'search_recog', 'source': 'SEARCH', 'dest': 'RECOGNIZE'},
            {'trigger': 'read_t', 'source': 'RECOGNIZE', 'dest': 'READ'},
            {'trigger': 'read_recog', 'source': 'READ', 'dest': 'RECOGNIZE'},
            {'trigger': 'finish', 'source': 'RECOGNIZE', 'dest': 'FINISH'},
            {'trigger': 'end', 'source': 'FINISH', 'dest': 'END'}]
        
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='GAY_BLIND_BALD')

        # Lista de lugares de la casa sobre la que se hara la tarea.
        self.list_places = ["bathroom","entrance","living_room","kitchen","bedroom","pantry"]
        self.current_place = "bedroom"
        
        #Variables absolutas 
        self.hearing = True
        self.isTouched = False
        self.navigating = False
        self.guide = False
        self.look_for_object = False
        self.read = False
        self.is_done = False
        self.lost = False

        #Subscribers
        self.headSensorSubscriber = rospy.Subscriber("/touch", touch_msg, self.callback_head_sensor_subscriber)
        self.hotWordSubscriber = rospy.Subscriber("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,self.callback_hot_word)
        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_orthogonal_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv")
        self.set_orthogonal_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv",set_security_distance_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_tangential_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_tangential_security_distance_srv")
        self.set_tangential_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_tangential_security_distance_srv",set_security_distance_srv)
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.set_current_place(self.current_place)
        self.tm.initialize_pepper()
        self.set_orthogonal_security_srv(0.3)
        self.set_tangential_security_srv(0.05)
        self.enable_hot_word_service()
        self.tm.talk("I am going to do the "+self.task_name+" task","English", wait=False)
        self.beggining()
        

        

    # ============================== STATE: RECOGNIZE HOT WORDS ==============================
    def on_enter_RECOGNIZE(self):
        """
        Nova should recognize three basic hotwords: 'guide', 'object' and 'read' to do the respective
        tasks to help the person.
        """ 
        print(self.consoleFormatter.format("RECOGNIZE", "HEADER"))
        #self.tm.talk("Hello! Today, I'll be assisting you with navigating through the house, locating objects, and reading any text that you require help with.", wait= False)
        #rospy.sleep(1)
        #self.tm.talk("Here's how we can communicate: Say 'guide' when you want to move to a different room, use the word 'object' if you need me to locate something in the room,tell me 'read' if you have some text that you need me to read out loud for you, and when you no longer need my assistance, just say 'bye' to let me know you're done.","English",  wait= False)

        while not self.is_done: 
            if self.guide: 
                self.guide_t()
                self.guide = False
            elif self.read: 
                self.read_t()
                self.read = False
            elif self.look_for_object: 
                self.search_t()
                self.look_for_object = False
            rospy.sleep(2)
        self.finish()
        

    # =============================== STATE: GUIDE THE PERSON TO WALK  ===============================
    def on_enter_GUIDE(self):
        print(self.consoleFormatter.format("GUIDE", "HEADER"))
        def get_user_place_request():
            """Solicita al usuario que especifique un lugar."""
            self.tm.talk("Now I will tell you the places to go", wait=False)
            for place in self.list_places:
                self.tm.talk(place, wait=False)
            
            return self.tm.speech2text_srv(5).lower()

        def validate_place_request(place_request):
            """Valida que la respuesta del usuario esté en la lista de lugares disponibles."""
            for place in self.list_places:
                if place in place_request:
                    return place

        user_place_request = get_user_place_request()
        next_place = validate_place_request(user_place_request)
        print(next_place)
        while next_place not in self.list_places:
            self.tm.talk("I'm not familiar with that location. Also, please make sure to speak slowly, loudly, and clearly for me to understand you better.", wait=False)
            user_place_request = get_user_place_request()
            next_place = validate_place_request(user_place_request)
        
        if self.current_place == "bedroom":
            self.tm.talk("Please wait until i turn around to grab onto my shoulders", wait=False)
            self.tm.go_to_defined_angle_srv(180)
        elif self.current_place == "pantry":
            self.tm.talk("Please wait until i turn around to grab onto my shoulders", wait=False)
            self.tm.go_to_defined_angle_srv(270)
        self.tm.talk("Please place your hand on my shoulder and keep it there until we reach our destination. I kindly ask that you do not let go", wait=False)
        
            
        self.navigating = True
        self.tm.talk(f"Fantastic! We're on our way to {next_place}. Please keep your hands on my shoulders while we're moving. If at any point you can't find me, just shout 'I Lost You' clearly and loudly.", wait=False)
        self.tm.go_to_place(next_place, wait=False)
        
        # Estanca la ejecucion hasta llegar al destino
        while self.tm.navigation_status != 2:
            # Manejo del caso en que el usuario quite la mano
            if self.lost:
                break
            rospy.sleep(1)
            
        if self.tm.navigation_status == 2:
            self.tm.talk(f"We've reached {next_place}. You may now remove your hand from my shoulder.", wait=False)
        self.navigating = False
        self.current_place = next_place
        self.guide_recog()
            

    # =============================== STATE: SEARCH  ===============================

    def on_enter_SEARCH(self):
        print(self.consoleFormatter.format("SEARCH", "HEADER"))
        self.tm.talk("Please tell me which object you are searching for, so I can help you find it.", wait=False)
        user_search_prompt = self.tm.speech2text_srv(5, "eng")
        object_2_search = self.tm.answer_question(
            f'Whenever I send you a message like "I want to search the [word]", respond with just the [word] and nothing else. Example: User: "I want to search the apple" Response: "apple". Now do it with this: {user_search_prompt}'
        )
        print(object_2_search)
        rospy.sleep(2)
        self.tm.talk(f"Great. I will have a look around the room to find the {object_2_search} for you.", wait=False)
        object_found = False
        while not object_found or :
            response = self.tm.img_description(
                f'Please find the specified object: {object_2_search} in the provided image. If the object is present, respond with "true". If the object is not present, respond with "false".'
            )["message"]
            object_found = response.lower() == "true"
            () == "true"
            if not object_found:
                self.tm.go_to_relative_point(0.0, 0.0, 90)
        object_location_description = self.tm.img_description(
            f"Please provide a specific and short description of where the {object_2_search} is placed and how to reach and grab it"
        )["message"]
        self.tm.talk(object_location_description, wait=False)
        self.search_recog()
    

    # =============================== STATE: READ TEXT ===============================

    def on_enter_READ(self):
        print(self.consoleFormatter.format("READ_TEXT", "HEADER"))
        # Initial check to see if the person is in sight with the paper
        self.tm.talk("If you have a document that you need me to read, approach me with it and keep coming closer until I instruct you to stop.", language="English")

        # Loop until a person with a paper is detected
        while True:
            detection_prompt = "Is there someone holding a paper within my view? Please respond with a yes or a no."
            detection_status = self.tm.img_description(detection_prompt, camera_name="front_camera")
            
            if detection_status["status"] and "yes" in detection_status["message"].lower():
                self.tm.talk("I see you now. Please stay still as I guide you to the perfect spot.", language="English")
                break
            else:
                self.tm.talk("Keep moving closer, please.", language="English")
                rospy.sleep(1)  # Giving time for the person to move

        # Guide the person to center and come close enough for clear vision
        while True:
            position_prompt = "Is the person centered and close enough for reading the full paper clearly? Answer with yes or no and give instructions to the person to move to left, right, closer or further to the robot ."
            position_status = self.tm.img_description(position_prompt, camera_name="front_camera")
            
            if position_status["status"] and "yes" in position_status["message"].lower():
                self.tm.talk("You are perfectly positioned. Hold the paper steady, I am going to read it now.", language="English")
                break
            else:
                self.tm.talk(position_status["message"], language="English")
                rospy.sleep(1)  # Continuous feedback loop

        # Attempt to read the text on the paper
        gpt_vision_prompt = "Read the text from the paper. Return the text only, dont make additional comments."
        response = self.tm.img_description(gpt_vision_prompt, camera_name="front_camera")
        
        # Verify if the text makes sense
        if response["status"] and response["message"]:
            if any(word.isalpha() for word in response["message"]):  # Simple check for alphabetic characters to ensure text makes sense
                self.tm.talk(response["message"], language="English")
            
            elif response["message"] == "":
                self.tm.talk("That side of the paper has no words to read, please try to show me the other side of the paper", language="English")

            else:
                self.tm.talk("I could not understand the text. Please rotate the paper.", language="English")
        else:
            self.tm.talk("I still can't read the paper. Please rotate it to a different position.", language="English")
            return False  # Stay in this state to try reading again
        
        # Assuming there's another function or state to handle post-reading actions
        self.read_recog()

    # =============================== STATE: FINISHING THE TASK ===============================


    def on_enter_FINISH(self):
        print(self.consoleFormatter.format("FINISH", "HEADER"))
        self.tm.talk("It has been my pleasure to assist you today, mr gay blind bald man. Take care!")
        self.end()

    # =============================== STATE: END OF THE TASK ===============================

    def on_enter_END(self):
        print(self.consoleFormatter.format("END", "HEADER"))
        os._exit(os.EX_OK)

    
    # =============================== Additional functions ===============================

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        self.start()

    def callback_hot_word(self,data):
        word = data.status
        print(word, "listened")
        if word == "goodbye": 
            self.is_done = True
        elif word == "lost": 
            self.lost = True
            self.tm.robot_stop_srv()
            self.tm.talk("Your hand seems to have slipped from my shoulder. Please place your hands back on my shoulders, and when you're ready to proceed, simply say 'guide' again.", wait=False)
        if not self.navigating:
            if word == "guide":
                self.guide = True   
            elif word == "object": 
                self.look_for_object = True
            elif word == "read": 
                self.read = True
    
    def enable_hot_word_service(self):
        """
        Enables hotwords detection from the toolkit of the robot.
        """
        print("Waiting for hot word service")
        try:
            if self.hearing:
                self.tm.hot_word(["guide", "read", "object", "lost","goodbye"], thresholds=[0.44,0.43,0.4,0.5,0.5])
                print("Hot word service connected!")
            
        except rospy.ServiceException as e:
            print("Service call failed")
        
    def callback_head_sensor_subscriber(self, msg: touch_msg):
        if "head" in msg.name:
            self.isTouched = msg.state
        

# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = GAY_BLIND_BALD()
    sm.run()
    rospy.spin()

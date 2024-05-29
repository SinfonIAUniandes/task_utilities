#!/usr/bin/env python3

"""
    This module contains the events for the Zeiss May 2024 task.
    First version made by: Alonso Hernandez @fai-aher
"""

# General Imports
import os
import sys
import rospy
import threading
import time
import random
import pandas as pd
import csv

# State Machine Library
from transitions import Machine

# Console Formatter
import ConsoleFormatter

# Task Module
from task_module import Task_module as tm

# Standard Messages
from std_msgs.msg import Bool

# Standard Services
from std_srvs.srv import SetBool

# Robot Toolkit Services
from robot_toolkit_msgs.srv import tablet_service_srv,  set_open_close_hand_srv, tablet_service_srvRequest, set_open_close_hand_srvRequest, motion_tools_srv, battery_service_srv, set_output_volume_srv

from robot_toolkit_msgs.msg import animation_msg, motion_tools_msg, leds_parameters_msg, speech_recognition_status_msg


# Class definition and implementation
class ZeissCustomersReception(object):
    
    #Initialization state
    def __init__(self) -> None:
        
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        
        #Name of the task
        self.task_name = "ZeissCustomersReception"
        self.hearing = False
        self.is_done = False
        self.hey_pepper=False
        self.already_asereje = False
        self.already_dance = False 
        self.haciendo_animacion = False
        
        # Task module initialization
        self.tm = tm(perception = True,speech=True,manipulation=False, navigation=False, pytoolkit=True)
        
        # Task module node initialization
        time.sleep(1)
        self.tm.initialize_node(self.task_name)
        
        # States definition
        self.states = [
            'INIT',
            
            # State with all the logic related to receiving the customers
            'RECEIVING_CUSTOMERS',
            
            # State with all the logic related to saying random facts about ZEISS
            'SAYING_FACTS',
            
            # Ending state
            'JOB_DONE',
            
            'MENU'
        ]
        
        # Reading the ZEISS facts csv
        self.facts_df = pd.read_csv('/home/sinfonia/sinfonia_ws/src/task_utilities/src/events/2024/ZEISS_event/facts.csv')
        
        # Setting a custom recursion limit
        sys.setrecursionlimit(3000)
        
        """
        1. STATES MACHINE
        """
        
        # States Machine transitions definition
        
        self.transitions = [
            
            # Transition 0: Going to and executing the first state
            {'trigger': 'start_task', 'source': 'ZeissCustomersReception', 'dest': 'INIT'},
         
            # Transition 1: Start the task: Nova starts receiving customers
            {'trigger': 'begin_reception', 'source': 'MENU', 'dest': 'RECEIVING_CUSTOMERS'},
            
            # Transition 2: Start saying random facts
            {'trigger': 'start_saying_facts', 'source': 'RECEIVING_CUSTOMERS', 'dest': 'SAYING_FACTS'},
            
            {'trigger': 'start_saying_facts', 'source': 'MENU', 'dest': 'SAYING_FACTS'},
            
            # Transition 3: Stop saying random facts
            {'trigger': 'stop_saying_facts', 'source': 'SAYING_FACTS', 'dest': 'MENU'},
            
            # Transition 4: Continue receiving customers
            {'trigger': 'continue_receiving_customers', 'source': 'RECEIVING_CUSTOMERS', 'dest': 'RECEIVING_CUSTOMERS'},
    
            # Transition 5: Finish the task
            {'trigger': 'finish_task_from_waiting', 'source': 'RECEIVING_CUSTOMERS', 'dest': 'JOB_DONE'}, # Finishes the task 
            
            {'trigger': 'return_menu', 'source': 'RECEIVING_CUSTOMERS','dest': 'MENU'},
            
            {'trigger': 'return_menu', 'source': 'INIT','dest': 'MENU'}
        ] 
        
        # States Machine initialization
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial='ZeissCustomersReception')
        
        # Thread for checking rospy
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        

        print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/show_image_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.show_image_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_image_srv",tablet_service_srv)
        
        print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/show_web_view_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_web_view_srv")
        self.show_web_page_proxy = rospy.ServiceProxy(
        "/pytoolkit/ALTabletService/show_web_view_srv", tablet_service_srv)
        
        
        """
        2. VARIABLES
        """
        
        # 1. ROS Callbacks variables
        self.labels = {}
        
        
    """
    3. CUSTOM FUNCTIONS
    """
    def get_random_fact(self):
        random_index = random.randint(0, len(self.facts_df) - 1)
        random_fact = self.facts_df.iloc[[random_index]]
        return random_fact['fact_content'].values[0], random_fact['fact_id'].values[0]
    
    def callback_get_labels(self,data):
        labels = data.labels
        x_coordinates = data.x_coordinates
        y_coordinates = data.y_coordinates
        widths = data.widths
        heights = data.heights
        ids = data.ids
        for i in range(len(labels)):
            if int(self.img_dimensions[0]*0.2)<x_coordinates[i]<int(self.img_dimensions[0]*0.8) and widths[i]>130:
                self.labels[labels[i]] = {"x":x_coordinates[i],"y":y_coordinates[i],"w":widths[i],"h":heights[i],"id":ids[i]}
    
    
    def enable_breathing_service(self):
        """
        Enables the breathing animations of the robot.
        """
        request = set_open_close_hand_srvRequest()
        request.hand = "All"
        request.state = "True"
        print("Waiting for breathing service")
        rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv")
        try:
            toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
            toggle_breathing_proxy(request)
            print("Breathing service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
        request = set_open_close_hand_srvRequest()
        request.hand = "Head"
        request.state = "False"
        print("Waiting for breathing service")
        rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv")
        try:
            toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
            toggle_breathing_proxy(request)
            print("Breathing service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
    def motion_tools_service(self):
        """
        Enables the motion Tools service from the toolkit of the robot.
        """
        request = motion_tools_msg()
        request.command = "enable_all"
        print("Waiting for motion tools service")
        rospy.wait_for_service('/robot_toolkit/motion_tools_srv')
        try:
            motion = rospy.ServiceProxy('/robot_toolkit/motion_tools_srv', motion_tools_srv)
            motion(request)
            print("Motion tools service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
            
    def enable_tracker_service(self):
        """
        Enables face tracking from the toolkit of the robot.
        """
        print("Waiting for tracking service")
        rospy.wait_for_service('/pytoolkit/ALTracker/start_tracker_srv')
        try:
            start_tracker_srv = rospy.ServiceProxy("/pytoolkit/ALTracker/start_tracker_srv", battery_service_srv)
            start_tracker_srv()
            print("tracking service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
            
            
    def disable_tracker_service(self):
        """
        Disables face tracking from the toolkit of the robot.
        """
        print("Waiting for disable tracking service")
        rospy.wait_for_service('/pytoolkit/ALTracker/stop_tracker_srv')
        try:
            stop_tracker_srv = rospy.ServiceProxy("/pytoolkit/ALTracker/stop_tracker_srv", battery_service_srv)
            stop_tracker_srv()
            print("disable-tracking service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
    def setLedsColor(self, r,g,b):
        """
        Function for setting the colors of the eyes of the robot.
        Args:
        r,g,b numbers
            r for red
            g for green
            b for blue
        """
        ledsPublisher = rospy.Publisher('/leds', leds_parameters_msg, queue_size=10)
        ledsMessage = leds_parameters_msg()
        ledsMessage.name = "FaceLeds"
        ledsMessage.red = r
        ledsMessage.green = g
        ledsMessage.blue = b
        ledsMessage.time = 0
        ledsPublisher.publish(ledsMessage)  #Inicio(aguamarina), Pepper esta ALSpeechRecognitionStatusPublisherlista para escuchar
        
    def gen_anim_msg(self, animation):
        anim_msg = animation_msg()
        anim_msg.family = "animations"
        anim_msg.animation_name = animation
        return anim_msg
    
    def set_hot_words(self):
        if self.hearing:
            self.tm.hot_word(["chao","detente","hey nova","baile","asereje","pose","musculos" ,"besos","foto","guitarra","cumpleaños","corazon","llama","helicoptero","zombi","carro","gracias"],thresholds=[0.45, 0.4, 0.398, 0.43, 0.43, 0.39, 0.4, 0.4, 0.5, 0.4, 0.4, 0.4, 0.41, 0.4, 0.4, 0.43, 0.4])

    
    def callback_hot_word(self,data):
        word = data.status
        print(word, "listened")
        if word=="detente":
            self.tm.setRPosture_srv("stand")
            self.haciendo_animacion = False
        if not self.haciendo_animacion:
            self.haciendo_animacion = True
            
            if word == "QR":
                self.begin_reception()
                
            elif word == "chao":
                self.is_done = True
                self.already_dance = False
                self.already_asereje = False
            elif word == "hey nova":
                self.hey_pepper = True
            elif word == "guitarra":
                anim_msg = self.gen_anim_msg("Waiting/AirGuitar_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "besos":
                anim_msg = self.gen_anim_msg("Gestures/Kisses_1")
                self.animationPublisher.publish(anim_msg)
                self.tm.talk("Muah!","Spanish")
            elif word == "baile":
                if not self.already_dance:
                    dance_thread = threading.Thread(target=self.dance_threadf,args=[1])
                    dance_thread.start()
                    self.already_dance = True
            elif word == "asereje":
                if not self.already_asereje:
                    dance_thread = threading.Thread(target=self.dance_threadf,args=[3])
                    dance_thread.start()
                    self.already_asereje = True
            elif word == "pose":
                anim_msg = self.gen_anim_msg("Gestures/ShowSky_8")
                self.animationPublisher.publish(anim_msg)
            elif word == "foto":
                anim_msg = self.gen_anim_msg("Waiting/TakePicture_1")
                self.animationPublisher.publish(anim_msg)
                rospy.sleep(2)
                self.show_picture_proxy()
            elif word == "cumpleaños":
                anim_msg = self.gen_anim_msg("Waiting/HappyBirthday_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "corazon":
                anim_msg = self.gen_anim_msg("Waiting/LoveYou_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "llama":
                anim_msg = self.gen_anim_msg("Waiting/CallSomeone_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "helicoptero":
                anim_msg = self.gen_anim_msg("Waiting/Helicopter_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "zombi":
                anim_msg = self.gen_anim_msg("Waiting/Zombie_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "carro":
                anim_msg = self.gen_anim_msg("Waiting/DriveCar_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "musculos":
                anim_msg = self.gen_anim_msg("Waiting/ShowMuscles_3")
                self.animationPublisher.publish(anim_msg)
            elif word == "gracias":
                anim_msg = self.gen_anim_msg("Gestures/BowShort_3")
                self.animationPublisher.publish(anim_msg)
                self.tm.talk("Con mucho gusto","Spanish")
            self.haciendo_animacion = False
            
            
            

    """
    4. TASK STATES LOGICAL IMPLEMENTATION
    """
    
    def on_enter_MENU(self):
        
        self.enable_tracker_service()
        self.tm.publish_filtered_image(filter_name="qr",camera_name="front_camera")
        request = tablet_service_srvRequest()
        request.url = "http://192.168.0.229:8000/" 
        self.show_web_page_proxy(request)
        #Set led color to white
        self.setLedsColor(224,224,224)
        rospy.sleep(30)
        self.start_saying_facts()
        

    
    #1. on enter INIT
    def on_enter_INIT(self):    
        
        self.enable_tracker_service()
    
        # Initialization message
        print(self.consoleFormatter.format("Initializing the task - ENTERING 'INIT' STATE", "HEADER"))
        self.animationPublisher = rospy.Publisher('/animations', animation_msg, queue_size=10)
        rospy.Subscriber("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,self.callback_hot_word)
        
        # Saying what Nova is going to do
        print(self.consoleFormatter.format("Nova: I am ready to do my task!", "HEADER"))
        
        # Greeting while the initialization completes
        self.tm.talk("""
                Por favor dame un momento mientras termino de iniciar mis herramientas.
                     
                ""","Spanish", animated=True, wait=False)
        
        # Starting the functions and publishers
        self.tm.initialize_pepper()
        self.motion_tools_service()
        self.enable_breathing_service()
        self.setLedsColor(255,255,255)
        self.tm.publish_filtered_image(filter_name="qr",camera_name="front_camera")
        request = tablet_service_srvRequest()
        request.url = "http://192.168.0.229:8000/" 
        self.show_web_page_proxy(request)
        #Set led color to white
        self.setLedsColor(224,224,224)

        self.tm.talk("""
                     
        Ya estoy lista para la tarea!
                     
                     """, "Spanish", wait=True, animated=True)
            
        print(self.consoleFormatter.format("Nova: Front camera enabled!", "HEADER"))
        
        # Greeting while the initialization completes
        self.tm.talk("""Hola! mi nombre es Nova, soy la robot de recepción e interacción en este evento.
 
                ""","Spanish", wait=False, animated=True)
        
        rospy.sleep(2)
        
        # Look for a person to begin greeting
        print(self.consoleFormatter.format("Nova: I will look for a person to greet him and start the reception of customers!", "HEADER"))
        self.tm.look_for_object("person")
        
        self.return_menu()
        
    
    # 2. on enter RECEIVING_CUSTOMERS
    def on_enter_RECEIVING_CUSTOMERS(self):
        
        # Head tracking service activated
        self.tm.show_topic("/perception_utilities/filtered_image")
        self.tm.talk("""Voy a registrar tu ingreso al evento.
                            """, "Spanish", wait=False, animated=True)
        
        # Initialization message
        print(self.consoleFormatter.format("Initializing the task - ENTERING 'RECEIVING_CUSTOMERS' STATE", "HEADER"))
        
        print(self.consoleFormatter.format("Nova: I'm waiting for someone to show me the QR", "HEADER"))
        
        # If Nova sees someone in front
        
        # Reading the QR code
        print(self.consoleFormatter.format("Nova: Someone is in front of me, let's see if the person has a QR code", "HEADER"))
        rospy.sleep(1.5)
        self.disable_tracker_service()
        
        print(self.consoleFormatter.format("Nova: I will stop my head tracker service to let the person show me the QR code", "HEADER"))
        self.setLedsColor(102,102,255)
        self.qr_code = self.tm.qr_read(12)
        
        self.enable_tracker_service()
        #Set led color to white
        self.setLedsColor(224,224,224)
        
        # If the person does not show a QR code, Nova tells the person to show it or to register
        if self.qr_code == "":
            print(self.consoleFormatter.format("Nova: I could not read any QR code, I will tell the person to register", "HEADER"))
            self.tm.talk("""No he podido leer el código, por favor inténtalo de nuevo.
                            """, "Spanish", wait=False, animated=True)
        else:
            # Play greeting animation
            anim_msg = self.gen_anim_msg("Gestures/BowShort_3")
            self.animationPublisher.publish(anim_msg)

            # Check if the scanned QR code matches any VIP guest's name
            with open('/home/sinfonia/sinfonia_ws/src/task_utilities/src/events/2024/ZEISS_event/guests_list.csv', 'r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    if row['guest_name'] == self.qr_code:
                        # Greet the special customer with custom message
                        self.tm.talk(row['custom_message'], "Spanish", wait=False)
                        break
                else:
                    # Greeting the person with a default message if not a VIP guest
                    self.tm.talk("Bienvenido al evento " + self.qr_code + "Espero que lo disfrutes mucho. Fue un placer atenderte!", "Spanish", wait=False)
            
            # Save the attendance to the CSV file
            with open('/home/sinfonia/sinfonia_ws/src/task_utilities/src/events/2024/ZEISS_event/attended_guests.csv', 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.qr_code, 'True'])
            
            
        # Transition: Moving to the same state to keep looking for QR codes
        self.return_menu()
        
        
    # 3. on enter SAYING_FACTS
    def on_enter_SAYING_FACTS(self):
        
        # Initialization message
        print(self.consoleFormatter.format("Initializing the task - ENTERING 'RECEIVING_CUSTOMERS' STATE", "HEADER"))
        self.setLedsColor(224,224,224)
        
        # Choosing the random fact
        rospy.sleep(15)
        fact_content, fact_id = self.get_random_fact()
        
        print(self.consoleFormatter.format(f"Nova: I chose to say the fact with ID = {fact_id}", "HEADER"))
        
        # Saying the random fact
        self.tm.talk(fact_content, "Spanish", wait=False, animated=True)        
        
        # Going back to scan QR codes
        self.stop_saying_facts()
        
        
    # 4. on enter JOB_DONE
    def on_enter_JOB_DONE(self):
        # Initialization message
        print(self.consoleFormatter.format("Finishing the task - ENTERING 'JOB_DONE' STATE", "HEADER"))
        print(self.consoleFormatter.format("Nova: I am done for today!", "HEADER"))
        self.check_rospy()
        
        
    """
    5. ROSPY FUNCTIONS
    """
    def check_rospy(self):
        # Ending all processes if rospy is not running
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start_task()


# Main function
if __name__ == "__main__":
    app_instance= ZeissCustomersReception()
    app_instance.run()
    rospy.spin()

        

        
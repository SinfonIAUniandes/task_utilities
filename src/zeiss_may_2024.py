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
from robot_toolkit_msgs.srv import tablet_service_srv,  set_open_close_hand_srv, set_open_close_hand_srvRequest, motion_tools_srv, battery_service_srv, set_output_volume_srv

from robot_toolkit_msgs.msg import animation_msg, motion_tools_msg, leds_parameters_msg


# Class definition and implementation
class ZeissCustomersReception(object):
    
    #Initialization state
    def __init__(self) -> None:
        
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        
        #Name of the task
        self.task_name = "ZeissCustomersReception"
        
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
            'JOB_DONE'
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
            {'trigger': 'begin_reception', 'source': 'INIT', 'dest': 'RECEIVING_CUSTOMERS'},
            
            # Transition 2: Start saying random facts
            {'trigger': 'start_saying_facts', 'source': 'RECEIVING_CUSTOMERS', 'dest': 'SAYING_FACTS'},
            
            # Transition 3: Stop saying random facts
            {'trigger': 'stop_saying_facts', 'source': 'SAYING_FACTS', 'dest': 'RECEIVING_CUSTOMERS'},
            
            # Transition 4: Continue receiving customers
            {'trigger': 'continue_receiving_customers', 'source': 'RECEIVING_CUSTOMERS', 'dest': 'RECEIVING_CUSTOMERS'},
    
            # Transition 5: Finish the task
            {'trigger': 'finish_task_from_waiting', 'source': 'RECEIVING_CUSTOMERS', 'dest': 'JOB_DONE'}, # Finishes the task 
        ] 
        
        # States Machine initialization
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial='ZeissCustomersReception')
        
        # Thread for checking rospy
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        

        print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/show_image_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.show_image_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_image_srv",tablet_service_srv)
        
        
        """
        2. VARIABLES
        """
        
        # 1. ROS Callbacks variables
        self.labels = {}
        
        # 2. Global variables
        self.registration_qr_img= "https://raw.githubusercontent.com/fai-aher/Airline-Coberture-WebApp/main/qr_zeiss.png"
        
        
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
    
    
    """
    4. TASK STATES LOGICAL IMPLEMENTATION
    """
    
    #1. on enter INIT
    def on_enter_INIT(self):    
    
        # Initialization message
        print(self.consoleFormatter.format("Initializing the task - ENTERING 'INIT' STATE", "HEADER"))
        self.animationPublisher = rospy.Publisher('/animations', animation_msg, queue_size=10)
        
        # Saying what Nova is going to do
        print(self.consoleFormatter.format("Nova: I am ready to do my task!", "HEADER"))
        
        # Greeting while the initialization completes
        self.tm.talk("""
                Por favor dame un momento mientras termino de iniciar mis herramientas para esta tarea.
                     
                ""","Spanish", animated=True, wait=False)
        
        # Starting the functions and publishers
        self.tm.initialize_pepper()
        self.motion_tools_service()
        self.enable_breathing_service()
        self.setLedsColor(255,255,255)
        self.tm.publish_filtered_image(filter_name="qr",camera_name="front_camera")
        #Set led color to white
        self.setLedsColor(224,224,224)

        self.tm.talk("""
                     
        Ya estoy lista para la tarea!
                     
                     """, "Spanish", wait=True, animated=True)
            
        print(self.consoleFormatter.format("Nova: Front camera enabled!", "HEADER"))
        
        # Showing the registration QR code in the tablet
        self.tm.show_image("https://raw.githubusercontent.com/fai-aher/Airline-Coberture-WebApp/main/qr_zeiss.png")
    
        self.tm.show_image("https://raw.githubusercontent.com/fai-aher/Airline-Coberture-WebApp/main/qr_zeiss.png")
        
        
        # Greeting while the initialization completes
        self.tm.talk("""Hola! mi nombre es Nova, soy la robot de recepción en este evento.
                
                Seré la encargada de leer los códigos QR para el ingreso al evento.
                Si no te has inscrito, escanea el QR de mi tablet.
                ""","Spanish", wait=False, animated=True)
        
        rospy.sleep(2)
        
        # Look for a person to begin greeting
        print(self.consoleFormatter.format("Nova: I will look for a person to greet him and start the reception of customers!", "HEADER"))
        self.tm.look_for_object("person")
        
        # Transition: Moving to the next state
        self.begin_reception()
        
    
    # 2. on enter RECEIVING_CUSTOMERS
    def on_enter_RECEIVING_CUSTOMERS(self):
        
        # Head tracking service activated
        self.enable_tracker_service()
        
        # Initialization message
        print(self.consoleFormatter.format("Initializing the task - ENTERING 'RECEIVING_CUSTOMERS' STATE", "HEADER"))
        
        print(self.consoleFormatter.format("Nova: I'm waiting for someone to show me the QR", "HEADER"))
        
        # If no person shows a QR code, Nova goes to the state of saying a random fact
        person_detected = self.tm.wait_for_object(8)
        
        if person_detected == False:
            print(self.consoleFormatter.format("Nova: I have seen no person in 10 seconds, I will start saying facts about ZEISS!", "HEADER"))
            self.start_saying_facts()
        
        # If Nova sees someone in front
        else:
            # Reading the QR code
            print(self.consoleFormatter.format("Nova: Someone is in front of me, let's see if the person has a QR code", "HEADER"))
            rospy.sleep(1.5)
            self.disable_tracker_service()
            
            print(self.consoleFormatter.format("Nova: I will stop my head tracker service to let the person show me the QR code", "HEADER"))
            self.setLedsColor(102,102,255)
            self.tm.show_topic("/perception_utilities/filtered_image")
            self.tm.talk("""
            Hola! te doy la bienvenida al evento de tsais. Por favor muestra el código QR de ingreso frente a mis ojos.
            ""","Spanish", wait=False, animated=True)
            qr_code = self.tm.qr_read(12)
            self.tm.show_image("https://raw.githubusercontent.com/fai-aher/Airline-Coberture-WebApp/main/qr_zeiss.png")
            #Set led color to white
            self.setLedsColor(224,224,224)
            
            # If the person does not show a QR code, Nova tells the person to show it or to register
            if qr_code == "":
                print(self.consoleFormatter.format("Nova: I could not read any QR code, I will tell the person to register", "HEADER"))
                self.tm.talk("""Si ya te registraste para el evento y tienes un código QR, por favor muéstralo frente a mis ojos.
                                Si aún no te has registrado, puedes hacerlo escaneando el código QR en mi tablet.
                             """, "Spanish", wait=False, animated=True)
            else:
                # Play greeting animation
                anim_msg = self.gen_anim_msg("Gestures/BowShort_3")
                self.animationPublisher.publish(anim_msg)

                # Check if the scanned QR code matches any VIP guest's name
                with open('vip_guests.csv', 'r') as file:
                    reader = csv.DictReader(file)
                    for row in reader:
                        if row['guest_name'] == qr_code:
                            # Greet the special customer with custom message
                            self.tm.talk(row['custom_message'], "Spanish", wait=False)
                            break
                    else:
                        # Greeting the person with a default message if not a VIP guest
                        self.tm.talk("Bienvenido al evento. Espero que lo disfrutes mucho. Fue un placer atenderte!", "Spanish", wait=False)
                
                # Save the attendance to the CSV file
                with open('attended_guests.csv', 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([qr_code, 'True'])
            
            
        # Transition: Moving to the same state to keep looking for QR codes
        self.continue_receiving_customers()
        
        
    # 3. on enter SAYING_FACTS
    def on_enter_SAYING_FACTS(self):
        
        # Initialization message
        print(self.consoleFormatter.format("Initializing the task - ENTERING 'RECEIVING_CUSTOMERS' STATE", "HEADER"))
        self.setLedsColor(224,224,224)
        
        # Choosing the random fact
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

        

        
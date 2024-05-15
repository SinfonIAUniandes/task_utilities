#!/usr/bin/env python3

"""
    This module contains the events for the Zeiss May 2024 task.
    First version made by: Alonso Hernandez @fai-aher
"""

# General Imports
import os
import rospy
import threading
import time
import random
import pandas as pd

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

from robot_toolkit_msgs.msg import animation_msg, motion_tools_msg

# Robot Perception Messages
from perception_msgs.msg import get_labels_msg


# Class definition and implementation
class ZeissCustomersReception(object):
    
    #Initialization state
    def __init__(self) -> None:
        
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        
        #Name of the task
        self.task_name = "ZeissCustomersReception"
        
        # Task module initialization
        self.tm = tm(perception = True,speech=True,manipulation=False, navigation=True, pytoolkit=True)
        
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
        
        # ROS callbacks initialization
        
        
        # 1. Awareness service
        #print(self.consoleFormatter.format("Waiting for pytoolkit/awareness...", "WARNING"))
        #rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        #self.awareness_srv = rospy.ServiceProxy("/pytoolkit/ALBasicAwareness/set_awareness_srv",SetBool)
        
        # 2. Move head service
        #print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/move_head...", "WARNING"))
        #rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        #self.move_head_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/move_head_srv",move_head_srv)
        
        # 3. Tablet service: Show image
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.show_image_proxy = rospy.ServiceProxy(
            "/pytoolkit/ALTabletService/show_image_srv", tablet_service_srv
        )
        
        # 4. Tablet service: Show topic
        #print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
        #rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
        #self.show_topic_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv",tablet_service_srv)
        
        # 5. Autonomous life service
        #print(self.consoleFormatter.format("Waiting for pytoolkit/autononumusLife...", "WARNING"))
        #rospy.wait_for_service("/pytoolkit/ALAutonomousLife/set_state_srv")
        #self.autonomous_life_srv = rospy.ServiceProxy("/pytoolkit/ALAutonomousLife/set_state_srv",SetBool)
        
        # 6. ROS subscribers (perception)
        #print(self.consoleFormatter.format("Waiting for /perception_utilities/get_labels_publisher", "WARNING"))
        #self.get_labels_publisher = rospy.Subscriber("/perception_utilities/get_labels_publisher", get_labels_msg, self.callback_get_labels)

        # 7. ROS Publishers
        #print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        #self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)
        
        
        """
        2. VARIABLES
        """
        
        # 1. ROS Callbacks variables
        self.labels = {}
        
        # 2. Global variables
        self.registration_qr_img= "https://media.discordapp.net/attachments/876543237270163498/1123649957791010939/logo_sinfonia_2.png"
        
        
        
    """
    3. CUSTOM FUNCTIONS
    """
    def get_random_fact(self):
        random_fact = self.facts_df.sample()
        return (random_fact['fact_content'].values[0], random_fact['fact_id'].values[0])
    
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
    
    
    """
    4. TASK STATES LOGICAL IMPLEMENTATION
    """
    
    #1. on enter INIT
    def on_enter_INIT(self):
    
        # Initialization message
        print(self.consoleFormatter.format("Initializing the task - ENTERING 'INIT' STATE", "HEADER"))
        
        # Saying what Nova is going to do
        self.tm.talk("I will begin my task as the ZEISS company event receptionist", "English")
        print(self.consoleFormatter.format("Nova: I am ready to do my task!", "HEADER"))
        
        # Greeting while the initialization completes
        self.tm.talk("""
                Por favor dame un momento mientras termino de iniciar mis herramientas necesarias para esta tarea.
                
                .bip bip bip.
                
                En este momento se están inicializando mis funcionalidades para ver e identificar personas.
                
                También se están inicializando las herramientas que me permiten moverme de manera más natural.
                
                Por último, estoy inicializando un servicio para poder simular que respiro y mejorar la naturalidad de mis interacciones.
                
                Gracias por esperar.     
                     
                ""","Spanish", wait=False)
        
        # Starting the front camera to read QR codes
        self.tm.initialize_pepper()
        self.motion_tools_service()
        self.enable_breathing_service()
        
        
        self.tm.talk("""
                     
                     
                     
                     """, "Spanish", wait=True)
            
        print(self.consoleFormatter.format("Nova: Front camera enabled!", "HEADER"))
        
        # Showing the registration QR code in the tablet
        self.show_image_proxy(self.registration_qr_img)
        print(self.consoleFormatter.format("Nova: I am now showing the registration QR in my tablet!", "HEADER"))
        
        # Greeting while the initialization completes
        self.tm.talk("""Hola! mi nombre es Nova, soy la robot de recepcion en este evento.
                
                Espero estén teniendo un bonito día.
                Seré la encargada de leer los códigos QR de las personas que ya se registraron con anterioridad.
                Aquellos que aún no se han registrado, pueden hacerlo al escanear el código QR sobre mi tablet.

                Por favor, acérquese a mi para que pueda leer su código QR con mis ojos de robot.
                ""","Spanish", wait=False)
        
        # Look for a person to begin greeting
        print(self.consoleFormatter.format("Nova: I will look for a person to greet him and start the reception of customers!", "HEADER"))
        self.tm.look_for_object("person")
        
        # Transition: Moving to the next state
        self.begin_reception()
        
    
    # 2. on enter RECEIVING_CUSTOMERS
    def on_enter_RECEIVING_CUSTOMERS(self):
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
            rospy.sleep(1)
            qr_code = self.tm.qr_read(10)
            
            # If the person does not show a QR code, Nova tells the person to show it or to register
            if qr_code == "":
                print(self.consoleFormatter.format("Nova: I could not read any QR code, I will tell the person to register", "HEADER"))
                self.tm.talk("""Si ya te registraste para el evento y tienes un código QR, por favor muéstralo frente a mis ojos.
                                Si aún no te has registrado, puedes hacerlo escaneando el código QR que se muestra en mi tablet.
                             """, "Spanish")
            else:
                # Greeting the person
                print(self.consoleFormatter.format("Nova: I recognized a QR code, I will greet the person", "HEADER"))
                self.tm.talk("Bienvenido al evento "+ qr_code + "Espero que lo disfrutes mucho. Fue un placer atenderte!", "Spanish")
                # TODO: Logic to save the person's assistance in the database
                # Idea: save the person's name and id in a file and report it to the company
            
            
        # Transition: Moving to the same state to keep looking for QR codes
        self.continue_receiving_customers()
        
        
    # 3. on enter SAYING_FACTS
    def on_enter_SAYING_FACTS(self):
        
        # Initialization message
        print(self.consoleFormatter.format("Initializing the task - ENTERING 'RECEIVING_CUSTOMERS' STATE", "HEADER"))
        
        # Choosing the random fact
        random_fact = self.get_random_fact()[0]
        random_fact_id = self.get_random_fact()[1]
        print(self.consoleFormatter.format(f"Nova: I chose to say the fact with ID = {random_fact_id}", "HEADER"))
        
        # Saying the random fact
        self.tm.talk(random_fact, "Spanish")
        
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

        

        
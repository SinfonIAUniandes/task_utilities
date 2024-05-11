"""
    This module contains the events for the Zeiss May 2024 task.
    First version made by: Alonso Hernandez @fai-aher
"""

# General Imports
import rospy
import os
import sys
import threading
import time
import random

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
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srv

from robot_toolkit_msgs.msg import animation_msg

# Robot Perception Messages
from perception_msgs.msg import get_labels_msg


# Class definition and implementation
class ZeissCustomersReception(object):
    
    #Initialization state
    def __init__(self):
        
        self.console_formatter = ConsoleFormatter.ConsoleFormatter()
        
        #Name of the task
        self.task_name = "Zeiss Customers Reception"
        
        # States definition
        self.states = [
            'INIT',
            
            # States when no customer is on sight
            'WAITING_FOR_CUSTOMERS', # Passively waiting
            'SEARCHING_CUSTOMERS', # Actively searching
            
            'SAYING_RANDOM_FACT',
            'SAYING_RANDOM_THOUGHT',
            
            # States when a customer is seen
            'CUSTOMER_SEEN',
            'GREETING_CUSTOMER',
            'TELLING_INSTRUCTIONS',
            'SHOWING_REGISTRATION_QR',
            
            # States for unregistered customers
            'ASKING_CUSTOMER_TO_REGISTER',
            'TELLING_CUSTOMER_ABOUT_REGISTRATION',
            'WAITING_FOR_CUSTOMER_REGISTRATION',
            
            # States for previously registered customers
            'ASKING_CUSTOMER_TO_SHOW_QR',
            'LOOKING_FOR_CUSTOMER_INFORMATION',
            'WELCOMING_REGISTERED_CUSTOMER',
            
            # Ending states
            'JOB_DONE'
        ]
        
        # Task module initialization
        self.tm = tm(perception = True,speech=True,manipulation=False, navigation=True, pytoolkit=True)
        
        # Task module node initialization
        self.tm.initialize_node(self.task_name)
        
        
        """
        1. STATES MACHINEs
        """
        
        # States Machine transitions definition
        
        self.transitions = [
         
            # Transition 1: Start the task
            {'trigger': 'start_task', 'source': 'INIT', 'dest': 'WAITING_FOR_CUSTOMERS'},
            
            # Transition 2: Begin to actively search for customers
            {'trigger': 'search_customers', 'source': 'WAITING_FOR_CUSTOMERS', 'dest': 'SEARCHING_CUSTOMERS'},
            # Transition 2.1: Stop searching for customers
            {'trigger': 'stop_searching_customers', 'source': 'SEARCHING_CUSTOMERS', 'dest': 'WAITING_FOR_CUSTOMERS'},
            
            #  IF Nova sees a customer standing in front of her for some time:
            
            # Transition 3: A customer is seen
            {'trigger': 'customer_seen', 'source': 'SEARCHING_CUSTOMERS', 'dest': 'CUSTOMER_SEEN'},
            # Transition 3.1: Customer lost
            {'trigger': 'customer_lost', 'source': 'CUSTOMER_SEEN', 'dest': 'SEARCHING_CUSTOMERS'}, # It is considered lost if the person does not stay in sight for 5 seconds
            
            # Transition 4: Greet the customer
            {'trigger': 'greet_customer', 'source': 'CUSTOMER_SEEN', 'dest': 'GREETING_CUSTOMER'}, # Says hello, welcome and asks if the customer is there for the event
            
            # Transition 5: Tell the customer the instructions
            {'trigger': 'tell_instructions', 'source': 'GREETING_CUSTOMER', 'dest': 'TELLING_INSTRUCTIONS'}, # Tells the customer the instructions for the event
            

                #   IF the customer is not registered:

            
            # Transition 6: Show the customer the registration QR
            {'trigger': 'show_registration_qr', 'source': 'TELLING_INSTRUCTIONS', 'dest': 'SHOWING_REGISTRATION_QR'}, # Shows the customer the registration QR in the pepper robot tablet
            
            # Transition 7: Ask the customer to register
            {'trigger': 'ask_customer_to_register', 'source': 'SHOWING_REGISTRATION_QR', 'dest': 'ASKING_CUSTOMER_TO_REGISTER'}, # Asks the customer to register in the event
                                                                                                                                # tell the customer that when he/she registers, he/she would need to show the robot the qr code in their mail
            # Transiiton 8: Wait for the registration of the customer
            { 'trigger': 'wait_for_customer_registration', 'source': 'SHOWING_REGISTRATION_QR', 'dest': 'WAITING_FOR_CUSTOMER_REGISTRATION'},
            
            
                #   IF the customer is registered:
            
            # Transition 9: Ask the customer to show their registration QR
            {'trigger': 'ask_customer_to_show_qr', 'source': 'TELLING_INSTRUCTIONS', 'dest': 'ASKING_CUSTOMER_TO_SHOW_QR'}, # Asks the customer to show the registration QR in front ot nova's eyes
            
            # Transition 10: Look for the customer information
            {'trigger': 'look_for_customer_information', 'source': 'ASKING_CUSTOMER_TO_SHOW_QR', 'dest': 'LOOKING_FOR_CUSTOMER_INFORMATION'}, # Looks for the customer information in the database
            
            # Transition 11: Welcome the registered customer
            {'trigger': 'welcome_registered_customer', 'source': 'LOOKING_FOR_CUSTOMER_INFORMATION', 'dest': 'WELCOMING_REGISTERED_CUSTOMER'}, # Nova should say the person's name and more information
            
            
            #   IF Nova has not seen a customer standing in front of her for certain time:
            
            # Transition 12: Say a random fact
            {'trigger': 'say_random_fact', 'source': 'WAITING_FOR_CUSTOMERS', 'dest': 'SAYING_RANDOM_FACT'}, # Says a random fact about the Zeiss company
            
            # Transition 13: Say a random thought
            {'trigger': 'say_random_thought', 'source': 'WAITING_FOR_CUSTOMERS', 'dest': 'SAYING_RANDOM_THOUGHT'}, # Says a random thought about the event
            
            
            #  When the job is done and the team would like to finish the task:
            
            # Transition 14: Finish the task
            {'trigger': 'finish_task', 'source': 'WELCOMING_REGISTERED_CUSTOMER', 'dest': 'JOB_DONE'}, # Finishes the task 
        ]
        
        
        # States machine initialization
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial='INIT')
        
        # Thread for checking rospy
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        # ROS callbacks initialization
        
        
        # 1. Awareness service
        print(self.consoleFormatter.format("Waiting for pytoolkit/awareness...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        self.awareness_srv = rospy.ServiceProxy("/pytoolkit/ALBasicAwareness/set_awareness_srv",SetBool)
        
        # 2. Move head service
        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/move_head...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        self.move_head_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/move_head_srv",move_head_srv)
        
        # 3. Tablet service: Show image
        print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/show_image_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.show_image_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_image_srv",tablet_service_srv)
        
        # 4. Tablet service: Show topic
        print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
        self.show_topic_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv",tablet_service_srv)
        
        # 5. Autonomous life service
        print(self.consoleFormatter.format("Waiting for pytoolkit/autononumusLife...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALAutonomousLife/set_state_srv")
        self.autonomous_life_srv = rospy.ServiceProxy("/pytoolkit/ALAutonomousLife/set_state_srv",SetBool)
        
       # 6. ROS subscribers (perception)
        print(self.consoleFormatter.format("Waiting for /perception_utilities/get_labels_publisher", "WARNING"))
        self.get_labels_publisher = rospy.Subscriber("/perception_utilities/get_labels_publisher", get_labels_msg, self.callback_get_labels)

        # 7. ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)
        
        
        """
        2. VARIABLES
        """
        
        # 1. ROS Callbacks variables
        self.labels = {}
        
        # 2. Global variables
        
        
        
        
        
        
        """
        3. CUSTOM FUNCTIONS
        """
        
        
        
        
        """
        4. TASK STATES LOGICAL IMPLEMENTATION
        """
        
        
        
        
        
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
                self.start()


# Main function
if __name__ == "__main__":
    app_instance= ZeissCustomersReception()
    app_instance.run()
    rospy.spin()

        

        
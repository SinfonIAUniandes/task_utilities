#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from std_msgs.msg import Bool
import ConsoleFormatter
import time
import random
import threading
import sys
import rospy
import math
import os
import numpy as np
from std_srvs.srv import SetBool

from navigation_msgs.srv import constant_spin_srv
from perception_msgs.msg import get_labels_msg
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srv,speech_recognition_srv,set_move_arms_enabled_srv
from robot_toolkit_msgs.msg import animation_msg, touch_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class FINAL(object):
    def __init__(self):
        
        # TODO
        """
        - Revisar si se meten los servicios de interface o del pytoolkit directamente (ojala nodo de interface)
        - Falta meter todos los servicios del pytoolkit al modulo para que se puedan llamar facilmente desde la maquina de estados.
        - Falta crear behaviours como el spin_until_object que es usado varias veces en varios tasks.
        - Falta revisar todos los angulos de navegacion al hacer look_4_something y la velocidad de giro 
        """

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        # Definir los estados posibles del semáforo
        self.task_name = "final"
        states = ['INIT', 'RECEIVE_GUEST', 'SELECT_INGREDIENTS', 'QA','RECIPE','MAKE_BREAKFAST']
        self.tm = tm(perception = True, speech=True, manipulation=True, navigation=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)

        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'FINAL', 'dest': 'INIT'},
            {'trigger': 'beginning', 'source': 'INIT', 'dest': 'RECEIVE_GUEST'},
            {'trigger': 'guest_received', 'source': 'RECEIVE_GUEST', 'dest': 'SELECT_INGREDIENTS'},
            {'trigger': 'ingredients_selected', 'source': 'SELECT_INGREDIENTS', 'dest': 'QA'},
            {'trigger': 'specifications_selected', 'source': 'QA', 'dest': 'RECIPE'},
            {'trigger': 'help_breakfast', 'source': 'RECIPE', 'dest': 'MAKE_BREAKFAST'},
            {'trigger': 'breakfast_made', 'source': 'MAKE_BREAKFAST', 'dest': 'END'},
            {'trigger': 'finish', 'source': 'RECIPE', 'dest': 'END'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='END')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        # ROS Callbacks
        print(self.consoleFormatter.format("Waiting for /robot_toolkit/speech_recognition_srv", "WARNING"))
        rospy.wait_for_service('/robot_toolkit/speech_recognition_srv')
        self.speech_recognition_srv = rospy.ServiceProxy('/robot_toolkit/speech_recognition_srv', speech_recognition_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_move_arms_enabled_srv", "WARNING"))
        rospy.wait_for_service('pytoolkit/ALMotion/set_move_arms_enabled_srv')
        self.setMoveArms_srv = rospy.ServiceProxy('pytoolkit/ALMotion/set_move_arms_enabled_srv', set_move_arms_enabled_srv)

                
        # ROS subscribers (perception)
        print(self.consoleFormatter.format("Waiting for /perception_utilities/get_labels_publisher", "WARNING"))
        self.get_labels_publisher = rospy.Subscriber("/perception_utilities/get_labels_publisher", get_labels_msg, self.callback_get_labels)

        self.headSensorSubscriber = rospy.Subscriber('/touch', touch_msg, self.callback_head_sensor_subscriber)

        # ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        ##################### ROS CALLBACK VARIABLES #####################
        self.labels = []
        
        ##################### GLOBAL VARIABLES #####################W
        self.init_place = "kitchen"
        self.cardinal_to_ordinal = {1:"first", 2:"second", 3:"third", 4:"fourth", 5:"fifth", 6:"sixth", 7:"seventh", 8:"eight", 9:"nineth"}
        self.ingredient_i = 1
        self.selectedIngredients = []
        self.isTouched = False
        self.recipe_info = {}
    

      
    ############## TASK STATES ##############

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.setMoveArms_srv.call(True, True)
        self.tm.set_current_place(self.init_place) 
        self.tm.show_image("sinfonia")
        self.tm.talk("I am going to do the  "+self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom", 2, 15) 
        self.tm.set_awareness(False)
        self.tm.go_to_pose("default_head")
        self.tm.go_to_place("door_kitchen") #TODO
        self.beginning()

    def on_enter_RECEIVE_GUEST(self):
        print(self.consoleFormatter.format("RECEIVE_GUEST", "HEADER"))
        self.tm.start_recognition("front_camera")
        self.tm.show_topic("/perception_utilities/yolo_publisher")
        time.sleep(0.2)
        self.tm.talk("Waiting for the special guest","English")
        self.tm.look_for_object("person")
        self.tm.wait_for_object(15)
        self.tm.look_for_object("",True)
        self.tm.talk("Welcome to our kitchen, esteemed guest! As a culinary robot, I'm here to create your dream breakfast. Get ready for a delightful breakfast experience!", "English")
        self.tm.talk("Follow me to the kitchen, please.")
        self.tm.go_to_place("circular_table")
        self.guest_received()

    def on_enter_SELECT_INGREDIENTS(self):
        print(self.consoleFormatter.format("SELECT_INGREDIENTS", "HEADER"))
        self.tm.talk("Before we begin, esteemed guest, I kindly request you to show me all the ingredients you desire for your breakfast. Simply grab them from the door besides me and place them in front of me, and I'll take note of each ingredient. Once you have everything ready, just give my head a gentle touch,")
        self.tm.talk("Are you ready to start?")
        answer = self.speech_recognition_srv(['yes', 'no'], 0.45)
        
        while answer not in ['yes', 'no']:
            self.tm.talk("Sorry, I did not understand you.")
            self.tm.talk("Are you ready to start?")
            answer = self.speech_recognition_srv(['yes', 'no'], 0.45)
        t1 =time.time()
        possible_ingredients = ["tuna", "tomato_soup", "canned_meat", "mustard", "strawberry_jello", "sugar"]
        while not self.isTouched:
            not_selected = True
            self.tm.talk("Could you please show me the " + self.cardinal_to_ordinal[self.ingredient_i]+ " ingredient")
            self.ingredient_i+=1
            while time.time()-t1<20 and not_selected:
                for label in self.labels:
                    if label["label"] in possible_ingredients:
                        self.selectedIngredients.append(label["label"])
                        t1 = time.time()
                        not_selected = False
                        break
            if not_selected:
                self.tm.talk("Sorry, I could not detect the ingredient that you are showing me, could you please try with a new one?")      
                self.ingredient_i-=1
            else:
                self.tm.talk(f"Thank you, I have detected you want to add the {self.selectedIngredients[-1]} ingredient to the recipe")
                self.tm.talk("Remember to touch my head when you have finished showing me the ingredients you want to use")
                rospy.sleep(5)
        self.ingredients_selected()

    def on_enter_QA(self):
        print(self.consoleFormatter.format("QA", "HEADER"))
        self.tm.talk("Now I will ask you a few questions about your preferences for your delightful breakfast")
        self.recipe_info["temeprature"] = self.tm.q_a_speech("temperature")
        self.recipe_info["sweet"] = self.tm.q_a_speech("sweet")
        self.recipe_info["veget"] = self.tm.q_a_speech("veget")
        self.recipe_info["healthy"] = self.tm.q_a_speech("healthy")
        self.specifications_selected()

    def on_enter_RECIPE(self):
        print(self.consoleFormatter.format("RECIPE", "HEADER"))
        #
        # TODO CHAT GPT
        #
        
        self.tm.talk("Do you want me to help you doing the recipie?")
        answer = self.speech_recognition_srv(['yes', 'no'], 0.45)
        
        while answer not in ['yes', 'no']:
            self.tm.talk("Sorry, I did not understand you.")
            self.tm.talk("Do you want me to help you doing the recipie?")
            answer = self.speech_recognition_srv(['yes', 'no'], 0.45)

        if answer == "yes":
            self.tm.talk("Perfect, I am glad to help you")
            self.help_breakfast()
        else:
            self.tm.talk("Sorry, I did not understand you.")
            self.tm.talk("Do you want me to help you doing the recipie?")
            answer = self.speech_recognition_srv(['yes', 'no'], 0.45)
            if answer == "yes":
                self.tm.talk("Perfect, I am glad to help you")
                self.help_breakfast()

    def on_enter_MAKE_BREAKFAST(self):
        print(self.consoleFormatter.format("MAKE_BREAKFAST", "HEADER"))
        self.tm.talk("I will start preparing your breakfast")
        self.setMoveArms_srv.call(False, False)
        for ingredient in self.selectedIngredients:
            self.tm.go_to_place("circular_table")
            self.tm.grasp_object(ingredient)
            self.tm.go_to_place("rectangular_table")
            self.tm.leave_object(ingredient)
        self.finish()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()

    def callback_head_sensor_subscriber(self, msg):
        if msg.name in["head_rear","head_middle","head_front"]:
            if msg.state:
                self.isTouched=msg.state


    def callback_get_labels(self,data):
        labels = data.labels
        x_coordinates = data.x_coordinates
        y_coordinates = data.y_coordinates
        widths = data.widths
        heights = data.heights
        ids = data.ids
        self.label= []
        for i in range(len(labels)):
            label_dict = {"label":labels[i], "x":x_coordinates[i], "y":y_coordinates[i], "w":widths[i], "h":heights[i], "id":ids[i]}
            self.labels.append(label_dict)
        
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = FINAL()
    sm.run()
    rospy.spin()

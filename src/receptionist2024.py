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
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srv
from robot_toolkit_msgs.msg import animation_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class RECEPTIONIST(object):
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
        self.task_name = "receptionist"
        states = ['INIT', 'WAIT4GUEST', 'QA', 'SAVE_FACE','INTRODUCE_NEW','INTRODUCE_OLD','GO2LIVING','GO2DOOR','LOOK4PERSON','LOOK4CHAIR','SIGNAL_SOMETHING']
        self.tm = tm(perception = True,speech=True,manipulation=False, navigation=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'RECEPTIONIST', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'WAIT4GUEST'},
            {'trigger': 'person_arrived', 'source': 'WAIT4GUEST', 'dest': 'QA'},
            {'trigger': 'person_met', 'source': 'QA', 'dest': 'SAVE_FACE'},
            {'trigger': 'save_face_failed', 'source': 'SAVE_FACE', 'dest': 'SAVE_FACE'},
            {'trigger': 'save_face_succeded', 'source': 'SAVE_FACE', 'dest': 'GO2LIVING'},
            {'trigger': 'arrived_to_point', 'source': 'GO2LIVING', 'dest': 'INTRODUCE_NEW'},
            {'trigger': 'introduced_new_person', 'source': 'INTRODUCE_NEW', 'dest': 'LOOK4PERSON'},
            {'trigger': 'person_not_found', 'source': 'LOOK4PERSON', 'dest': 'LOOK4PERSON'},
            {'trigger': 'person_found', 'source': 'LOOK4PERSON', 'dest': 'INTRODUCE_OLD'},
            {'trigger': 'introduced_old_person', 'source': 'INTRODUCE_OLD', 'dest': 'LOOK4PERSON'},
            {'trigger': 'introduced_everyone', 'source': 'LOOK4PERSON', 'dest': 'LOOK4CHAIR'},
            {'trigger': 'chair_found', 'source': 'LOOK4CHAIR', 'dest': 'SIGNAL_SOMETHING'},
            {'trigger': 'person_accomodated', 'source': 'SIGNAL_SOMETHING', 'dest': 'GO2DOOR'},
            {'trigger': 'wait_new_guest', 'source': 'GO2DOOR', 'dest': 'WAIT4GUEST'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='RECEPTIONIST')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        # ROS Callbacks

        # ROS Services (PyToolkit)
        print(self.consoleFormatter.format("Waiting for pytoolkit/awareness...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        self.awareness_srv = rospy.ServiceProxy("/pytoolkit/ALBasicAwareness/set_awareness_srv",SetBool)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/move_head...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        self.move_head_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/move_head_srv",move_head_srv)

        print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/show_image_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.show_image_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_image_srv",tablet_service_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
        self.show_topic_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv",tablet_service_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/autononumusLife...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALAutonomousLife/set_state_srv")
        self.autonomous_life_srv = rospy.ServiceProxy("/pytoolkit/ALAutonomousLife/set_state_srv",SetBool)
                
        # ROS subscribers (perception)
        print(self.consoleFormatter.format("Waiting for /perception_utilities/get_labels_publisher", "WARNING"))
        self.get_labels_publisher = rospy.Subscriber("/perception_utilities/get_labels_publisher", get_labels_msg, self.callback_get_labels)

        # ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        ##################### ROS CALLBACK VARIABLES #####################
        self.labels ={}
        ##################### GLOBAL VARIABLES #####################

        self.initial_place ="init_living_room"
        self.sinfonia_url_img="https://media.discordapp.net/attachments/876543237270163498/1123649957791010939/logo_sinfonia_2.png"
        self.img_dimensions = (320,240)
        self.recognize_person_counter = 0
        self.all_persons = {"Charlie":{"name":"Charlie","age": self.categorize_age(21),"drink":"Milk","gender":"Man","pronoun":"he"}}
        self.introduced_persons = []
        self.actual_person={}
        self.old_person = ""
        self.failed_saving_face=False
        self.angle_index = 0
        self.chair_angles = [260,220,180,140]
        self.checked_chair_angles = []
        self.empty_chair_angles = []
        self.first_time = True
        self.host_name = "Charlie"
    
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

    def categorize_age(self,age):
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
    
    ############## TASK STATES ##############

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.set_current_place(self.initial_place)
        self.tm.talk("I am going to do the  "+self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",2,10) 
        # self.awareness_srv(False)
        self.tm.go_to_place("door_living_room")
        self.beggining()
                
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.show_topic_srv("/perception_utilities/yolo_publisher")
        self.tm.start_recognition("front_camera")
        time.sleep(0.2)
        self.tm.talk("Waiting for guests","English")
        self.tm.look_for_object("person",False)
        self.tm.wait_for_object(-1)        
        self.tm.look_for_object("",True)
        self.person_arrived()
        
    def on_enter_QA(self):
        print(self.consoleFormatter.format("QA", "HEADER"))
        clothes_color = self.tm.get_clothes_color()
        self.actual_person["clothes_color"]=clothes_color
        self.move_head_srv("up")
        name=self.tm.q_a_speech("name")
        drink=self.tm.q_a_speech("drink")
        self.actual_person = {"name":name,"drink":drink}
        self.tm.start_recognition("")
        self.person_met()

    def on_enter_SAVE_FACE(self):

        print(self.consoleFormatter.format("SAVE_FACE", "HEADER"))
        # self.move_head_srv("up")
        if not self.failed_saving_face:
            self.tm.publish_filtered_image("face","front_camera")
            self.show_topic_srv("/perception_utilities/filtered_image")
            self.tm.talk("Hey {}, I will take some pictures of your face to recognize you in future occasions".format(self.actual_person["name"]),"English")
        success = self.tm.save_face(self.actual_person["name"],5)
        attributes = self.tm.get_person_description()
        print("attributes: ",attributes)
        # attributes = {"age":25,"gender":"Man","race":"White"}
        print("success ",success)
        if success and attributes!={}:
            time.sleep(1)
            self.actual_person["age"]=self.categorize_age(attributes["age"])
            self.actual_person["gender"]=attributes["gender"]
            self.actual_person["race"]=attributes["race"]
            self.actual_person["pronoun"]= "he" if  attributes["gender"] == "Man" else "she"
            self.actual_person["has_glasses"]=attributes["has_glasses"]
            self.actual_person["has_beard"]=attributes["has_beard"]
            self.actual_person["has_hat"]=attributes["has_hat"]
            self.all_persons[self.actual_person["name"]] = self.actual_person
            self.move_head_srv("default")  
            self.failed_saving_face=False
            self.show_image_srv(self.sinfonia_url_img)
            self.save_face_succeded()
        else:
            self.failed_saving_face=True
            self.tm.talk("I am sorry {}, I was not able to save your face, can you please see my tablet and fit your face".format(self.actual_person["name"]),"English")
            self.save_face_failed()
    
    def on_enter_GO2LIVING(self):
        print(self.consoleFormatter.format("GO2LIVING", "HEADER"))
        self.tm.talk("Please {}, follow me to the living room".format(self.actual_person["name"]),"English",wait=False)
        self.tm.go_to_place("receptionist")
        self.arrived_to_point()

    def on_enter_INTRODUCE_NEW(self):
        print(self.consoleFormatter.format("INTRODUCE_NEW", "HEADER"))
        self.tm.talk("Please {}, stand besides me".format(self.actual_person["name"]),"English")
        time.sleep(2)
        has_beard_str = "has a beard" if self.actual_person["has_beard"] else "does not have a beard"
        has_glasses_str = "wears glasses" if self.actual_person["has_glasses"] else "does not wear glasses"
        has_hat_str = "wears a hat" if self.actual_person["has_hat"] else "does not wear a hat"
        self.tm.talk(f'Hello everyone, this is {self.actual_person["name"]}, {self.actual_person["pronoun"]} is a {self.actual_person["gender"]}.  {self.actual_person["pronoun"]} is {self.actual_person["age"]}, and {self.actual_person["pronoun"]} likes to drink {self.actual_person["drink"]}. {self.actual_person["pronoun"]} {has_beard_str}, {has_hat_str} and {has_glasses_str}',"English")
        #Turns on recognition and looks for  person
        self.tm.start_recognition("front_camera")
        # Reiniciar las variables de presentacion de personas y sillas
        self.introduced_persons=[]
        self.introduced_persons.append(self.actual_person["name"])
        self.checked_chair_angles=[]
        self.empty_chair_angles=self.chair_angles.copy()
        self.angle_index=0
        self.show_topic_srv("/perception_utilities/yolo_publisher")
        self.introduced_new_person()
    
    def on_enter_LOOK4PERSON(self):
        print(self.consoleFormatter.format("LOOK4PERSON", "HEADER"))
        # El robot ya fue a todas las posiciones de personas o introdujo a todas las personas
        if (len(self.checked_chair_angles)==len(self.chair_angles)) or (len(self.introduced_persons)==len(self.all_persons)):
            if len(self.introduced_persons)==len(self.all_persons):
                print(self.consoleFormatter.format("INTRODUCED_EVERYONE", "OKGREEN"))
                self.introduced_everyone()
            else:
                print(self.consoleFormatter.format("FAILED_INTRODUCING", "FAIL"))
                not_introduced = []
                for person in self.all_persons:
                    if person not in self.introduced_persons:
                        not_introduced.append(person)
                not_introduced_persons = ", ".join(not_introduced)
                self.tm.talk("{} I am sorry I was not able to recognize you, please introduce yourself to {}".format(not_introduced_persons,self.actual_person["name"]),"English",wait=True)
                self.introduced_everyone()
        # El robot busca una persona
        else:
            self.move_head_srv("default")
            self.tm.go_to_defined_angle_srv(self.chair_angles[self.angle_index])
            self.checked_chair_angles.append(self.chair_angles[self.angle_index])
            self.angle_index+=1
            t1 = time.time()
            self.labels ={}
            while time.time()-t1<5:
                if "person" in self.labels:
                    self.tm.talk("Recognizing person","English")
                    self.person_found()
                    break
            self.person_not_found()
            
    def on_enter_INTRODUCE_OLD(self):
        print(self.consoleFormatter.format("INTRODUCE_OLD", "HEADER"))
        person_name = ""
        while person_name == "" and self.recognize_person_counter<3:
            person_name = self.tm.recognize_face(3)
            print("saw: "+person_name)
            self.recognize_person_counter+=1
            self.recognize_person_counter=0
        if person_name not in self.introduced_persons and person_name in self.all_persons:
            self.empty_chair_angles.remove(self.checked_chair_angles[-1])
            person_introduce = self.all_persons[person_name]
            print("Person introduce: ",person_introduce)
            #TODO manipulacion animations/poses
            self.animations_publisher.publish("animations","Gestures/TakePlace_2")
            self.tm.talk(f' {self.actual_person["name"]} I introduce to you {person_name}. {person_introduce["pronoun"]} is a {person_introduce["gender"]}. {person_name} is around {person_introduce["age"]} years old and likes to drink {person_introduce["drink"]}',"English")
            self.introduced_persons.append(person_name)
        self.introduced_old_person()
    
    def on_enter_LOOK4CHAIR(self):
        print(self.consoleFormatter.format("LOOK4CHAIR", "HEADER"))
        self.show_image_srv(self.sinfonia_url_img)
        if len(self.empty_chair_angles)!=0:
            chair_angle = random.choice(self.empty_chair_angles)
            print("chair_angle ",chair_angle)
        else:
            chair_angle=180
        self.tm.go_to_defined_angle_srv(chair_angle)
        self.chair_found()

    def on_enter_SIGNAL_SOMETHING(self):
        print(self.consoleFormatter.format("SIGNAL_SOMETHING", "HEADER"))
        self.animations_publisher.publish("animations","Gestures/TakePlace_2")
        self.tm.talk("Please, take a seat {}".format(self.actual_person["name"]),"English")
        time.sleep(0.5)
        self.person_accomodated()

    def on_enter_GO2DOOR(self):
        print(self.consoleFormatter.format("GO2DOOR", "HEADER"))
        self.tm.talk("Waiting for other guests to come","English",wait=False)
        self.tm.go_to_place("door_living_room")
        self.wait_new_guest()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
        
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = RECEPTIONIST()
    sm.run()
    rospy.spin()

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
from robot_toolkit_msgs.srv import tablet_service_srv
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
        - Poner una redundancia para cuando el robot asigna un nuevo a id a una misma persona para que no la presente (lista con los nombres de los que ya presento, incluyendo el actual)
        """

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "receptionist"
        states = ['INIT', 'WAIT4GUEST', 'QA', 'SAVE_FACE','INTRODUCE_NEW','INTRODUCE_OLD','GO2LIVING','GO2DOOR','LOOK4PERSON','LOOK4CHAIR','SIGNAL_SOMETHING']
        self.tm = tm(perception = True,speech=True,manipulation=False, navigation=True)
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
        print(self.consoleFormatter.format("Waiting for /amcl_pose", "WARNING"))
        subscriber_odom = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_spin_until)

        print(self.consoleFormatter.format("Waiting for /look_for_object_publisher", "WARNING"))
        subscriber_look_for_object = rospy.Subscriber("/perception_utilities/look_for_object_publisher", Bool, self.callback_look_for_object)


        # ROS Services (PyToolkit)
        print(self.consoleFormatter.format("Waiting for pytoolkit/awareness...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        self.awareness_srv = rospy.ServiceProxy("/pytoolkit/ALBasicAwareness/set_awareness_srv",SetBool)

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
        self.img_dimensions = (320,240)
        self.angle_stop_looking_person = 190
        self.recognize_person_counter = 0
        self.all_persons = {"Charlie":{"name":"Charlie","age":"20","drink":"beer"}}
        self.introduced_persons = []
        self.actual_person={}
        self.old_person = ""
        self.failed_saving_face=False
        self.angle_index = 0
        self.chair_angles = [90,130,160]
        self.checked_chair_angles = []
        self.empty_chair_angles = []

    
    def callback_get_labels(self,data):
        labels = data.labels
        x_coordinates = data.x_coordinates
        y_coordinates = data.y_coordinates
        widths = data.widths
        heights = data.heights
        ids = data.ids
        for i in range(len(labels)):
            if self.img_dimensions[0]//3<x_coordinates<int(self.img_dimensions[0]*2/3):
                self.labels[labels[i]] = {"x":x_coordinates[i],"y":y_coordinates[i],"w":widths[i],"h":heights[i],"id":ids[i]}

    def on_enter_INIT(self):
        self.autonomous_life_srv(False)
        self.tm.talk("I am going to do the  "+self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",1,15) 
        self.awareness_srv(False)
        self.tm.go_to_place("door")
        self.beggining()
                
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.show_topic_srv("/perception_utilities/yolo_publisher")
        self.tm.start_recognition("front_camera")
        time.sleep(0.2)
        #TODO mostrar el topico de yolo en la pantalla
        self.tm.talk("Waiting for guests","English")
        self.tm.look_for_object("person",False)
        self.tm.wait_for_object(-1)
        #TODO apagar el topico de yolo en la pantalla
        
        self.tm.look_for_object("",True)
        self.person_arrived()
        
    def on_enter_QA(self):
        print(self.consoleFormatter.format("QA", "HEADER"))
        name=self.tm.q_a_speech("name")
        age=self.tm.q_a_speech("age")
        drink=self.tm.q_a_speech("drink")
        self.actual_person = {"name":name,"age":age,"drink":drink}
        self.all_persons[name] = {"name":name,"age":age,"drink":drink}
        self.tm.start_recognition("")
        self.person_met()

    def on_enter_SAVE_FACE(self):
        print(self.consoleFormatter.format("SAVE_FACE", "HEADER"))
        self.tm.publish_filtered_image("face","front_camera")
        self.show_topic_srv("/perception_utilities/filtered_image")
        #TODO encender el awareness para que siga la cara de la persona
        # self.awareness_srv(False)
        time.sleep(0.5)
        self.animations_publisher.publish("animations","Gestures/Look_1")
        if not self.failed_saving_face:
            self.tm.talk("Hey {}, I will take some pictures of your face to recognize you in future occasions".format(self.actual_person["name"]),"English")
        succed = self.tm.save_face(self.actual_person["name"],5)
        print("succed ",succed)
        #TODO apagar el topico con el filtro para meter la cara en la pantalla
        if succed:
            #TODO apagar el awareness y fijar la posicion de la cabeza para que mire al frente
            # self.awareness_srv(False)
            time.sleep(1)
            self.animations_publisher.publish("animations","Gestures/Maybe_1")
            self.failed_saving_face=False
            #TODO show sinfonia img
            self.show_topic_srv("/robot_toolkit_node/camera/front/image_raw")
            self.save_face_succeded()
        else:
            self.failed_saving_face=True
            self.tm.talk("I am sorry {}, I was not able to save your face, can you please see my tablet and fit your face".format(self.actual_person["name"]),"English")
            self.save_face_failed()
    
    def on_enter_GO2LIVING(self):
        print(self.consoleFormatter.format("GO2LIVING", "HEADER"))
        self.tm.talk("Please {}, follow me to the living room".format(self.actual_person["name"]),"English",wait=False)
        self.tm.go_to_place("living_room")
        #TODO revisar el angulo de llegada del robot
        self.arrived_to_point()

    def on_enter_INTRODUCE_NEW(self):
        print(self.consoleFormatter.format("INTRODUCE_NEW", "HEADER"))
        self.introduced_persons=[]
        self.introduced_persons.append(self.actual_person["name"])
        self.tm.talk("Hello everyone, this is {}, he is {} years old and he likes to drink {}".format(self.actual_person["name"],self.actual_person["age"],self.actual_person["drink"]),"English")
        # self.awareness_srv(True)
        time.sleep(0.5)
        self.animations_publisher.publish("animations","Gestures/Maybe_1")
        time.sleep(0.5)
        # self.awareness_srv(False)
        #Turns on recognition and looks for  person
        self.tm.start_recognition("front_camera")
        self.checked_chair_angles=[]
        self.empty_chair_angles=[]
        self.introduced_new_person()
    
    def on_enter_LOOK4PERSON(self):
        print(self.consoleFormatter.format("LOOK4PERSON", "HEADER"))
        # El robot ya fue a todas las posiciones de personas
        if len(self.checked_chair_angles)==len(self.chair_angles):
            if len(self.introduced_persons)==len(self.all_persons):
                print(self.consoleFormatter.format("INTRODUCED_EVERYONE", "OKGREEN"))
                self.introduced_everyone()
            else:
                not_introduced = []
                for person in self.all_persons:
                    if person not in self.introduced_persons:
                        not_introduced.append(person)
                not_introduced_persons = ", ".join(not_introduced)
                self.tm.talk("{} I am sorry I was not able to recognize you, please introduce yourself to {}".format(not_introduced_persons,self.actual_person["name"]),"English")
                self.introduced_everyone()
        # El robot busca una perosona
        else:
            self.tm.go_to_defined_angle_srv(self.chair_angles[self.angle_index])
            self.checked_chair_angles.append(self.chair_angles[self.angle_index])
            self.angle_index+=1
            t1 = time.time()
            while time.time()-t1<3:
                if "person" in self.labels:
                    self.tm.talk("Recognizing person","English")
                    self.person_found()
                    break
            self.empty_chair_angles.append(self.checked_chair_angles[-1])
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
            person_introduce = self.all_persons[person_name]
            #TODO manipulacion animations/poses
            self.animations_publisher.publish("animations","Gestures/TakePlace_2")
            self.tm.talk(" {} I introduce to you {} he is {} years old and he likes to drink {}".format(self.actual_person["name"],person_name,person_introduce["age"],person_introduce["drink"]),"English")
            self.animations_publisher.publish("animations","Gestures/Maybe_1")
            self.introduced_persons.append(person_name)
        self.introduced_old_person()
    
    def on_enter_LOOK4CHAIR(self):
        print(self.consoleFormatter.format("LOOK4CHAIR", "HEADER"))
        chair_angle = random.choice(self.empty_chair_angles)
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
        self.tm.talk("I am going to receive new guests","English",wait=False)
        self.tm.go_to_place("door")
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

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
import numpy as np

from navigation_msgs.srv import constant_spin_srv
from navigation_msgs.msg import simple_feedback_msg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class RECEPTIONIST(object):
    def __init__(self):

        
        # Definir los estados posibles del semáforo
        self.task_name = "receptionist"
        states = ['INIT', 'WAIT4GUEST', 'QA', 'SAVE_FACE','INTRODUCE_NEW','INTRODUCE_OLD','GO2LIVING','GO2DOOR','LOOK4PERSON','LOOK4CHAIR','SIGNAL_SOMETHING']
        self.tm = tm(perception = False,speech=False,manipulation=False, navigation=True)
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
            {'trigger': 'person_found', 'source': 'LOOK4PERSON', 'dest': 'INTRODUCE_OLD'},
            {'trigger': 'introduced_old_person', 'source': 'INTRODUCE_OLD', 'dest': 'LOOK4PERSON'},
            {'trigger': 'introduced_everyone', 'source': 'LOOK4PERSON', 'dest': 'LOOK4CHAIR'},
            {'trigger': 'chair_found', 'source': 'LOOK4CHAIR', 'dest': 'SIGNAL_SOMETHING'},
            {'trigger': 'person_accomodated', 'source': 'SIGNAL_SOMETHING', 'dest': 'GO2DOOR'},
            {'trigger': 'wait_new_guest', 'source': 'GO2DOOR', 'dest': 'WAIT4GUEST'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='RECEPTIONIST')

        # ROS Callbacks
        subscriber_odom = rospy.Subscriber("/odom", Odometry, self.callback_spin_until)
        subscriber_look_for_object = rospy.Subscriber("/perception_utilities/look_for_object_publisher", Bool, self.callback_look_for_object)

        ##################### ROS CALLBACK VARIABLES #####################
        self.angle = 0
        self.object_found = False
        self.stop_rotation = False
        ##################### GLOBAL VARIABLES #####################

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.all_persons = {"charlie":{"name":"charlie","age":"20","drink":"beer"}}
        self.actual_person={}
        self.old_person = ""


    def callback_spin_until(self,data):
        self.angle = int(np.degrees(euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2] % (2 * math.pi)))
        return self.angle
    
    def callback_look_for_object(self,data):
        self.object_found=data.data
        if self.object_found:
            self.stop_rotation=True
        return data
    
    def on_enter_INIT(self):
        self.tm.talk("I am going to do the  "+self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","enable",0,0)  
        self.tm.start_recognition("front_camera")
        self.beggining()
                
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.talk("Waiting for guests","English")
        self.tm.look_for_object("person",True)
        self.tm.wait_for_object(-1)
        self.tm.look_for_object("",True)
        self.person_arrived()
        
    def on_enter_QA(self):
        print(self.consoleFormatter.format("QA", "HEADER"))
        name=self.tm.q_a_speech("name")
        age=self.tm.q_a_speech("age")
        drink=self.tm.q_a_speech("drink")
        self.actual_person = {"name":name,"age":age,"drink":drink}
        self.all_persons[name] = {"name":name,"age":age,"drink":drink}
        self.person_met()

    def on_enter_SAVE_FACE(self):
        print(self.consoleFormatter.format("SAVE_FACE", "HEADER"))
        self.tm.talk("Hey {}, I will take some pictures of your face to recognize you in future occasions".format(self.actual_person["name"]),"English")
        succed = self.tm.save_face(self.actual_person["name"],7)
        if succed:
            self.save_face_succeded()
        else:
            self.tm.talk("I am sorry {}, I was not able to save your face, can you please see my tablet and fit your face".format(self.actual_person["name"]),"English")
            self.save_face_failed()
    
    def on_enter_GO2LIVING(self):
        print(self.consoleFormatter.format("GO2LIVING", "HEADER"))
        self.tm.talk("Please {}, follow me to the living room".format(self.actual_person["name"]),"English")
        self.tm.go_to_place("living_room")
        self.tm.wait_go_to_place()
        self.arrived_to_point()

    def on_enter_INTRODUCE_NEW(self):
        print(self.consoleFormatter.format("INTRODUCE_NEW", "HEADER"))
        self.tm.talk("Hello everyone, this is {}, he is {} years old and he likes to drink {}".format(self.actual_person["name"],self.actual_person["age"],self.actual_person["drink"]),"English")
        self.introduced_new_person()
    
    def on_enter_LOOK4PERSON(self):
        print(self.consoleFormatter.format("LOOK4PERSON", "HEADER"))
        self.tm.look_for_object("person",True)
        self.tm.constant_spin_proxy(10.0)
        while self.angle<270 and not self.stop_rotation:
            time.sleep(0.1)
        self.tm.robot_stop_srv()
        self.tm.look_for_object("",True)
        self.stop_rotation=False
        if self.angle>=270:
            self.introduced_everyone()
        else:
            self.person_found()
        
    def on_enter_INTRODUCE_OLD(self):
        print(self.consoleFormatter.format("INTRODUCE_OLD", "HEADER"))
        person_name = self.tm.recognize_face(3)
        person_introduce = self.all_persons[person_name]
        self.tm.talk(" {} I introduce to you {} he is {} years old and he likes to drink {}".format(self.actual_person["name"],person_name,person_introduce["age"],person_introduce["drink"]),"English")
        self.introduced_old_person()
    
    def on_enter_LOOK4CHAIR(self):
        print(self.consoleFormatter.format("LOOK4CHAIR", "HEADER"))
        self.tm.go_to_place("living_room")
        self.tm.turn_camera("bottom_camera","custom",1,15)
        self.tm.start_recognition("bottom_camera")
        #TODO Chair without person
        self.tm.look_for_object("chair",True)
        self.tm.constant_spin_proxy(10.0)
        while self.angle<270 and not self.stop_rotation:
            time.sleep(0.1)
        self.tm.robot_stop_srv()
        self.tm.look_for_object("",True)
        self.chair_found()

    def on_enter_SIGNAL_SOMETHING(self):
        print(self.consoleFormatter.format("SIGNAL_SOMETHING", "HEADER"))
        self.tm.talk("Please, take a seat {}".format(self.actual_person["name"]),"English")
        #TODO
        #self.tm.go_to_pose("signal")
        self.person_accomodated()

    def on_enter_GO2DOOR(self):
        print(self.consoleFormatter.format("GO2DOOR", "HEADER"))
        self.tm.talk("Waiting for guests to come","English")
        self.tm.go_to_place("door")
        self.tm.wait_go_to_place()
        self.wait_new_guest()

    def run(self):
        while not rospy.is_shutdown():
            self.start()
        
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = RECEPTIONIST()
    sm.run()
    rospy.spin()

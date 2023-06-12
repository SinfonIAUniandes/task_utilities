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

from navigation_msgs.msg import simple_feedback_msg

class RECEPTIONIST(object):
    def __init__(self):
        # Definir los estados posibles del semáforo
        self.task_name = "receptionist"
        states = ['INIT', 'WAIT4GUEST', 'QA', 'SAVE_FACE','RECOG_PERSON','INTRODUCE_NEW','INTRODUCE_OLD','GO2LIVING','GO2DOOR','LOOK4PERSON','LOOK4CHAIR','SIGNAL_SOMETHING', 'FINISH']
        self.tm = tm(perception = True,speech=False,manipulation=False, navigation=False)
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

        ##################### GLOBAL VARIABLES #####################

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.actual_person={}


    def on_enter_INIT(self):
        self.tm.initialize_node(self.task_name)
        self.tm.talk("English","I am going to do the  "+self.task_name+"task",2)
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","enable",0,0)
        self.tm.start_recognition("front_camera")
        self.beggining()
    
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.talk("English","Waiting for guests",2)
        self.tm.look_for_object("person")
        self.tm.wait_for_object(-1)
        self.tm.look_for_object("")
        self.person_arrived()
        
    def on_enter_QA(self):
        print(self.consoleFormatter.format("QA", "HEADER"))
        questions = ["name","age","drink"]
        #TODO
        # person=self.tm.q_a(questions)
        # self.actual_person = {"name":person[0],"age":person[1],"drink":person[2]}
        self.person_met()

    def on_enter_SAVE_FACE(self):
        print(self.consoleFormatter.format("SAVE_FACE", "HEADER"))
        self.tm.talk("English","Hey {}, I will take some pictures of your face to recognize you in future occasions".format(self.actual_person["name"]),8)
        succed = self.tm.save_face(self.actual_person["name"])
        if succed:
            self.save_face_succeded()
        else:
            self.save_face_failed()
    
    def on_enter_GO2LIVING(self):
        print(self.consoleFormatter.format("GO2LIVING", "HEADER"))
        self.tm.talk("English","Please {}, follow me to the living room".format(self.actual_person["name"]),6)
        self.tm.go_to_place("living_room")
        self.tm.wait_go_to_place()
        self.arrived_to_point()

    def on_enter_INTRODUCE_NEW(self):
        print(self.consoleFormatter.format("INTRODUCE_NEW", "HEADER"))
        self.tm.talk("English","Hello everyone, this is {}, he is {} years old and he likes to drink {}".format(self.actual_person["name"],self.actual_person["age"],self.actual_person["drink"]),8)
        self.introduced_new_person()
    
    def on_enter_LOOK4PERSON(self):
        #TODO
        # Revisar como se hace para revisar el angulo
        # self.tm.look_for_object("person")
        # self.tm.wait_for_object(-1)
        # TODO
        # self.tm.spin()
        pass
        self.person_found()
        
    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = RECEPTIONIST()
    sm.run()
    rospy.spin()

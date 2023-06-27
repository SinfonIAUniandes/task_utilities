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
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.msg import animation_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class GPSR(object):
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
        self.task_name = "GPSR"
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
        self.awareness_srv = rospy.ServiceProxy("/pytoolkit/ALBasicAwareness/set_awareness_srv",SetBool)

        print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
        self.show_topic_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv",SetBool)

        # ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        ##################### ROS CALLBACK VARIABLES #####################
        self.angle = 0
        self.recognize_person_counter = 0
        self.angle_stop_looking_person = 190
        self.object_found = False
        self.stop_rotation = False
        ##################### GLOBAL VARIABLES #####################


    def callback_spin_until(self,data):
        self.angle = int(np.degrees(euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2] % (2 * math.pi)))
        return self.angle
    
    def callback_look_for_object(self,data):
        self.object_found=data.data
        if self.object_found:
            self.stop_rotation=True
        return data
    
    def on_enter_INIT(self):
        print(self.angle)
        self.tm.talk("I am going to do the  "+self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",1,15) 
        self.tm.turn_camera("bottom_camera","custom",1,15)
        self.tm.go_to_place("door")
        self.tm.wait_go_to_place()
        self.beggining()
                
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.start_recognition("front_camera")
        #TODO mostrar el topico de yolo en la pantalla
        self.tm.talk("Waiting for guests","English")
        self.tm.look_for_object("person",True)
        self.tm.wait_for_object(-1)
        #TODO apagar el topico de yolo en la pantalla
        self.tm.start_recognition("")
        self.tm.look_for_object("",True)
        self.person_arrived()
        
    def on_enter_QA(self):
        # self.show_topic_srv("/robot_toolkit_node/camera/bottom/image_raw")
        print(self.consoleFormatter.format("QA", "HEADER"))
        name=self.tm.q_a_speech("name")
        age=self.tm.q_a_speech("age")
        drink=self.tm.q_a_speech("drink")
        self.actual_person = {"name":name,"age":age,"drink":drink}
        self.all_persons[name] = {"name":name,"age":age,"drink":drink}
        self.person_met()


    def on_enter_GO2DOOR(self):
        print(self.consoleFormatter.format("GO2DOOR", "HEADER"))
        self.tm.talk("Waiting for guests to come","English")
        self.tm.go_to_place("door")
        self.tm.wait_go_to_place()
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
    sm = GPSR()
    sm.run()
    rospy.spin()

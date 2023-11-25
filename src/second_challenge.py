#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from perception_msgs.msg import get_labels_msg
import ConsoleFormatter
import time
import threading
import rospy
import os


class SECOND_CHALLENGE(object):
    def __init__(self):
        
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "second challenge"
        states = ['INIT', 'LOOK4OBJECT', 'RETURN', 'SHARE']
        self.tm = tm(perception = True, speech=True, manipulation=False, navigation=True)
        self.tm.initialize_node("first_challenge")
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'SECOND_CHALLENGE', 'dest': 'INIT'},
            {'trigger': 'look', 'source': 'INIT', 'dest': 'LOOK4OBJECT'},
            {'trigger': 'ret', 'source': 'LOOK4OBJECT', 'dest': 'RETURN'},
            {'trigger': 'share', 'source': 'RETURN', 'dest': 'SHARE'},

        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='SECOND_CHALLENGE')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        #Subscriber:
        self.get_labels_publisher = rospy.Subscriber("/perception_utilities/get_labels_publisher", get_labels_msg, self.callback_get_labels)


        ##################### ROS CALLBACK VARIABLES #####################
        self.label = ""

        ##################### GLOBAL VARIABLES #####################
        self.object = ""
        

    def on_enter_INIT(self):
        self.tm.talk("Voy a realizar el segundo reto","Spanish")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",2,15)
        self.tm.start_recognition("front_camera")
        self.tm.talk("Mi misión es ver qué objeto hay en el fondo y compartirlo con ustedes","Spanish")
        self.tm.go_to_relative_point(2,0,0)
        self.look()   

    def on_enter_LOOK4OBJECT(self):
        self.tm.talk("Buscando objeto","Spanish")
        objectFound = False
        while(objectFound == False):
            if("bottle" in self.label):
                self.object = "Botella"
                objectFound = True
            elif("laptop" in self.label):
                self.object = "laptop"
                objectFound = True
        self.tm.talk("Objeto identificado","Spanish")
        self.ret()  

    def on_enter_RETURN(self):
        self.tm.talk("Ahora daré la vuelta para poder compartir el objeto","Spanish")
        self.tm.go_to_relative_point(0,0,180)
        self.tm.go_to_relative_point(2,0,0)
        self.share()  

    def on_enter_SHARE(self):
        self.tm.talk("El objeto del fondo es una " + self.object,"Spanish")
        time.sleep(5)
        self.tm.talk("Si necesitas otra cosa no dudes en decirme","Spanish")
        
    # Callbacks for the perception        
    def callback_get_labels(self,data):
        self.label = data.labels
        
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
    sm = SECOND_CHALLENGE()
    sm.run()
    rospy.spin()

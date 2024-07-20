#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import random
import threading
import sys
import rospy
import math
import os
import numpy as np
import re
import ast
# from code_generation import ls_generate as gen
from local_gpsr_code_generation import init as local_generator_init
from local_gpsr_code_generation import robot_code_generator_aux as local_generator
from code_generation import generate_utils 
from code_generation.database.models import Model
from std_msgs.msg import String
from sensor_msgs.msg import Range
from std_srvs.srv import Empty
from robot_toolkit_msgs.msg import speech_recognition_status_msg

class GPSR(object):
    def __init__(self):
        # self.gen = gen.LongStringGenerator()

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "GPSR"
        states = ['INIT', 'WAIT4GUEST', 'GPSR', 'GO2GPSR']
        self.tm = tm(perception = True,speech=True,manipulation=True, navigation=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'GPSR', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO2GPSR'},
            {'trigger': 'go_to_gpsr', 'source': 'GO2GPSR', 'dest': 'WAIT4GUEST'},
            {'trigger': 'person_arrived', 'source': 'WAIT4GUEST', 'dest': 'GPSR'},
            {'trigger': 'GPSR_done', 'source': 'GPSR', 'dest': 'GO2GPSR'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='GPSR')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        ############################# GLOBAL VARIABLES #############################
        self.gpsr_location = "gpsr_location"
        self.init_place = "entrance"
        self.task_counter = 0 
        self.generated_code = ""
        subscriber_sonar= rospy.Subscriber("/sonar/front", Range, self.callback_sonar)
        self.clearCostmapServiceClient = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        ##################### ROS CALLBACK VARIABLES #####################
        self.sonar = False

    def on_enter_INIT(self):
        local_generator_init()
        generate_utils.load_code_gen_config() 
        self.tm.initialize_pepper()
        self.tm.turn_camera("bottom_camera","custom",1,15)
        # CAMBIAR EN LA COMPETENCIA, EL ROBOT NO INICIA EN GPSR_LOCATION DEBE NAVEGAR ALLA TODO
        self.tm.current_place = "house_door"
        self.tm.set_current_place(self.init_place)
        print("initial position: ",self.init_place)
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.talk("I am going to do the  "+ self.task_name + " task","English")
        self.tm.talk("Please open the door to start","English",wait=False)
        start_time = rospy.get_time()
        while not self.sonar and rospy.get_time() - start_time < 20 :
            print("Waiting for door to open ",self.sonar )
            time.sleep(0.2)
        self.clearCostmapServiceClient()
        self.tm.talk("The door has been opened","English")
        self.beggining()

    def on_enter_GPSR(self):
        print(self.consoleFormatter.format("GPSR", "HEADER"))
        self.tm.look_for_object("")
        self.tm.talk("Hello guest, please tell me what you want me to do, I will try to execute the task you give me. Please talk loud and say the task once. You can talk to me when my eyes are blue: ","English")
        task = self.tm.speech2text_srv(0)
        self.generated_code = ""
        code_gen_thread = threading.Thread(target=self.code_gen_t,args=[task])
        code_gen_thread.start()
        self.tm.talk(f"Your command is {task}. If that is correct, please touch my head. If not, please wait until my eyes are blue again ","English", wait=False)
        print("antes de touch")
        correct = self.tm.wait_for_head_touch(message="", message_interval=100, timeout=13)
        print("despues de touch")
        while not correct:
            self.tm.talk("I am sorry, please repeat your command","English", wait=True)
            task = self.tm.speech2text_srv(0)
            code_gen_thread = threading.Thread(target=self.code_gen_t,args=[task])
            code_gen_thread.start()
            self.tm.talk(f"Your command is {task}. If that is correct, please touch my head","English", wait=True)
            correct = self.tm.wait_for_head_touch(message="", message_interval=13, timeout=13)
        print("processing your request")
        while self.generated_code == "":
            rospy.sleep(0.1)
        if self.generated_code != "impossible":
            try:
                exec(self.generated_code)
            except:
                self.tm.talk("I am sorry but i cannot complete this task! Please give me another task.", "English")

        print(f"Task: {task}")
        self.GPSR_done()

    def on_enter_GO2GPSR(self):
        if self.task_counter == 3:
            self.tm.talk("GPSR task completed succesfully", "English", wait=True)
            os._exit(os.EX_OK)

        print(self.consoleFormatter.format("GO2GPSR", "HEADER"))
        self.tm.talk("I am going to the GPSR location","English", wait=False)
        self.tm.go_to_place(self.gpsr_location,graph=1)
        self.go_to_gpsr()

    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.go_to_pose("default_head")
        self.tm.setRPosture_srv("stand")
        self.tm.setMoveHead_srv.call("up")
        self.tm.talk("Waiting for guests","English")
        self.tm.look_for_object("person")
        self.tm.wait_for_object(-1)
        self.person_arrived()


    def code_gen_t(self, task):
        
        print(self.consoleFormatter.format("GEN CODE THREAD", "HEADER"))
        
        contador = 0
        code = ""
        while contador<1:
            code = local_generator(task).replace("`","").replace("python","")
            pattern = r'self\.tm\.go_to_place\((.*?)\)'
            code = re.sub(pattern, r'self.tm.go_to_place(\1, lower_arms=False)', code)
            print(code)
            if not "I am sorry but I cannot complete this task" in code:
                print("\nIt is possible to execute the request")
                if self.is_valid_syntax(code):
                    self.generated_code = code
                    self.task_counter += 1
                    contador = 5
            contador += 1
        if contador==1:
            self.generated_code = "impossible"
            self.tm.talk("I cannot the following task: " + task,"English")

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)


    def callback_sonar(self,data):
        if(data.range > 0.5):
            self.sonar = True
        else:
            self.sonar = False


    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
    def is_valid_syntax(self, code):
        try:
            ast.parse(code)
            return True
        except SyntaxError:
            return False
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = GPSR()
    sm.run()
    rospy.spin()
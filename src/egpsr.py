#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os
import re
import math
import ast
from code_generation import ls_generate as gen
from code_generation import generate_utils 
from code_generation.database.models import Model
class EGPSR(object):
    def __init__(self):
        self.gen = gen.LongStringGenerator()
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
        self.task_name = "EGPSR"
        states = ['INIT', 'SEARCH4GUEST', 'EGPSR']
        self.tm = tm(perception = True,speech=True,manipulation=True, navigation=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'EGPSR', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'SEARCH4GUEST'},
            {'trigger': 'person_arrived', 'source': 'SEARCH4GUEST', 'dest': 'EGPSR'},
            {'trigger': 'again', 'source': 'SEARCH4GUEST', 'dest': 'SEARCH4GUEST'},
            {'trigger': 'GPSR_done', 'source': 'EGPSR', 'dest': 'SEARCH4GUEST'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='EGPSR')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        ############################# GLOBAL VARIABLES #############################
        self.gpsr_location = "house_door"
        self.init_place = "house_door"
        self.places_names = ["kitchen","living_room","main room","dining","bedroom"]
        self.head_angles = [60,-60]
        

    def on_enter_INIT(self):
        self.tm.initialize_pepper()
        self.tm.turn_camera("bottom_camera","custom",1,15)
        # CAMBIAR EN LA COMPETENCIA, EL ROBOT NO INICIA EN GPSR_LOCATION DEBE NAVEGAR ALLA TODO
        self.tm.current_place = "house_door"
        self.tm.set_current_place(self.init_place)
        print("initial position: ",self.init_place)
        self.tm.turn_camera("depth_camera","custom",1,15)
        self.tm.toggle_filter_by_distance(True,2,["person"])
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.talk("I am going to do the  "+ self.task_name + " task","English")
        self.tm.talk("I am going to the EGPSR location","English")
        print("gpsr_location:", self.gpsr_location )
        print(self.tm.last_place)
        if self.tm.last_place != self.gpsr_location:
            self.tm.go_to_place(self.gpsr_location)
        self.beggining()
        
        

    def on_enter_EGPSR(self):
        print(self.consoleFormatter.format("EGPSR", "HEADER"))
        self.tm.look_for_object("")
        self.tm.talk("Hello guest, please tell me what you want me to do, I will try to execute the task you give me. Please talk loud and say the task once. You can talk to me when my eyes are blue: ","English")
        task = self.tm.speech2text_srv(0)
        self.generated_code = ""
        code_gen_thread = threading.Thread(target=self.code_gen_t,args=[task])
        code_gen_thread.start()
        self.tm.talk(f"Your command is {task}","English", wait=True)
        self.tm.talk(f"If that is correct, please touch my head. If not, please wait until my eyes are blue again ","English", wait=False)
        correct = self.tm.wait_for_head_touch(message="", message_interval=100, timeout=13)
        while not correct:
            self.tm.talk("I am sorry, please repeat your command","English", wait=True)
            task = self.tm.speech2text_srv(0)
            code_gen_thread = threading.Thread(target=self.code_gen_t,args=[task])
            code_gen_thread.start()
            self.tm.talk(f"Your command is {task}. If that is correct, please touch my head","English", wait=True)
            correct = self.tm.wait_for_head_touch(message="", message_interval=13, timeout=13)
        
        self.tm.talk("Processing your request")
        while self.generated_code == "":
            rospy.sleep(0.1)
        if self.generated_code != "impossible":
            exec(self.generated_code)
        print(f"Task: {task}")
        self.GPSR_done()
        
        

    def on_enter_SEARCH4GUEST(self):
        print(self.consoleFormatter.format("SEARCH4GUEST", "HEADER"))
        self.tm.go_to_pose("default_head")
        self.tm.go_to_pose("standard")
        self.current_place = self.places_names[0]
        self.tm.talk(f"I'm going to search a guest in {self.current_place}","English")
        self.tm.go_to_place(self.current_place)
        self.tm.look_for_object("person")
        for angle in self.head_angles:
            self.tm.set_angles_srv(["HeadYaw"], [math.radians(angle)],0.05)
            person_response = self.tm.img_description(
                    "Observe the person in front of you and respond only with True if the person is raising their hand. Otherwise, respond False. Here is an example output: False",
                    "front_camera"
                    )
            is_raisin = True if person_response["message"] == "True" else False
        self.places_names.pop(0)
        if is_raisin:
            self.person_arrived()
        else:
            self.tm.talk(f"I didn't find a guest raising their hand in {self.current_place}","English")
            self.again()
        
    
    def code_gen_t(self, task):
        
        print(self.consoleFormatter.format("GEN CODE THREAD", "HEADER"))
        
        contador = 0
        code = ""
        while contador<1:
            code = self.gen.generate_code(task, Model.GPT4).replace("`","").replace("python","")
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
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

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
    sm = EGPSR()
    sm.run()
    rospy.spin()

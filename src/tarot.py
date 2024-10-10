#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import threading
import rospy
import os
import json
import random

from std_srvs.srv import SetBool
from robot_toolkit_msgs.msg import animation_msg, leds_parameters_msg
from robot_toolkit_msgs.srv import set_stiffnesses_srv, set_stiffnesses_srvRequest

from perception_msgs.srv import read_qr_srvRequest, read_qr_srv

class Tarot(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "Tarot"
        states = ['INIT', 'TALK', 'WAIT4GUEST', 'Q_A', 'MOVE_CARD', 'ASK4QR', 'ANSWER_TAROT', 'GIVE_CARD']
        self.tm = tm(perception = False,speech=True, pytoolkit=True, manipulation=True)
        self.tm.initialize_node(self.task_name)
        
        self.signo = ""

        transitions = [
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'WAIT4GUEST'},
            {'trigger': 'person_found', 'source': 'WAIT4GUEST', 'dest': 'Q_A'},
            {'trigger': 'q_answered', 'source': 'Q_A', 'dest': 'MOVE_CARD'},
            {'trigger': 'card_in_place', 'source': 'MOVE_CARD', 'dest': 'ASK4QR'},
            {'trigger': 'card_recognized', 'source': 'ASK4QR', 'dest': 'GIVE_CARD'},
            {'trigger': 'restart', 'source': 'GIVE_CARD', 'dest': 'WAIT4GUEST'}
        ]
        
        self.qr_node_service = rospy.ServiceProxy("/vision_utilities/read_qr_srv", read_qr_srv)
        self.tm.initialize_pepper()
        self.motion_set_stiffnesses_client = rospy.ServiceProxy("pytoolkit/ALMotion/set_stiffnesses_srv", set_stiffnesses_srv)
        req_stiffnesses = set_stiffnesses_srvRequest()
        req_stiffnesses.names = "Body"
        req_stiffnesses.stiffnesses = 1
        self.motion_set_stiffnesses_client(req_stiffnesses)
        self.set_arms_security = rospy.ServiceProxy("pytoolkit/ALMotion/set_arms_security_srv", SetBool)
        self.set_arms_security(True)
        
        
        ############################# GLOBAL VARIABLES #############################
        self.cartas_pose = ["der4", "der3", "izq2", "izq1"]
        
        self.poses = {
            "izq1": ["tarot_l_initial", "tarot_l_1_2", "tarot_l_final"],
            "izq2": ["tarot_l_initial", "tarot_l_2_2", "tarot_l_final"],
            "der3": ["tarot_r_initial", "tarot_r_3_2", "tarot_r_final"],
            "der4": ["tarot_r_initial", "tarot_r_4_2", "tarot_r_final"]
            }
        
        self.random_card_text = [
            "En el aire flota un secreto... Recibe tu carta y descubre tu destino.",
            "Un destino aguardando ser revelado... ¿te atreves a tomar tu carta?",
            "La magia está en tus manos... recoge la carta y desvela tu futuro.",
            "Como un mago en la noche, te invito a que recibas tu carta misteriosa.",
            "Los astros han alineado este momento... recibe tu carta y descúbrelo.",
            "En la penumbra, una carta te llama... ¿la tomarás?",
            "Siente la energía a tu alrededor... recibe tu carta y sorpréndete.",
            "Con un susurro del universo, recibe la carta que te hable.",
            "Un enigma aguarda... ¿ será esta la carta que revelará tu camino?",
            "Cada carta es un paso hacia lo desconocido... ¡recibe la tuya con valentía!"
        ]
            
        self.diccionario_signos= {
            "Aries": 0, 
            "Tauro": 1, 
            "Géminis": 2, 
            "Cáncer": 3, 
            "Leo": 4, 
            "Virgo": 5, 
            "Libra": 6, 
            "Escorpio": 7, 
            "Sagitario": 8, 
            "Capricornio": 9, 
            "Acuario": 10, 
            "Piscis": 11
        }


        self.diccionario_cartas = {
            "0": "El Loco", 
            "1": "El Mago", 
            "2": "La Sacerdotisa", 
            "3": "La Emperatriz", 
            "4": "El Emperador", 
            "5": "El Hierofante", 
            "6": "El Enamorado", 
            "7": "El Carro", 
            "8": "La Fuerza", 
            "9": "El Ermitaño", 
            "10": "La Rueda de la Fortuna", 
            "11": "La Justicia", 
            "12": "El Colgado"
        }        
        with open('src/task_utilities/src/another.json', 'r') as file:
            self.cartas = json.load(file)
            
            
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='INIT')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()


    def on_enter_INIT(self):
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        
                    
            
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.go_to_pose("tarotist",velocity=0.1)
        #Apagar el stiffness para no sobrecalentar al robot
        req_stiffnesses = set_stiffnesses_srvRequest()
        req_stiffnesses.names = "LArm"
        req_stiffnesses.stiffnesses = 0
        self.motion_set_stiffnesses_client(req_stiffnesses)
        
        req_stiffnesses.names = "RArm"
        req_stiffnesses.stiffnesses = 0
        self.motion_set_stiffnesses_client(req_stiffnesses)
        
        self.tm.wait_for_arm_touch()
        self.person_found()

    def on_enter_Q_A(self):
        print(self.consoleFormatter.format("Q_A", "HEADER"))
        self.tm.talk(text="Por favor dime tu nombre", language="Spanish", wait=True, animated=False)
        nombre = self.tm.speech2text_srv(0,lang="esp")
        self.tm.talk(text="Por favor dime tu fecha de nacimiento", language="Spanish", wait=True, animated=False)
        fecha = self.tm.speech2text_srv(0,lang="esp")
        self.signo = self.tm.answer_question(f"Cual es el signo zodiacal de una persona con esta fecha de nacimiento? {fecha}. Responde en una sola palabra")
        self.q_answered()

    def on_enter_MOVE_CARD(self): 
        print(self.consoleFormatter.format("MOVE_CARD", "HEADER"))
        
        self.tm.talk("Cierra los ojos y siente la energía, con esa energía eligiré la carta para tí", "Spanish", wait=False)
        
        req_stiffnesses = set_stiffnesses_srvRequest()
        req_stiffnesses.names = "LArm"
        req_stiffnesses.stiffnesses = 1
        self.motion_set_stiffnesses_client(req_stiffnesses)
        
        req_stiffnesses.names = "RArm"
        req_stiffnesses.stiffnesses = 1
        self.motion_set_stiffnesses_client(req_stiffnesses)
        
        card = random.choice(self.cartas_pose)
        if card in self.poses:
            self.tm.go_to_pose(self.poses[card][0])
            self.tm.go_to_pose(self.poses[card][1])
            self.tm.go_to_pose(self.poses[card][2])

            txt = random.choice(self.random_card_text)
            self.tm.talk(txt, "Spanish", wait=False)
            
            self.tm.go_to_pose(self.poses[card][2])
            self.tm.go_to_pose(self.poses[card][1])
            self.tm.go_to_pose(self.poses[card][0])
            
            self.tm.go_to_pose("tarotist")

            req_stiffnesses = set_stiffnesses_srvRequest()
            req_stiffnesses.names = "LArm"
            req_stiffnesses.stiffnesses = 0
            self.motion_set_stiffnesses_client(req_stiffnesses)
            
            req_stiffnesses.names = "RArm"
            req_stiffnesses.stiffnesses = 0
            self.motion_set_stiffnesses_client(req_stiffnesses)
            
        self.card_in_place()

    def on_enter_ASK4QR(self): 
        print(self.consoleFormatter.format("ASK4QR", "HEADER"))
        self.tm.talk("Por favor toma la carta que te acerque y muestrame el codigo QR",language="Spanish", wait=True, animated=False)
        read_qr_message = read_qr_srvRequest()
        read_qr_message.timeout = 20
        card_number = int(self.qr_node_service(read_qr_message).text)
        print(card_number)
        self.signo ="Acuario"
        signo_numero = self.diccionario_signos[self.signo]
        respuesta = random.choice(self.cartas[card_number][signo_numero])
        self.tm.talk(respuesta,language="Spanish",wait=True)
        self.card_recognized()

    def on_enter_GIVE_CARD(self): 
        print(self.consoleFormatter.format("GIVE_CARD", "HEADER"))
        self.tm.talk(text="Gracias por participar, toma una tarjeta y un sticker de la mesa", language="Spanish")
        self.restart()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.beggining()

    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = Tarot()
    sm.run()
    rospy.spin()
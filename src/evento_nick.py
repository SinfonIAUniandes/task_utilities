#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from datetime import datetime
import ConsoleFormatter
import threading
import rospy
import os
from robot_toolkit_msgs.msg import speech_recognition_status_msg, animation_msg, motion_tools_msg, leds_parameters_msg
from robot_toolkit_msgs.srv import tablet_service_srv,  set_open_close_hand_srv, set_open_close_hand_srvRequest, motion_tools_srv, battery_service_srv, set_output_volume_srv, tablet_service_srvRequest

class Evento(object):
    def __init__(self):
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "Event"
        self.hearing = True
        self.is_done = False
        self.hey_pepper=False
        self.already_asereje = False 
        self.already_dance = False 
        self.haciendo_animacion = False
        states = ['INIT', 'TALK']
        self.tm = tm(perception = True,speech=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'TALK', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'TALK'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='TALK')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        ############################# GLOBAL VARIABLES #############################

    def on_enter_INIT(self):
        self.tm.talk("Iniciando modo de Demostración","Spanish")
        self.tm.initialize_pepper()
        self.motion_tools_service()
        self.enable_breathing_service()
        print(self.consoleFormatter.format("Waiting for pytoolkit/ALTabletService/show_picture_srv...", "WARNING"))
        rospy.wait_for_service('pytoolkit/ALTabletService/show_picture_srv')
        self.show_picture_proxy = rospy.ServiceProxy('pytoolkit/ALTabletService/show_picture_srv', battery_service_srv)
        self.animationPublisher = rospy.Publisher('/animations', animation_msg, queue_size=10)
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.beggining()

    def on_enter_TALK(self):
        print(self.consoleFormatter.format("TALK", "HEADER"))
        anim_msg = self.gen_anim_msg("Gestures/BowShort_3")
        self.animationPublisher.publish(anim_msg)
        self.hearing = False
        self.tm.talk("Bienvenidos, soy Nova, es un gusto conocerlos","Spanish",animated=False)
        self.tm.talk("Veo que estan buscando una pista, esta bien les dare la pista que buscan, pero deberan ayudarme a buscar mi objeto favorito. Cuando crean que lo tienen muestrenmelo y toquen mi cabeza","Spanish",animated=True)
        rospy.sleep(0.9)
        self.tm.start_tracker_proxy()
        self.hearing = True
        self.tm.wait_for_head_touch(message="",timeout=-1)
        anim_msg = self.gen_anim_msg("Waiting/TakePicture_1")
        self.animationPublisher.publish(anim_msg)
        rospy.sleep(2)
        self.show_picture_proxy()
        answer = self.tm.img_description("Answer about the person in the front and center of the picture: Are they holding a ball? Answer only True or False",camera_name="both")
        while not "true" in answer.lower():
            self.tm.talk("No, ese no es mi objeto favorito, el objeto que busco es muy importante para la Robocap. Por favor muestrame otro objeto y toca mi cabeza","Spanish",animated=True)
            self.tm.wait_for_head_touch(message="",timeout=-1)
            anim_msg = self.gen_anim_msg("Waiting/TakePicture_1")
            self.animationPublisher.publish(anim_msg)
            rospy.sleep(2)
            self.show_picture_proxy()
            answer = self.tm.img_description("Answer about the person in the front and center of the picture: Are they holding a ball? Answer only True or False",camera_name="both")
        self.tm.talk("Ese es! Ese es mi objeto favorito una pelotita! En la Robocup mi amigo NAO juega futbol con ellas en la Robocap y es muy bueno, espero verlo este anio en Aindjoven","Spanish",animated=True)
        self.tm.talk("Esa es la pista, la pista es Eindhoven","Spanish",animated=True)
        os._exit(os.EX_OK)
        self.start()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
    def enable_breathing_service(self):
        """
        Enables the breathing animations of the robot.
        """
        request = set_open_close_hand_srvRequest()
        request.hand = "All"
        request.state = "True"
        print("Waiting for breathing service")
        rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv")
        try:
            toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
            toggle_breathing_proxy(request)
            print("Breathing service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
        request = set_open_close_hand_srvRequest()
        request.hand = "Head"
        request.state = "False"
        print("Waiting for breathing service")
        rospy.wait_for_service("/pytoolkit/ALMotion/toggle_breathing_srv")
        try:
            toggle_breathing_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/toggle_breathing_srv", set_open_close_hand_srv)
            toggle_breathing_proxy(request)
            print("Breathing service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
    def motion_tools_service(self):
        """
        Enables the motion Tools service from the toolkit of the robot.
        """
        request = motion_tools_msg()
        request.command = "enable_all"
        print("Waiting for motion tools service")
        rospy.wait_for_service('/robot_toolkit/motion_tools_srv')
        try:
            motion = rospy.ServiceProxy('/robot_toolkit/motion_tools_srv', motion_tools_srv)
            motion(request)
            print("Motion tools service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
            
    def gen_anim_msg(self, animation):
        anim_msg = animation_msg()
        anim_msg.family = "animations"
        anim_msg.animation_name = animation
        return anim_msg
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = Evento()
    sm.run()
    rospy.spin()

#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from datetime import datetime
import ConsoleFormatter
import threading
import pandas as pd
import rospy
import random
import os
import csv
from speech_msgs.srv import speech2text_srv
from robot_toolkit_msgs.msg import speech_recognition_status_msg, animation_msg, motion_tools_msg, leds_parameters_msg
from robot_toolkit_msgs.srv import tablet_service_srv,  set_open_close_hand_srv, set_open_close_hand_srvRequest, motion_tools_srv, battery_service_srv, set_output_volume_srv, tablet_service_srvRequest

class Evento(object):
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
        self.task_name = "Event"
        self.hearing = False
        self.is_done = False
        self.hey_pepper=False
        self.already_asereje = False 
        self.already_dance = False 
        self.haciendo_animacion = False
        states = ['INIT', 'WAIT4GUEST', 'TALK']
        self.tm = tm(perception = True,speech=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'TALK', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'WAIT4GUEST'},
            {'trigger': 'person_arrived', 'source': 'WAIT4GUEST', 'dest': 'TALK'},
            {'trigger': 'TALK_done', 'source': 'TALK', 'dest': 'WAIT4GUEST'}
        ]
        
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='TALK')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        ############################# GLOBAL VARIABLES #############################

    def on_enter_INIT(self):
        self.tm.talk("Iniciando modo de Demostración","Spanish")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        subscriber = rospy.Subscriber("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,self.callback_hot_word)
        self.motion_tools_service()
        self.enable_breathing_service()
        self.enable_hot_word_service()
        print(self.consoleFormatter.format("Waiting for pytoolkit/ALTabletService/show_picture_srv...", "WARNING"))
        rospy.wait_for_service('pytoolkit/ALTabletService/show_picture_srv')
        print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/show_image_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.show_image_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_image_srv",tablet_service_srv)
        self.show_picture_proxy = rospy.ServiceProxy('pytoolkit/ALTabletService/show_picture_srv', battery_service_srv)
        self.stop_tracker_srv = rospy.ServiceProxy("/pytoolkit/ALTracker/stop_tracker_srv", battery_service_srv)
        self.play_dance_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/play_dance_srv", set_output_volume_srv)
        self.animationPublisher = rospy.Publisher('/animations', animation_msg, queue_size=10)
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.show_web_page_proxy = rospy.ServiceProxy(
            "/pytoolkit/ALTabletService/show_web_view_srv", tablet_service_srv
        )
        request = tablet_service_srvRequest()
        request.url = "http://192.168.0.229:8000/" 
        self.show_web_page_proxy(request)
        self.tm.publish_filtered_image(filter_name="qr",camera_name="front_camera")
        self.facts_df = pd.read_csv('/home/sinfonia/sinfonia_ws/src/task_utilities/src/events/2024/ZEISS_event/facts.csv')
        self.beggining()

    def on_enter_TALK(self):
        print(self.consoleFormatter.format("TALK", "HEADER"))
        anim_msg = self.gen_anim_msg("Gestures/BowShort_3")
        self.animationPublisher.publish(anim_msg)
        self.tm.talk("Bienvenido, soy Nova, es un gusto conocerte","Spanish")
        rospy.sleep(2)
        self.enable_tracker_service()
        self.tm.talk("Oprime Hey Nova cuando quieras decirme algo, y chao cuando no quieras seguir hablando","Spanish",animated=True)
        while not self.is_done:
            if self.hey_pepper:
                #self.tm.show_words_proxy()
                self.hey_pepper_function()
                self.hey_pepper=False
            rospy.sleep(0.1)
        self.tm.talk("Adios","Spanish",animated=True)
        self.TALK_done()

    def on_enter_WAIT4GUEST(self):
        self.stop_tracker_srv()
        #self.tm.show_words_proxy()
        self.is_done=False
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.setRPosture_srv("stand")
        self.tm.setMoveHead_srv.call("up")
        self.tm.talk("Esperando personas","Spanish",animated=True)
        self.tm.look_for_object("person")
        self.tm.wait_for_object(-1)
        self.person_arrived()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
    def enable_tracker_service(self):
        """
        Enables face tracking from the toolkit of the robot.
        """
        print("Waiting for tracking service")
        rospy.wait_for_service('/pytoolkit/ALTracker/start_tracker_srv')
        try:
            start_tracker_srv = rospy.ServiceProxy("/pytoolkit/ALTracker/start_tracker_srv", battery_service_srv)
            start_tracker_srv()
            print("tracking service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
    def enable_hot_word_service(self):
        """
        Enables hotwords detection from the toolkit of the robot.
        """
        print("Waiting for hot word service")
        rospy.wait_for_service('/pytoolkit/ALSpeechRecognition/set_hot_word_language_srv')
        try:
            hot_word_language_srv = rospy.ServiceProxy("/pytoolkit/ALSpeechRecognition/set_hot_word_language_srv", tablet_service_srv)
            hot_word_language_srv("Spanish")
            self.set_hot_words()
            print("Hot word service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
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
            
    def get_random_fact(self):
        random_index = random.randint(0, len(self.facts_df) - 1)
        random_fact = self.facts_df.iloc[[random_index]]
        return random_fact['fact_content'].values[0], random_fact['fact_id'].values[0]
    
    def hey_pepper_function(self):
        #self.tm.hot_word([])
        self.tm.talk("Dímelo manzana","Spanish",animated=True)
        text = self.tm.speech2text_srv(7, "esp")
        anim_msg = self.gen_anim_msg("Waiting/Think_3")
        self.animationPublisher.publish(anim_msg)
        if not ("None" in text):
            request = f"""La persona dijo: {text}."""
            if "hoy" in text or "día" in text:
                fecha_actual = datetime.now()
                dia = fecha_actual.day
                mes = fecha_actual.month
                m = ['nan','enero','febrero','marzo','abril','mayo','junio','julio','agosto','septiembre','octubre','noviembre','diciembre']
                answer="Hoy es: {} de {}".format(dia, m[mes])
            elif "hora" in text or "horas" in text:
                fecha_actual = datetime.now()
                hora = fecha_actual.hour
                minuto = fecha_actual.minute
                answer="Son las: {} y {} minutos".format(hora, minuto)
            else:
                answer=self.tm.answer_question(request) 
            self.tm.talk(answer,"Spanish",animated=True)
        else:
            self.tm.talk("Disculpa, no te entendi, puedes hablar cuando mis ojos esten azules. Por favor habla mas lento","Spanish",animated=True)
        self.set_hot_words()

    def set_hot_words(self):
        if self.hearing:
            self.tm.hot_word(["chao","detente","hey nova","baile","asereje","pose","musculos" ,"besos","foto","guitarra","cumpleaños","corazon","llama","helicoptero","zombi","carro","gracias"],thresholds=[0.45, 0.4, 0.398, 0.43, 0.43, 0.39, 0.4, 0.4, 0.5, 0.4, 0.4, 0.4, 0.41, 0.4, 0.4, 0.43, 0.4])

    def gen_anim_msg(self, animation):
        anim_msg = animation_msg()
        anim_msg.family = "animations"
        anim_msg.animation_name = animation
        return anim_msg
    
    def dance_threadf(self, number):
        #Se ponen los bailes en un thread porque estancan la ejecucion y no permiten checkear otras hotwords, como detente
        self.haciendo_animacion = True
        self.play_dance_srv(number)
        self.haciendo_animacion = False
    
    def setLedsColor(self, r,g,b):
        """
        Function for setting the colors of the eyes of the robot.
        Args:
        r,g,b numbers
            r for red
            g for green
            b for blue
        """
        ledsPublisher = rospy.Publisher('/leds', leds_parameters_msg, queue_size=10)
        ledsMessage = leds_parameters_msg()
        ledsMessage.name = "FaceLeds"
        ledsMessage.red = r
        ledsMessage.green = g
        ledsMessage.blue = b
        ledsMessage.time = 0
        ledsPublisher.publish(ledsMessage)  #Inicio(aguamarina), Pepper esta ALSpeechRecognitionStatusPublisherlista para escuchar
    
    
    def disable_tracker_service(self):
        """
        Disables face tracking from the toolkit of the robot.
        """
        print("Waiting for disable tracking service")
        rospy.wait_for_service('/pytoolkit/ALTracker/stop_tracker_srv')
        try:
            stop_tracker_srv = rospy.ServiceProxy("/pytoolkit/ALTracker/stop_tracker_srv", battery_service_srv)
            stop_tracker_srv()
            print("disable-tracking service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
    def callback_hot_word(self,data):
        word = data.status
        print(word, "listened")
        if word=="detente":
            self.tm.setRPosture_srv("stand")
            self.haciendo_animacion = False
            request = tablet_service_srvRequest()
            request.url = "http://192.168.0.229:8000/" 
            self.show_web_page_proxy(request)
            
        if not self.haciendo_animacion:
            self.haciendo_animacion = True
            if word == "chao":
                self.is_done = True
                self.already_dance = False
                self.already_asereje = False
                
            elif word == "Registro":
                self.tm.show_image("https://raw.githubusercontent.com/SinfonIAPepperTeam/Public-Images/main/QR-registro.jpg")
                
            elif word == "QR":
                self.tm.show_topic("/perception_utilities/filtered_image")
                self.tm.talk("""Voy a registrar tu ingreso al evento.
                                    """, "Spanish", wait=False, animated=True)
                rospy.sleep(1.5)
                self.disable_tracker_service()
                self.setLedsColor(102,102,255)
                self.qr_code = self.tm.qr_read(12)
                self.enable_tracker_service()
                #Set led color to white
                self.setLedsColor(224,224,224)
                request = tablet_service_srvRequest()
                request.url = "http://192.168.0.229:8000/" 
                self.show_web_page_proxy(request)# If the person does not show a QR code, Nova tells the person to show it or to register
                if self.qr_code == "":
                    print(self.consoleFormatter.format("Nova: I could not read any QR code, I will tell the person to register", "HEADER"))
                    self.tm.talk("""No he podido leer el código, por favor inténtalo de nuevo.
                                    """, "Spanish", wait=False, animated=True)
                else:
                    # Play greeting animation
                    anim_msg = self.gen_anim_msg("Gestures/BowShort_3")
                    self.animationPublisher.publish(anim_msg)

                    # Check if the scanned QR code matches any VIP guest's name
                    with open('/home/sinfonia/sinfonia_ws/src/task_utilities/src/events/2024/ZEISS_event/guests_list.csv', 'r') as file:
                        reader = csv.DictReader(file)
                        for row in reader:
                            if row['guest_name'] == self.qr_code:
                                # Greet the special customer with custom message
                                self.tm.talk(row['custom_message'], "Spanish", wait=False)
                                break
                        else:
                            # Greeting the person with a default message if not a VIP guest
                            self.tm.talk("Bienvenido al evento " + self.qr_code + "Espero que lo disfrutes mucho. Fue un placer atenderte!", "Spanish", wait=False)
                    
                    # Save the attendance to thehttps://raw.githubusercontent.com/SinfonIAUniandes/task_utilities/dev/src/static/QR-registro.jpg?token=GHSAT0AAAAAACRUXAJQA7DMHMONYDMTNQFSZSXYM3A CSV file
                    with open('/home/sinfonia/sinfonia_ws/src/task_utilities/src/events/2024/ZEISS_event/attended_guests.csv', 'a', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow([self.qr_code, 'True'])
            elif word == "heynova":
                self.hey_pepper = True
            elif word == "fact":
                fact_content, fact_id = self.get_random_fact()
                self.tm.talk(fact_content, "Spanish", wait=False, animated=True)   
            elif word == "guitarra":
                anim_msg = self.gen_anim_msg("Waiting/AirGuitar_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "besos":
                anim_msg = self.gen_anim_msg("Gestures/Kisses_1")
                self.animationPublisher.publish(anim_msg)
                self.tm.talk("Muah!","Spanish")
            elif word == "baile":
                if not self.already_dance:
                    dance_thread = threading.Thread(target=self.dance_threadf,args=[1])
                    dance_thread.start()
                    self.already_dance = True
            elif word == "asereje":
                if not self.already_asereje:
                    dance_thread = threading.Thread(target=self.dance_threadf,args=[3])
                    dance_thread.start()
                    self.already_asereje = True
            elif word == "pose":
                anim_msg = self.gen_anim_msg("Gestures/ShowSky_8")
                self.animationPublisher.publish(anim_msg)
            elif word == "foto":
                anim_msg = self.gen_anim_msg("Waiting/TakePicture_1")
                self.animationPublisher.publish(anim_msg)
                rospy.sleep(2)
                self.show_picture_proxy()
            elif word == "cumpleaños":
                anim_msg = self.gen_anim_msg("Waiting/HappyBirthday_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "corazon":
                anim_msg = self.gen_anim_msg("Waiting/LoveYou_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "llama":
                anim_msg = self.gen_anim_msg("Waiting/CallSomeone_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "helicoptero":
                anim_msg = self.gen_anim_msg("Waiting/Helicopter_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "zombi":
                anim_msg = self.gen_anim_msg("Waiting/Zombie_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "carro":
                anim_msg = self.gen_anim_msg("Waiting/DriveCar_1")
                self.animationPublisher.publish(anim_msg)
            elif word == "musculos":
                anim_msg = self.gen_anim_msg("Waiting/ShowMuscles_3")
                self.animationPublisher.publish(anim_msg)
            elif word == "gracias":
                anim_msg = self.gen_anim_msg("Gestures/BowShort_3")
                self.animationPublisher.publish(anim_msg)
                self.tm.talk("Con mucho gusto","Spanish")
            self.haciendo_animacion = False
    
# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = Evento()
    sm.run()
    rospy.spin()

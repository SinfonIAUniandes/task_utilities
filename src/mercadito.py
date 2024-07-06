#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from std_msgs.msg import Bool, String
import ConsoleFormatter
import time
import threading
import rospy
import os

from robot_toolkit_msgs.msg import speech_recognition_status_msg
class MERCADITO(object):
    def __init__(self):

        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "MERCADITO"
        self.is_done = False
        self.hey_pepper = False
        self.language = "English"
        states = ["MERCADITO", "INIT", "FOLLOW_YOU", "FINISH", "MERCADITO_DONE"]
        self.tm = tm(
            perception=True,
            speech=True,
            manipulation=True,
            navigation=False,
            pytoolkit=True,
        )
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {"trigger": "start", "source": "MERCADITO", "dest": "INIT"},
            {"trigger": "beggining", "source": "INIT", "dest": "FOLLOW_YOU"},
            {"trigger": "market_ready", "source": "FOLLOW_YOU", "dest": "FINISH"},
            {"trigger": "finish", "source": "FINISH", "dest": "MERCADITO_DONE"},
        ]

        # Crear la máquina de estados
        self.machine = Machine(
            model=self, states=states, transitions=transitions, initial="MERCADITO"
        )
        subscriber = rospy.Subscriber(
            "/pytoolkit/ALSpeechRecognition/status",
            speech_recognition_status_msg,
            self.callback_hot_word,
        )

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ############################# STATES #############################

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.initialize_pepper()
        self.tm.show_topic("/perception_utilities/yolo_publisher")
        self.tm.talk("I am going to do the shopping task", "English")

        print(
            self.consoleFormatter.format(
                "Inicializacion del task: " + self.task_name, "HEADER"
            )
        )
        print("levantar brazos")
        self.tm.go_to_pose("basket", 0.1)
        self.tm.go_to_pose("open_both_hands", 0.1)

        if self.language == "Spanish":
            self.tm.talk(
                "Hola soy nova, te voy a ayudar con tus compras hoy, cuando estes listo pon la canasta en mis manos",
                self.language,
            )
        else:
            self.tm.talk(
                "Hello I will help you with your shopping today, when you are ready put your basket in my hands",
                "English",
            )

        self.beggining()

    def on_enter_FOLLOW_YOU(self):
        print(self.consoleFormatter.format("FOLLOW_YOU", "HEADER"))
        if self.language == "Spanish":

            self.tm.talk(
                "Cuando tengas una pregunta sobre la comida por favor di Hey nova y espera a que yo responda. Si quieres que pare y te entregue la canasta por favor di detente nova",
                "Spanish",
                wait=False,
            )
        else:
            self.tm.talk(
                "When you have a question regarding your food please say Hey nova. If you want me to stop and hand you the basket please say Stop. Please talk slow, clear and loud.",
                "English",
            )
        time.sleep(1)
        if self.language == "Spanish":
            self.tm.hot_word(
                ["hey nova", "oye nova", "stop", "detente nova"],
                thresholds=[0.4, 0.3, 0.5, 0.3],
            )
        else:
            self.tm.hot_word(["hey nova", "stop"], thresholds=[0.45, 0.5])
        self.tm.get_labels(True)
        self.tm.follow_you(True)
        while not self.is_done:
            if self.hey_pepper:
                print(self.consoleFormatter.format("Hey Pepper detected ", "WARNING"))
                self.hey_pepper_function()
                self.hey_pepper = False
            time.sleep(0.1)
        print(self.consoleFormatter.format("Stop detected ", "WARNING"))
        self.tm.talk(
            "I am pleased to serve you in your shopping today, you may grab your basket now. Have a nice day"
        )
        self.market_ready()

    def on_enter_FININSH(self):
        print(self.consoleFormatter.format("FINISH", "HEADER"))
        self.tm.follow_you(False)
        self.finish()

    def on_enter_MERCADITO_DONE(self):
        print(self.consoleFormatter.format("MERCADITO_DONE", "HEADER"))
        if self.language == "Spanish":
            self.tm.talk("He terminado la tarea de mercado", "Spanish")
        else:
            self.tm.talk("I have finished the " + self.task_name + " task", "English")
        os._exit(os.EX_OK)

    def hey_pepper_function(self):
        self.tm.get_labels(True)
        if self.language == "Spanish":
            self.tm.talk("Cual es tu pregunta?", "Spanish")
        else:
            self.tm.talk("What is your question?", "English")
        text = self.tm.speech2text_srv()
        # labels=self.tm.get_labels(False)
        gpt_vision_prompt = f"You are a Pepper robot named Nova from Universidad de los Andes Bogota, Colombia, in this precise moment you are assisting in a supermakert, your function is to answer questions from the users. You can make assumptions. If you don't see anything relevant in the image, just ignore the image. The person asked {text}, please answer concisely, make sure that your answers are short and to the point. Do not try to keep up with the conversation, never ask questions to the person."
        answer = self.tm.img_description(gpt_vision_prompt)["message"]
        # if self.language =="Spanish":
        #     request = f"""La persona pregunto: {text}. Mientras la persona habló, tu viste los siguientes objetos: {labels}. Por favor contesta en español."""
        # else:
        #     request = f"""The person asked: {text}.While the person spoke, you saw the next objects: {labels}"""
        # answer=self.tm.answer_question(request)
        self.tm.talk(answer, self.language, wait=False)
        self.tm.get_labels(True)
        self.tm.follow_you(True)

    def callback_hot_word(self, data):
        word = data.status
        print(word)
        if word == "stop" or word == "detente nova":
            self.tm.follow_you(False)
            self.is_done = True
        elif word == "hey nova" or word == "oye nova":
            self.tm.follow_you(False)
            self.tm.set_move_arms_enabled(True)
            self.tm.go_to_pose("basket", 0.1)
            self.hey_pepper = True

    def check_rospy(self):
        # Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()

    # def callback_hand_sensor_subscriber(self, msg: touch_msg):
    #     if "hand" in msg.name:
    #         self.hey_pepper = True
    #     if "head" in msg.name:
    #         print("head_touched")
    #         self.tm.follow_you(False)
    #         self.is_done = True


# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = MERCADITO()
    sm.run()
    rospy.spin()

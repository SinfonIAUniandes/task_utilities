#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import threading
import rospy
import os


class GetLabelsTest(object):
    def __init__(self):

        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "get labels test"
        states = ["INIT", "RETURN", "SHARE"]
        self.tm = tm(
            perception=True, speech=False, manipulation=False, navigation=False
        )
        self.tm.initialize_node("get_labels_test")
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {"trigger": "start", "source": "GET_LABELS_TEST", "dest": "INIT"},
        ]

        # Crear la máquina de estados
        self.machine = Machine(
            model=self,
            states=states,
            transitions=transitions,
            initial="GET_LABELS_TEST",
        )

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ##################### ROS CALLBACK VARIABLES #####################

        ##################### GLOBAL VARIABLES #####################
        self.name = ""

    def on_enter_INIT(self):
        self.tm.talk("Testeando el publisher de get labels", "Spanish")
        print(
            self.consoleFormatter.format(
                "Inicializacion del task: " + self.task_name, "HEADER"
            )
        )

        self.tm.get_labels(True)
        labels = self.tm.get_labels(False)
        print(labels)

    def check_rospy(self):
        # Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()


# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = GetLabelsTest()
    sm.run()
    rospy.spin()

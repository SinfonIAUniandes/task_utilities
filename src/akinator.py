from transitions import Machine
from task_module import Task_module as tm
import akipy
import rospy
import threading
from robot_toolkit_msgs.msg import speech_recognition_status_msg, animation_msg, motion_tools_msg, leds_parameters_msg, touch_msg
from robot_toolkit_msgs.srv import tablet_service_srv,  set_open_close_hand_srv, set_open_close_hand_srvRequest, motion_tools_srv, battery_service_srv, set_output_volume_srv, tablet_service_srvRequest, set_stiffnesses_srv, set_stiffnesses_srvRequest
import os
import ConsoleFormatter

class Akinator(object):
    def __init__(self) -> None:
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.task_name = "AKINATOR"
        self.hearing = True
        self.new_game = False

        states = states = ["AKINATOR", "INIT", "WAIT4GUEST", "ASK"]
        self.tm = tm(perception = False,speech=True, pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        
        #Definir transiciones
        transitions = [
            {"trigger": "start", "source": "AKINATOR", "dest": "INIT"},
            {"trigger": "begin", "source": "INIT", "dest": "WAIT4GUEST"},
            {"trigger": "new_game", "source": "WAIT4GUEST", "dest": "AKINATOR"},
            {"trigger": "finish", "source": "AKINATOR", "dest": "WAIT4GUEST"},
        ]

        self.machine = Machine(model=self, states=states, transitions=transitions, initial='AKINATOR')
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.initialize_pepper()
        self.begin()

    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        while not self.new_game:
            self.new_game_function()
        self.aki = akipy.Akinator()
        self.aki.start_game()
        self.tm.talk("Ya estoy listo para jugar", "Spanish")
        #TODO - Agregar imprimir en la tablet
        self.tm.show_words()
        self.new_game()

    def on_enter_AKINATOR(self):
        print(self.consoleFormatter.format("AKINATOR", "HEADER"))
        while not self.aki.win:
            print(self.consoleFormatter.format("Pregunta del Akinator: ", "INFO"))
            question = self.aki.question
            #TODO - Agregar imprimir en la tablet
            self.tm.talk(question, "Spanish")
            #Agregar reconocer de la tablet, tener en cuenta que solo se reciba una respuesta de alguno de los 2 lados
            self.tm.hot_word(["", "", ""], thresholds=[0.45, 0.5, 0.5])
            answer = self.tm.speech2text_srv(seconds=0,lang="esp")
            
            self.aki.answer(answer)

            if self.aki.win:
                self.tm.talk("Adivine!", "Spanish")
                #TODO - Pose de celebracion ?
                self.finish()
                self.new_game = False
        
        #TODO - Revisar que pasa cuando despues de mucho tiempo no es capaz de adivinar
        if not self.aki.win:
            
            self.finish()
            

    #Funcion para esperar a que alguien quiera jugar
    def new_game_function(self):
        text = self.tm.speech2text_srv(seconds=0,lang="esp")
        if not ("None" in text):
            if "jugar" in text:
                self.new_game = True


    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
    
if __name__ == "__main__":
    sm = Akinator()
    sm.run()
    rospy.spin()        

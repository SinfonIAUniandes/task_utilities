#!/usr/bin/env python3
import os
import time
import rospy
import threading
import ConsoleFormatter

from transitions import Machine
from task_module import Task_module as tm

class CLEAN_THE_TABLE(object):
    def __init__(self) -> None:
        
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, speech=True, perception = True, pytoolkit=True)
        self.tm.initialize_node('CLEAN_THE_TABLE')
 
        self.STATES = ['INIT', 'GO_TO_TABLE', 'GRAB_OBJECT', 'GO_DISHWASHER', 'DROP_OBJECT', 'CLEAN', 'END'] 

        self.TRANSITIONS = [{'trigger': 'zero', 'source': 'CLEAN_THE_TABLE', 'dest': 'INIT'},
                            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_TO_TABLE'},
                            {'trigger': 'grab_ingredient', 'source': 'GO_TO_TABLE', 'dest': 'GRAB_OBJECT'},
                            {'trigger': 'go_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_DISHWASHER'},
                            {'trigger': 'drop_object', 'source': 'GO_DISHWASHER', 'dest': 'DROP_OBJECT'},
                            {'trigger': 'again', 'source': 'DROP_OBJECT', 'dest': 'GO_TO_TABLE'},
                            {'trigger': 'clean_table', 'source': 'DROP_OBJECT', 'dest': 'CLEAN'},
                            {'trigger': 'end', 'source': 'CLEAN', 'dest': 'END'}]
        
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='CLEAN_THE_TABLE')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                           PARÁMETROS AJUSTABLES
        # -------------------------------------------------------------------------------------------------------------------------------------------
        
        #TODO Parametrizar variables para que sea mas facil editar
        # Movements Speed
        self.fast_movement = 0.15
        self.normal_movement = 0.1
        self.slow_movement = 0.05

        self.item = 0 # Contador para saber que ingrediente se esta manipulando
        self.items = ["cup", "bowl", "spoon", "fork", "dish"] # Orden de los ingredientes (izquierda a derecha)
        self.distance_between_items = 0.2 # Distancia promedio entre los objetos en la cupboard

        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                            ESTADOS / TRANSICIONES
        # -------------------------------------------------------------------------------------------------------------------------------------------

    #TODO Crear logica y estados para la task
    def on_enter_INIT(self): 
        self.tm.set_current_place("init")
        self.tm.go_to_pose("standard", self.fast_movement)
        self.tm.go_to_pose("default_head", self.fast_movement)
        self.tm.set_security_distance(False)
        self.tm.initialize_pepper()
        self.tm.go_to_pose("up_head", 0.1)
        self.tm.talk("Have you finished eating? Let me clean the table.", "English", wait=True)
        self.start()
        
    def on_enter_STATE_0(self): 
        self.finish()

    def on_enter_END(self):
        self.tm.talk("I finished cleaning the table")
        return

    def check_rospy(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)
    
    def run(self):
        while not rospy.is_shutdown():
            self.zero()
            
            
    # -------------------------------------------------------------------------------------------------------------------------------------------
    #                                                      FIN DE ESTADOS / TRANSICIONES
    # -------------------------------------------------------------------------------------------------------------------------------------------
    
        
    # -------------------------------------------------------------------------------------------------------------------------------------------
    #                                                         FUNCIÓN PRINCIPAL
    # -------------------------------------------------------------------------------------------------------------------------------------------
    
if __name__ == "__main__":
    sm = CLEAN_THE_TABLE()
    sm.run()
    rospy.spin()

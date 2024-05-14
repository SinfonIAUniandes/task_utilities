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
 
        #TODO Definir estados
        self.STATES = ['INIT', 'END'] 
        #TODO Definir transiciones entre los estados
        self.TRANSITIONS = [{'trigger': 'zero', 'source': 'CLEAN_THE_TABLE', 'dest': 'STATE_0'},
                            {'trigger': 'finish', 'source': 'STATE_0', 'dest': 'END'}]
        
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='CLEAN_THE_TABLE')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                           PARÁMETROS AJUSTABLES
        # -------------------------------------------------------------------------------------------------------------------------------------------
        
        #TODO Parametrizar variables para que sea mas facil editar

        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                            ESTADOS / TRANSICIONES
        # -------------------------------------------------------------------------------------------------------------------------------------------

    #TODO Crear logica y estados para la task
    def on_enter_INIT(self): 
        self.start()
        
    def on_enter_STATE_0(self): 
        self.finish()

    def on_enter_END(self):
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

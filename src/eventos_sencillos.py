#!/usr/bin/env python3
import os
import time
import rospy
import threading
import ConsoleFormatter

from transitions import Machine
from task_module import Task_module as tm

class EVENTOS(object):
    def __init__(self) -> None:
        
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, speech=True, perception = True, pytoolkit=True)
        self.tm.initialize_node('EVENTOS')
 
        self.STATES = ['INIT', 'GO_2_CUPBOARD','GRAB_OBJECT', 'GO_DROP_PLACE', 'DROP_OBJECT', 'MAKE_BREAKFAST', 'END']
        self.TRANSITIONS = [{'trigger': 'zero', 'source': 'EVENTOS', 'dest': 'INIT'},
                            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_2_CUPBOARD'},
                            {'trigger': 'grab_ingredient', 'source': 'GO_2_CUPBOARD', 'dest': 'GRAB_OBJECT'},
                            {'trigger': 'go_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_DROP_PLACE'},
                            {'trigger': 'drop_object', 'source': 'GO_DROP_PLACE', 'dest': 'DROP_OBJECT'},
                            {'trigger': 'again', 'source': 'DROP_OBJECT', 'dest': 'GO_2_CUPBOARD'},
                            {'trigger': 'make_breakfast', 'source': 'DROP_OBJECT', 'dest': 'MAKE_BREAKFAST'},
                            {'trigger': 'end', 'source': 'MAKE_BREAKFAST', 'dest': 'END'}]
        
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='EVENTOS')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                            ESTADOS / TRANSICIONES
        # -------------------------------------------------------------------------------------------------------------------------------------------
        

    def on_enter_INIT(self):
        self.tm.go_to_pose("standard", self.fast_movement)
        self.tm.go_to_pose("default_head", self.fast_movement)
        self.tm.set_security_distance(False)
        self.tm.initialize_pepper()
        self.tm.talk("I will serve the breakfast", "English", wait=True)
        self.start()

    def on_enter_GO_2_CUPBOARD(self):
        self.actual_item = self.items[self.item]
        self.tm.talk(f"I am going to go pick up the {self.actual_item} for breakfast")
        #self.tm.go_to_place("kitchen")
        self.grab_ingredient()
    

    def on_enter_END(self):
        self.tm.talk("I finished serving breakfast, enjoy it")
        return

    def check_rospy(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        self.tm.set_security_distance(True)
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
    sm = EVENTOS()
    sm.run()
    rospy.spin()

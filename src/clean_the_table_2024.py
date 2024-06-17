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
                            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_2_TABLE'},
                            {'trigger': 'grab_ingredient', 'source': 'GO_2_TABLE', 'dest': 'GRAB_OBJECT'},
                            {'trigger': 'go_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_DISHWASHER'},
                            {'trigger': 'drop_object', 'source': 'GO_DISHWASHER', 'dest': 'DROP_OBJECT'},
                            {'trigger': 'again', 'source': 'DROP_OBJECT', 'dest': 'GO_2_TABLE'},
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
        self.table_approach_distance = 0.4


        self.fast_movement = 0.15
        self.normal_movement = 0.1
        self.slow_movement = 0.05
        
        self.away_from_table = 0.2
        self.drop_dish_point = -0.3
        self.drop_bowl_point = 0
        self.drop_cup_point = 0.3
        
        self.get_closer = 0.2



        self.item = 0 # Contador para saber que ingrediente se esta manipulando
        self.items = ["cup", "bowl", "dish", "spoon", "fork"] # Orden de los ingredientes (izquierda a derecha)
        self.distance_between_items = 0.2 # Distancia promedio entre los objetos en la cupboard
        self.info_grab_items = {
        "dish": ["open_both_hands", "both_arms_milk", self.away_from_table, "close_arms_milk", "raise_arms_milk", "finish"],
        "bowl": ["mid_arms_bowl", "both_arms_bowl", self.away_from_table, "close_arms_bowl", "raise_arms_bowl", self.relative_bowl_distance, "finish"],
        "cup": ["open_both_hands", "both_arms_cereal", self.away_from_table, "close_arms_cereal", "raise_arms_cereal", self.relative_cereal_distance, "finish"]
        }
        
        self.info_drop_items = {
            "dish": ["close_arms_milk", "both_arms_milk", "finish"],
            "bowl":["close_arms_bowl", "both_arms_bowl", "finish"],
            "cup":["close_arms_cereal", "both_arms_cereal", "finish"]
            }
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                            ESTADOS / TRANSICIONES
        # -------------------------------------------------------------------------------------------------------------------------------------------

    def on_enter_INIT(self): 
        self.tm.set_current_place("init")
        self.tm.go_to_pose("standard", self.fast_movement)
        self.tm.go_to_pose("default_head", self.fast_movement)
        self.tm.set_security_distance(False)
        self.tm.initialize_pepper()
        self.tm.go_to_pose("up_head", 0.1)
        self.tm.talk("I am going to clean the table.", "English", wait=False)
        self.start()

    def on_enter_GO_2_TABLE(self):
        self.actual_item = self.items[self.item]
        self.tm.talk("on my way to the kitchen!", "English", wait=False)
        self.tm.go_to_place("kitchen")
        self.tm.talk(f"I am going to pick up the {self.actual_item}", "English", wait=False)
        self.grab_ingredient()

    def on_enter_GRAB_OBJECT(self):
        self.tm.go_to_relative_point(self.table_approach_distance, 0.0, 0.0)
        time.sleep(1)
        self.tm.go_to_pose("almost_down_head", self.normal_movement)
        actions = self.info_grab_items[self.actual_item]
        for action in actions:
            if isinstance(action, str):
                if action != "finish":
                    if self.actual_item == "cup":
                        self.tm.talk("Please, could you put the spoon in the cup so I can take it to the next table?", "English", wait=False)
                        time.sleep(5)
                        self.tm.talk("Thank you!", "English", wait=False)
                        
                    self.tm.go_to_pose(action, self.slow_movement)
                    print(f"se ejcuto {action}")
            else:
                self.tm.go_to_relative_point(action, 0.0, 0.0)
            time.sleep(3)
                
        self.tm.talk(f"Now I will leave the {self.actual_item} in the dishwasher", "English", wait=False)
        self.tm.go_to_relative_point(-(self.cupboard_approach_distance), 0.0, 0.0)
        self.go_drop_place()
     
    def on_enter_GO_DISHWASHER(self):
        self.tm.go_to_place("kitchen_dishwasher")
        self.tm.go_to_relative_point(0.0,0.0,90)
        time.sleep(1)
        self.drop_object()

    def on_enter_DROP_OBJECT(self):
        self.tm.talk(f"Now, I will leave the {self.actual_item} on the dishwasher tab", "English", wait=False)
        actions = self.info_drop_items[self.actual_item]
        drop_point = getattr(self, f'drop_{self.actual_item}_point')
        #TODO: Verficiar como vamos a dejar los objetos (altura de brazos, posicion manos etc)
        self.tm.go_to_relative_point(0.0, drop_point, 0.0)
        self.tm.go_to_relative_point(self.get_closer, 0.0, 0.0)
        for action in actions:
            if isinstance(action, str):
                if action != "finish":
                    #TODO: Que vamos a hacer con la spoon y el fork 
                        
                    self.tm.go_to_pose(action, self.slow_movement)
                    time.sleep(3)
                    print(f"se ejcuto {action}")
            else:
                self.tm.go_to_relative_point(action, 0.0, 0.0)
                time.sleep(1)   
        
        self.tm.go_to_pose("standard")
        if self.items[-1] == self.actual_item:
            self.clean_table()
        else:
            self.item+=1
            self.again()
            
    def on_enter_CLEAN(self):
        self.tm.go_to_place("kitchen_dishwasher")   
        self.end()    

    def on_enter_END(self):
        self.tm.talk("I finished cleaning the table")
        os._exit(os.EX_OK)


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

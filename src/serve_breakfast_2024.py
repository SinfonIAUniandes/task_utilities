#!/usr/bin/env python3
import os
import time
import rospy
import threading
import ConsoleFormatter

from transitions import Machine
from task_module import Task_module as tm

class SERVE_BREAKFAST(object):
    def __init__(self) -> None:
        
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, speech=True, perception = True, pytoolkit=True)
        self.tm.initialize_node('SERVE_BREAKFAST')
 
        self.STATES = ['INIT', 'GO_2_CUPBOARD','GRAB_OBJECT', 'GO_DROP_PLACE', 'DROP_OBJECT', 'MAKE_BREAKFAST', 'END']
        self.TRANSITIONS = [{'trigger': 'zero', 'source': 'SERVE_BREAKFAST', 'dest': 'INIT'},
                            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_2_CUPBOARD'},
                            {'trigger': 'grab_ingredient', 'source': 'GO_2_CUPBOARD', 'dest': 'GRAB_OBJECT'},
                            {'trigger': 'go_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_DROP_PLACE'},
                            {'trigger': 'drop_object', 'source': 'GO_DROP_PLACE', 'dest': 'DROP_OBJECT'},
                            {'trigger': 'again', 'source': 'DROP_OBJECT', 'dest': 'GO_2_CUPBOARD'},
                            {'trigger': 'make_breakfast', 'source': 'DROP_OBJECT', 'dest': 'MAKE_BREAKFAST'},
                            {'trigger': 'end', 'source': 'MAKE_BREAKFAST', 'dest': 'END'}]
        
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='SERVE_BREAKFAST')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                           PARÁMETROS AJUSTABLES
        # -------------------------------------------------------------------------------------------------------------------------------------------
        
        # Movements Speed
        self.fast_movement = 0.15
        self.normal_movement = 0.1
        self.slow_movement = 0.05
        
        # Relative Distances to Cupboard
        # TODO Ajustar distancias para recoger
        self.cupboard_approach_distance = 0.45 
        self.right_corner_cupboard_table = -0.4
        self.relative_milk_distance = 0.35
        self.relative_bowl_distance = 0.35
        self.relative_cereal_distance = 0.35
        
        # Relative Distances to Serve Table
        # TODO Ajustar distancias para soltar
        self.serve_table_center = 0.45
        self.away_from_table = 0.2
        self.drop_milk_point = -0.3
        self.drop_cereal_point = 0.3
        
        
        self.item = 0 # Contador para saber que ingrediente se esta manipulando
        self.items = ["milk_cardboard", "bowl", "cereal_box", "spoon"] # Orden de los ingredientes (izquierda a derecha)
        self.info_items = {"milk_cardboard": ["open_both_hands", "both_arms_milk" ,self.relative_milk_distance, "close_arms_milk", "raise_arms_milk" , "kitchen"],
                           "bowl":["mid_arms_bowl","both_arms_bowl", self.relative_bowl_distance, "close_arms_bowl", "raise_arms_bowl" , self.relative_bowl_distance],
                           "cereal_box":["","both_arms_cereal", self.relative_bowl_distance, "close_arms_cereal", "raise_arms_cereal" , self.relative_cereal_distance]}
        self.distance_between_items = 0.2 # Distancia aproximada entre los objetos
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                          FIN DE PARÁMETROS AJUSTABLES
        # -------------------------------------------------------------------------------------------------------------------------------------------
        
        
        
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
        self.tm.talk("Good morning! Today I would like to help you with the preparation of a delicious breakfast", "English", wait=False)
        self.start()

    def on_enter_GO_2_CUPBOARD(self):
        self.actual_item = self.items[self.item]
        self.tm.talk("I will navigate to the kitchen door and look for the ingredients. I will look for the milk cardboard, the cereal box and the bowl. in that order", "English", wait=False)
        time.sleep(0.5)
        self.tm.talk(f"on my way to the kitchen!", "English", wait=False)
        self.tm.go_to_place("kitchen")
        self.tm.talk(f"First, I am going to pick up the {self.actual_item}", "English", wait=False)
        self.grab_ingredient()

    def on_enter_GRAB_OBJECT(self):
        self.tm.go_to_relative_point(self.cupboard_approach_distance, 0.0, 0.0)
        time.sleep(1)
        self.tm.go_to_relative_point(0.0, self.right_corner_cupboard_table, 0.0)
        self.tm.go_to_pose("almost_down_head", self.normal_movement)
        
        actions = self.info_items[self.actual_item]
        #eliminar cuando perception tenga milk_cardboard 
        if self.actual_item == "milk_cardboard":
            self.tm.talk("I will move a litle bit to position myself in front of the cardboard", "English", wait=True)
            self.tm.align_with_object("bottle")
            counter = 0
            
            for i in actions:
                if isinstance(i, str):
                    
                    if i != "kitchen":
                        self.tm.go_to_pose(actions[counter], self.normal_movement)
                        time.sleep(2)

                    else:
                        self.tm.go_to_place("kitchen_house")
                        self.tm.go_to_place("house")
                        time.sleep(1)
                
                else:
                    self.tm.go_to_relative_point(actions[counter], 0.0, 0.0)
                    time.sleep(3)
                    
                counter += 1
                
        self.go_drop_place()

    def on_enter_GO_DROP_PLACE(self):
        self.tm.go_to_place("dining")
        self.tm.go_to_relative_point(0.0,0.0,-90)
        time.sleep(2)
        self.drop_object()

    def on_enter_DROP_OBJECT(self): 
        self.tm.talk(f"Now, I will leave the {self.actual_item} above the table and then go for the {self.items[self.item+1]}", "English", wait=False)
        
        if self.items[-1] == self.actual_item:
            self.make_breakfast()
        else:
            self.item+=1
            self.again()
        
    
    def on_enter_MAKE_BREAKFAST(self):
        self.item = 0
        # TODO Crear las animaciones para servir el cereal y la leche.
            
        self.end()

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
    sm = SERVE_BREAKFAST()
    sm.run()
    rospy.spin()

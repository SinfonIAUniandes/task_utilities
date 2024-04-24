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
        self.tm = tm(navigation=True, manipulation=True, speech=True, perception = False, pytoolkit=True)
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
        
        self.j = 0 # Contador para saber que ingrediente se esta manipulando
        self.ingredients = ["milk_carton", "bowl", "cereal_box", "spoon"] # Orden de los ingredientes (izquierda a derecha)
        self.distance_between_items = 0.2 # Distancia aproximada entre los objetos
        
        # Movements Speed
        self.fast_movement = 0.15
        self.normal_movement = 0.1
        self.slow_movement = 0.5
        
        # Relative Distances to Cupboard
        # TODO Ajustar distancias para recoger
        self.cupboard_approach_distance = 0.45 
        self.right_corner_cupboard_table = 0.4
        self.relative_milk_distance = 0.35
        self.relative_bowl_distance = 0.35
        self.relative_cereal_distance = 0.35
        
        # Relative Distances to Serve Table
        # TODO Ajustar distancias para soltar
        self.serve_table_center = 0.45
        self.away_from_table = 0.2
        self.drop_milk_point = -0.3
        self.drop_milk_point = 0.3

    def on_enter_INIT(self):
        self.tm.go_to_pose("standard",self.fast_movement)
        self.tm.go_to_pose("default_head",self.fast_movement)
        self.tm.set_security_distance(False)
        self.tm.talk("I will serve the breakfast", "English", wait=True)
        self.start()

    def on_enter_GO_2_CUPBOARD(self):
        self.actual_item = self.ingredients[self.j]
        self.tm.talk(f"I am going to go pick up the {self.actual_item} for breakfast")
        self.tm.go_to_place("kitchen")
        self.grab_ingredient()

    def on_enter_GRAB_OBJECT(self):
        
        self.tm.go_to_relative_point(self.cupboard_approach_distance,0,0);
        time.sleep(1)
        self.tm.go_to_relative_point(0.0,self.right_corner_cupboard_table,0);
        self.right_corner_cupboard_table -= self.distance_between_items
        
        if self.actual_item == "milk_carton":
            self.tm.focus_with_object(self.actual_item)
            self.tm.go_to_pose("open_arms_milk",self.normal_movement)
            time.sleep(5)
            self.tm.go_to_relative_point(self.relative_milk_distance,0.0)
            self.tm.go_to_pose("close_arms_milk",self.slow_movement)
            time.sleep(3)
            self.tm.go_to_pose("raise_arms_milk", self.slow_movement)
            time.sleep(1)
            self.tm.go_to_relative_point(-(self.relative_milk_distance),0.0,0.0)
            
        elif self.actual_item == "bowl":
            self.tm.focus_with_object(self.actual_item)
            self.tm.go_to_pose("open_arms_bowl",self.normal_movement)
            time.sleep(5)
            self.tm.go_to_relative_point(self.relative_bowl_distance,0.0)
            self.tm.go_to_pose("close_arms_bowl",self.slow_movement)
            time.sleep(3)
            self.tm.go_to_pose("raise_arms_bowl", self.slow_movement)
            time.sleep(1)
            self.tm.go_to_relative_point(-(self.relative_bowl_distance),0.0,0.0)
            
        elif self.actual_item== "cereal_box":
            self.tm.focus_with_object(self.actual_item)
            self.tm.go_to_pose("open_arms_cereal_box",self.normal_movement)
            time.sleep(5)
            self.tm.go_to_relative_point(self.relative_cereal_distance,0.0)
            self.tm.go_to_pose("close_arms_cereal_box",self.slow_movement)
            time.sleep(3)
            self.tm.go_to_pose("raise_arms_cereal_box", self.slow_movement)
            time.sleep(2)
            self.tm.go_to_relative_point(-(self.relative_cereal_distance),0.0,0.0)
            
        elif self.actual_item== "spoon":
            pass
        
        self.tm.go_to_relative_point(-(self.cupboard_approach_distance),0,0)
        self.go_drop_place()

    def on_enter_GO_DROP_PLACE(self):
        self.tm.talk(f"I am going to take the {self.actual_item} to the table, please wait", "English", wait=False)
        self.tm.go_to_place("dinner_room")
        self.drop_object()

    def on_enter_DROP_OBJECT(self):
        self.tm.go_to_relative_point(self.serve_table_center,0.0,0.0)
        time.sleep(1)
        self.tm.go_to_relative_point(0.0,self.away_from_table,0.0)
        time.sleep(1)
        self.tm.go_to_relative_point(0.0,0.0,-1.57)
        if self.actual_item == "milk_carton":
            self.tm.go_to_relative_point(0.0, self.drop_milk_point, 0.0)
            time.sleep(1)
            self.tm.go_to_relative_point(-(self.away_from_table), 0.0, 0.0)
            self.tm.go_to_pose("close_arms_milk",self.slow_movement)
            time.sleep(3)
            self.tm.go_to_pose("open_arms_milk",self.normal_movement)
            time.sleep(3)
            
        elif self.actual_item== "bowl":
            self.tm.go_to_relative_point(-(self.away_from_table), 0.0, 0.0)
            self.tm.go_to_pose("close_arms_bowl",self.slow_movement)
            time.sleep(3)
            self.tm.go_to_pose("open_arms_bowl",self.normal_movement)
            time.sleep(3)
            
        elif self.actual_item == "cereal_box":
            self.tm.go_to_relative_point(0.0, self.drop_ceral_point, 0.0)
            time.sleep(1)
            self.tm.go_to_relative_point(-(self.away_from_table), 0.0, 0.0)
            self.tm.go_to_pose("close_arms_cereal_box",self.slow_movement)
            time.sleep(3)
            self.tm.go_to_pose("open_arms_cereal_box",self.normal_movement)
            time.sleep(3)
            
        elif self.actual_item == "spoon":
            pass
        
        self.tm.go_to_relative_point(0.0,self.away_from_table,0.0)
        time.sleep(1)
        self.tm.go_to_pose("standard", 0.15)
        time.sleep(2)
        
        
        if self.actual_item == self.ingredients[-1]:
            self.tm.talk("I am ready to prepare breakfast", "English", wait=True)
            self.make_breakfast()
        else:
            self.j += 1
            self.again()
    
    def on_enter_MAKE_BREAKFAST(self):
        self.j = 0
        # TODO Crear las animaciones para servir el cereal y la leche.
            
        self.end()

    def on_enter_END(self):
        self.tm.talk("I finished serving breakfast, enjoy it")
        return

    def check_rospy(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)
    
    def run(self):
        while not rospy.is_shutdown():
            self.zero()

if __name__ == "__main__":
    sm = SERVE_BREAKFAST()
    sm.run()
    rospy.spin()

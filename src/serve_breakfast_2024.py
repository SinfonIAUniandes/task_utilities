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
        
        self.j = 0 #Contador para saber que ingrediente se esta manipulando
        self.ingredients = ["milk_carton", "bowl", "cereal_box", "spoon"] # Lista de ingredientes
        
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

    def on_enter_INIT(self):
        self.tm.go_to_pose("standard",0.15)
        self.tm.go_to_pose("default_head",0.15)
        self.tm.set_security_distance(False)
        self.tm.talk("I will serve the breakfast", "English", wait=True)
        self.start()

    def on_enter_GO_2_CUPBOARD(self):
        self.tm.talk(f"I am going to go pick up the {self.ingredients[self.j]} for breakfast")
        # self.tm.go_to_place("kitchen")
        self.grab_ingredient()

    def on_enter_GRAB_OBJECT(self):
        
        self.tm.go_to_relative_point(0.45,0,0);
        
        if self.ingredients[self.j] == "milk_carton":
            self.tm.go_to_relative_point(0.0,0.345,0.0)
            self.tm.go_to_pose("open_both_arms",0.1)
            time.sleep(7)
            self.tm.go_to_relative_point(0.405, 0.0, 0.0)
            self.tm.go_to_pose("open_both_hands",0.1)
            time.sleep(3)
            self.tm.go_to_pose("close_both_arms_milk",0.1)
            time.sleep(5)
            self.tm.go_to_pose("raise_both_arms_milk",0.1)
            time.sleep(10)
            self.tm.go_to_pose("default_head",0.1)
            self.tm.go_to_relative_point(-0.41, 0.0, 0.0)
            time.sleep(1)
            self.tm.go_to_relative_point(0.0, -0.35, 0.0)
        elif self.ingredients[self.j] == "bowl":
            self.tm.go_to_relative_point(0.0,0.-0.345,0.0)
            self.tm.go_to_pose("open_arms_bowl",0.1)
            time.sleep(7)
            self.tm.go_to_relative_point(0.41, 0.0, 0.0)
            self.tm.go_to_pose("open_both_hands",0.1)
            time.sleep(3)
            self.tm.go_to_pose("close_arms_bowl",0.1)
            time.sleep(5)
            self.tm.go_to_pose("raise_arms_bowl",0.1)
            time.sleep(10)
            self.tm.go_to_pose("default_head",0.1)
            self.tm.go_to_relative_point(-0.41, 0.0, 0.0)
            time.sleep(1)
            self.tm.go_to_relative_point(0.0, 0.34, 0.0)
        elif self.ingredients[self.j] == "cereal_box":
            self.tm.go_to_pose("open_both_arms",0.1)
            time.sleep(7)
            self.tm.go_to_relative_point(0.3, 0.0, 0.0)
            self.tm.go_to_pose("open_both_hands",0.1)
            time.sleep(3)
            self.tm.go_to_pose("close_both_arms_cereal",0.1)
            time.sleep(5)
            self.tm.go_to_pose("raise_both_arms_milk",0.1)
            time.sleep(10)
            self.tm.go_to_pose("default_head",0.1)
            self.tm.go_to_relative_point(-0.3, 0.0, 0.0)
        elif self.ingredients[self.j] == "spoon":
            # TODO definir el lugar del objeto y poner un relative_point
            # TODO crear y añadir la animacion correspoondiente
            pass
        
        self.tm.go_to_relative_point(-0.45,0,0)
            
        self.go_drop_place()

    def on_enter_GO_DROP_PLACE(self):
        self.tm.talk(f"I am going to take the {self.ingredients[self.j]} to the table, please wait", "English", wait=False)
        # self.tm.go_to_place("kitchen")
        self.drop_object()

    def on_enter_DROP_OBJECT(self):
        if self.ingredients[self.j] == "milk_carton":
            self.tm.go_to_relative_point(0.45,0.0,0.0)
            self.tm.go_to_relative_point(0, 0.345, 0)
            self.tm.go_to_relative_point(0.45,0.0,0.0)
            self.tm.go_to_pose("close_both_arms_milk",0.1)
            time.sleep(2)
            self.tm.go_to_pose("open_both_arms",0.1)
            self.tm.go_to_relative_point(-0.3,0.0,0.0)
            time.sleep(1)
            self.tm.go_to_relative_point(0.0,-0.45,0.0)
            
        elif self.ingredients[self.j] == "bowl":
            # TODO definir el lugar del objeto y poner un relative_point (tiene que estar en la mitad de los objetos)
            # TODO crear y añadir la pose correspoondiente para soltar
            pass
        elif self.ingredients[self.j] == "cereal_box":
            self.tm.go_to_relative_point(0.3,0.0,0.0)
            time.sleep(2)
            self.tm.go_to_pose("close_both_arms_cereal",0.1)
            time.sleep(2)
            self.tm.go_to_pose("open_both_arms",0.1)
            self.tm.go_to_relative_point(-0.3,0.0,0.0)
        elif self.ingredients[self.j] == "spoon":
            # TODO definir el lugar del objeto y poner un relative_point
            # TODO TODO crear y añadir la pose correspoondiente para soltar
            pass
        self.tm.go_to_pose("standard", 0.15)
        if self.ingredients[self.j] == self.ingredients[-1]:
            self.tm.talk("I am ready to prepare breakfast", "English", wait=True)
            self.make_breakfast()
        else:
            self.j += 1
            self.again()
    
    def on_enter_MAKE_BREAKFAST(self):
        self.j = 0
        # TODO Crear las animaciones para servir el cereal y la leche. El bowl ya esta posicionado en el centro
            
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

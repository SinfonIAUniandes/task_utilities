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
 
        self.STATES = ['INIT', 'GO_2_CUPBOARD', 'LOOK_FOR_OBJECTS' ,'GRAB_OBJECT', 'GO_DROP_PLACE', 'DROP_OBJECT', 'MAKE_BREAKFAST', 'END']
        self.TRANSITIONS = [{'trigger': 'zero', 'source': 'SERVE_BREAKFAST', 'dest': 'INIT'},
                            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_2_CUPBOARD'},
                            {'trigger': 'look_for_object', 'source': 'GO_2_CUPBOARD', 'dest': 'LOOK_FOR_OBJECTS'},
                            {'trigger': 'grab_ingredient', 'source': 'LOOK_FOR_OBJECTS', 'dest': 'GRAB_OBJECT'},
                            {'trigger': 'go_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_DROP_PLACE'},
                            {'trigger': 'drop_object', 'source': 'GO_DROP_PLACE', 'dest': 'DROP_OBJECT'},
                            {'trigger': 'again', 'source': 'DROP_OBJECT', 'dest': 'GO_GRAB_PLACE'},
                            {'trigger': 'make_breakfast', 'source': 'DROP_OBJECT', 'dest': 'MAKE_BREAKFAST'},
                            {'trigger': 'end', 'source': 'MAKE_BREAKFAST', 'dest': 'END'}]
        
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='SERVE_BREAKFAST')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()


    def on_enter_INIT(self):
        self.tm.go_to_pose("standard")
        self.tm.go_to_pose("default_head")
        self.tm.talk("I will serve breakfast")
        time.sleep(5)
        self.start()

    def on_enter_GO_2_CUPBOARD(self):
        #TODO Poner el lugar de destino para recoger las cosas cuando este listo el mapa
        self.tm.talk("I am going to go pick up the stuff for breakfast")
        self.tm.go_to_place("")
        self.look_for_object()

    def on_enter_LOOK_FOR_OBJECTS(self):
        self.tm.go_to_pose("down_head")
        ingredients_prompt = f"Just give me in a python list the objects that are considered realted with food
        or ingredients that you see below, from left to right. Do not answer aboslutely anything else. For example the answer
        will be [milk, cereal, spon, bowl]"
        self.ingredients = list(self.tm.img_description(ingredients_prompt)["message"])
        self.tm.talk("I can see ")
        for i in self.ingredients:
            self.tm.talk( f"a {str(i)}", "English", wait=False)
        self.j = 0
        self.grab_ingredient()

    def on_enter_GRAB_OBJECT(self):
        object_prompt = f"Just tell me the number of what I am going to ask you, don't answer anything else.
        How much (in meters) do I have to move to be exactly aligned with the object {int(self.ingredients[self.j])}.
        Taking into account that we are at a reference point 0, to the left are negative numbers and to
        the right are positive numbers. In case that you are alligned with the object only answer 0. For example,
        your answer will be 1 or 0 or -1, etc"
        self.approach = self.tm.img_description(object_prompt)["message"]
        self.tm.go_to_relative_point(0,self.approach,0)
        # if self.ingredients[self.j] == "milk":
        #     # TODO crear y añadir la pose correspondiente
        #     self.tm.play_action("")
        # elif self.ingredients[self.j] == "bowl":
        #     # TODO crear y añadir la pose correspondiente
        #     self.tm.play_action("")
        # elif self.ingredients[self.j] == "cereal":
        #     # TODO crear y añadir la pose correspondiente
            self.tm.go_to_pose("both_arms")
            time.sleep(10)
            self.tm.go_to_pose("close_both_arms")
            time.sleep(5)
            self.tm.go_to_pose("raise_closed_arms")
            time.sleep(10)
            self.tm.go_to_pose("default_head")
        #     self.tm.play_action("")
        # elif self.ingredients[self.j] == "spon":
        #     # TODO crear y añadir la pose correspondiente
        #     self.tm.play_action("")
        self.go_drop_place()

    def on_enter_GO_DROP_PLACE(self):
        self.tm.talk(f"I am going to take the {self.ingredients[self.j]} to the table, please wait", "English", wait=False)
        #TODO Poner el lugar de destino para dejar las cosas cuando este listo el mapa
        self.tm.go_to_place("")
        self.drop_object()

    def on_enter_DROP_OBJECT(self):
        #TODO Crear y añadir pose para soltar objeto
        self.tm.go_to_pose("both_arms")
        time.sleep(2)
        self.tm.go_to_pose("standard")
        if self.ingredients[self.j] == self.ingredients[-1]:
            self.make_breakfast()
        else:
            self.j += 1
            self.again()
    
    def on_enter_MAKE_BREAKFAST(self):
        self.tm.talk("I am ready to prepare breakfast", "English", wait=False)
        #TODO Crear estado para preparar el desayuno (para esto se pueden crear más estados)
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

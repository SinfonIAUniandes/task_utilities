#!/usr/bin/env python3
import os
import rospy
import threading
import numpy as np
import ConsoleFormatter

from transitions import Machine
from nav_msgs.msg import Odometry
from task_module import Task_module as tm
from robot_toolkit_msgs.msg import touch_msg
from robot_toolkit_msgs.srv import set_move_arms_enabled_srv, set_security_distance_srv,  misc_tools_srv, misc_tools_srvRequest, set_security_distance_srvRequest


class SERVE_BREAKFAST(object):
    def __init__(self) -> None:
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, speech=True, perception = True, pytoolkit=True)
        self.tm.initialize_node('SERVE_BREAKFAST')

        self.STATES = ['INIT', 'GO_2_CUPBOARD', 'LOOK_FOR_OBJECT' ,'GRAB_OBJECT', 'GO_DROP_PLACE', 'DROP_OBJECT', 'MAKE_BREAKFAST', 'END']
        self.TRANSITIONS = [{'trigger': 'init', 'source': 'SERVE_BREAKFAST', 'dest': 'INIT'},
                            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_2_CUPBOARD'},
                            {'trigger': 'look_for_object', 'source': 'GO_2_CUPBOARD', 'dest': 'LOOK_FOR_OBJECT'},
                            {'trigger': 'grab_ingredient', 'source': 'LOOK_FOR_OBJECT', 'dest': 'GRAB_OBJECT'},
                            {'trigger': 'go_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_DROP_PLACE'},
                            {'trigger': 'drop_object', 'source': 'GO_DROP_PLACE', 'dest': 'DROP_OBJECT'},
                            {'trigger': 'again', 'source': 'DROP_OBJECT', 'dest': 'GO_GRAB_PLACE'},
                            {'trigger': 'make_breakfast', 'source': 'DROP_OBJECT', 'dest': 'MAKE_BREAKFAST'},
                            {'trigger': 'end', 'source': 'MAKE_BREAKFAST', 'dest': 'END'}]
        
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='SERVE_BREAKFAST')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()


    def on_enter_INIT(self):
        self.tm.talk("I am going to serve breakfast")
        self.start()

    def on_enter_GO_GRAB_PLACE(self):
        self.tm.go_to_place("cupboard")
        self.look_for_object()

    def on_enter_LOOK_FOR_OBJECTS(self):
        self.tm.go_to_state("down_head")
        ingredients_prompt = f"Just give me in a python list the objects that are considered as food
        or ingredients that you see below, from left to right. Do not answer aboslutely anything else."
        self.ingredients = self.tm.img_description(ingredients_prompt)["message"]
        for i in self.ingredients:
            self.tm.talk("The objects I am looking at are, " + str(i), "English", wait=False)
        self.j = 0
        self.grab_object()

    def on_enter_GRAB_OBJECT(self):
        object_prompt = f"Just tell me the number of what I am going to ask you, don't answer anything else.
        How much do I have to move to be exactly aligned with the object {self.ingredients[self.j]}.
        Taking into account that we are at a reference point 0, to the left are negative numbers and to
        the right are positive numbers."
        self.approach = self.tm.img_description(object_prompt)["message"]
        self.tm.go_to_relative_point(0,self.approach,0)
        

        self.go_drop_place()

    def on_enter_GO_DROP_PLACE(self):
        self.drop_object()

    def on_enter_DROP_OBJECT(self):
            self.end()

    def on_enter_END(self):
        self.tm.talk("I have finished serving breakfast")
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

#!/usr/bin/env python3
import os
import rospy
import threading
from transitions import Machine
from task_module import Task_module as tm
from robot_toolkit_msgs.srv import set_angle_srv, set_security_distance_srv
import ConsoleFormatter

class ServeBreakfast(object):
    def __init__(self) -> None:
        self.console_formatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, speech=True, perception=True, pytoolkit=True)
        self.tm.initialize_node('SERVE_BREAKFAST')
        self.states = ['INIT', 'GO_TO_CUPBOARD', 'LOOK_4_ITEM', 'GRAB_OBJECT', 'GO_TO_DROP_PLACE', 'DROP', 'MAKE_BREAKFAST', 'END']
        
        self.transitions = [
            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_TO_CUPBOARD'},
            {'trigger': 'scan_for_objects', 'source': 'GO_TO_CUPBOARD', 'dest': 'LOOK_4_ITEM'},
            {'trigger': 'grab_ingredient', 'source': 'LOOK_4_ITEM', 'dest': 'GRAB_OBJECT'},
            {'trigger': 'go_to_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_TO_DROP_PLACE'},
            {'trigger': 'drop_object', 'source': 'GO_TO_DROP_PLACE', 'dest': 'DROP'},
            {'trigger': 'serve', 'source': 'DROP', 'dest': 'MAKE_BREAKFAST'},
            {'trigger': 'again', 'source': 'DROP', 'dest': 'GO_TO_CUPBOARD'},
            {'trigger': 'again2', 'source': 'MAKE_BREAKFAST', 'dest': 'GO_TO_CUPBOARD'},
            {'trigger': 'end', 'source': 'MAKE_BREAKFAST', 'dest': 'END'}
        ]
        
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial='SERVE_BREAKFAST')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        self.fast_movement = 0.15 
        self.normal_movement = 0.1
        self.slow_movement = 0.05
        
        self.items = ["bowl", "cereal_box", "milk_carton"] 
        
        self.relative_item_angle = {
            "bowl": 0,
            "cereal_box": 0,
            "milk_carton": 0
        }
                
        self.grab_items_poses = {
            "bowl": ["mid_arms_bowl", "bowl_hands", "close_arms_bowl", "raise_arms_bowl"],
            "cereal_box": ["open_both_hands", "both_arms_cereal", "close_arms_cereal","bowl_hands" , "raise_arms_cereal"],
            "milk_carton": ["open_both_hands", "both_arms_milk", "close_arms_milk", "bowl_hands" ,"raise_arms_milk"]
        }
        
        self.drop_and_serve_position = {
            "bowl": 0.8,
            "milk_carton": 0.2,
            "cereal_box": 1.0 
        }
        
        self.drop_items_poses = {
            "bowl": ["close_arms_bowl","open_both_hands", "both_arms_bowl"],
            "milk_carton": ["close_arms_milk", "both_arms_milk", "close_arms_milk"],
            "cereal_box": ["close_arms_cereal", "both_arms_cereal", "close_arms_cereal"]
        }
        
        self.serve_items_poses = {
            "milk_carton": ["close_arms_milk", "open_both_hands", "both_arms_milk", "close_arms_milk" ],
            "cereal_box": ["close_arms_cereal", "open_both_hands","both_arms_cereal", "close_arms_cereal", "serve_cereal_0", "serve_cereal_1", "serve_cereal_2"]
        }
                
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        self.relative_drop_position=0.2
        
        self.crouch_4_drop = -0.4
        
        # ---------------------------------------------------------------------------
        #                       SERVICES
        # ---------------------------------------------------------------------------
        
        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_orthogonal_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv")
        self.set_orthogonal_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv",set_security_distance_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_tangential_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_tangential_security_distance_srv")
        self.set_tangential_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_tangential_security_distance_srv",set_security_distance_srv)
        
        rospy.wait_for_service("/pytoolkit/ALMotion/set_angle_srv")
        self.set_angle_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_angle_srv",set_angle_srv)
        
        
        
    # ---------------------------------------------------------------------------
    #                       ESTADOS / TRANSICIONES
    # ---------------------------------------------------------------------------

    def on_enter_INIT(self):
        self.tm.go_to_pose("standard", self.normal_movement)
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.talk("Hi! Today I would serve you a cereal", "English", wait=False)
        self.tm.set_current_place("entrance")
        self.tm.initialize_pepper()
        self.start()



    def on_enter_GO_TO_CUPBOARD(self):
        print(self.consoleFormatter.format("GO_TO_CUPBOARD", "HEADER"))
        self.tm.talk("Now I will go to the Kitchen", "English", wait=False)
        self.set_orthogonal_security_srv(0.3)
        self.set_tangential_security_srv(0.05)
        self.tm.go_to_place("kitchen", lower_arms=False)
        self.scan_for_objects()

    
    
    def on_enter_LOOK_4_ITEM(self):
        print(self.consoleFormatter.format("LOOK_4_ITEM", "HEADER"))
        self.actual_item=self.items[0]
        self.tm.talk(f"I will look for the {self.actual_item}.", "English", wait=False)
        actual_relative_angle = self.relative_item_angle[self.actual_item]
        self.tm.go_to_relative_point(0.0,0.0,actual_relative_angle)
        response = self.tm.img_description(
                f"Describe the {self.actual_item} you see in front of you, add some details about the size and color. "
                "Then describe the place where it is located naming objects around it. "
                "Don't add any other words to your response. Give a short answer of maximum 2 lines. "
                "If you don't see a {object_name}, answer just with an empty string and don't say sorry or anything more.",
                "front_camera")
        if response["message"] != "":
            self.tm.talk(f"{response["message"]}. Just right there", "English", wait=False)
        else:
            self.tm.talk(f"I see the {self.actual_item} right there", "English", wait=False)
        self.tm.go_to_pose("point_there", self.normal_movement)
        self.items.pop(0)
        self.grab_ingredient()



    def on_enter_GRAB_OBJECT(self):
        print(self.consoleFormatter.format("GRAB_OBJECT", "HEADER"))
        actions = self.grab_items_poses[self.actual_item]
        self.tm.show_image(f"http://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/grab_{self.actual_item}.jpeg")
        self.tm.go_to_pose("both_arms_cereal")
        
        if self.actual_item == "bowl":
            self.tm.talk("Could you please put the spoon in the bowl?", "English", wait=False)     
            rospy.sleep(4)
            self.tm.talk("Thank you!", "English", wait=False) 
        
        for action in actions:
            if action == "mid_arms_bowl" :
                self.tm.talk("Please place the bowl in my hands just like the image in my tablet shows", wait=False)

            if action == "both_arms_cereal" or action == "both_arms_milk": 
                self.tm.talk(f"Please hold the {self.actual_item} in middle of my hands like the image in my tablet shows! I will tell you when to stop holding it.", "English", wait=False)   
            
            self.tm.go_to_pose(action, self.slow_movement)
            rospy.sleep(2)
            
            if action  == "raise_arms_cereal" or action  == "raise_arms_milk" or action  == "raise_arms_bowl":
                self.carry_to_serve = True
                carry_thread = threading.Thread(target=self.carry_thread,args=[action])
                carry_thread.start()

        self.tm.show_words_proxy()
        self.go_to_drop_place()
        


    def on_enter_GO_TO_DROP_PLACE(self):
        print(self.consoleFormatter.format("GO_TO_DROP_PLACE", "HEADER"))
        self.tm.talk("On my way to the dining room", "English", wait=False)
        self.tm.set_move_arms_enabled(False)
        self.tm.go_to_place("dining", lower_arms=False)
        self.carry_to_serve = False
        self.drop_object()
        


    def on_enter_DROP(self):
        print(self.consoleFormatter.format("DROP", "HEADER"))
        self.tm.set_security_distance(False)
        drop_relative_point = self.drop_and_serve_position[self.actual_item]
        self.tm.go_to_relative_point(0.0, drop_relative_point, 0.0)
        actions = self.drop_items_poses[self.actual_item] 
        self.tm.go_to_relative_point(self.relative_drop_position, 0.0, 0.0)
        self.tilt_hip(self.crouch_4_drop)
        
        for action in actions:
            self.tm.go_to_pose(action, self.slow_movement)
            rospy.sleep(2)
            
        self.tm.go_to_relative_point(-(self.relative_drop_position), 0.0, 0.0)
        self.tm.go_to_pose("standard")
        
        if self.actual_item == "cereal_box" or self.actual_item == "milk_carton":
            self.serve()
        
        elif self.actual_item == "bowl":
            self.tm.set_security_distance(True)
            self.set_orthogonal_security_srv(0.3)
            self.set_tangential_security_srv(0.05)
            self.again()

            
    def on_enter_MAKE_BREAKFAST(self):
        print(self.consoleFormatter.format("MAKE_BREAKFAST", "HEADER"))
        self.tm.talk(f"I will serve the {self.actual_item}", "English", wait=False) 
        actions = self.serve_items_poses[self.actual_item]
        for action in actions:
            self.tm.go_to_pose(action, self.slow_movement)
            if action == "serve_cereal_3":
                self.shake_cereal()
            rospy.sleep(2)
        if self.items == []:
            self.end()
        else:
            self.again2()
        

    def on_enter_END(self):
        print(self.consoleFormatter.format("END", "HEADER"))
        self.tm.talk("I finished serving breakfast, enjoy it", "English", wait=False)
        return

    def check_rospy(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.console_formatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def carry_thread(self, pose):
        print(self.consoleFormatter.format("Carring", "OKBLUE"))
        while self.carry_to_serve:
            self.tm.go_to_pose(pose)
            rospy.sleep(1)
        
    def shake_cereal(self):
        print(self.consoleFormatter.format("Shaking", "OKBLUE"))
        shake_poses = ["shake_cereal_1", "shake_cereal_2"]
        for _ in range(10):
            for pose in shake_poses:
                self.tm.go_to_pose(pose, self.fast_movement)

    def run(self):
        while not rospy.is_shutdown():
            self.zero()
    # ---------------------------------------------------------------------------
    #                       FUNCIÓN PRINCIPAL
    # ---------------------------------------------------------------------------
    
if __name__ == "__main__":
    serve_breakfast = ServeBreakfast()
    serve_breakfast.run()
    rospy.spin()

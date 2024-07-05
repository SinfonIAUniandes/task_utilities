#!/usr/bin/env python3
import os
import ast
import rospy
import threading
import ConsoleFormatter

from transitions import Machine
from task_module import Task_module as tm

from robot_toolkit_msgs.srv import set_angle_srv

class STORING_GROCERIES(object):
    def __init__(self) -> None:
        
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, speech=True, perception = True, pytoolkit=True)
        self.tm.initialize_node('STORING_GROCERIES')
 
        self.STATES = ['INIT', 'GO_2_TABLE', 'ASK_E_GRAB_OBJECT', 'GO_2_CABINET', 'PLACE_OBJECT', 'BACK_2_TABLE','END'] 

        self.TRANSITIONS = [
            {'trigger': 'zero', 'source': 'STORING_GROCERIES', 'dest': 'INIT'},
            {'trigger': 'go_2_table', 'source': 'INIT', 'dest': 'GO_2_TABLE'},
            {'trigger': 'analice_objects', 'source': 'GO_2_TABLE', 'dest': 'ANALIZE_OBJECTS'},
            {'trigger': 'ask_e_grab', 'source': 'ANALIZE_OBJECTS', 'dest': 'ASK_E_GRAB_OBJECT'},
            {'trigger': 'go_2_cabinet', 'source': 'ASK_E_GRAB_OBJECT', 'dest': 'GO_2_CABINET'},
            {'trigger': 'place_object', 'source': 'GO_2_CABINET', 'dest': 'PLACE_OBJECT'},
            {'trigger': 'back_2_table', 'source': 'PLACE_OBJECT', 'dest': 'BACK_2_TABLE'},
            {'trigger': 'new_object', 'source': 'BACK_2_TABLE', 'dest': 'ASK_E_GRAB_OBJECT'},
            {'trigger': 'finish', 'source': 'PLACE_OBJECT', 'dest': 'END'},
            ]
        
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='STORING_GROCERIES')
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                           ROS
        # -------------------------------------------------------------------------------------------------------------------------------------------

        rospy.wait_for_service("/pytoolkit/ALMotion/set_angle_srv")
        self.set_angle_srv = rospy.ServicePcategorizeroxy("/pytoolkit/ALMotion/set_angle_srv",set_angle_srv)
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                           PARÁMETROS AJUSTABLES
        # -------------------------------------------------------------------------------------------------------------------------------------------

        # Movement Parameters
        self.fast_movement = 0.15
        self.normal_movement = 0.1
        self.slow_movement = 0.05
        
        # Relative angle to cabinet from the main table
        self.cabinet_angle = -90
        
        # Relative distance to cabinet from the main table
        self.cabinet_approach_distance = 0.1
        
        # Cabinets categories
        self.cabinets_categories = ["sports stuff","healthy food","junk food"]
        
        # Cabinets and altures
        self.cabinets = {
            "sports stuff": 0.3,
            "healthy food": 0.0,
            "junk food":  -0.3
        }
        
        # Objects list (just in case gpt vision fails)
        self.objects_list = ["apple", "tenis ball", "ketchup", "tuna can", "orange juice"]
        
        # Grab actions list dictionary by object
        self.grab_actions = {
            "small_object":["prepare_2_grab_small", "bowl_hands"],
            "one_hand":["grab_one_hand", "close_both_hands"]
        }
        
        # Relative distance to place the object
        self.cabinet_place_distance = 0.1
        
        # Drop actions list dictionary by object
        self.drop_actions = {
            "small_object":["open_both_hands","drop"],
            "one_hand":["open_both_hands"]
        }
        
        # -------------------------------------------------------------------------------------------------------------------------------------------
        #                                                            ESTADOS / TRANSICIONES
        # -------------------------------------------------------------------------------------------------------------------------------------------

    def on_enter_INIT(self): 
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.go_to_pose("standard", self.fast_movement)
        self.tm.go_to_pose("default_head", self.fast_movement)
        self.tm.set_current_place("house_door")
        self.tm.talk("Hello! I will assist you in storing groceries.", "English", wait=False)
        self.tm.initialize_pepper()
        self.start()
        
        
    
    def on_enter_GO_2_TABLE(self):
        print(self.consoleFormatter.format("GO_2_TABLE", "HEADER"))
        self.tm.go_to_place("dining")
        self.analice_objects()
        
        
    
    def on_enter_ANALIZE_OBJECTS(self):
        print(self.consoleFormatter.format("ANALIZE_OBJECTS", "HEADER"))
        objects_response = self.tm.img_description(
                "Please analyze the image and describe all the objects you see on the table. Focus on identifying each object, their characteristics, and their approximate locations on the table. Ignore any items that are not on the table. Provide a brief and clear description.",
                "front_camera"
                )
        objects_description = objects_response["message"]
        self.tm.talk(f"{objects_description}", "English", wait=False)
        objects_list_response = self.tm.img_description(
                f"Based on this '{objects_description}', return a Python list with the names of the objects you mentioned. Then analyze the image again to verify and ensure the list is accurate. Ignore any items that are not on the table. Provide your final response in the format of a Python list.",
                "front_camera"
                )
        objects_list = objects_list_response["message"]
        try:
            self.objects_list = eval(objects_list_response.get("message", "[]"))
            if not isinstance(objects_list, list):
                raise ValueError("Response is not a list")
        except (SyntaxError, ValueError) as e:
            print(f"Error parsing objects list: {e}")
    
        self.tm.talk(f"The list of objects is: {self.objects_list}", "English", wait=False)
        self.ask_e_grab()
        
        
    
    def on_enter_ASK_E_GRAB_OBJECT(self):
        print(self.consoleFormatter.format("ASK_E_GRAB_OBJECT", "HEADER"))
        self.actual_item= self.objects_list[0]
        categorize_prompt = (
        f"I will provide you with an item, and I need you to categorize this item strictly into one of the following categories: "
        f"{', '.join(self.cabinets_categories)}. Please categorize the following item: {self.actual_item}. "
        "Respond with only the category name from the list above."

        )
        self.actual_item_category = self.tm.answer_question(categorize_prompt, 0)
        
        self.tm.talk(f"Please help me take the {self.actual_item} from the table. I described it earlier.", "English", wait=False)
        rospy.sleep(6)
        self.tm.talk(f"Thank you!", "English", wait=False)
        
        
        object_type_prompt = (
        f"""
        I will provide you with an item and I need you to categorize this item strictly into one of the following categories: 'small_object' or 'one_hand'. Please use only one of these two categories in your response, and respond with the category name exactly as it is listed.

        - 'small_object': Small but relatively heavy or long items, such as a can, a beverage, or a banana.
        - 'one_hand': Small and lightweight items that can be comfortably held with one hand, such as a fruit like an apple or a small ball.

        Here is the item: {self.actual_item}
        """
        )
        
        self.actual_item_type = self.tm.answer_question(object_type_prompt, 0)
        
        actions = self.grab_actions[self.actual_item_type]
        self.tm.show_image(f"http://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/grab_{self.actual_item}.jpeg")
        
        for action in actions:
            if action == "grab_one_hand" or "prepare_2_grab_small":
                self.tm.talk(f"Please place the {self.actual_item} as the example is shown on my tablet until I can hold it properly.", "English", wait=False)
                rospy.sleep(2)
            self.tm.go_to_pose(action,self.slow_movement)
                
        self.tm.talk("Thank you!", "English", wait=False)
        self.go_2_cabinet()
        
        
        
    def on_enter_GO_2_CABINET(self):
        print(self.consoleFormatter.format("GO_2_CABINET", "HEADER"))
        self.tm.go_to_relative_point(0.0, 0.0, self.cabinet_angle)
        rospy.sleep(2)
        self.tm.go_to_relative_point(self.cabinet_approach_distance, 0.0, 0.0)
        self.tm.talk(f"I have arrived at the cabinet. Now, I'm going tell you how to place the {self.actual_item} on the {self.actual_item_category} shelf.", "English", wait=False)
        self.place_object()
        
        
        
    def on_enter_PLACE_OBJECT(self):
        print(self.consoleFormatter.format("PLACE_OBJECT", "HEADER"))  
        self.tm.talk(f"Please take the {self.actual_item} from my hand. Then, I will point to the {self.actual_item_category} shelf where you need to place the {self.actual_item}.", "English", wait=False)
        rospy.sleep(2)
        self.tm.go_to_pose("open_both_hands", 0.01)
        rospy.sleep(9)
        point_angle = self.cabinets[self.actual_item_category]
        self.set_angle_srv((["LShoulderPitch"], [point_angle], self.normal_movement))
        rospy.sleep(5)
        self.tm.talk(f"Perfect! Now follow me to grab the next item", "English", wait=False)
        
        self.objects_list.pop(0)
        self.tm.go_to_pose("standard")
        
        if self.objects_list == []:
            self.finish()
        else:
            self.back_2_table()
            
            
        
    def on_enter_BACK_2_TABLE(self):
        print(self.consoleFormatter.format("BACK_2_TABLE", "HEADER"))
        self.tm.go_to_relative_point((self.cabinet_approach_distance), 0.0, 0.0)
        rospy.sleep(2)
        self.tm.go_to_relative_point(0.0, 0.0, -(self.cabinet_angle))
        self.new_object()
        


    def on_enter_END(self):
        print(self.consoleFormatter.format("END", "HEADER"))
        self.tm.talk("I have finished storing the groceries.", "English", wait=False)
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
    #                                                         FUNCIÓN PRINCIPAL
    # -------------------------------------------------------------------------------------------------------------------------------------------
    
if __name__ == "__main__":
    sm = STORING_GROCERIES()
    sm.run()
    rospy.spin()

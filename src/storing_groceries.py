#!/usr/bin/env python3
import os
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
 
        self.STATES = ['INIT', 'GO_2_TABLE', 'ASK_E_GRAB_OBJECT', 'GO_2_CABINET','ANALIZE_OBJECTS','CATEGORIZE_CABINETS' ,'PLACE_OBJECT', 'BACK_2_TABLE','END'] 

        self.TRANSITIONS = [
            {'trigger': 'zero', 'source': 'STORING_GROCERIES', 'dest': 'INIT'},
            {'trigger': 'go_2_table', 'source': 'INIT', 'dest': 'GO_2_TABLE'},
            {'trigger': 'analice_objects', 'source': 'GO_2_TABLE', 'dest': 'ANALIZE_OBJECTS'},
            {'trigger': 'ask_e_grab', 'source': 'ANALIZE_OBJECTS', 'dest': 'ASK_E_GRAB_OBJECT'},
            {'trigger': 'go_2_cabinet', 'source': 'ASK_E_GRAB_OBJECT', 'dest': 'GO_2_CABINET'},
            {'trigger': 'categorize', 'source': 'GO_2_CABINET', 'dest': 'CATEGORIZE_CABINETS'},
            {'trigger': 'place_first', 'source': 'CATEGORIZE_CABINETS', 'dest': 'PLACE_OBJECT'},
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
        self.set_angle_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_angle_srv",set_angle_srv)
        
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
        self.cabinet_approach_distance = 0.3
        
        # Cabinets categories
        self.cabinets_categories = ["sports stuff","healthy food","junk food"]
        
        self.categories = ["","",""]
        
        # Cabinets and altures
        self.cabinets = {
            self.categories[0]: 0.6,
            self.categories[0]: 0.0,
            self.categories[0]:  -0.6
        }
        
        # Objects list (just in case gpt vision fails)
        self.objects_list = ["apple", "tenis ball", "ketchup", "tuna can", "orange juice"]
        
        # Grab actions list dictionary by object
        self.grab_actions = {
            "small_object":["prepare_2_grab_small", "bowl_hands"],
            "one_hand":["grab_one_hand", "close_both_hands"]
        }
        
        
        # Drop actions list dictionary by object
        self.drop_actions = {
            "small_object":["open_both_hands","drop"],
            "one_hand":["open_both_hands"]
        }
        
        self.first_time = True
        self.storing_objects_counter = 0
        
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
        self.go_2_table()
        
        
    
    def on_enter_GO_2_TABLE(self):
        print(self.consoleFormatter.format("GO_2_TABLE", "HEADER"))
        self.tm.go_to_place("dining")
        self.analice_objects()
        
        
    
    def on_enter_ANALIZE_OBJECTS(self):
        print(self.consoleFormatter.format("ANALIZE_OBJECTS", "HEADER"))
        self.tm.set_security_distance(False)
        self.tm.go_to_pose("down_head")
        self.tm.talk("I arrived at the table, I will now look for 5 items to store them in the shelf, for that I will need your valuable help!", language="English", wait=False)
     
        self.ask_e_grab()
        
        
    
    def on_enter_ASK_E_GRAB_OBJECT(self):
        print(self.consoleFormatter.format("ASK_E_GRAB_OBJECT", "HEADER"))
        
        object_chosen_request = """Please look at the table that is in front of you, the closest items and don't mind about the other items in the back of the picture like computers or people.
        Now, you will notice that there are a set of objects over a table. I need you to pick only one of them and describe it.
        
        Answer ONLY with the following structure: "{object name},I can see there is a {object size} {object name} in the table in front of me. It is located {describe of the items surrounding it, and its relative position in the table, be concrete and use few words}. Let's store it in the shelf!,{object type}"
        
        Replace the parts inside { } with text according to what you see and the object you chose. The {object name} must be the same in both cases.
        In {object type} please answer small_object or one_hand without '' or "" depending if you consider that object can be grabbed by a pepper robot with one hand or with both hands in a bowl position.
        Return your response as a single string and the three parts separated by one comma and no spaces after or before the commas. Dont include additional text.
        I will do a .split(",") to your response with python, so leave it in the best structure to split threse 3 parts of your response and get a list with three strings in my program.
        
        Remember to answer with the structure provided and nothing more. Don't add extra spaces between the commas.
        
        """
        
        object_chosen_answer = self.tm.img_description(object_chosen_request, "front_camera")
        
        # Splitting the answers of the prompt
        answer_dict = object_chosen_answer["message"].split(",")
        
        # Chosen object
        object_chosen = answer_dict[0]
        
        # Object description
        object_description = answer_dict[1]
        self.tm.talk(object_description, "English", True)
        
        # Object type
        object_type = answer_dict[2]
        
        # Actual item
        self.actual_item = object_chosen
        
        # Actual item type
        self.actual_item_type = object_type
        print("The chosen object: " + object_chosen + " was categorized as a: " + self.actual_item_type + " object.")
        
        self.tm.talk(f"Please help me take {self.actual_item} from the table and leave it in your hands for a second.", "English", wait=True)
        rospy.sleep(5)
        
        self.tm.talk(f"Thank you!", "English", wait=False)
        
        actions = self.grab_actions[self.actual_item_type]
        
        self.tm.show_image(f"http://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/grab_{self.actual_item}.jpeg")
        
        self.tm.talk(f"Now please place the {self.actual_item} as the example I am showing in my tablet until I can hold it properly.", "English", wait=False)
        
        for action in actions:
            if action == "prepare_2_grab_small" or action == "grab_one_hand":
                rospy.sleep(6)
                
            self.tm.go_to_pose(action,self.normal_movement)
            rospy.sleep(2)
            
        rospy.sleep(4)
        self.tm.talk("Thank you very much for your help!", "English", wait=False)
        self.go_2_cabinet()
        
    
    def on_enter_GO_2_CABINET(self):
        print(self.consoleFormatter.format("GO_2_CABINET", "HEADER"))
        self.tm.go_to_pose("up_head")
        self.tm.go_to_relative_point(0.0, 0.0, self.cabinet_angle)
        rospy.sleep(2)
        self.tm.go_to_relative_point(self.cabinet_approach_distance, 0.0, 0.0)
        self.tm.talk(f"I have arrived at the shelf. Now I need your help to place the {self.actual_item}. I'm going to tell you how to place it inside the shelf.", "English", wait=False)
        
        if self.first_time:
            self.categorize()
        else:
            self.place_object()
    
    
    
    def on_enter_CATEGORIZE_CABINETS(self):
        print(self.consoleFormatter.format("CATEGORIZE_CABINETS", "HEADER"))
        
        category_recognition_request = self.tm.img_description(
                f"'I will show you an image of a piece of furniture (like a shelf) with cabinets. Inside these cabinets, there are various objects stored in different places according to their object category. I need you to help me categorize this object: {self.actual_item} in one of those categories you saw. Return your answer ONLY with the following format: 'After analyzing the categories in the shelf, I recognized that the {self.actual_item} that I have in my hands belong to the _category_name_ category in the shelf','_category_name_'' . Replace _category_name_ with the items category you identified in the shelf in the both places it appears. Include the whole answer inside "". ",
                "front_camera"
                )
        
        category_recognition_answer = category_recognition_request["message"]
        category_recognition_answer = category_recognition_answer.split(",")
        
        # Category full answer
        category_full_answer = category_recognition_answer[0]
        
        # Object category
        self.object_category_name = category_recognition_answer[1]
        
        self.tm.talk(category_full_answer, "English", True)
        
        self.place_first()
        
        
    def on_enter_PLACE_OBJECT(self):
        
        print(self.consoleFormatter.format("PLACE_OBJECT", "HEADER"))
        
        self.tm.go_to_pose("standard")
        
        place_to_store_request = f"I need your help to give indications to a person to store an item in one place inside a shelf. The item is {self.actual_item} and it belongs to the {self.object_category_name} of objects, which are in one side inside the shelf. Provide relative indications to a person in order to place that item inside the shelf in a place where other items of that same category are located. Return your answer in the following format: 'Please take the {self.actual_item} from my hand and help me place it in the shelf with the other items of the same category. That category is located in _relative_location_ of the shelf!'. Please replace _relative_kocation_ with the relative position of those objects belonging to one same category, for example: The category is located in the top right part of the shelf, where you can see another objects like milk, chocolate and bread. Include the "" in your answer. Return your response as a single string with the three parts separated by one comma and no spaces after or before the commas. Dont include additional text."
        
        # Giving indications to store
        self.tm.talk(place_to_store_request, "English", False)
        rospy.sleep(5)
        
        # Updating the storing objects counter
        self.storing_objects_counter += 1
        
        for action in self.drop_actions:
                
            self.tm.go_to_pose(action,self.slow_movement)
            
            rospy.sleep(3)
        
        if self.storing_objects_counter == 5:
            self.finish()
        else:
            self.back_2_table()
            
            
        
    def on_enter_BACK_2_TABLE(self):
        print(self.consoleFormatter.format("BACK_2_TABLE", "HEADER"))
        self.tm.go_to_relative_point(-(self.cabinet_approach_distance), 0.0, 0.0)
        rospy.sleep(2)
        self.tm.go_to_relative_point(0.0, 0.0, -(self.cabinet_angle))
        self.new_object()
        


    def on_enter_END(self):
        print(self.consoleFormatter.format("END", "HEADER"))
        self.tm.talk("I have finished storing the groceries! Thanks for your help!", "English", wait=False)
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

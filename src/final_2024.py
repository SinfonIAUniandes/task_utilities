#!/usr/bin/env python3
from transitions import Machine
from openai import AzureOpenAI
import os
import threading
import datetime
from task_module import Task_module as tm
import ConsoleFormatter
import rospy
from robot_toolkit_msgs.msg import animation_msg, motion_tools_msg
from robot_toolkit_msgs.srv import motion_tools_srv

class DINNER_WITH_PEPPER(object):
    
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        self.task_name = "dinner_with_pepper"
        
        self.breads = ["wheat bread","garlic bread", "rice bread", "potato bread"]
        self.proteins = ["ham", "bacon", "eggs", "pork", "chicken"]
        self.vegetables = ["lettuce", "tomato", "onion", "olives", "spinach"]
        self.sauces = ["mayonnaise", "ketchup", "mustard"]
        self.available_ingredients = self.breads + self.proteins + self.vegetables + self.sauces + ["cheese"]
        self.order_taken = False
        
        self.context_prompt = """You are a Pepper robot, you work as chef aid. You are inside a house, the costumer is inside a room, while the chef is inside the kitchen. Your job is to take a personalized order from the costumer, including the ingredientes, the costumer's dietary reestrictions and preferences. You will have a short conversation with the costumer where he answers your questions about the order and tells you about his preferences. Then you will have to deliver the order to the chef and help them prepare the food. You will receive a personalized costumer's order and you will help the chef to prepare the food. You will have a short conversation with the chef where he tells you about the order and you will have to help him with the food preparation. The chef could ask you about the ingredients, the recipe or the cooking time. You will have to answer the chef's questions and help him with the food preparation. All your answers must be polite, amicable and short."""
        
        self.gpt_messages = [{"role":"system","content": self.context_prompt}]
        
        self.order = ""
        self.preferences = ""
        self.order_dict = {}

        # States
        states = ['INIT', 'WAIT4GUEST', 'TAKE_ORDER', 'GO2CHEF', "HELP_CHEF", "ORDER_READY", "DELIVER_ORDER", "END"]

        # Task Module initialization
        self.tm = tm(perception = True, speech=True, manipulation=True, navigation=False, pytoolkit=True)
        self.tm.initialize_node(self.task_name)

        # Transitions
        transitions = [
            {'trigger': 'start', 'source': 'DINNER_WITH_PEPPER', 'dest': 'INIT'},
            {'trigger': 'beginning', 'source': 'INIT', 'dest': 'WAIT4GUEST'},
            {'trigger': 'person_arrived', 'source': 'WAIT4GUEST', 'dest': 'TAKE_ORDER'},
            {'trigger': 'took_order', 'source': 'TAKE_ORDER', 'dest': 'GO2CHEF'},
            {'trigger': 'arrived_to_kitchen', 'source': 'GO2CHEF', 'dest': 'HELP_CHEF'},
            {'trigger': 'food_cooked', 'source': 'HELP_CHEF', 'dest': 'RECEIVE_FOOD'},
            {'trigger': 'food_received', 'source': 'RECEIVE_FOOD', 'dest': 'DELIVER_ORDER'},
            {'trigger': 'order_delivered', 'source': 'DELIVER_ORDER', 'dest': 'END'}
        ]

        # States machine initialization
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='DINNER_WITH_PEPPER')

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        self.clientGPT = AzureOpenAI(
            azure_endpoint= "https://sinfonia.openai.azure.com/",
            api_key= os.getenv("GPT_API"),
            api_version="2024-02-01",
        )
        self.animationPublisher = rospy.Publisher('/animations', animation_msg, queue_size=10)

        # ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        # Global Variables
        # TODO set initial place in NAVIGATION
        self.init_place = ""


    # Task States Implementation
    def on_enter_INIT(self):

        # Initializing Pepper
        self.tm.initialize_pepper()
        self.tm.turn_camera("depth_camera","custom",1,15)
        self.motion_tools_service()
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.set_current_place("hallway")
        rospy.sleep(3)
        anim_msg = self.gen_anim_msg("Gestures/BowShort_3")
        self.animationPublisher.publish(anim_msg)
        self.tm.talk("Hello everyone! my name is Pepper. I am a social robot built to help people with different daily tasks.", animated=False, language = "English", wait = True)
        rospy.sleep(1)
        self.tm.talk("Today, I would love to help with taking some sandwich orders for people who don't have a clear idea about the menu.", animated=True, language = "English", wait = True)
        rospy.sleep(1)

        self.tm.toggle_filter_by_distance(True,2,["person"])
        # Going to the next state
        self.beginning()
        
    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.tm.talk("Waiting for customers.","English")
        rospy.sleep(1)
        self.tm.look_for_object("person")
        self.tm.wait_for_object(-1)
        self.tm.talk("Hello! I am Pepper. I will help you with your sandwich order.", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.person_arrived()

    def on_enter_TAKE_ORDER(self):
        print(self.consoleFormatter.format("TAKE_ORDER", "HEADER"))
        self.tm.talk("I will ask you some questions to know what ingredients you want in your sandwich.", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.tm.talk("You will be able to choose a bread, a protein, some vegetables, and some sauces. Then you will be able to make changes to your order.", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.tm.talk("Please say it to me loud and clear, and only when my eyes turn blue.", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        
        # TODO make ingredient options pictures for each ingredient type and show then in the tablet
        print(self.consoleFormatter.format("Asking for ingredients...", "HEADER"))
        self.ASK4BREAD()
        self.ASK4PROTEIN()
        self.ASK4VEGETABLES()
        self.ASK4SAUCES()
        self.ASK4CHEESE()
        self.ASK4CHANGES()
        
        self.gpt_messages.append({"role":"user","content": f"The order is the following: {str(self.order_dict)}"})
        rospy.sleep(1)
        self.gpt_messages.append({"role":"user","content": f"These are the costumer's preferences and changes: {self.preferences}"})
        rospy.sleep(1)
        self.gpt_messages.append({"role":"user","content": "Summarize the order for asking the chef for help. Your answer must begin with 'The order is a sandwich with'."})
        rospy.sleep(1)
        prediction= self.clientGPT.chat.completions.create(
                model="GPT-4o", 
                messages=self.gpt_messages, 
                temperature=0, 
                max_tokens=100
            )
        self.order = prediction.choices[0].message.content
        self.gpt_messages.append({"role":"user","content": f"The final version of the order is: {self.order}"})
        print(f"Order: {self.order}")
        self.order_taken = True
        self.tm.talk(f"I have taken your order. {self.order}.", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.took_order()

    def on_enter_GO2CHEF(self):
        print(self.consoleFormatter.format("GO2CHEF", "HEADER"))
        self.tm.talk("Alright! I will go and tell the chef your order, take a seat while the chef is cooking!",wait=False,animated=False)
        rospy.sleep(1)
        self.tm.go_to_place("kitchen_counter")
        self.arrived_to_kitchen()
        
    def on_enter_HELP_CHEF(self):
        print(self.consoleFormatter.format("HELP_CHEF" "HEADER"))
        self.tm.talk("Hello chef, it's nice to see you! I have an order for you.", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.tm.talk(self.order, animated = True, language = "English", wait = True)
        self.tm.talk("If you need help with the order you can ask me anytime by touching my head. Or you can tell me when the order is done to deliver it", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        helping_chef = True
        while helping_chef:
            self.tm.wait_for_head_touch(timeout=100000)
            self.tm.talk("Please tell me what you need!")
            chef_request_message = self.tm.speech2text_srv(seconds=0).lower()
            if ("done" in chef_request_message) or ("finished" in chef_request_message) or ("ready" in chef_request_message) or ("complete" in chef_request_message):
                helping_chef = False
            elif "today" in chef_request_message or "date" in chef_request_message:
                date = datetime.now()
                print(datetime.now())
                day = date.day
                month = date.month
                month_names_list = ['nan','enero','febrero','marzo','abril','mayo','junio','julio','agosto','septiembre','octubre','noviembre','diciembre']
                answer = f"Today is the {day} of {month_names_list[month]}"
            elif "hour" in chef_request_message:
                date = datetime.now()
                hour = date.hour
                minute = date.minute
                answer = f"Alright, its {hour}:{minute}"
            else:
                self.gpt_messages.append({"role":"user","content":chef_request_message})
                prediction = self.clientGPT.chat.completions.create(
                    model="GPT-4o", 
                    messages=self.gpt_messages, 
                    temperature=0, 
                    max_tokens=100
                )
                answer = prediction.choices[0].message.content
            self.tm.talk(answer, animated = True, language = "English", wait = True)
        self.food_cooked()

    def on_enter_RECEIVE_FOOD(self):
        print(self.consoleFormatter.format("RECEIVE_FOOD", "HEADER"))
        
        self.tm.go_to_pose("open_both_hands")
        self.tm.go_to_pose("carry")
        self.tm.talk("Alright i am ready to deliver this order, please put the dish in my hands, when you are finished, please touch my head!", animated = False, language = "English", wait = False)
        self.tm.wait_for_head_touch(timeout=100000, message="Touch my head to get going!")
        self.food_received()

    def on_enter_DELIVER_ORDER(self):
        print(self.consoleFormatter.format("DELIVER_ORDER", "HEADER"))
        
        self.tm.go_to_pose("close_both_hands")
        self.tm.talk("Thank you! I will now deliver this order", animated = False, language = "English", wait = True)
        rospy.sleep(1)
        self.tm.go_to_place("hallway",lower_arms = False)
        self.tm.talk("Here you go, the chef and I worked really hard on this sandwich, we hope that you like it!", language = "English", wait = True)
        rospy.sleep(1)
        self.tm.talk("Please take it and touch my head when you have done so!", language = "English", wait = False)
        head_touched = False
        while not head_touched:
            touched_head = self.tm.wait_for_head_touch(timeout=5000)
            self.tm.talk("Your order is ready, please take it", language="English", animated=False, wait=True)
        
        self.order_delivered()
        
    def on_enter_END(self):
        print(self.consoleFormatter.format("END", "HEADER"))
        
        self.tm.talk("Hope you enjoy sandwich! It was a pleasure serving you!", language="English", animated=True, wait=True)
        rospy.sleep(1)
        os._exit(os.EX_OK)
        
    def ASK4BREAD(self):
        breads_str = ", ".join(self.breads)
        self.tm.talk(f"What would you like to choose for your bread? The options are : {breads_str}. The rice bread and potato bread are gluten free.", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.order_dict["bread"] = self.listen_for_ingredient("bread")
        
    def ASK4PROTEIN(self):
        proteins_str = ", ".join(self.proteins)
        self.tm.talk(f"What would you like to choose for your protein? The options are {proteins_str}. If you don't want any protein, say no.", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.order_dict["protein"] = self.listen_for_ingredient("protein")
        
    def ASK4VEGETABLES(self):
        vegetebles_str = ", ".join(self.vegetables)
        self.tm.talk(f"What would you like to choose for your vegetables? The options are {vegetebles_str}. If you don't any vegetables, say no", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.order_dict["vegetables"] = self.listen_for_ingredient("vegetables")
        
    def ASK4SAUCES(self):
        sauces_str = ", ".join(self.sauces)
        self.tm.talk(f"What would you like to choose for your sauces? The options are {sauces_str}. If you don't want any sauces, say no", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.order_dict["sauces"] = self.listen_for_ingredient("sauces")
        
    def ASK4CHEESE(self):
        self.tm.talk("Would you like to add cheese to your sandwich?", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.order_dict["cheese"] = self.listen_for_ingredient("cheese")

    
    def listen_for_ingredient(self, ingredient_type):
        print(self.consoleFormatter.format(f"Listening for {ingredient_type}...", "HEADER"))
        rospy.sleep(1)
        touched_head = False
        while not touched_head:
            ingredient_order = self.tm.speech2text_srv(seconds=0).lower()
            self.tm.talk("Processing you answer", language = "English", wait=True)
            valid_ingredient = False
            ingredient = ""
            for available_ingredient in self.available_ingredients:
                if available_ingredient in ingredient_order:
                    valid_ingredient = True
                    ingredient = available_ingredient
            if ("no" in ingredient_order) or ("dont" in ingredient_order) or ("don't" in ingredient_order):
                ingredient = "no"
                self.tm.talk(f"I heard you don't want {ingredient_type} in your sandwich.", animated = True, language = "English", wait = True)
                rospy.sleep(1)
                if ingredient_type == "bread":
                    self.tm.talk("How curious, a sandwich without bread, but I will make sure to tell the chef.", animated = True, language = "English", wait = True)
            elif ingredient_type == "cheese" and ("yes" in ingredient or "yeah" in ingredient):
                ingredient = "yes"
                self.tm.talk(f"I heard you would like {ingredient_type} in your sandwich.", animated = True, language = "English", wait = True)
                rospy.sleep(1)
            elif not valid_ingredient:
                self.tm.talk(f"I am sorry, the requested ingredient is not available.", language = "English", wait=True)
            else:
                self.tm.talk(f"I heard you would like {ingredient} in your sandwich.", animated = True, language = "English", wait = True)
                rospy.sleep(1)
            self.tm.talk("If that is correct please touch my head, if not please wait until my eyes turn blue again", wait = False)
            touched_head = self.tm.wait_for_head_touch(timeout=5)
            if not touched_head:
                self.tm.talk("I am sorry, please repeat the ingredient","English", wait=True)
                rospy.sleep(1)
        print(f"{ingredient_type}: {ingredient}")
        return ingredient


    def ASK4CHANGES(self):
        current_order = self.order_dict
        self.tm.talk("Would you like to make any changes to your order? You can add or remove ingredients or make any other change you want. Remember to touch my head when you are satisfied with your order.", animated = True, language = "English", wait = True)
        rospy.sleep(1)
        self.gpt_messages.append({"role":"user","content": "You will now talk to the costumer about his order, you can ask him if he wants to make any other changes to the order. During the conversation you can ask the costumer about his dietary reestrictions, and about other preferences."})
        
        head_touched = False
        while not head_touched:
            self.tm.wait_for_head_touch(timeout=100000)
            self.tm.talk("Please tell me what changes you would like to make to your order.")
            rospy.sleep(1)
            changes = self.tm.speech2text_srv(seconds=0).lower()
            self.gpt_messages.append({"role":"user","content": changes})
            prediction = self.clientGPT.chat.completions.create(
                model="GPT-4o", 
                messages=self.gpt_messages, 
                temperature=0, 
                max_tokens=100
            )
            response = prediction.choices[0].message.content
            self.tm.talk(response, animated = True, language = "English", wait = True)
            rospy.sleep(1)
            head_touched = self.tm.wait_for_head_touch(timeosut=5)
        
        
    def motion_tools_service(self):
        """
        Enables the motion Tools service from the toolkit of the robot.
        """
        request = motion_tools_msg()
        request.command = "enable_all"
        print("Waiting for motion tools service")
        rospy.wait_for_service('/robot_toolkit/motion_tools_srv')
        try:
            motion = rospy.ServiceProxy('/robot_toolkit/motion_tools_srv', motion_tools_srv)
            motion(request)
            print("Motion tools service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")

    def check_rospy(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()
            
    def gen_anim_msg(self, animation):
        anim_msg = animation_msg()
        anim_msg.family = "animations"
        anim_msg.animation_name = animation
        return anim_msg

if __name__ == "__main__":
    sm = DINNER_WITH_PEPPER()
    sm.run()
    rospy.spin()
#!/usr/bin/env python3

# --------------- GENERAL LIBRARIES IMPORTS ---------------
import os
import re
import time
import math
import rospy
import threading
import ConsoleFormatter
import numpy as np
from transitions import Machine
from task_module import Task_module as tm

# --------------- MESSAGES AND SERVICES IMPORTS ---------------
from std_msgs.msg import String
from robot_toolkit_msgs.srv import set_angle_srvRequest

# --------------- RESTAURANT TASK CLASS DEFINITION ---------------
class RESTAURANT(object):
    
    # --------------- Constructor ---------------    
    def __init__(self) -> None:
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, perception=True, speech=True, pytoolkit=True)
        self.tm.initialize_node("restaurant")

        # --------------- Machine States ---------------

        self.STATES = [
            "INIT",
            "LOOK_FOR_CUSTOMER",
            "GO2CUSTOMER",
            "TAKE_ORDER",
            "GO_BACK",
            "SERVE_ORDER",
            "END",
        ]
        
        # --------------- Machine Transitions ---------------
        
        self.TRANSITIONS = [
            {"trigger": "zero", "source": "RESTAURANT", "dest": "INIT"},
            {"trigger": "start", "source": "INIT", "dest": "LOOK_FOR_CUSTOMER"},
            {"trigger": "found_customer", "source": "LOOK_FOR_CUSTOMER", "dest": "GO2CUSTOMER"},
            {"trigger": "arrived_customer", "source": "GO2CUSTOMER", "dest": "TAKE_ORDER"},
            {"trigger": "order_taken", "source": "TAKE_ORDER", "dest": "GO_BACK"},
            {"trigger": "arrived_bar", "source": "GO_BACK", "dest": "SERVE_ORDER"},
            {"trigger": "delivered_order", "source": "SERVE_ORDER", "dest": "GO_BACK"},
            {"trigger": "end", "source": "GO_BACK", "dest": "END"},
        ]
        
        # --------------- Machine Declaration ---------------

        self.machine = Machine(
            model=self,
            states=self.STATES,
            transitions=self.TRANSITIONS,
            initial="RESTAURANT",
        )
        
        # --------------- Robot State Variables ---------------
        
        self.choosing = False
        
        
        # --------------- Subscribers ---------------
        
        self.posePublisherSubscriber = rospy.Subscriber(
            "perception_utilities/pose_publisher", String, self.pose_publisher_callback
        )
        
        # --------------- Absolute Variables ---------------
        
        #self.angles_to_check = [0,-30,-60,30,60]
        self.angles_to_check = [30]
        self.hand_raised = False
        self.gpt_hand_raised_id = -1
        self.customer_count = 0 
        self.current_customer = 0
        self.resolution = 2
        self.orders = []
        self.customers_coordinates = []
        self.delivering = False
        self.taking_order = False
        self.pose = "small_object_left_hand"
        self.menu = ["bottle of water, orange, apple, pear"]
        self.customer1_counter = 0
        self.customer2_counter = 0
        
        # --------------- ROSPY Check ---------------
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()


    # --------------------------------------------- STATE FUNCTIONS DEFINITION ---------------------------------------------
     
    # --------------- FIRST STATE: INIT ---------------
    
    def on_enter_INIT(self):
        
        self.tm.initialize_pepper()
        if self.resolution == 2:
            self.tm.turn_camera("front_camera","custom",2,10)
            self.tm.turn_camera("depth_camera","custom",1,10)
        else:
            self.tm.turn_camera("depth_camera","custom",1,15)
        self.tm.pose_srv("front_camera", True)
        
        # --------------- INIT STATE PROCCEDURE ---------------
        
        current_position = self.tm.get_absolute_position_proxy()
        current_angle = current_position.theta
        print("adding place:bar_table")
        self.tm.add_place("bar_table",with_coordinates=True,x=20,y=20,theta=180+current_angle)
        self.tm.set_current_place("bar_table")
        rospy.sleep(1)

        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.talk("Hello i am ready to help the restaurant", wait=True)
        
        # Moving to LOOK_FOR_BAG state
        self.start()



    # --------------- SECOND STATE: LOOK FOR BAG ---------------
    
    def on_enter_LOOK_FOR_CUSTOMER(self):
        
        print(self.consoleFormatter.format("LOOK_FOR_CUSTOMER", "HEADER"))
        self.tm.talk("I am looking for calling customers",wait=False)
        self.tm.setRPosture_srv("stand")
        
        self.choosing = False
        
        for attempt in [1,2,3]:
            if self.customer_count == 1:
                break
            self.look_around()
            
        self.tm.setRPosture_srv("stand")
        self.found_customer()
        

    # --------------- THIRD STATE: GO 2 CUSTOMER ---------------

    def on_enter_GO2CUSTOMER(self):
        print(self.consoleFormatter.format("GO2CUSTOMER", "HEADER"))
        self.current_customer += 1
        self.tm.posture_thread = True
        self.arrived_customer()
        
    
    # --------------- FOURTH STATE: TAKE ORDER ---------------
    
    def on_enter_TAKE_ORDER(self):
        
        print(self.consoleFormatter.format("TAKE_ORDER", "HEADER"))
        self.tm.talk("Hello, i am Pepper and i will take your order, please tell me what you want when my eyes are blue!",wait=True)
        order = self.tm.speech2text_srv(0)
        self.tm.talk(f"Your order is: {order}.",wait=True)
        self.tm.talk("If that is correct, please touch my head. If not, please wait until my eyes are blue again","English", wait=False)
        
        correct = self.tm.wait_for_head_touch(message="", message_interval=100, timeout=13)
        
        while not correct:
            self.tm.talk("I am sorry, may you please repeat your order?","English", wait=True)
            order = self.tm.speech2text_srv(0)
            
            self.tm.talk(f"Your order is: {order}.",wait=True)
            self.tm.talk("If that is correct, please touch my head. If not, please wait until my eyes are blue again","English", wait=False)
            correct = self.tm.wait_for_head_touch(message="", message_interval=13, timeout=13)
        
        self.orders.append(order)
        
        # Moving to GO_BACK state
        self.order_taken()

    # --------------- FIFTH STATE: GO BACK ---------------
    
    def on_enter_GO_BACK(self):
        
        print(self.consoleFormatter.format("GO_BACK", "HEADER"))
        
        self.tm.setRPosture_srv("stand")
        self.tm.talk("I am going back to the bar",wait=False)
        
        print("adding place:last_customer_place"+ str(self.customer_count))
        self.tm.add_place(
                    "last_customer_place"+ str(self.customer_count),
                    edges=["customer_place"+ str(self.customer_count) + str(self.customer1_counter - 1)]
                )
        
        self.tm.robot_stop_srv()
        self.tm.set_current_place("last_customer_place"+ str(self.customer_count))
        
        # Going back to the initial position
        self.tm.go_to_place("bar_table",graph=1)
        
        self.tm.posture_thread = False
        
        self.tm.setRPosture_srv("stand")
        
        self.arrived_bar()


    # --------------- SIXTH STATE: SERVE ORDER ---------------
    
    def on_enter_SERVE_ORDER(self):
        
        print(self.consoleFormatter.format("SERVER ORDER", "HEADER"))
        
        self.tm.talk("I'm back at the bar",wait=False)
        
        self.tm.talk(f"Hello barman, my name is pepper! I have a couple of orders from customers in the restaurant!",wait=True)
        
        self.tm.talk("I am going to deliver these items one by one, please hand them to me and touch my head when you've placed them in my hand",wait=True)
        
        for order in self.orders:
            self.tm.talk(f"The order is: {order}",wait=True)
            self.tm.go_to_pose("small_object_left_hand")
            self.pose = "small_object_left_hand"
            list_foods = self.obtener_lista_de_alimentos(order,self.menu)
            for food in list_foods:
                self.tm.talk(f"Listen carefully. Please hand me an {food}. Keep it in my hand until you've touched my head and then i will close my hand",wait=True)
                self.tm.wait_for_head_touch(message="Touch my head to get going!")
                self.tm.go_to_pose("close_both_hands")
                self.tm.talk("Thank you! I will now deliver this item", "English",wait=False)
                carry_thread = threading.Thread(target=self.carry_thread,args=[self.pose])
                carry_thread.start()
                self.tm.go_to_place("last_customer_place"+ str(self.customer_count))
            
            #self.tm.go_to_place("customer" + str(self.current_customer),graph=1)
            self.delivering = False
                
                
        self.delivered_order()
        
        
    # --------------------------------------------- ADITIONAL FUNCTIONS ---------------------------------------------

    def obtener_lista_de_alimentos(self,transcripcion, alimentos):
        # Lista para almacenar el resultado
        resultado = []

        # Iterar sobre cada alimento
        for alimento in alimentos:
            # Buscar coincidencias de cantidades con el alimento
            coincidencias = re.findall(rf"(\d+)\s*{alimento}", transcripcion)
            for coincidencia in coincidencias:
                # Añadir el alimento a la lista según la cantidad encontrada
                resultado.extend([alimento] * int(coincidencia))

        return resultado

    def look_around(self):
        
        for angle in self.angles_to_check:
            
            self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), 0],0.1)
            
            if angle == 0:
                rospy.sleep(1)
                
            elif angle == -30:
                rospy.sleep(1.5)
                
            elif angle == -60:
                rospy.sleep(1.5)
                
            elif angle == 30:
                rospy.sleep(2.5)
                
            elif angle == 60:
                rospy.sleep(2.5)
                
            self.tm.labels = dict()
            
            rospy.sleep(3)
                
            persons = self.tm.labels.get("person", [])
            for person in persons:
                self.tm.center_head_with_label(person,resolution = self.resolution, height=0)
                
                
                self.choosing = True
                
                verify_raised_hand_thread = threading.Thread(target=self.verify_raised_hand)
                verify_raised_hand_thread.start()
                self.hand_raised = False
                start_time = rospy.get_time()
        
                while not self.hand_raised and rospy.get_time() - start_time < 5:
                    rospy.sleep(0.1)
                    
                self.choosing = False
        
                if self.hand_raised or self.gpt_hand_raised_id == person[0]:
                    self.tm.talk(f"I found a calling customer",wait=False)
                    toggle_msg =  set_angle_srvRequest()
                    toggle_msg.name = ["HeadYaw"]
                    toggle_msg.angle = []
                    toggle_msg.speed = 0
                    self.tm.toggle_get_angles_topic_srv(toggle_msg)
                    current_position = self.tm.get_absolute_position_proxy()
                    current_angle = current_position.theta
                    person_x = person[1]
                    person_y = person[2]
                    person_width = person[3]*1.3
                    person_height = person[4]
                    rospy.sleep(3)
                    depth_from_current_place = self.tm.calculate_depth_of_label(person_x, person_y, person_width, person_height)
                    print(depth_from_current_place)
                    if depth_from_current_place == 0 or depth_from_current_place == np.nan:
                        self.tm.talk(f"But they are too far away for me to accurately reach them, i might need some help",wait=False)
                    else:
                        person_center = (person_x + person_width/2)
                        # 54 grados caben en la camara y hay 320 pixeles en resolution 1
                        factor = 54 / 320
                        if self.resolution == 2:
                            # 54 grados caben en la camara y hay 640 pixeles en resolution 2
                            factor = 54 / 640
                        person_degree_yolo = (person_center*factor) - 27
                        current_head_angle = self.tm.angles
                        person_degree = math.radians(person_degree_yolo - current_head_angle)
                        depth_from_current_place = depth_from_current_place - 0.3
                        if depth_from_current_place>5:
                            depth_from_current_place = depth_from_current_place -1
                        nav_x = depth_from_current_place*math.cos(person_degree)
                        nav_y = depth_from_current_place*math.sin(person_degree)
                        self.customers_coordinates.append([nav_x,-nav_y])
                        self.customer_count += 1
                        print("add place:","customer" + str(self.customer_count))
                        self.tm.add_place("customer_place" + str(self.customer_count) + str(self.customer1_counter), edges=["bar_table"])
                        self.customer1_counter += 1
                        print("depth:",depth_from_current_place)
                        print("degree:",person_degree)
                        print("x:",nav_x)
                        print("y:",-nav_y)
                        current_position = self.tm.get_absolute_position_proxy()
                        current_angle = current_position.theta
                        self.taking_order = True
                        save_place_thread = threading.Thread(target=self.save_place)
                        save_place_thread.start()
                        self.tm.Navigate_to_srv(nav_x,-nav_y)
                        self.taking_order = False
                        self.tm.go_to_defined_angle_srv(current_angle)
                        #self.tm.go_to_place("customer" + str(self.customer_count),graph=1)
                self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), 0],0.1)
                if self.customer_count == 1:
                    break   
            if self.customer_count == 1:
                break   

    # --------------- SAVE PLACE FUNCTION ---------------
    def save_place(self):
        
        # Adding the place where the robot is standing, along with the previous node
        
        while self.taking_order:
            
            if not self.tm.avoiding_obstacle:
                print("adding place:","customer_place"+ str(self.customer_count) + str(self.customer1_counter))
                self.tm.add_place(
                    "customer_place"+ str(self.customer_count) + str(self.customer1_counter),
                    edges=["customer_place"+ str(self.customer_count) + str(self.customer1_counter - 1)]
                )
                
                self.customer1_counter += 1
                
                # Saving each 2 seconds
                rospy.sleep(2)
                

    # --------------- POSE PUBLISHER CALLBACK FUNCTION ---------------
    def pose_publisher_callback(self, msg):
        if self.choosing:
            if "left" in msg.data.lower() or "right" in msg.data.lower():
                self.hand_raised = True
                
                
    # --------------- GPTVISION VERIFY RAISED HAND THREAD ---------------
    def verify_raised_hand(self):
        print(self.consoleFormatter.format("VERIFY RAISED HAND", "HEADER"))
        gpt_vision_prompt = f"If the person in the center is raising their hand return their ID as a single integer number"
        answer = self.tm.img_description(gpt_vision_prompt,camera_name="yolo")["message"].lower()
        print("gpt answer:",answer)
        if answer.isdigit():
            self.gpt_hand_raised_id = int(answer)
    
    # --------------- CARRY THREAD FUNCTION --------------- 
    def carry_thread(self, pose):
        while self.pose:
            self.tm.go_to_pose(pose)
            rospy.sleep(1)
                
    # --------------- MACHINE INITIALIZATION FUNCTION ---------------            
    def run(self):
        while not rospy.is_shutdown():
            self.zero()

    # --------------- CHECK ROSPY FUNCTION DEFINITION --------------- 
    def check_rospy(self):
        
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)


# --------------- INITIALIZATION OF THE TASK CLASS CONSTRUCTOR --------------- 
if __name__ == "__main__":
    sm = RESTAURANT()
    sm.run()
    rospy.spin()
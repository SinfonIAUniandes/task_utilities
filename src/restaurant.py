#!/usr/bin/env python3

# --------------- GENERAL LIBRARIES IMPORTS ---------------
import os
import time
import math
import rospy
import threading
import ConsoleFormatter
from transitions import Machine
from task_module import Task_module as tm

# --------------- MESSAGES AND SERVICES IMPORTS ---------------
from std_msgs.msg import String

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
            "SERVE_ORDER"
            "END",
        ]
        
        # --------------- Machine Transitions ---------------
        
        self.TRANSITIONS = [
            {"trigger": "zero", "source": "RESTAURANT", "dest": "INIT"},
            {"trigger": "start", "source": "INIT", "dest": "LOOK_FOR_CUSTOMER"},
            {"trigger": "found_customer()", "source": "LOOK_FOR_CUSTOMER", "dest": "GO2CUSTOMER"},
            {"trigger": "arrived_customer", "source": "GO2CUSTOMER", "dest": "TAKE_ORDER"},
            {"trigger": "order_taken()", "source": "TAKE_ORDER", "dest": "GO_BACK"},
            {"trigger": "arrived_bar", "source": "GO_BACK", "dest": "SERVE_ORDER"},
            {"trigger": "end", "source": "SERVE_ORDER", "dest": "END"},
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
        
        self.angles_to_check = [0,-60,60]
        self.hand_raised = False
        self.gpt_hand_raised_id = -1
        
        # --------------- ROSPY Check ---------------
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()


    # --------------------------------------------- STATE FUNCTIONS DEFINITION ---------------------------------------------
     
    # --------------- FIRST STATE: INIT ---------------
    
    def on_enter_INIT(self):
        
        self.tm.initialize_pepper()
        self.tm.turn_camera("depth_camera","custom",1,15)
        
        # --------------- INIT STATE PROCCEDURE ---------------
        
        current_position = self.tm.get_absolute_position_proxy()
        current_angle = current_position.theta
        print("adding place:bar_table")
        self.tm.add_place("bar_table",with_coordinates=True,x=20,y=20,theta=current_angle)
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
            
        print("Signaled")
        
        self.choosing = False
        
        for angle in self.angles_to_check:
            
            print("angulo actual:",angle)
            self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
            
            if angle==0:
                rospy.sleep(1)
                
            elif angle==-60:
                rospy.sleep(3)
                
            elif angle==60:
                rospy.sleep(5)
                
            self.tm.labels = dict()
            
            rospy.sleep(2)
                
            persons = self.tm.labels.get("person", [])
            
            for person in persons:
                
                print("centrando persona:",person)
                self.tm.center_head_with_label(person)
                
                print("revisando si esta la mano levantada")
                
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
                    current_position = self.tm.get_absolute_position_proxy()
                    current_angle = current_position.theta
                    person_x = person[1]
                    
                    self.tm.add_place("customer",with_coordinates=True,x=20,y=20,theta=current_angle)
                    
                print("calling customer found")
                self.tm.set_angles_srv(["HeadYaw","HeadPitch"],[math.radians(angle), -0.1],0.1)
            
        self.tm.setRPosture_srv("stand")
        
        # Moving to the GO2NEXT state
        self.rules_checked()
        self.grab_bag()
        

    # --------------- THIRD STATE: GRAB BAG ---------------

    def on_enter_GRAB_BAG(self):
        
        print(self.consoleFormatter.format("GRAB_BAG", "HEADER"))
        
        self.tm.set_move_arms_enabled(False)
        self.tm.show_image(f"https://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/carry_bag.jpeg")
        self.tm.talk("Please place the bag in my hand just like you can see in my tablet, when you have finished please touch my head!","English",wait=True)
        
        self.tm.wait_for_head_touch(message="Touch my head to get going!")
                
        # The pose is relative to the robot, so the opposite arm is used
        
        print("close hand:",self.hand_raised )
        
        if self.hand_raised=="right":
            self.tm.go_to_pose("close_left_hand")
            rospy.sleep(1)
            self.tm.go_to_pose("small_object_left_high_2")
            rospy.sleep(4)
            self.tm.go_to_pose("small_object_left_high_3")
            
        else:
            self.tm.go_to_pose("close_right_hand")
            rospy.sleep(1)
            self.tm.go_to_pose("small_object_right_high_2")
            rospy.sleep(4)
            self.tm.go_to_pose("small_object_right_high_3")
            
        # Thanking the user
        self.tm.show_words_proxy()
        self.tm.show_topic("/perception_utilities/yolo_publisher")
        self.tm.talk("Thank you!", "English",wait=False)
        self.follow_you()
        
    
    # --------------- FOURTH STATE: FOLLOW YOU ---------------
    
    def on_enter_FOLLOW_YOU(self):
        
        print(self.consoleFormatter.format("FOLLOW_YOU", "HEADER"))
        
        print("Follow you activated!")
        
        self.following = True
        
        # Following thread initialization
        carry_thread = threading.Thread(target=self.carry_thread,args=[self.pose])
        carry_thread.start()
        
        # Save place thread initialization
        save_place_thread = threading.Thread(target=self.save_place)
        save_place_thread.start()
        
        # Stopping the execution
        self.tm.follow_you(rotate=False)
        self.following = False
        
        self.tm.talk("We have arrived! Could you pick up your bag?", "English")
        self.tm.robot_stop_srv()
        
        # Openning hand to release the bag
        if self.hand_raised=="right":
            self.tm.go_to_pose("small_object_left_high_2")
            rospy.sleep(4)
            self.tm.go_to_pose("small_object_left_hand")
            rospy.sleep(4)
            self.tm.go_to_pose("open_left_hand")
            
        else:
            self.tm.go_to_pose("small_object_right_high_2")
            rospy.sleep(4)
            self.tm.go_to_pose("small_object_right_hand")
            rospy.sleep(4)
            self.tm.go_to_pose("open_right_hand")
            
        rospy.sleep(5)
        # Thanking the user 
        self.tm.talk("Thank you for using my services, have a nice day!",wait=False)
        self.tm.set_move_arms_enabled(True)
        
        # Moving to GO_BACK state
        self.go_back()


    # --------------- FIFTH STATE: GO BACK ---------------
    
    def on_enter_GO_BACK(self):
        
        print(self.consoleFormatter.format("GO_BACK", "HEADER"))
        
        self.tm.setRPosture_srv("stand")
        self.tm.talk("I am going back to my initial position",wait=False)
        
        self.tm.posture_thread = True
        posture_thread = threading.Thread(target=self.tm.posture_srv_thread)
        posture_thread.start()
        
        # --------------- PARAMETERS ---------------
        
        # Super safe parameters
        self.tm.set_orthogonal_security_srv(0.3)
        self.tm.set_tangential_security_srv(0.05)
        
        print("adding place:last_place")
        self.tm.add_place(
                    "last_place",
                    edges=["place" + str(self.place_counter - 1)],
                )
        
        self.tm.robot_stop_srv()
        self.tm.set_current_place("last_place")
        self.tm.robot_stop_srv()
        
        # Going back to the initial position
        self.tm.go_to_place("place0",graph=1)
        
        
        self.tm.posture_thread = False
        
        self.tm.setRPosture_srv("stand")
        # Task completed
        self.tm.talk("carry my luggage task completed succesfully",wait=False)
        
        # Finish the execution
        os._exit(os.EX_OK)
        
        
    # --------------------------------------------- ADITIONAL FUNCTIONS ---------------------------------------------


    def execute_after_delay(self):
        print("timeout")
        self.tm.robot_stop_srv()
        self.tm.robot_stop_srv()
        self.tm.robot_stop_srv()
        self.tm.posture_thread = False
        self.tm.setRPosture_srv("stand")
        # Task completed
        self.tm.talk("carry my luggage task completed succesfully",wait=False)
        
        # Finish the execution
        os._exit(os.EX_OK)

    # --------------- SAVE PLACE FUNCTION ---------------
    def save_place(self):
        
        # Adding the place where the robot is standing, along with the previous node
        
        while self.following:
            
            if not self.tm.avoiding_obstacle:
                current_position = self.tm.get_absolute_position_proxy()
                current_x = current_position.x
                current_y = current_position.y
                current_angle = current_position.theta
                print("adding place:","place" + str(self.place_counter))
                self.tm.add_place(
                    "place" + str(self.place_counter),
                    edges=["place" + str(self.place_counter - 1)],
                    with_coordinates=True,
                    x=current_x,
                    y=current_y,
                    theta=current_angle
                )
                
                self.place_counter += 1
                
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
    
    # --------------- CARRY THREAD FUNCTION --------------- 
    def carry_thread(self, pose):
        
        while self.following:
            self.tm.go_to_pose(pose)
            rospy.sleep(1)


# --------------- INITIALIZATION OF THE TASK CLASS CONSTRUCTOR --------------- 
if __name__ == "__main__":
    sm = CARRY_MY_LUGGAGE()
    sm.run()
    rospy.spin()
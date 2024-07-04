#!/usr/bin/env python3

# --------------- GENERAL LIBRARIES IMPORTS ---------------
import os
import rospy
import threading
import ConsoleFormatter
import time
from transitions import Machine
from task_module import Task_module as tm

# --------------- MESSAGES AND SERVICES IMPORTS ---------------
from std_msgs.msg import String
from std_srvs.srv import SetBool
from robot_toolkit_msgs.msg import touch_msg
from robot_toolkit_msgs.srv import (
    set_move_arms_enabled_srv,
    set_security_distance_srv
)
from robot_toolkit_msgs.srv import (
    tablet_service_srv,
    move_head_srv,
    battery_service_srv,
)
from robot_toolkit_msgs.msg import (
    touch_msg,
    speech_recognition_status_msg)


# --------------- CARRY MY LUGGAGE TASK CLASS DEFINITION ---------------
class CARRY_MY_LUGGAGE(object):
    
    # --------------- Constructor ---------------    
    def __init__(self) -> None:
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, perception=True, speech=True, pytoolkit=True)
        self.tm.initialize_node("carry_my_luggage")

        # --------------- Machine States ---------------

        self.STATES = [
            "INIT",
            "LOOK_FOR_BAG",
            "GRAB_BAG",
            "FOLLOW_YOU",
            "GO_BACK",
            "END",
        ]
        
        # --------------- Machine Transitions ---------------
        
        self.TRANSITIONS = [
            {"trigger": "zero", "source": "CARRY_MY_LUGGAGE", "dest": "INIT"},
            {"trigger": "start", "source": "INIT", "dest": "LOOK_FOR_BAG"},
            {"trigger": "grab_bag", "source": "LOOK_FOR_BAG", "dest": "GRAB_BAG"},
            {"trigger": "grab_bag_again", "source": "GRAB_BAG", "dest": "GRAB_BAG"},
            {"trigger": "follow_you", "source": "GRAB_BAG", "dest": "FOLLOW_YOU"},
            {"trigger": "go_back", "source": "FOLLOW_YOU", "dest": "GO_BACK"},
            {"trigger": "end", "source": "GO_BACK", "dest": "END"},
        ]
        
        # --------------- Machine Declaration ---------------

        self.machine = Machine(
            model=self,
            states=self.STATES,
            transitions=self.TRANSITIONS,
            initial="CARRY_MY_LUGGAGE",
        )
        
        # --------------- ROSPY Services ---------------

        self.setMoveArms_srv = rospy.ServiceProxy(
            "pytoolkit/ALMotion/set_move_arms_enabled_srv", set_move_arms_enabled_srv
        )

        print(
            self.consoleFormatter.format(
                "Waiting for pytoolkit/awareness...", "WARNING"
            )
        )
        rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        self.awareness_srv = rospy.ServiceProxy(
            "/pytoolkit/ALBasicAwareness/set_awareness_srv", SetBool
        )

        print(
            self.consoleFormatter.format(
                "Waiting for pytoolkit/ALMotion/move_head...", "WARNING"
            )
        )
        rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        self.move_head_srv = rospy.ServiceProxy(
            "/pytoolkit/ALMotion/move_head_srv", move_head_srv
        )

        print(
            self.consoleFormatter.format(
                "Waiting for pytoolkit/show_topic...", "WARNING"
            )
        )
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
        self.show_topic_srv = rospy.ServiceProxy(
            "/pytoolkit/ALTabletService/show_topic_srv", tablet_service_srv
        )
        
        print(
            self.consoleFormatter.format(
                "Waiting for pytoolkit/stop_tracker...", "WARNING"
            )
        )
        rospy.wait_for_service("/pytoolkit/ALTracker/stop_tracker_srv")
        self.stop_tracker_proxy = rospy.ServiceProxy(
            "/pytoolkit/ALTracker/stop_tracker_srv", battery_service_srv
        )
        
        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_orthogonal_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv")
        self.set_orthogonal_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_orthogonal_security_distance_srv",set_security_distance_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_tangential_security_distance_srv...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/set_tangential_security_distance_srv")
        self.set_tangential_security_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/set_tangential_security_distance_srv",set_security_distance_srv)
        
        
        # --------------- Robot State Variables ---------------
        
        self.choosing = False
        
        # --------------- Subscribers ---------------
        
        self.posePublisherSubscriber = rospy.Subscriber(
            "perception_utilities/pose_publisher", String, self.pose_publisher_callback
        )
        
        # --------------- Absolute Variables ---------------
        
        self.bag_place = "none"
        self.gpt_bag_place = "right"
        self.tm.waiting_touch = False
        self.following = False
        self.place_counter = 0
        self.pose = ""
        self.tm.head_touched = False
        
        # --------------- ROSPY Check ---------------
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()


    # --------------------------------------------- STATE FUNCTIONS DEFINITION ---------------------------------------------
     
    # --------------- FIRST STATE: INIT ---------------
    
    def on_enter_INIT(self):
        
        self.tm.initialize_pepper()
        
        # --------------- PARAMETERS ---------------
        
        self.set_orthogonal_security_srv(0.1)
        self.set_tangential_security_srv(0.01)
        
        # --------------- INIT STATE PROCCEDURE ---------------
        
        self.tm.pose_srv("front_camera", True)
        current_position = self.tm.get_absolute_position_proxy()
        current_angle = current_position.theta
        self.tm.add_place("arena_outside",with_coordinates=True,x=20,y=20,theta=current_angle)
        self.tm.set_current_place("arena_outside")
        rospy.sleep(1)
        
        self.tm.add_place("place" + str(self.place_counter))
        self.place_counter += 1
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.talk("Hello, I am ready to carry your luggage", wait=True)
        
        # Moving to LOOK_FOR_BAG state
        self.start()



    # --------------- SECOND STATE: LOOK FOR BAG ---------------
    
    def on_enter_LOOK_FOR_BAG(self):
        
        print(self.consoleFormatter.format("LOOK_FOR_BAG", "HEADER"))
        
        self.bag_place = "none"
        self.choosing = True
        self.tm.show_image("https://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/cml2.png")
        self.move_head_srv("up")
        self.tm.talk("I am looking for the bag you want me to carry, please point to it, raise your hand clearly. Just like you see in my tablet",wait=False)
        
        look_4_bag_thread = threading.Thread(target=self.look_4_bag)
        look_4_bag_thread.start()
        
        start_time = rospy.get_time()
        
        while self.bag_place=="none" and rospy.get_time() - start_time < 10:
            rospy.sleep(0.1)
            
        print("Signaled")
        print(f"bag at {self.bag_place}")
        
        self.choosing = False
        
        if self.bag_place == "none":
            self.bag_place = self.gpt_bag_place
        self.tm.talk(f"I found your bag to your {self.bag_place}",wait=True)
        

        # The pose is relative to the robot, so the opposite arm is used
        
        if self.bag_place=="right":
            self.pose = "small_object_left_high_3"
            self.tm.go_to_pose("small_object_left_hand", 0.1)
            
        else:
            self.pose = "small_object_right_high_3"
            self.tm.go_to_pose("small_object_right_hand", 0.1)
            
            
        # Moving to GRAB_BAG state
        self.grab_bag()
        

    # --------------- THIRD STATE: GRAB BAG ---------------

    def on_enter_GRAB_BAG(self):
        
        print(self.consoleFormatter.format("GRAB_BAG", "HEADER"))
        
        self.setMoveArms_srv.call(False, False)
        self.tm.show_image(f"https://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/carry_bag.jpeg")
        self.tm.talk("Please place the bag in my hand just like you can see in my tablet, when you have finished please touch my head!","English",wait=True)
        
        self.tm.wait_for_head_touch(message="Touch my head to get going!")
                
        # The pose is relative to the robot, so the opposite arm is used
        
        print("close hand:",self.bag_place )
        
        if self.bag_place=="right":
            self.tm.go_to_pose("close_left_hand")
            rospy.sleep(1)
            self.tm.go_to_pose("small_object_left_high_2")
            rospy.sleep(2)
            self.tm.go_to_pose("small_object_left_high_3")
            
        else:
            self.tm.go_to_pose("close_right_hand")
            rospy.sleep(1)
            self.tm.go_to_pose("small_object_right_high_2")
            rospy.sleep(2)
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
        if self.bag_place=="right":
            self.tm.go_to_pose("small_object_left_high_2")
            rospy.sleep(2)
            self.tm.go_to_pose("small_object_left_hand")
            rospy.sleep(2)
            self.tm.go_to_pose("open_left_hand")
            
        else:
            self.tm.go_to_pose("small_object_right_high_2")
            rospy.sleep(2)
            self.tm.go_to_pose("small_object_right_hand")
            rospy.sleep(2)
            self.tm.go_to_pose("open_right_hand")
            
        # Thanking the user 
        self.tm.talk("Thank you for using my services, have a nice day!",wait=False)
        self.setMoveArms_srv.call(True, True)
        
        # Moving to GO_BACK state
        self.go_back()


    # --------------- FIFTH STATE: GO BACK ---------------
    
    def on_enter_GO_BACK(self):
        
        print(self.consoleFormatter.format("GO_BACK", "HEADER"))
        
        self.tm.setRPosture_srv("stand")
        self.tm.talk("I am going back to my initial position",wait=False)
        
        
        # --------------- PARAMETERS ---------------
        
        # Super safe parameters
        self.set_orthogonal_security_srv(0.3)
        self.set_tangential_security_srv(0.05)
        
        last_place = "place" + str(self.place_counter)
        print("last_place:", last_place)
        
        self.tm.add_place(
                    last_place,
                    edges=["place" + str(self.place_counter - 1)],
                )
        
        self.tm.robot_stop_srv()
        self.tm.set_current_place(last_place)
        self.tm.robot_stop_srv()
        
        # Going back to the initial position
        self.tm.go_to_place("place0",graph=1)
        
        # Task completed
        self.tm.talk("carry my luggage task completed succesfully",wait=False)
        
        # Finish the execution
        os._exit(os.EX_OK)
        
        
    # --------------------------------------------- ADITIONAL FUNCTIONS ---------------------------------------------

    # --------------- SAVE PLACE FUNCTION ---------------
    def save_place(self):
        
        # Adding the place where the robot is standing, along with the previous node
        
        while self.following:
            
            if not self.tm.avoiding_obstacle:
                current_position = self.tm.get_absolute_position_proxy()
                current_x = current_position.x
                current_y = current_position.y
                current_angle = current_position.theta
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
                rospy.sleep(1.5)
                

    # --------------- POSE PUBLISHER CALLBACK FUNCTION ---------------
    def pose_publisher_callback(self, msg):
        if self.choosing:
            if "left" in msg.data.lower():
                self.bag_place = "left"
            elif "right" in msg.data.lower():
                self.bag_place = "right"
                
                
    # --------------- GPTVISION LOOK4BAG THREAD ---------------
    def look_4_bag(self):
        print(self.consoleFormatter.format("LOOK4BAG", "HEADER"))
        gpt_vision_prompt = f"Is the person in the center of the picture pointing to their Left or to their Right? Answer only with Left or Right"
        answer = self.tm.img_description(gpt_vision_prompt,camera_name="both")["message"].lower()
        if "right" in answer or "left" in answer:
            self.gpt_bag_place = answer
                
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
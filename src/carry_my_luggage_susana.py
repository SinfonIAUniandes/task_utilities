#!/usr/bin/env python3
import os
import rospy
import threading
import ConsoleFormatter
import time
from transitions import Machine
from task_module import Task_module as tm

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
from robot_toolkit_msgs.msg import touch_msg, speech_recognition_status_msg


class CARRY_MY_LUGGAGE(object):
    def __init__(self) -> None:
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, perception=True, speech=True, pytoolkit=True)
        self.tm.initialize_node("carry_my_luggage")

        self.STATES = [
            "INIT",
            "LOOK_FOR_BAG",
            "GRAB_BAG",
            "FOLLOW_YOU",
            "GO_BACK",
            "END",
        ]
        self.TRANSITIONS = [
            {"trigger": "zero", "source": "CARRY_MY_LUGGAGE", "dest": "INIT"},
            {"trigger": "start", "source": "INIT", "dest": "LOOK_FOR_BAG"},
            {"trigger": "grab_bag", "source": "LOOK_FOR_BAG", "dest": "GRAB_BAG"},
            {"trigger": "grab_bag_again", "source": "GRAB_BAG", "dest": "GRAB_BAG"},
            {"trigger": "follow_you", "source": "GRAB_BAG", "dest": "FOLLOW_YOU"},
            {"trigger": "go_back", "source": "FOLLOW_YOU", "dest": "GO_BACK"},
            {"trigger": "end", "source": "GO_BACK", "dest": "END"},
        ]

        self.machine = Machine(
            model=self,
            states=self.STATES,
            transitions=self.TRANSITIONS,
            initial="CARRY_MY_LUGGAGE",
        )

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
        
        self.choosing = False
        # --------------- Subscribers ---------------
        self.posePublisherSubscriber = rospy.Subscriber(
            "perception_utilities/pose_publisher", String, self.posePublisherCallback
        )
        self.headSensorSubscriber = rospy.Subscriber(
            "/touch", touch_msg, self.callback_head_sensor_subscriber
        )
        # --------------- Variables Absolutas ---------------
        self.bag_place = "none"
        self.is_ready = False
        self.following = False
        self.place_counter = 0
        self.pose = ""
        self.isTouched = False
        self.closest_person = {
            "id": None,
            "width": None,
            "height": None,
            "x": None,
            "y": None,
            "status": None,
        }
        self.rate = rospy.Rate(10)
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

    # --------------- Funciones ON_ENTER ---------------
    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.initialize_pepper()
        # Super safe parameters
        #self.set_orthogonal_security_srv(0.3)
        #self.set_tangential_security_srv(0.05)
        # Maybe unsafe
        self.set_orthogonal_security_srv(0.1)
        self.set_tangential_security_srv(0.01)
        #self.tm.setDistance_srv(0)
        self.tm.pose_srv("front_camera", True)
        self.tm.set_current_place("arena_outside")
        rospy.sleep(1)
        self.tm.add_place("place" + str(self.place_counter))
        self.place_counter += 1
        self.tm.talk("Hello, I am ready to carry your luggage", wait=False)
        self.start()

    def on_enter_LOOK_FOR_BAG(self):
        print(self.consoleFormatter.format("LOOK_FOR_BAG", "HEADER"))
        self.bag_place = "none"
        self.choosing = True
        self.tm.show_image("https://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/cml2.png")
        self.move_head_srv("up")
        self.tm.talk("I am looking for the bag you want me to carry, please point to it, raise your hand clearly. Just like you see in my tablet",wait=False)
        start_time = rospy.get_time()
        while self.bag_place=="none" and rospy.get_time() - start_time < 10:
            rospy.sleep(0.1)
        print("Signaled")
        print(f"bag at {self.bag_place}")
        self.choosing = False
        if self.bag_place=="none":
            self.bag_place="right"
        self.tm.talk(f"I found your bag to your {self.bag_place}",wait=False)
        # La pose es relativa al robot, por lo tanto se usa el brazo contrario
        if self.bag_place=="right":
            self.pose = "small_object_left_high"
        else:
            self.pose = "small_object_right_high"
        self.tm.go_to_pose(self.pose, 0.1)
        self.grab_bag()

    def on_enter_GRAB_BAG(self):
        print(self.consoleFormatter.format("GRAB_BAG", "HEADER"))
        self.setMoveArms_srv.call(False, False)
        self.tm.talk("Please place the bag in my hand, when you have finished please touch my head!","English",wait=False)
        start_time = rospy.get_time()
        last_talk_time = rospy.get_time()
        while (not self.isTouched) and rospy.get_time() - start_time < 15:
            rospy.sleep(0.1)
            if rospy.get_time()-last_talk_time > 5:
                self.tm.talk("Touch my head to get going!","English",wait=False)
                last_talk_time = rospy.get_time()
        # La pose es relativa al robot, por lo tanto se usa el brazo contrario
        print("close hand:",self.bag_place )
        if self.bag_place=="right":
            self.tm.go_to_pose("close_left_hand")
        else:
            self.tm.go_to_pose("close_right_hand")
        self.tm.show_topic("/perception_utilities/yolo_publisher")
        self.tm.talk("Thank you. I will follow you now! Please touch my head when we arrive", "English",wait=False)
        self.follow_you()

    def on_enter_FOLLOW_YOU(self):
        print(self.consoleFormatter.format("FOLLOW_YOU", "HEADER"))
        self.tm.follow_you(True, speed=0.5,avoid_obstacles=True)
        self.tm.talk("If i can't see you anymore please come back!", "English", wait=False)
        self.tm.setMoveHead_srv("up")
        print("Follow you activated!")
        self.following = True
        carry_thread = threading.Thread(target=self.carry_thread,args=[self.pose])
        carry_thread.start()
        save_place_thread = threading.Thread(target=self.save_place)
        save_place_thread.start()
        last_talk_time = rospy.get_time()
        while (not self.isTouched):
            rospy.sleep(0.1)
            if rospy.get_time()-last_talk_time >10:
                self.tm.talk("Remember to touch my head when we arrive","English",wait=False)
                last_talk_time = rospy.get_time()
        self.tm.follow_you(False)
        self.following = False
        if self.bag_place=="right":
            self.tm.go_to_pose("open_left_hand")
        else:
            self.tm.go_to_pose("open_right_hand")
        self.tm.talk("We have arrived! Could you pick up your bag?", "English")
        self.tm.robot_stop_srv()
        rospy.sleep(5)
        self.tm.talk("Thank you for using my services, have a nice day!",wait=False)
        self.setMoveArms_srv.call(True, True)
        self.go_back()

    def on_enter_GO_BACK(self):
        print(self.consoleFormatter.format("GO_BACK", "HEADER"))
        self.tm.setRPosture_srv("stand")
        self.tm.talk("I am going back to my initial position",wait=False)
        self.tm.add_place(
                    "place" + str(self.place_counter),
                    edges=["place" + str(self.place_counter - 1)],
                )
        self.tm.robot_stop_srv()
        self.tm.set_current_place("place" + str(self.place_counter))
        self.tm.robot_stop_srv()
        approved = self.tm.go_to_place("place0")
        self.tm.talk("carry my luggage task completed succesfully",wait=False)
        os._exit(os.EX_OK)

    def save_place(self):
        # anadir el lugar donde esta parado, junto con el anterior nodo
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
                # Guardar cada 3 segundos
                rospy.sleep(3)

    def posePublisherCallback(self, msg):
        if self.choosing:
            if "left" in msg.data.lower():
                self.bag_place = "left"
            elif "right" in msg.data.lower():
                self.bag_place = "right"

    def run(self):
        while not rospy.is_shutdown():
            self.zero()

    def callback_head_sensor_subscriber(self, msg: touch_msg):
        if "head" in msg.name:
            self.isTouched = msg.state

    def check_rospy(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def carry_thread(self, pose):
        while self.following:
            self.tm.go_to_pose(pose)
            rospy.sleep(1)

if __name__ == "__main__":
    sm = CARRY_MY_LUGGAGE()
    sm.run()
    rospy.spin()
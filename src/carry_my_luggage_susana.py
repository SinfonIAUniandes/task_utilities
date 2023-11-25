#!/usr/bin/env python3
import os
import rospy
import threading
import numpy as np
import ConsoleFormatter
import time
from transitions import Machine
from nav_msgs.msg import Odometry
from task_module import Task_module as tm

from std_msgs.msg import String
from std_srvs.srv import SetBool
from perception_msgs.msg import get_labels_msg
from robot_toolkit_msgs.msg import touch_msg
from robot_toolkit_msgs.srv import (
    set_move_arms_enabled_srv,
    set_security_distance_srv,
    misc_tools_srv,
    misc_tools_srvRequest,
    set_security_distance_srvRequest,
)
from speech_msgs.srv import hot_word_srv
from navigation_msgs.srv import constant_spin_srv, follow_you_srv
from perception_msgs.msg import get_labels_msg
from navigation_msgs.msg import simple_feedback_msg
from robot_toolkit_msgs.srv import (
    tablet_service_srv,
    move_head_srv,
    misc_tools_srv,
    battery_service_srv,
)
from robot_toolkit_msgs.msg import animation_msg, touch_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class CARRY_MY_LUGGAGE(object):
    def __init__(self) -> None:
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, perception=True, speech=True)
        self.tm.initialize_node("carry_my_luggage")

        self.STATES = [
            "INIT",
            "LOOK_FOR_BAG",
            "MOVE_TO_BAG",
            "GRAB_BAG",
            "FOLLOW_YOU",
            "GO_BACK",
            "END",
        ]
        self.TRANSITIONS = [
            {"trigger": "zero", "source": "CARRY_MY_LUGGAGE", "dest": "INIT"},
            {"trigger": "start", "source": "INIT", "dest": "LOOK_FOR_BAG"},
            {"trigger": "move_to_bag", "source": "LOOK_FOR_BAG", "dest": "MOVE_TO_BAG"},
            {"trigger": "grab_bag", "source": "MOVE_TO_BAG", "dest": "GRAB_BAG"},
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
        self.securityDistance_srv = rospy.ServiceProxy(
            "pytoolkit/ALMotion/set_security_distance_srv", set_security_distance_srv
        )
        self.securityDistance_srv.call(set_security_distance_srvRequest(0.05))

        self.hot_word_srv = rospy.ServiceProxy(
            "speech_utilities/hot_word_srv", hot_word_srv
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

        self.follow_you_srv = rospy.ServiceProxy(
            "/navigation_utilities/follow_you_srv", follow_you_srv
        )

        self.getLabelsSubscriber = rospy.Subscriber(
            "/perception_utilities/get_labels_publisher",
            get_labels_msg,
            self.callback_get_labels_subscriber,
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
        # --------------- Subscribers ---------------
        self.posePublisherSubscriber = rospy.Subscriber(
            "perception_utilities/pose_publisher", String, self.posePublisherCallback
        )
        self.headSensorSubscriber = rospy.Subscriber(
            "/touch", touch_msg, self.callback_head_sensor_subscriber
        )
        # --------------- Variables Absolutas ---------------
        self.isBagAtTheLeft = False
        self.bagSelectedLeft = False
        self.isBagAtTheRight = False
        self.bagSelectedRight = False
        self.save_places = False
        self.place_counter = 0
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
        self.tm.talk("Hello, I am ready to carry your luggage")
        self.tm.turn_camera("front_camera", "custom", 1, 15)
        self.tm.start_recognition("front_camera")
        self.tm.pose_srv("front_camera", True)
        time.sleep(1)
        self.tm.add_place("place" + str(self.place_counter))
        self.place_counter += 1
        self.awareness_srv(False)
        self.start()

    def on_enter_LOOK_FOR_BAG(self):
        print(self.consoleFormatter.format("LOOK_FOR_BAG", "HEADER"))
        self.isBagAtTheLeft = False
        self.isBagAtTheRight = False
        self.tm.show_image(
            "https://steamuserimages-a.akamaihd.net/ugc/941712200376608618/779F3940B9610C85543AAC2F2BCEE7E451410DD9/?imw=512&&ima=fit&impolicy=Letterbox&imcolor=%23000000&letterbox=false"
        )
        self.move_head_srv("default")
        self.tm.talk(
            "I am looking for the bag you want me to carry, please point to it, raise your hand clearly. Just like you see in my tablet"
        )
        start_time = rospy.get_time()
        while (
            self.isBagAtTheLeft == False
            and self.isBagAtTheRight == False
            and (rospy.get_time() - start_time) < 25.0
        ):
            rospy.sleep(0.05)
        print("Signaled")
        if self.isBagAtTheRight == True:
            self.bagSelectedRight = True
            self.tm.talk("I found your bag to my left")
            self.tm.go_to_pose("small_object_left_hand", 0.1)
        elif self.isBagAtTheLeft == True:
            self.bagSelectedLeft = True
            self.tm.talk("I found your bag to my right")
            self.tm.go_to_pose("small_object_right_hand", 0.1)
        else:
            self.isBagAtTheLeft = True
            self.bagSelectedLeft = True
            self.tm.talk("I found your bag to my left")
            self.tm.go_to_pose("small_object_left_hand", 0.1)

        self.move_to_bag()

    def on_enter_MOVE_TO_BAG(self):
        print(self.consoleFormatter.format("MOVE_TO_BAG", "HEADER"))
        self.setMoveArms_srv.call(True, True)
        self.tm.show_image(
            "https://media.discordapp.net/attachments/876543237270163498/1123649957791010939/logo_sinfonia_2.png"
        )
        if self.isBagAtTheLeft:
            self.tm.talk("Im moving right", wait=False)
            self.tm.go_to_relative_point(0.2, -0.5, 0)
        elif self.isBagAtTheRight:
            self.tm.talk("Im moving left", wait=False)
            self.tm.go_to_relative_point(0.2, 0.5, 0)
        self.grab_bag()

    def on_enter_GRAB_BAG(self):
        print(self.consoleFormatter.format("GRAB_BAG", "HEADER"))
        self.setMoveArms_srv.call(False, False)
        if self.bagSelectedRight:
            self.tm.go_to_pose("small_object_right_hand", 0.1)
        else:
            self.tm.go_to_pose("small_object_left_hand", 0.1)
        self.tm.talk(
            "Please place the bag in my hand, when you have finished say ready!",
            "English",
        )
        rospy.sleep(7)
        # answer = self.hot_word_srv("ready",0.5,6)
        # if answer:
        if self.bagSelectedRight:
            self.tm.go_to_pose("close_right_hand")
        else:
            self.tm.go_to_pose("close_left_hand")
        self.tm.talk("Thank you", "English")
        self.follow_you()
        # else:
        #     self.grab_bag_again()

    def on_enter_FOLLOW_YOU(self):
        print(self.consoleFormatter.format("FOLLOW_YOU", "HEADER"))
        rospy.sleep(3)
        self.tm.talk("Please stand in front me so I can follow you")
        while self.closest_person["status"] != "ok":
            if self.closest_person["status"] == "far":
                self.tm.talk("Please come closer")
            elif self.closest_person["status"] == "close":
                self.tm.talk("You are too close")
        self.tm.talk(
            "Perfect! Walk slow, Please try to keep this distance while I follow you"
        )
        self.tm.talk("If you want me to stop please touch my head!")
        rospy.sleep(3)
        self.tm.talk("Go ahead, I am following you")
        self.stop_tracker_proxy()
        self.stop_tracker_proxy()
        self.stop_tracker_proxy()
        self.follow_you_srv.call(True)
        self.stop_tracker_proxy()
        self.stop_tracker_proxy()
        self.stop_tracker_proxy()
        self.save_places = True
        # answer = self.hot_word_srv("stop", 0.5, 1000000)
        while self.isTouched == False:
            rospy.sleep(0.1)
            print("Following")
        save_place_thread = threading.Thread(target=self.save_place)
        save_place_thread.start()
        self.save_place = False
        self.follow_you_srv.call(False)
        self.tm.talk("We have arrived! Could you pick up your bag?", "English")
        if self.bagSelectedRight:
            self.tm.go_to_pose("open_right_hand")
        else:
            self.tm.go_to_pose("open_left_hand")
        rospy.sleep(5)
        self.tm.talk("Thank you for using my services, have a nice day!")
        self.setMoveArms_srv.call(True, True)
        self.go_back()

    def on_enter_GO_BACK(self):
        print(self.consoleFormatter.format("GO_BACK", "HEADER"))
        self.tm.talk("I am going back to my initial position")
        self.tm.go_to_place("place0")
        self.tm.talk("carry my luggage task completed succesfully")
        os._exit(os.EX_OK)

    def save_place(self):
        # anadir el lugar donde esta parado, junto con el anterior nodo
        while self.save_places:
            self.tm.add_place(
                "place" + str(self.place_counter),
                edges=["place" + str(self.place_counter - 1)],
            )
            self.place_counter += 1
            self.move_head_srv("up")
            # Guardar cada 3 segundos
            time.sleep(10)

    def posePublisherCallback(self, msg):
        if msg.data == "Pointing to the left":
            self.isBagAtTheLeft = True
        elif msg.data == "Pointing to the right":
            self.isBagAtTheRight = True
        elif "up" in msg.data:
            self.isBagAtTheCenter = True

    def callback_get_labels_subscriber(self, msg):
        self.labels = {}
        labels_msg = msg.labels
        x_coordinates_msg = msg.x_coordinates
        y_coordinates_msg = msg.y_coordinates
        widths = msg.widths
        heights = msg.heights
        ids = msg.ids
        closest_person = {
            "id": None,
            "width": None,
            "height": None,
            "x": None,
            "y": None,
            "status": None,
        }
        for label in range(len(labels_msg)):
            if labels_msg[label] == "person" and (
                closest_person["id"] is None or closest_person["width"] < widths[label]
            ):
                closest_person["id"] = ids[label]
                closest_person["width"] = widths[label]
                closest_person["height"] = heights[label]
                closest_person["x"] = x_coordinates_msg[label]
                closest_person["y"] = y_coordinates_msg[label]
                if closest_person["width"] < 130:
                    closest_person["status"] = "far"
                elif closest_person["width"] > 220:
                    closest_person["status"] = "close"
                else:
                    closest_person["status"] = "ok"
        self.closest_person = closest_person

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


if __name__ == "__main__":
    sm = CARRY_MY_LUGGAGE()
    sm.run()
    rospy.spin()

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
        
        # --------------- Subscribers ---------------
        self.posePublisherSubscriber = rospy.Subscriber(
            "perception_utilities/pose_publisher", String, self.posePublisherCallback
        )
        self.headSensorSubscriber = rospy.Subscriber(
            "/touch", touch_msg, self.callback_head_sensor_subscriber
        )
        # --------------- Variables Absolutas ---------------
        self.isBagAtTheLeft = False
        self.is_ready = False
        self.bagSelectedLeft = False
        self.isBagAtTheRight = False
        self.bagSelectedRight = False
        self.save_places = False
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
        self.set_orthogonal_security_srv(0.3)
        self.set_tangential_security_srv(0.05)
        self.tm.pose_srv("front_camera", True)
        time.sleep(1)
        self.tm.add_place("place" + str(self.place_counter))
        self.place_counter += 1
        self.tm.talk("Hello, I am ready to carry your luggage", wait=False)
        self.start()

    def on_enter_LOOK_FOR_BAG(self):
        print(self.consoleFormatter.format("LOOK_FOR_BAG", "HEADER"))
        self.isBagAtTheLeft = False
        self.isBagAtTheRight = False
        self.tm.show_image("http://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/carrymyluggage.jpg")
        self.move_head_srv("default")
        self.tm.talk("I am looking for the bag you want me to carry, please point to it, raise your hand clearly. Just like you see in my tablet",)
        start_time = rospy.get_time()
        while (self.isBagAtTheLeft == False and self.isBagAtTheRight == False and (rospy.get_time() - start_time) < 10.0):
            rospy.sleep(0.05)
        print("Signaled")
        if self.isBagAtTheRight:
            self.bagSelectedRight = True
            self.tm.talk("I found your bag to my left",wait=False)
            self.pose = "small_object_left_high"
            self.tm.go_to_pose("small_object_left_high", 0.1)
        elif self.isBagAtTheLeft:
            self.bagSelectedLeft = True
            self.tm.talk("I found your bag to my right",wait=False)
            self.pose = "small_object_right_high"
            self.tm.go_to_pose("small_object_right_high", 0.1)

        self.move_to_bag()

    def on_enter_MOVE_TO_BAG(self):
        print(self.consoleFormatter.format("MOVE_TO_BAG", "HEADER"))
        self.setMoveArms_srv.call(True, True)
        self.grab_bag()

    def on_enter_GRAB_BAG(self):
        print(self.consoleFormatter.format("GRAB_BAG", "HEADER"))
        self.setMoveArms_srv.call(False, False)
        if self.bagSelectedRight:
            self.tm.go_to_pose("small_object_left_high", 0.1)
        else:
            self.tm.go_to_pose("small_object_right_high", 0.1)
        self.tm.talk(
            "Please place the bag in my hand, when you have finished please touch my head!",
            "English",wait=False
        )
        while not self.isTouched:
            time.sleep(0.1)
        if self.bagSelectedRight:
            self.tm.go_to_pose("close_left_hand")
        else:
            self.tm.go_to_pose("close_right_hand")
        self.tm.talk("Thank you", "English",wait=False)
        self.follow_you()

    def on_enter_FOLLOW_YOU(self):
        print(self.consoleFormatter.format("FOLLOW_YOU", "HEADER"))
        self.tm.follow_you(True)
        self.tm.talk("I will follow you now. Please walk backwards and look at my face at all times and walk slowly. If i can't see you anymore please come back!", "English", wait=False)
        self.tm.setMoveHead_srv("up")
        print("Follow you activated!")
        self.following = True
        carry_thread = threading.Thread(target=self.carry_thread,args=[self.pose])
        carry_thread.start()
        save_place_thread = threading.Thread(target=self.save_place)
        save_place_thread.start()
        while not self.isTouched:
            time.sleep(0.1)
        self.tm.follow_you(False)
        self.save_places = False
        self.following = False
        if self.bagSelectedRight:
            self.tm.go_to_pose("open_left_hand")
        else:
            self.tm.go_to_pose("open_right_hand")
        self.tm.talk("We have arrived! Could you pick up your bag?", "English")
        rospy.sleep(5)
        self.tm.talk("Thank you for using my services, have a nice day!",wait=False)
        self.setMoveArms_srv.call(True, True)
        self.go_back()

    def on_enter_GO_BACK(self):
        print(self.consoleFormatter.format("GO_BACK", "HEADER"))
        self.tm.setRPosture_srv("stand")
        self.tm.talk("I am going back to my initial position",wait=False)
        self.tm.go_to_place("place0")
        self.tm.talk("carry my luggage task completed succesfully",wait=False)
        os._exit(os.EX_OK)

    def save_place(self):
        self.save_places = True
        # anadir el lugar donde esta parado, junto con el anterior nodo
        while self.save_places:
            self.tm.add_place(
                "place" + str(self.place_counter),
                edges=["place" + str(self.place_counter - 1)],
            )
            self.place_counter += 1
            # Guardar cada 3 segundos
            time.sleep(5)

    def posePublisherCallback(self, msg):
        if msg.data == "Pointing to the left":
            self.isBagAtTheLeft = True
        elif msg.data == "Pointing to the right":
            self.isBagAtTheRight = True
        elif "up" in msg.data:
            self.isBagAtTheCenter = True

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
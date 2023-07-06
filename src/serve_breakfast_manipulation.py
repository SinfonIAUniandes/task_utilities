#!/usr/bin/env python3
import os
import rospy
import threading
import numpy as np
import ConsoleFormatter

from transitions import Machine
from nav_msgs.msg import Odometry
from task_module import Task_module as tm
from perception_msgs.msg import get_labels_msg
from robot_toolkit_msgs.msg import touch_msg
from robot_toolkit_msgs.srv import set_move_arms_enabled_srv, set_security_distance_srv,  misc_tools_srv, misc_tools_srvRequest, set_security_distance_srvRequest


class SERVE_BREAKFAST(object):
    def __init__(self) -> None:
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.tm = tm(navigation=True, manipulation=True, perception=True, speech=True)
        self.tm.initialize_node('serve_breakfast')

        self.STATES = ['INIT', 'GO_GRAB_PLACE', 'LOOK_FOR_OBJECT' ,'GRAB_OBJECT', 'GO_DROP_PLACE', 'DROP_OBJECT', 'END']
        self.TRANSITIONS = [{'trigger': 'zero', 'source': 'SERVE_BREAKFAST', 'dest': 'INIT'},
                            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_GRAB_PLACE'},
                            {'trigger': 'look_for_object', 'source': 'GO_GRAB_PLACE', 'dest': 'LOOK_FOR_OBJECT'},
                            {'trigger': 'grab_object', 'source': 'LOOK_FOR_OBJECT', 'dest': 'GRAB_OBJECT'},
                            {'trigger': 'go_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_DROP_PLACE'},
                            {'trigger': 'drop_object', 'source': 'GO_DROP_PLACE', 'dest': 'DROP_OBJECT'},
                            {'trigger': 'again', 'source': 'DROP_OBJECT', 'dest': 'GO_GRAB_PLACE'},
                            {'trigger': 'end', 'source': 'DROP_OBJECT', 'dest': 'END'}]
        
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='INIT')

        #self.enableTouch_srv = rospy.ServiceProxy('robot_toolkit/misc_tools_srv', misc_tools_srv)
        #misc_request = misc_tools_srvRequest()
        #misc_request.data.command = "enable_all"
        #self.enableTouch_srv.call(misc_request)
        self.isTouched = False

        self.setMoveArms_srv = rospy.ServiceProxy('pytoolkit/ALMotion/set_move_arms_enabled_srv', set_move_arms_enabled_srv)

        self.securityDistance_srv = rospy.ServiceProxy('pytoolkit/ALMotion/set_security_distance_srv', set_security_distance_srv)
        self.securityDistance_srv.call(set_security_distance_srvRequest(0.01))
        
        # Subscribers
        self.getLabelsSubscriber = rospy.Subscriber('/perception_utilities/get_labels_publisher', get_labels_msg, self.callback_get_labels_subscribers)
        self.headSensorSubscriber = rospy.Subscriber('/touch', touch_msg, self.callback_head_sensor_subscriber)
        self.currentPoseOdomSuscriber = rospy.Subscriber('/odom', Odometry, self.callback_odom_subscriber)
        
        self.objects = ['bowl', 'spoon', 'cereal box', 'milk']
        self.object_i = 0

        self.object_instructions = {
            "bowl":{"grap_place":"bowl_grap_place", "drop_place":"bowl_drop_place", "forward_distance":0.35, "backward_distance":0.35},
            "spoon":{"grap_place":"spoon_grap_place", "drop_place":"spoon_drop_place", "forward_distance":0.4, "backward_distance":0.4},
            "cereal box":{"grap_place":"cereal_box_grap_place", "drop_place":"cereal_box_drop_place", "forward_distance":0.35, "backward_distance":0.35},
            "milk":{"grap_place":"milk_grap_place", "drop_place":"milk_drop_place", "forward_distance":0.35, "backward_distance":0.35}
        }

        self.rate = rospy.Rate(10)
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()


    def on_enter_INIT(self):
        self.tm.talk("I am going to serve breakfast")
        self.tm.turn_camera("front_camera", "custom", 1, 15)
        self.start()

    def on_enter_GO_GRAB_PLACE(self):
        navigation_grap_place = self.object_instructions[self.objects[self.object_i]]["grap_place"]
        self.tm.talk("I am navigating to the "+navigation_grap_place)
        self.tm.go_to_place(navigation_grap_place)
        self.look_for_object()

    def on_enter_LOOK_FOR_OBJECT(self):
        self.tm.start_recognition("front_camera")
        self.tm.talk("I am looking for the "+self.objects[self.object_i])
        while self.objects[self.object_i] not in self.labels:
            rospy.sleep(0.1)
        self.tm.talk("I have found the "+self.objects[self.object_i])
        self.grab_object()


    def on_enter_GRAB_OBJECT(self):
        self.setMoveArms_srv.call(False, False)
        if self.objects[self.object_i] == "bowl":
            self.tm.go_to_pose('medium_object', 0.1)
            rospy.sleep(2)
            self.tm.talk("Can you place the "+self.objects[self.object_i]+" inside my arms, please?, when you are ready touch my head")
            # TODO ready wait word
            while not self.isTouched:
                rospy.sleep(0.1)
            
        elif self.objects[self.object_i] == "spoon":
            self.tm.go_to_pose('small_object_right_hand', 0.1)
            rospy.sleep(2)
            self.tm.talk("Can you place the "+self.objects[self.object_i]+" inside my hand, please?, when you are ready touch my head")
            while not self.isTouched:
                rospy.sleep(0.1)
            self.tm.go_to_pose('close_right_hand', 0.2)
            rospy.sleep(1)

        elif self.objects[self.object_i] == "cereal" or self.objects[self.object_i] == "milk":
            self.tm.go_to_pose('box', 0.2)
            rospy.sleep(2)
            self.tm.talk("Could you place the "+self.objects[self.object_i]+" between my hands, please?, when you are ready touch my head")
            while not self.isTouched:
                rospy.sleep(0.1)
            self.tm.go_to_pose('cylinder', 0.1)
            rospy.sleep(2)
            self.tm.go_to_pose('close_both_hands', 0.2)
            rospy.sleep(1)

        self.go_drop_place()

    def on_enter_GO_DROP_PLACE(self):
        navigation_drop_place = self.object_instructions[self.objects[self.object_i]]["drop_place"]
        self.tm.talk("I am navigating to the "+navigation_drop_place)
        self.tm.go_to_place(navigation_drop_place)
        self.drop_object()

    def on_enter_DROP_OBJECT(self):
        self.tm.talk("I am going to place the "+self.objects[self.object_i]+" on the table")
        previousPosition = self.currentPositionOdom.position
        decreaseDistance = 0
        while True:
            distanceTraveled = self.calculateEuclideanDistance(self.currentPositionOdom.position.x, self.currentPositionOdom.position.y, previousPosition.x, previousPosition.y) 
            if distanceTraveled < 0.1-decreaseDistance:
                self.tm.go_to_relative_point(self.object_instructions[self.objects[self.object_i]]["forward_distance"]-(distanceTraveled), 0, 0)
                decreaseDistance += 0.05
            else:
                break
        if self.objects[self.object_i] == "spoon":
            self.tm.execute_trayectory("place_right_arm")
        else:
            self.tm.execute_trayectory("place_both_arms")
        self.tm.go_to_relative_point(-self.object_instructions[self.objects[self.object_i]]["backward_distance"], 0, 0)
        self.setMoveArms_srv.call(True, True)
        self.object_i += 1
        if self.object_i < len(self.objects):
            self.tm.talk("I am ready for grabbing the "+self.objects[self.object_i])
            self.again()
        else:
            self.end()

    def on_enter_END(self):
        self.tm.talk("I have finished serving breakfast, thank you for your help")
        return

    def callback_get_labels_subscribers(self, msg):
        labels_msg = msg.labels
        x_coordinates_msg = msg.x_coordinates
        y_coordinates_msg = msg.y_coordinates
        for label in range(len(labels_msg)):
            self.labels[labels_msg[label]] = (x_coordinates_msg[label], y_coordinates_msg[label])

    def callback_head_sensor_subscriber(self, msg:touch_msg):
        if msg.name == "head_rear":
            sensorRear = msg.state
        elif msg.state == "head_middle":
            sensorMiddle = msg.state
        elif msg.state == "head_front":
            sensorFront = msg.state
        if sensorFront or sensorMiddle or sensorRear:
            self.isTouched=True
        else:
            self.isTouched=False

    def calculateEuclideanDistance(self, xPoint1, yPoint1, xPoint2, yPoint2):
        return np.linalg.norm(np.array([xPoint1, yPoint1])-np.array([xPoint2, yPoint2]))
    
    def callback_odom_subscriber(self,msg:Odometry):
        self.currentPositionOdom = msg.pose.pose

    def check_rospy(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

if __name__ == "__main__":
    sm = SERVE_BREAKFAST()
    sm.run()
    rospy.spin()

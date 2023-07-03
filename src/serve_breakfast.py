#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from perception_msgs.msg import get_labels_msg
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from robot_toolkit_msgs.msg import touch_msg, animation_msg
from robot_toolkit_msgs.srv import point_at_srv, set_move_arms_enabled_srv, point_at_srvRequest, set_security_distance_srv,  misc_tools_srv, misc_tools_srvRequest, set_angle_srv, set_angle_srvRequest, set_open_close_hand_srv, set_open_close_hand_srvRequest, set_security_distance_srvRequest
import ConsoleFormatter
import rospy
import os
import time
import threading
import numpy as np
from nav_msgs.msg import Odometry


class BREAKFAST(object):
    def __init__(self) -> None:
        
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        self.task_name = "Serve the breakfast"
        states = ["INIT", "GO2PANTRY", "LOOKFORBOWL", "REQUESTHELP", "GO2TABLE", "DROP", "END"]
        
        self.tm = tm(navigation=True, speech=True, perception=True,manipulation=True)
        self.tm.initialize_node("serve_the_breakfast")

        transitions = [{'trigger': 'start', 'source': 'BREAKFAST', 'dest': 'INIT'},
                       {'trigger': 'init_go2pantry', 'source': 'INIT', 'dest': 'GO2PANTRY'},
                       {'trigger': 'go2pantry_lookforbowl', 'source': 'GO2PANTRY', 'dest': 'LOOKFORBOWL'},
                       {'trigger': 'lookforbowl_requesthelp', 'source': 'LOOKFORBOWL', 'dest': 'REQUESTHELP'},
                       {'trigger': 'requesthelp_go2table', 'source': 'REQUESTHELP', 'dest': 'GO2TABLE'},
                       {'trigger': 'go2table_drop', 'source': 'GO2TABLE', 'dest': 'DROP'},
                       {'trigger': 'drop_go2pantry', 'source': 'DROP', 'dest': 'GO2PANTRY'},
                       {'trigger': 'go2pantry_end', 'source': 'DROP', 'dest': 'END'}]

        self.machine = Machine(model=self, states=states, transitions=transitions, initial="BREAKFAST")

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        self.enableTouch_srv = rospy.ServiceProxy('robot_toolkit/misc_tools_srv', misc_tools_srv)
        misc_request = misc_tools_srvRequest()
        misc_request.data.command = "enable_all"
        self.enableTouch_srv.call(misc_request)
        
        self.setMoveArms_srv = rospy.ServiceProxy('pytoolkit/ALMotion/set_move_arms_enabled_srv', set_move_arms_enabled_srv)
        self.pointAt_srv = rospy.ServiceProxy('pytoolkit/ALTracker/point_at_srv', point_at_srv)
        self.securityDistance_srv = rospy.ServiceProxy('pytoolkit/ALMotion/set_security_distance_srv', set_security_distance_srv)
        self.securityDistance_srv.call(set_security_distance_srvRequest(0.01))
        self.setAngle_srv = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)
        self.openCloseHand_srv = rospy.ServiceProxy('pytoolkit/ALMotion/set_open_close_hand_srv', set_open_close_hand_srv)

        # Publishers
        self.cmd_velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        # Subscribers
        self.getLabelsSubscriber = rospy.Subscriber('/perception_utilities/get_labels_publisher', get_labels_msg, self.callback_get_labels_subscribers)
        self.currentPoseSubscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_amcl_pose_subscriber)
        self.headSensorSubscriber = rospy.Subscriber('/touch', touch_msg, self.callback_head_sensor_subscriber)
        self.currentPoseOdomSuscriber = rospy.Subscriber('/odom', Odometry, self.callback_odom_subscriber)

        self.navigation_grap_goal = "bowl_grap_place"
        self.navigation_drop_goal = "bowl_drop_place"
        self.object_name = "bowl"

        self.object_instructions = {
            "bowl": {"grap_place":"bowl_grap_place", "drop_place":"bowl_drop_place", "grap_angles":[0.334408, 0.0352817, -0.707165, -0.863631, -1.84084, 0.342078, -0.0475533, 0.710233, 0.857495, 1.83769], "drop_angles":[0.00375926, 1.56199, 0.0494703, -0.00880247, -6.13969E-05, 0.00385512, -1.56205, 0.00377709, 0.0087872, 4.07988E-05], "forward_distance":0.35, "backward_distance":0.35},
            "cereal":{"grap_place":"pantry", "drop_place":"table", "grap_angles":[0.334408, 0.0352817, -0.707165, -0.863631, -1.84084, 0.342078, -0.0475533, 0.710233, 0.857495, 1.83769], "drop_angles":[0.00375926, 1.56199, 0.0494703, -0.00880247, -6.13969E-05, 0.00385512, -1.56205, 0.00377709, 0.0087872, 4.07988E-05], "forward_distance":0.35, "backward_distance":0.35},
            "spoon":{"grap_place":"milk_grap_place", "drop_place":"milk_drop_place", "grap_angles":[1.309, 0.139626, -1.32645, -0.0174533, 0.0698132, 0.10472, -0.0523599, 1.39626, 0.191986, 1.81514], "drop_angles":[0.00385512, -1.56205, 0.00377709, 0.0087872, 4.07988E-05], "forward_distance":0.4, "backward_distance":0.4}
        }

        self.labels = {}

        self.currentPositionOdom = None
        self.work_space = 0.3

        self.bowl_coordinates = None

        self.isTouched = False
        self.sensorFront = False
        self.sensorMiddle = False
        self.sensorRear = False

        self.rate = rospy.Rate(10)  # Publish at a rate of 10 Hz

    def on_enter_INIT(self):
        self.tm.talk("I am going to do the "+self.task_name+" task", "English")
        self.tm.turn_camera("front_camera", "custom", 1, 15)
        self.init_go2pantry()


    def on_enter_GO2PANTRY(self):
        self.tm.talk("Navigating to the "+self.navigation_grap_goal)
        self.tm.go_to_place(self.navigation_grap_goal)
        self.go2pantry_lookforbowl()


    def on_enter_LOOKFORBOWL(self):
        self.tm.start_recognition("front_camera")
        self.tm.talk("I am looking for the "+self.object_name)
        while "cereal box" not in self.labels:
            time.sleep(0.1)
        self.tm.talk("I found the "+self.object_name)
        self.lookforbowl_requesthelp()


    def on_enter_REQUESTHELP(self):
<<<<<<< HEAD
        self.setMoveArms_srv.call(False, False)
        self.setMoveArms_srv.call(False, False)
        if self.object_name == "cereal":
            jointsRequest = set_angle_srvRequest()
            print("xd")
            jointsRequest.name = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            jointsRequest.angle =  [0.0,0.3,0.0,-0.4,-1.4,0.0,-0.3,0.0,0.4,1.4]
            jointsRequest.speed = 0.2
            self.setAngle_srv.call(jointsRequest)
            print("yd")
            rospy.sleep(2.2)
            self.tm.talk("Can you please put the "+self.object_name+" between my hands?, when you are ready touch my head")
            while not self.isTouched:
                time.sleep(0.1)
            jointsRequest = set_angle_srvRequest()
            print("xd")
            jointsRequest.name = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            jointsRequest.angle = [0.0,0.3,0.0,-1.0,-1.4,0.0,-0.3,0.0,1.0,1.4]
            jointsRequest.speed = 0.2
            self.setAngle_srv.call(jointsRequest)
            print("yd")
            rospy.sleep(2.2)
            self.openCloseHand_srv.call("both", "close")
            self.requesthelp_go2table()
        elif self.object_name == "bowl":
            jointsRequest = set_angle_srvRequest()
            jointsRequest.name = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            jointsRequest.angle = self.object_instructions[self.object_name]["grap_angles"]
            jointsRequest.speed = 0.1
            self.setAngle_srv.call(jointsRequest)
            rospy.sleep(2)
            self.tm.talk("Can you please put the "+self.object_name+" inside my arms?, when you are ready touch my head")
            while not self.isTouched:
                time.sleep(0.1)
            self.requesthelp_go2table()
        elif self.object_name == "spoon":
            jointsRequest = set_angle_srvRequest()
            jointsRequest.name = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            jointsRequest.angle = self.object_instructions[self.object_name]["grap_angles"]
            jointsRequest.speed = 0.1
            self.setAngle_srv.call(jointsRequest)
            self.tm.talk("Can you please put the "+self.object_name+" inside my right hand?, when you are ready touch my head")
            self.openCloseHand_srv.call("right", "open")
            while not self.isTouched:
                time.sleep(0.1)
            self.openCloseHand_srv.call("right", "close")
            self.requesthelp_go2table()

=======
        self.tm.goToState("bowl")
        self.tm.talk("Can you please put the bowl inside my arms?, when you are ready touch my head")
        while not self.isTouched:
            time.sleep(0.1)
        self.requesthelp_go2table()
>>>>>>> ae3a3e01e0f137026c89d8055c8a42f6f536cd86

    def on_enter_GO2TABLE(self):
        self.tm.talk("Navigating to the "+self.navigation_drop_goal)
        self.tm.go_to_place(self.navigation_drop_goal)
        self.go2table_drop()


    def on_enter_DROP(self):

        self.tm.talk("I am going to drop the "+self.object_name)
        previousPosition = self.currentPositionOdom.position
        while self.calculateEuclideanDistance(self.currentPositionOdom.position.x, self.currentPositionOdom.position.y, previousPosition.x, previousPosition.y) < 0.1:
            self.tm.go_to_relative_point(self.object_instructions[self.object_name]["forward_distance"]-(self.calculateEuclideanDistance(self.currentPositionOdom.position.x, self.currentPositionOdom.position.y, previousPosition.x, previousPosition.y)+0.1),0,0)
        jointsRequest = set_angle_srvRequest()
        print("xd")
        jointsRequest.name = ["HipPitch"]
        jointsRequest.angle = [-0.35]
        jointsRequest.speed = 0.1
        rospy.sleep(2)
        self.setAngle_srv.call(jointsRequest)
        if self.object_name == "spoon":
            jointsRequest = set_angle_srvRequest()
            print("xd")
            jointsRequest.name = ["RWristYaw"]
            jointsRequest.angle = [-1.6]
            jointsRequest.speed = 0.2
            self.setAngle_srv.call(jointsRequest)
            print("yd")
            rospy.sleep(2.2)
            self.openCloseHand_srv.call("right", "open")
            rospy.sleep(1.2)
            jointsRequest = set_angle_srvRequest()
            jointsRequest.name = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            jointsRequest.angle = self.object_instructions[self.object_name]["drop_angles"]
            jointsRequest.speed = 0.1
            self.setAngle_srv.call(jointsRequest)
            rospy.sleep(2)
            jointsRequest.angle = [1.60385512, -1.56205, 0.00377709, 0.0087872, 4.07988E-05]
            jointsRequest.speed = 0.25
            self.setAngle_srv.call(jointsRequest)
            rospy.sleep(0.5)
            jointsRequest.angle = [1.75007, -0.111032, 1.6967, 0.102538, -0.0100479]
            jointsRequest.speed = 0.15
            self.setAngle_srv.call(jointsRequest)
            rospy.sleep(2)
        else:
            jointsRequest = set_angle_srvRequest()
            jointsRequest.name = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            jointsRequest.angle = self.object_instructions[self.object_name]["drop_angles"]
            jointsRequest.speed = 0.1
            self.setAngle_srv.call(jointsRequest)
            rospy.sleep(2)
            jointsRequest.angle = [1.60375926, 1.56199, 0.0494703, -0.00880247, -6.13969E-05, 1.60385512, -1.56205, 0.00377709, 0.0087872, 4.07988E-05]
            jointsRequest.speed = 0.25
            self.setAngle_srv.call(jointsRequest)
            rospy.sleep(0.5)
            jointsRequest.angle = [1.75783, 0.1081, -1.72724, -0.130764, 0.0432976, 1.75007, -0.111032, 1.6967, 0.102538, -0.0100479]
            jointsRequest.speed = 0.15
            self.setAngle_srv.call(jointsRequest)
            rospy.sleep(2)
        
        #self.tm.spin_srv(180)
        
        self.tm.go_to_relative_point(-self.object_instructions[self.object_name]["backward_distance"],0,0)
        #self.animations_publisher.publish("animations","Gestures/Maybe_1")
        self.setMoveArms_srv.call(True, True)
        self.setMoveArms_srv.call(True, True)
        self.tm.talk("I am ready for grabbing the next object")
        if self.object_name == "bowl":
            self.navigation_grap_goal = "pantry"
            self.navigation_drop_goal = "table"
            self.object_name = "cereal"
            self.drop_go2pantry()
        elif self.object_name == "cereal":
            self.navigation_grap_goal = "milk_grap_place"
            self.navigation_drop_goal = "milk_drop_place"
            self.object_name = "spoon"
            self.drop_go2pantry()
        else:
            self.go2pantry_end()
        
        

    def on_enter_END(self):
        self.tm.talk("I finished the task, thank you for your help")

    def move_x_meters(self, x, linear_x, linear_y):
        initial_position = [self.currentPositionOdom.position.x, self.currentPositionOdom.position.y]
        twistMsg = Twist()
        twistMsg.linear.x = linear_x
        twistMsg.linear.y = linear_y
        while self.calculateEuclideanDistance(initial_position[0], initial_position[1], self.currentPositionOdom.position.x, self.currentPositionOdom.position.y) < x-0.1:
            self.cmd_velPublisher.publish(twistMsg)
            self.rate.sleep()
        

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()

    def callback_get_labels_subscribers(self, msg):
        labels_msg = msg.labels
        x_coordinates_msg = msg.x_coordinates
        y_coordinates_msg = msg.y_coordinates
        for label in range(len(labels_msg)):
            self.labels[labels_msg[label]] = (x_coordinates_msg[label], y_coordinates_msg[label])


    def callback_amcl_pose_subscriber(self, msg:PoseWithCovarianceStamped):
        self.currentPositionAmcl = msg.pose.pose

    def callback_head_sensor_subscriber(self, msg:touch_msg):
        if msg.name == "head_rear":
            self.sensorRear = msg.state
        elif msg.state == "head_middle":
            self.sensorMiddle = msg.state
        elif msg.state == "head_front":
            self.sensorFront = msg.state
        if self.sensorFront or self.sensorMiddle or self.sensorRear:
            self.isTouched=True
        else:
            self.isTouched=False

    def calculateEuclideanDistance(self, xPoint1, yPoint1, xPoint2, yPoint2):
        return np.linalg.norm(np.array([xPoint1, yPoint1])-np.array([xPoint2, yPoint2]))
<<<<<<< HEAD
    
    def callback_odom_subscriber(self,msg:Odometry):
        """
        Callback for /odom subscriber: It changes the currentPositionOdom attribute with the information of the
        input message.

        Args:
            msg (Odometry): Robot's estimated pose by the odom.
        """
        self.currentPositionOdom = msg.pose.pose
    
=======
>>>>>>> ae3a3e01e0f137026c89d8055c8a42f6f536cd86


if __name__ == "__main__":
    sm = BREAKFAST()
    sm.run()
    rospy.spin()
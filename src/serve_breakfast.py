#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from perception_msgs.msg import get_labels_msg
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from robot_toolkit_msgs.msg import touch_msg, animation_msg
from robot_toolkit_msgs.srv import point_at_srv, get_segmentation3D_srv, point_at_srvRequest
import ConsoleFormatter
import rospy
import os
import time
import threading
import numpy as np

class BREAKFAST(object):
    def __init__(self) -> None:
        
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        self.task_name = "Serve the breakfast"
        states = ["INIT", "GO2PANTRY", "LOOKFORBOWL", "REQUESTHELP", "GO2TABLE", "DROP"]
        
        self.tm = tm(navigation=True, speech=True, perception=True)
        self.tm.initialize_node("serve_the_breakfast")

        transitions = [{'trigger': 'start', 'source': 'BREAKFAST', 'dest': 'INIT'},
                       {'trigger': 'init_go2pantry', 'source': 'INIT', 'dest': 'GO2PANTRY'},
                       {'trigger': 'go2pantry_lookforbowl', 'source': 'GO2PANTRY', 'dest': 'LOOKFORBOWL'},
                       {'trigger': 'lookforbowl_requesthelp', 'source': 'LOOKFORBOWL', 'dest': 'REQUESTHELP'},
                       {'trigger': 'requesthelp_go2table', 'source': 'REQUESTHELP', 'dest': 'GO2TABLE'},
                       {'trigger': 'go2table_drop', 'source': 'GO2TABLE', 'dest': 'DROP'}]

        self.machine = Machine(model=self, states=states, transitions=transitions, initial="BREAKFAST")

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        self.segmentation_srv = rospy.ServiceProxy("pytoolkit/ALSegmentation3D/get_segmentation3D_srv", get_segmentation3D_srv)
        self.pointAt_srv = rospy.ServiceProxy('pytoolkit/ALTracker/point_at_srv', point_at_srv)

        # Publishers
        self.cmd_velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        # Subscribers
        self.getLabelsSubscriber = rospy.Subscriber('/perception_utilities/get_labels_publisher', get_labels_msg, self.callback_get_labels_subscribers)
        self.currentPoseSubscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_amcl_pose_subscriber)
        self.headSensorSubscriber = rospy.Subscriber('/touch', touch_msg, self.callback_head_sensor_subscriber)


        self.labels = {}

        self.currentPositionAmcl = None
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
        self.tm.talk("Navigating to the pantry")
        self.tm.go_to_place("pantry")
        self.go2pantry_lookforbowl()

    def on_enter_LOOKFORBOWL(self):
        self.tm.start_recognition("front_camera")
        self.tm.talk("I am looking for the bowl")
        while "bottle" not in self.labels:
            time.sleep(0.1)
        self.tm.talk("I found the bowl")
        self.lookforbowl_requesthelp()

    def on_enter_REQUESTHELP(self):
        # self.tm.saveState()
        self.tm.talk("Can you please put the bowl inside my arms?, when you are ready touch my head")
        while not self.isTouched:
            time.sleep(0.1)
        self.requesthelp_go2table()

    def on_enter_GO2TABLE(self):
        self.tm.talk("Navigating to the table")
        self.tm.go_to_place("table")
        self.go2table_drop()

    def on_enter_DROP(self):

        start_time = rospy.get_rostime()
        message = Twist()
        message.linear.x = 0.25
        while (rospy.get_rostime() - start_time).to_sec() < 1:
            self.cmd_velPublisher.publish(message)
            self.rate.sleep()
        self.tm.talk("I am going to drop the bowl")
        point_msg = point_at_srvRequest()
        point_msg.x = 0
        point_msg.y = -0.5
        point_msg.z = 0.1
        point_msg.effector_name = "RArm"
        point_msg.frame = 0
        point_msg.speed = 0.3
        self.pointAt_srv.call(point_msg)
        point_msg = point_at_srvRequest()
        point_msg.x = 0
        point_msg.y = 0.5
        point_msg.z = 0.1
        point_msg.effector_name = "LArm"
        point_msg.frame = 0
        point_msg.speed = 0.3
        self.pointAt_srv.call(point_msg)
        self.animations_publisher.publish("animations","Gestures/Maybe_1")
        self.tm.talk("I dropped the bowl xd")



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


if __name__ == "__main__":
    sm = BREAKFAST()
    sm.run()
    rospy.spin()
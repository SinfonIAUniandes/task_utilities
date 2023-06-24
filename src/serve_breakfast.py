#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from perception_msgs.msg import get_labels_msg
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from robot_toolkit_msgs.srv import point_at_srv, get_segmentation3D_srv
import ConsoleFormatter
import rospy
import threading
import numpy as np

class BREAKFAST(object):
    def __init__(self) -> None:
        
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        self.task_name = "Serve the breakfast"
        states = ["INIT", "GO2PANTRY", "LOOKFORCEREAL", "GETCLOSER", "GRAB"]
        
        self.tm = tm(navigation=True, speech=True, perception=True)
        self.tm.initialize_node("serve_the_breakfast")

        transitions = [{'trigger': 'init_go2pantry', 'source': 'INIT', 'dest': 'GO2PANTRY'},
                       {'trigger': 'go2pantry_lookforcereal', 'source': 'GO2PANTRY', 'dest': 'LOOKFORCEREAL'},
                       {'trigger': 'lookforcereal_getcloser', 'source': 'LOOKFORCEREAL', 'dest': 'GETCLOSER'},
                       {'trigger': 'getcloser_grab', 'source': 'GETCLOSER', 'dest': 'GRAB'}]

        self.machine = Machine(model=self, states=states, transitions=transitions, initial="")

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        self.segmentation_srv = rospy.ServiceProxy("pytoolkit/ALSegmentation3D/get_segmentation3D_srv", get_segmentation3D_srv)
        self.pointAt_srv = rospy.ServiceProxy('pytoolkit/ALTracker/point_at_srv', point_at_srv)

        # Publishers
        self.cmd_velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        self.getLabelsSubscriber = rospy.Subscriber('/perception_utilities/get_labels_publisher', get_labels_msg, self.callback_get_labels_subscribers)
        self.currentPoseSubscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_amcl_pose_subscriber)


        self.labels = {}

        self.currentPositionAmcl = None
        self.work_space = 0.5

        self.cereal_coordinates = None

    def on_enter_INIT(self):
        self.tm.talk("I am going to do the "+self.task_name+" task", "English")
        self.tm.turn_camera("front_camera", "custom", 1, 15)
        self.init_go2pantry()


    def on_enter_GO2PANTRY(self):
        self.tm.talk("Navigating to the pantry")
        self.tm.go_to_place("pantry")
        self.go2pantry_lookforcereal()

    def on_enter_LOOKFORCEREAL(self):
        self.tm.start_recognition("front_camera")
        self.tm.talk("I am looking for the cereal")
        while "cup" not in self.labels or abs(self.lables["cup"][0]-160) > 10:
            twist = Twist()
            twist.linear.y = 0.1
            self.cmd_velPublisher.publish(twist)
        self.cmd_velPublisher.publish(Twist())
        self.tm.talk("I found the cereal")
        self.lookforcereal_getcloser()

    def on_enter_GETCLOSER(self):
        self.cereal_coordinates = self.segmentation_srv.call()
        need_to_move = self.cereal_coordinates[0]-self.work_space
        actual_position = self.currentPositionAmcl
        self.tm.talk("I am getting closer to the cereal")
        while self.calculateEuclideanDistance(actual_position.pose.pose.position.x, actual_position.pose.pose.position.y, self.currentPositionAmcl.pose.pose.position.x, self.currentPositionAmcl.pose.pose.position.y) < need_to_move:
            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_velPublisher.publish(twist)
        self.cmd_velPublisher.publish(Twist())
        self.tm.talk("I am close enough to the cereal")
        self.getcloser_grab()

    def on_enter_GRAB(self):
        self.tm.talk("I am grabbing the cereal")
        point_at_srvRequest = point_at_srvRequest()
        point_at_srvRequest.x = self.work_space
        point_at_srvRequest.y = self.cereal_coordinates[1]
        point_at_srvRequest.z = self.cereal_coordinates[2]
        point_at_srvRequest.effector_name = "Arms"
        point_at_srvRequest.frame = 0
        point_at_srvRequest.speed = 0.5
        self.pointAt_srv.call(point_at_srvRequest)
        self.tm.talk("I grabbed the cereal")



    def callback_get_labels_subscribers(self, msg):
        labels_msg = msg.labels
        x_coordinates_msg = msg.x_coordinates
        y_coordinates_msg = msg.y_coordinates
        for label in range(len(labels_msg)):
            self.labels[labels_msg[label]] = (x_coordinates_msg[label], y_coordinates_msg[label])


    def callback_amcl_pose_subscriber(self, msg:PoseWithCovarianceStamped):
        self.currentPositionAmcl = msg.pose.pose

    def calculateEuclideanDistance(self, xPoint1, yPoint1, xPoint2, yPoint2):
        return np.linalg.norm(np.array([xPoint1, yPoint1])-np.array([xPoint2, yPoint2]))
    

    


if __name__ == "__main__":
    sm = BREAKFAST()
    sm.run()
    rospy.spin()
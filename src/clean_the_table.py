#!/usr/bin/env python3
from transitions import Machine #pylint: disable=import-error
from task_module import Task_module as tm
from perception_msgs.msg import get_labels_msg #pylint: disable=import-error
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from robot_toolkit_msgs.msg import touch_msg, animation_msg #pylint: disable=import-error
from robot_toolkit_msgs.srv import point_at_srv, set_move_arms_enabled_srv, point_at_srvRequest, set_security_distance_srv,  misc_tools_srv, misc_tools_srvRequest, set_angle_srv, set_angle_srvRequest, set_open_close_hand_srv, set_open_close_hand_srvRequest, set_security_distance_srvRequest #pylint: disable=import-error
import ConsoleFormatter
import rospy
import os
import time
import threading
import numpy as np
from nav_msgs.msg import Odometry


class CLEAN_THE_TABLE(object):
    def __init__(self) -> None:
        
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        self.task_name = "Clean the table"
        states = ["INIT", "CTT", "GO2KITCHEN", "GO_GRAB_PLACE", "GO_POSE_OBJECT", "GO_DROP_PLACE", "GO_DROP_OBJECT", "DROP_OBJECT", "END"]
        
        self.tm = tm(navigation=True, speech=True, perception=True, manipulation=True)
        self.tm.initialize_node("clean_the_table")

        transitions = [{'trigger': 'start', 'source': 'CTT', 'dest': 'INIT'},
                       {'trigger': 'go2kitchen', 'source': 'INIT', 'dest': 'GO2KITCHEN'},
                       {'trigger': 'go_grab_place', 'source': 'GO2KITCHEN', 'dest': 'GO_GRAB_PLACE'},
                       {'trigger': 'grab_object', 'source': 'GO_GRAB_PLACE', 'dest': 'GO_POSE_OBJECT'},
                       {'trigger': 'grab_object_again', 'source': 'GO_GRAB_PLACE', 'dest': 'GO_GRAB_PLACE'},
                       {'trigger': 'drop_place', 'source': 'GO_POSE_OBJECT', 'dest': 'GO_DROP_PLACE'},
                       {'trigger': 'drop_object', 'source': 'GO_DROP_PLACE', 'dest': 'DROP_OBJECT'},
                       {'trigger': 'drop_object_again', 'source': 'GO_DROP_PLACE', 'dest': 'GO_DROP_PLACE'},
                       {'trigger': 'again', 'source': 'DROP_OBJECT', 'dest': 'GO_GRAB_PLACE'},
                       {'trigger': 'end', 'source': 'DROP_OBJECT', 'dest': 'END'}]
 
        self.machine = Machine(model=self, states=self.STATES, transitions=self.TRANSITIONS, initial='INIT')

        self.enableTouch_srv = rospy.ServiceProxy('robot_toolkit/misc_tools_srv', misc_tools_srv)
        misc_request = misc_tools_srvRequest()
        misc_request.data.command = "enable_Sall"
        self.enableTouch_srv.call(misc_request)
        self.isTouched = False

        # Set security distance 
        self.setMoveArms_srv = rospy.ServiceProxy('pytoolkit/ALMotion/set_move_arms_enabled_srv', set_move_arms_enabled_srv)
        self.securityDistance_srv = rospy.ServiceProxy('pytoolkit/ALMotion/set_security_distance_srv', set_security_distance_srv)
        self.securityDistance_srv.call(set_security_distance_srvRequest(0.01))
        
        # Subscribers
        self.getLabelsSubscriber = rospy.Subscriber('/perception_utilities/get_labels_publisher', get_labels_msg, self.callback_get_labels_subscribers)
        self.headSensorSubscriber = rospy.Subscriber('/touch', touch_msg, self.callback_head_sensor_subscriber)
        self.currentPoseOdomSuscriber = rospy.Subscriber('/odom', Odometry, self.callback_odom_subscriber)
        
        # Variables de estado
        self.object = ""
        self.times_i = 0
        self.centinel = True
        self.arm = True
        self.object_instructions = {
            "spoon":{"grap_place":"table", "drop_place":"dishwasher", "forward_distance":0.35, "backward_distance":0.35},
            "fork":{"grap_place":"table", "drop_place":"dishwasher", "forward_distance":0.4, "backward_distance":0.4},
            "knife":{"grap_place":"table", "drop_place":"dishwasher", "forward_distance":0.35, "backward_distance":0.35},
            "dish":{"grap_place":"table", "drop_place":"dishwasher", "forward_distance":0.35, "backward_distance":0.35},
        }

        self.rate = rospy.Rate(10)
        
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()


    def on_enter_INIT(self):
        self.tm.set_current_place(self.initial_place)
        self.autonomous_life_srv(False)
        self.tm.talk("I am going to do "+str(self.task_name),"English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",1,15) 
        self.awareness_srv(False)
        self.go2kitchen()

    def on_enter_GO2KITCHEN(self):
        self.tm.talk("I am going to navigate to the kitchen","English")
        self.tm.go_to_place("kitchen")
        self.tm.talk("I am in the kitchen","English")
        self.go_grab_place()  

    def on_enter_GO_GRAB_PLACE(self):
        if(self.object==""):
            navigation_grap_place = "table"
        else:
            navigation_grap_place = self.object_instructions[self.object]["grap_place"]
        
        if(self.times_i==1):
            self.grab_object()
        
        else:
            self.tm.talk("I am going to navigate to the "+navigation_grap_place)
            self.tm.go_to_place(navigation_grap_place)
            self.tm.talk("I am in the "+ navigation_grap_place,"English")
            self.grab_object()

    def on_enter_GO_POSE_OBJECT(self):
        while(self.centinel):
            if(self.labels == "spoon"):
                self.tm.talk("I have found the spoon")
                if(self.arm == True):
                    self.tm.go_to_pose("small_object_left_hand")
                    self.arm = False
                else:
                    self.tm.go_to_pose("small_object_right_hand")
                self.object = "spoon"
                self.centinel = False
                    
            elif(self.labels == "fork"):
                self.tm.talk("I have found the fork")
                if(self.arm == True):
                    self.tm.go_to_pose("small_object_left_hand")
                    self.arm = False
                else:
                    self.tm.go_to_pose("small_object_right_hand")
                self.object = "fork"
                self.centinel = False

            elif(self.labels == "knife"):
                self.tm.talk("I have found the knife")
                if(self.arm == True):
                    self.tm.go_to_pose("small_object_left_hand")
                    self.arm = False
                else:
                    self.tm.go_to_pose("small_object_right_hand")
                self.object = "knife"
                self.centinel = False

            elif(self.labels == "dish"):
                self.tm.talk("I have found the dish")
                self.tm.go_to_pose("bowl")
                self.object = "bowl"
                self.centinel = False
        
        self.times_i=self.times_i+1
        self.tm.talk("Can you place the "+self.object+" inside my arms, please?, when you are ready touch my head")
        while not self.isTouched:
            rospy.sleep(0.1)
        if(self.times_i<2):
            self.grab_object_again()
        else:
            self.drop_place()
        
    def on_enter_GO_DROP_PLACE(self):
        navigation_drop_place = self.object_instructions[self.object]["drop_place"]
        self.tm.talk("I am navigating to the "+navigation_drop_place)
        self.tm.go_to_place(navigation_drop_place)
        self.tm.talk("I am in the "+navigation_drop_place)
        self.drop_object()

    def on_enter_DROP_OBJECT(self):
        self.tm.talk("I am going to place the "+self.object+" on the dishwasher")
        previousPosition = self.currentPositionOdom.position
        decreaseDistance = 0
        while True:
            distanceTraveled = self.calculateEuclideanDistance(self.currentPositionOdom.position.x, self.currentPositionOdom.position.y, previousPosition.x, previousPosition.y) 
            if distanceTraveled < 0.1-decreaseDistance:
                self.tm.go_to_relative_point(self.object_instructions[self.object]["forward_distance"]-(distanceTraveled), 0, 0)
                decreaseDistance += 0.05
            else:
                break
        
        if self.times_i==2:
            self.tm.execute_trayectory("place_left_arm")
            self.tm.talk("Object placed in dishwasher")
            self.tm.execute_trayectory("place_right_arm")
            self.tm.talk("Object placed in dishwasher")
        else:
            self.tm.execute_trayectory("place_both_arms")

        self.tm.go_to_relative_point(-self.object_instructions[self.object]["backward_distance"], 0, 0)
        self.setMoveArms_srv.call(True, True)
        if self.times_i < 6:
            self.tm.talk("I am ready for grabbing the next object")
            self.again()
        else:
            self.end()

    def callback_get_labels(self,data):
        self.labels = data.labels
        self.ids = data.ids
        self.widths = data.widths
        self.heights = data.heights
        for i in range(0, len(self.heights)):
            size_dict = {"label": self.labels[i], "width": self.widths[i], "height": self.heights[i]}
            if self.labels[i] == 'person':
                self.person_ids.append(self.ids[i])
            self.objects[self.ids[i]] = size_dict
        
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

    def callback_odom_subscriber(self,msg:Odometry):
        self.currentPositionOdom = msg.pose.pose
            
    def calculateEuclideanDistance(self, xPoint1, yPoint1, xPoint2, yPoint2):
        return np.linalg.norm(np.array([xPoint1, yPoint1])-np.array([xPoint2, yPoint2]))
    
    def check_rospy(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

if __name__ == "__main__":
    sm = SERVE_BREAKFAST()
    sm.run()
    rospy.spin()

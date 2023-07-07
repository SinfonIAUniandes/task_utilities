#!/usr/bin/env python3
import task_module as tm
import rospy
import time
from perception_msgs.srv import start_recognition_srv, start_recognition_srvRequest, look_for_object_srv, look_for_object_srvRequest, save_face_srv,save_face_srvRequest, recognize_face_srv, recognize_face_srvRequest, save_image_srv,save_image_srvRequest, set_model_recognition_srv,set_model_recognition_srvRequest
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Int32, String


tm = tm.Task_module(pytoolkit=True)
tm.initialize_node("test_node")
tm.show_image("sinfonia")


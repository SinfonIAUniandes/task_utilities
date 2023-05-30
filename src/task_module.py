#!/usr/bin/env python3
import rospy
import time 
import rospkg
import rosservice

#All imports from tools
# from robot_toolkit_msgs.msg import speech_msg
# from navigation_msgs.msg import simple_feedback_msg
# from navigation_msgs.srv import go_to_place_srv, go_to_place_srvRequest, spin_srv, spin_srvRequest,correct_position_srv, correct_position_srvRequest
from speech_utilities_msgs.srv import q_a_speech_srv, q_a_speech_srvRequest, talk_speech_srv, talk_speech_srvRequest
from perception_msgs.srv import start_recognition_srv, start_recognition_srvRequest, look_for_object_srv, look_for_object_srvRequest, save_face_srv,save_face_srvRequest, recognize_face_srv, recognize_face_srvRequest, save_image_srv,save_image_srvRequest, set_model_recognition_srv,set_model_recognition_srvRequest,read_qr_srv,read_qr_srvRequest,turn_camera_srv,turn_camera_srvRequest
# from manipulation_msgs.srv import go_to_pose_srv, go_to_pose_srvRequest, execute_trajectory_srv, execute_trajectory_srvRequest
from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import Trigger, TriggerRequest


class Task_module:

    def __init__(self):
        self.object_found = 0

    def initialize_node(self):
        rospy.init_node('task_module_node') 

    ################### PERCEPTION SERVICES ###################

    def turn_camera(self,camera_name:str,command:str,resolution=2,fps=10)->bool:
        """
        Input: 
        camera_name: "front_camera" || "bottom_camera" || "depth_camera"
        command: "enable" || "disable" || "custom"
        resolution: RGB (0-4) DEPTH (0-1)
        fps: 1-30
        Output: True if the service was called correctly, False if not
        ----------
        Turns on/off camera_name, if command is custom, it sets the resolution and fps
        """
        try:
            print("Waiting for turn camera service")
            rospy.wait_for_service('/perception_utilities/turn_camera_srv')
            turn_camera = rospy.ServiceProxy('/perception_utilities/turn_camera_srv', turn_camera_srv)
            approved = turn_camera(camera_name,command,resolution,fps)
            print("TODO approved",approved)
            if approved=="approved":
                return True
            else:
                return False
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False

    def start_recognition(self,camera_name:str)->bool:
        """
        Input: camera_name
        Output: True if the service was called correctly, False if not
        ----------
        Starts recognition for camera_name, if camera_name is empty, it shuts down all recognition
        """
        try:
            print("Waiting for start recognition service")
            rospy.wait_for_service('/perception_utilities/start_recognition_srv')
            start_recognition = rospy.ServiceProxy('/perception_utilities/start_recognition_srv', start_recognition_srv)
            approved = start_recognition(camera_name)
            if approved=="approved":
                return True
            else:
                return False
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False


    def look_for_object(self,object_name:str)->bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Looks for object_name in the current get_labels topic, publishes T/F in look_for_object_publisher
        """
        try:
            print("Waiting for look for object service")
            rospy.wait_for_service('/perception_utilities/look_for_object_srv')
            look_for_object = rospy.ServiceProxy('/perception_utilities/look_for_object_srv', look_for_object_srv)
            approved = look_for_object(object_name)
            if approved=="approved":
                return True
            else:
                return False
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
        
    def wait_for_object(self,timeout:float)->bool:
        """
        Input: timeout in seconds
        Output: True->Found || False->Timeout/Fail
        ----------
        Waits for object to be found for a max of <timeout> seconds
        """
        try:
            print("Waiting for object")
            t_start = time.time()
            finish=False
            response = False
            rospy.Subscriber("/perception_utilities/look_for_object_publisher", Bool, self.callback_look_for_object)
            while not finish:
                print("El object found es: ", self.object_found)
                rospy.sleep(0.05) 
                t_now = time.time()
                if self.object_found:
                    finish=True
                    response = True
                elif t_now-t_start>timeout:
                    finish=True
                    response = False
            return response                
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
        
    def save_face(self,name:str,num_pics:int)->bool:
        """
        Input: name, num_pics
        Output: True if the service was called correctly, False if not
        ----------
        Saves num_pics of the face of the person with name
        """
        try:
            print("Waiting for save face service")
            rospy.wait_for_service('/perception_utilities/save_face_srv')
            save_face = rospy.ServiceProxy('/perception_utilities/save_face_srv', save_face_srv)
            approved = save_face(name,num_pics)
            if approved=="approved":
                return True
            else:
                return False
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
        
    def recognize_face(self, num_pics:int)->str:
        """
        Input: num_pics of the face to recognize
        Output: name of the recognized person
        ----------
        Recognizes the face of the person with num_pics of person
        """
        try:
            print("Waiting for recognize face service")
            rospy.wait_for_service('/perception_utilities/recognize_face_srv')
            recognize_face = rospy.ServiceProxy('/perception_utilities/recognize_face_srv', recognize_face_srv)
            name = recognize_face(num_pics)
            return name
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return ""

    def qr_read(self,timeout:float)->str:
        """
        Input: timeout in seconds
        Output: "text" read from qr || "" if timeout or fail
        ----------
        Reads a qr code for a max of <timeout> seconds
        """
        try:
            rospy.wait_for_service('perception_utilities/read_qr_srv')
            qr_read = rospy.ServiceProxy('/perception_utilities/read_qr_srv', read_qr_srv)
            read_qr_message = read_qr_srvRequest()
            read_qr_message.timeout = timeout
            text = qr_read(read_qr_message)
            print(text)
            return text.text
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return ""
        
    ################### SPEECH SERVICES ###################
        
    def talk(self,text:str,time:float,language:str)->bool:
        print("esperando el servicio de talk")
        rospy.wait_for_service('/speech_utilities/talk_speech_srv')
        print("el servicio de talk ya esta disponible")
        try:
            talk = rospy.ServiceProxy('/speech_utilities/talk_speech_srv', talk_speech_srv)
            talk(text, time,language)
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
        
    ################ CALLBACKS ################
    def callback_look_for_object(self,data):
        self.object_found=data.data
        return data



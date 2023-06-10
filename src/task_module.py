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
from manipulation_msgs.srv import go_to_pose_srv, go_to_pose_srvRequest, execute_trajectory_srv, execute_trajectory_srvRequest
from navigation_utilities.srv import set_current_place_srv, set_current_place_srvRequest, go_to_relative_point_srv, go_to_relative_point_srvRequest, go_to_place_srv, go_to_place_srvRequest, start_random_navigation_srv, start_random_navigation_srvRequest, add_place_srv, add_place_srvRequest, follow_you_srv, follow_you_srvRequest, robot_stop_srv, robot_stop_srvRequest, spin_srv, spin_srvRequest, go_to_defined_angle_srv, go_to_defined_angle_srvRequest, get_absolute_position_srv, get_absolute_position_srvRequest, get_route_guidance_srv, get_route_guidance_srvRequest, correctPosition_srv, correctPosition_srvRequest
from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import Trigger, TriggerRequest


class Task_module:

    def __init__(self, perception: bool, speech: bool, manipulation:bool, navigation: bool):
        
        self.perception= perception
        
        if perception: 
            rospy.wait_for_service('/perception_utilities/turn_camera_srv')
            self.turn_camera_proxy = rospy.ServiceProxy('/perception_utilities/turn_camera_srv', turn_camera_srv)
            rospy.wait_for_service('/perception_utilities/start_recognition_srv')
            self.start_recognition_proxy = rospy.ServiceProxy('/perception_utilities/start_recognition_srv', start_recognition_srv)
            rospy.wait_for_service('/perception_utilities/look_for_object_srv')
            self.look_for_object_proxy = rospy.ServiceProxy('/perception_utilities/look_for_object_srv', look_for_object_srv)
            rospy.wait_for_service('/perception_utilities/save_face_srv')
            self.save_face_proxy = rospy.ServiceProxy('/perception_utilities/save_face_srv', save_face_srv)
            rospy.wait_for_service('/perception_utilities/recognize_face_srv')
            self.recognize_face_proxy = rospy.ServiceProxy('/perception_utilities/recognize_face_srv', recognize_face_srv)
            rospy.wait_for_service('perception_utilities/read_qr_srv')
            self.qr_read_proxy = rospy.ServiceProxy('/perception_utilities/read_qr_srv', read_qr_srv)
            
            
        self.speech=speech
        
        if speech: 
            rospy.wait_for_service('/speech_utilities/talk_speech_srv')
            self.talk_proxy = rospy.ServiceProxy('/speech_utilities/talk_speech_srv', talk_speech_srv)

        self.object_found = 0

        if navigation:
            rospy.wait_for_service('/navigation_utilities/set_current_place_srv')
            self.set_current_place_proxy = rospy.ServiceProxy('/navigation_utilities/set_current_place_srv', set_current_place_srv)
            
            rospy.wait_for_service('navigation_utilities/go_to_relative_point_srv')
            self.go_to_relative_point_proxy = rospy.ServiceProxy('/navigation_utilities/go_to_relative_point_srv', go_to_relative_point_srv)
            
            rospy.wait_for_service('navigation_utilities/go_to_place_srv')
            self.go_to_place_proxy = rospy.ServiceProxy('/navigation_utilities/go_to_place_srv', go_to_place_srv)
            
            rospy.wait_for_service('navigation_utilities/start_random_navigation_srv')
            self.start_random_navigation_proxy = rospy.ServiceProxy('/navigation_utilities/start_random_navigation_srv', start_random_navigation_srv)
            
            rospy.wait_for_service('navigation_utilities/add_place_srv')
            self.add_place_proxy = rospy.ServiceProxy('/navigation_utilities/add_place_srv', add_place_srv)
            
            rospy.wait_for_service('navigation_utilities/follow_you_srv')
            self.follow_you_proxy = rospy.ServiceProxy('/navigation_utilities/follow_you_srv', follow_you_srv)
            
            rospy.wait_for_service('navigation_utilities/robot_stop_srv')
            self.robot_stop_proxy = rospy.ServiceProxy('/navigation_utilities/robot_stop_srv', robot_stop_srv)
            
            rospy.wait_for_service('navigation_utilities/spin_srv')
            self.spin_proxy = rospy.ServiceProxy('/navigation_utilities/spin_srv', spin_srv)

            rospy.wait_for_service('navigation_utilities/go_to_defined_angle_srv')
            self.go_to_defined_angle_proxy = rospy.ServiceProxy('/navigation_utilities/go_to_defined_angle_srv', go_to_defined_angle_srv)

            rospy.wait_for_service('navigation_utilities/get_absolute_position_srv')
            self.get_absolute_position_proxy = rospy.ServiceProxy('/navigation_utilities/get_absolute_position_srv', get_absolute_position_srv)

            rospy.wait_for_service('navigation_utilities/get_route_guidance_srv')
            self.get_route_guidance_proxy = rospy.ServiceProxy('/navigation_utilities/get_route_guidance_srv', get_route_guidance_srv)

            rospy.wait_for_service('navigation_utilities/correctPosition_srv')
            self.correctPosition_proxy = rospy.ServiceProxy('/navigation_utilities/correctPosition_srv', correctPosition_srv)


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
        if self.perception: 
            try:
                approved = self.turn_camera_proxy(camera_name,command,resolution,fps)
                print("TODO approved",approved)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("perception as false")
            return False

    def start_recognition(self,camera_name:str)->bool:
        """
        Input: camera_name
        Output: True if the service was called correctly, False if not
        ----------
        Starts recognition for camera_name, if camera_name is empty, it shuts down all recognition
        """
        if self.perception:
            try:
                approved = self.start_recognition_proxy(camera_name)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("perception as false")    
            return False        


    def look_for_object(self,object_name:str)->bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Looks for object_name in the current get_labels topic, publishes T/F in look_for_object_publisher
        """
        if self.perception:
            try:
                approved = self.look_for_object_proxy(object_name)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("perception as false") 
            return False
            
        
    def wait_for_object(self,timeout:float)->bool:
        """
        Input: timeout in seconds
        Output: True->Found || False->Timeout/Fail
        ----------
        Waits for object to be found for a max of <timeout> seconds
        """
        if self.perception:
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
        else: 
            print("perception as false") 
            return False            
        
    def save_face(self,name:str,num_pics:int)->bool:
        """
        Input: name, num_pics
        Output: True if the service was called correctly, False if not
        ----------
        Saves num_pics of the face of the person with name
        """
        if self.perception:
            try:
                approved = self.save_face_proxy(name,num_pics)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("perception as false") 
            return False
        
    def recognize_face(self, num_pics:int)->str:
        """
        Input: num_pics of the face to recognize
        Output: name of the recognized person
        ----------
        Recognizes the face of the person with num_pics of person
        """
        if self.perception:
            try:
                name = self.recognize_face_proxy(num_pics)
                return name
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return ""
        else:
            print("perception as false") 
            return False

    def qr_read(self,timeout:float)->str:
        """
        Input: timeout in seconds
        Output: "text" read from qr || "" if timeout or fail
        ----------
        Reads a qr code for a max of <timeout> seconds
        """
        if self.perception: 
            try:
                read_qr_message = read_qr_srvRequest()
                read_qr_message.timeout = timeout
                text = self.qr_read_proxy(read_qr_message)
                print(text)
                return text.text
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return ""
        else: 
            print("perception as false") 
            return ""
    ################### SPEECH SERVICES ###################
        
    def talk(self,text:str,time:float,language:str)->bool:
        if self.speech:
            try:
                self.talk_proxy(text, time,language)
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("speech as false")
            return False
        
    ################### NAVIGATION SERVICES ###################

    def set_current_place(self,place_name:str)->bool:
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Sets the pose of the robot to the coordinates of the place specified in
        the request message, then it clears the global costmap.
        """
        if self.navigation:
            try:
                approved = self.set_current_place_proxy(place_name)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("navigation as false")
            return False
        
    def go_to_relative_point(self,x:float,y:float,theta:float)->bool:
        """
        Input: x, y, theta
        Output: True if the service was called correctly, False if not
        ----------
        Sends the robot to the coordinates (in meters and relative to the robot)
        specified in the request message.
        """
        if self.navigation:
            try:
                approved = self.go_to_relative_point_proxy(x,y,theta)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("navigation as false")
            return False
        
    def go_to_place(self,place_name:str, graph:int)->bool:
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Goes to place_name
        """
        if self.navigation:
            try:
                approved = self.go_to_place_proxy(place_name, graph)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("navigation as false")
            return False
        
    def start_random_navigation_srv(self)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Starts random navigation
        """
        if self.navigation:
            try:
                approved = self.start_random_navigation_proxy()
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("navigation as false")
            return False
        
    def add_place_srv(self, place_name:str, persist:int, edges:list)->bool:
        """
        Input: place_name, persist, edges
        Output: True if the service was called correctly, False if not
        ----------
        Adds place_name to the graph
        """
        if self.navigation:
            try:
                approved = self.add_place_proxy(place_name, persist, edges)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("navigation as false")
            return False
        
    def follow_you_srv(self, place_name: str)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Starts following you
        """
        if self.navigation:
            try:
                approved = self.follow_you_proxy(place_name)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("navigation as false")
            return False

    def robot_stop_srv(self)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Stops the robot
        """
        if self.navigation:
            try:
                approved = self.robot_stop_proxy()
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("navigation as false")
            return False
        
    def spin_srv(self, degrees:float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Spins the robot
        """
        if self.navigation:
            try:
                approved = self.spin_proxy(degrees)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else: 
            print("navigation as false")
            return False
        
    def go_to_defined_angle_srv(self, degrees:float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Goes to defined angle
        """
        if self.navigation:
            try:
                approved = self.go_to_defined_angle_proxy(degrees)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("navigation as false")
            return False

    def get_absolute_position_srv(self):
        """
        Input: None
        Output: get_absolute_position_srv response message. {x(float64),y(float64),theta(float64)}.
        ----------
        Gets absolute position
        """
        if self.navigation:
            try:
                absolute_position = self.get_absolute_position_proxy()
                return absolute_position.x, absolute_position.y, absolute_position.theta
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def get_route_guidance_srv(self, place_name: str):
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Gets route guidance
        """
        if self.navigation:
            try:
                instructions = self.get_route_guidance_proxy(place_name)
                return instructions
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False
        
    def correctPosition_srv(self, degrees: float):
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Corrects position
        """
        if self.navigation:
            try:
                approved = self.correctPosition_proxy(degrees)
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False
        
            
    ################ CALLBACKS ################
    def callback_look_for_object(self,data):
        self.object_found=data.data
        return data



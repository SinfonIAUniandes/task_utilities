#!/usr/bin/env python3
import rospy
import time
import rospkg
import rosservice
import ConsoleFormatter


from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import Trigger, TriggerRequest

# All imports from tools

# from robot_toolkit_msgs.msg import speech_msg

from speech_utilities_msgs.srv import q_a_speech_srv, talk_speech_srv, saveAudio_srv, q_a_speech_srvRequest, talk_speech_srvRequest, saveAudio_srvRequest

from perception_msgs.srv import start_recognition_srv, start_recognition_srvRequest, look_for_object_srv, look_for_object_srvRequest, save_face_srv,save_face_srvRequest, recognize_face_srv, recognize_face_srvRequest, save_image_srv,save_image_srvRequest, set_model_recognition_srv,set_model_recognition_srvRequest,read_qr_srv,read_qr_srvRequest,turn_camera_srv,turn_camera_srvRequest,filtered_image_srv,filtered_image_srvRequest

from manipulation_msgs.srv import SaveState, SaveStateRequest, GoToState, GoToStateRequest

from navigation_msgs.srv import set_current_place_srv, set_current_place_srvRequest, go_to_relative_point_srv, go_to_relative_point_srvRequest, go_to_place_srv, go_to_place_srvRequest, start_random_navigation_srv, start_random_navigation_srvRequest, add_place_srv, add_place_srvRequest, robot_stop_srv, robot_stop_srvRequest, spin_srv, spin_srvRequest, go_to_defined_angle_srv, go_to_defined_angle_srvRequest, get_absolute_position_srv, get_absolute_position_srvRequest, get_route_guidance_srv, get_route_guidance_srvRequest, correct_position_srv, correct_position_srvRequest, constant_spin_srv, constant_spin_srvRequest
from navigation_msgs.msg import simple_feedback_msg

class Task_module:

    def __init__(self, perception=False, speech=False, manipulation=False, navigation=False):
        """Initializer for the Task_module class

        Args:
            perception (bool): Enables or disables perception services
            speech (bool): Enables or disables speech services
            manipulation (bool): Enables or disables manipulation services
            navigation (bool): Enables or disables navigation services
        """
    ################### PERCEPTION SERVICES ###################

    def find_object(self,object_name:str, timeout=25)->bool:
        """
        Input:
        object_name: label of the object to look for options -> classes names depends of the actual model (see set_model service)
        timeout: time in seconds to look for the object while spinning
        Output: True if the object was found, False if not
        ----------
        Spins while looking for <object_name> for <timeout> seconds while spinning at 15 deg/s
        """

    def set_model(self,model_name:str)->bool:
        """
        Input: model name -> "default" || "objects" || "fruits"
        Output: True if the service was called correctly, False if not
        ----------
        Sets the model to use for the object recognition.
        classes by model:
        default: ["person","bench","backpack","handbag","suitcase","bottle","cup","fork","knife","spoon","bowl","chair","couch","bed","laptop"]
        objects: ["spam"(carne enlatada), "cleanser", "sugar", "jello"(gelatina roja), "mug", "tuna", "bowl", "tomato_soup", "footwear", "banana", "mustard", "coffee_grounds", "cheezit"]
        fruits: ["apple", "lemon", "orange", "peach", "pear", "plum","strawberry"]
        """

    ################### SPEECH SERVICES ###################

    def talk(self,text:str,language="English",wait=True,animated=False)->bool:
        """
        Input:
        text : text that robot will say
        language: English || Spanish
        wait(wait until the robot finishes talking)
        animates: gesture hands
        Output: True if everything ok || False if not
        ----------
        Allows the robot to say the input of the service.
        """

    def speech2text_srv(self, file_name="prueba",seconds=0,transcription=True)->bool:
        """
        Input: 
        seconds: 0 for automatic stop || > 0 for seconds to record
        file_name: name of the file
        transcription: True || False
        Output: text that the robot heard
        ----------
        Allows the robot to save audio and saves it to a file.
        """

    def q_a_speech(self, tag:str)->str:
        """
        Input: tag in lowercase: options -> ("age", "name", "drink")
        Output: answer
        ----------
        Returns a specific answer for predefined questions.
        """

    ################### NAVIGATION SERVICES ###################

    def go_to_place(self,place_name:str, graph=1, wait=True)->bool:
        """
        Input: 
        place_name: options -> ("door","living_room")
        graph: 0 no graph || 1 graph
        wait: True (waits until the robot reaches) || False (doesn't wait)
        Output: True if the service was called correctly, False if not
        ----------
        Goes to place_name
        """

    def follow_you(self, start:bool)->bool:
        """
        Input:
        start: True start following || False stops following
        Output: True if the service was called correctly, False if not
        ----------
        Follows the person in front of the robot until the service is called again with start=False
        """

    def add_place(self,name:str,persist=0,edges=[])->bool:
        """
        Input: name, edges, persist
        Output: True if the service was called correctly, False if not
        ----------
        Adds a place to the graph
        """

    ############ MANIPULATION SERVICES ###############    
    
    def go_to_pose(self,pose:str)->bool:
        """
        Input: pose options ->("bowl","box","cylinder","medium_object", "small_object_left_hand","small_object_right_hand","tray","head_up","head_down","head_default")
        Output: True if the service was called correctly, False if not
        ----------
        Goes to pose with hands or head
        """

    def execute_trayectory(self,trayectory:str)->bool:
        """
        Input: trayectory options ->("place_both_arms","place_left_arm","place_right_arm")
        Output: True if the service was called correctly, False if not
        ----------
        Executes trayectory with the hands
        """
        
    def grasp_object(self,object_name:str)->bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Grasp the <object_name>
        """

    ################ PYTOOLKIT ################

    def set_awareness(self, state:bool)->bool:
        """
        Input: True turn on || False turn off
        Output: True if the service was called correctly, False if not
        ----------
        Sets the awareness of the robot
        """

    def set_autonomous_life(self, state:bool)->bool:
        """
        Input: True turn on || False turn off
        Output: True if the service was called correctly, False if not
        ----------
        Sets the autonomous life of the robot
        """
     
    ################ SUBSCRIBER CALLBACKS ################

    def callback_look_for_object(self,data):
        self.object_found=data.data
        return data

    def callback_simple_feedback_subscriber(self, msg):
        self.navigation_status = msg.navigation_status
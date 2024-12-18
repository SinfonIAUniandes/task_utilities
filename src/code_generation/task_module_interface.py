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

from speech_msgs.srv import q_a_speech_srv, talk_speech_srv, q_a_speech_srvRequest, talk_speech_srvRequest, saveAudio_srvRequest, speech2text_srv

from perception_msgs.srv import  set_model_recognition_srv,set_model_recognition_srvRequest

from manipulation_msgs.srv import SaveState, SaveStateRequest, GoToState, GoToStateRequest

from navigation_msgs.srv import set_current_place_srv, set_current_place_srvRequest, go_to_relative_point_srv, go_to_relative_point_srvRequest, go_to_place_srv, go_to_place_srvRequest, start_random_navigation_srv, start_random_navigation_srvRequest, add_place_srv, add_place_srvRequest, robot_stop_srv, robot_stop_srvRequest, spin_srv, spin_srvRequest, go_to_defined_angle_srv, go_to_defined_angle_srvRequest, get_absolute_position_srv, get_absolute_position_srvRequest, get_route_guidance_srv, get_route_guidance_srvRequest, correct_position_srv, correct_position_srvRequest, constant_spin_srv, constant_spin_srvRequest
from navigation_msgs.msg import simple_feedback_msg

class Task_module:

    ################### PERCEPTION SERVICES ###################

    def find_object(self,object_name:str, timeout=25)->bool:
        """
        Input:
        object_name: label of the object to look for options -> classes names depends of the actual model (see set_model service)
        timeout: time in seconds to look for the object while spinning ()
        Output: True if the object was found, False if not
        ----------
        Spins while looking for <object_name> for <timeout> seconds while spinning at 15 deg/s
        """
        
    def count_objects(self,object_name:str)->int:
        """
        Input: object_name, classes names depends of the actual model (see set_model service)
        Output: Number of objects seen when rotating 360
        ----------
        Spins 360 degrees and then returns the number of objects of <object_name> seen
        """

    def set_model(self,model_name:str)->bool:
        """
        Input: model name -> "default" || "objects" || "fruits"
        Output: True if the service was called correctly, False if not
        ----------
        Sets the model to use for the object recognition.
        classes by model:
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
        Input: tag in lowercase: options -> ("age", "name", "drink", "gender")
        Output: answer
        ----------
        Returns a specific answer for predefined questions.
        """

    ################### NAVIGATION SERVICES ###################

    def go_to_place(self,place_name:str, graph=1, wait=True)->bool:
        """
        Input:
        place_name: the name of the place
        graph: 0 no graph || 1 graph
        wait: True (waits until the robot reaches) || False (doesn't wait)
        Output: True if the service was called correctly, False if not
        ----------
        Goes to place_name
        """

    def robot_stop_srv(self)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Stops the robot
        """

    def spin_srv(self, degrees:float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Spins the robot a number of degrees
        """

    def go_to_defined_angle_srv(self, degrees:float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Goes to defined angle
        """

    def follow_you(self)->bool:
        """
        Input: 
        Output: True if the service was called correctly, False if not
        ----------
        Follows the person in front of the robot until the person touches the head of the robot,
        the service finishes by its own
        """

    def add_place(self,name:str,persist=0,edges=[])->bool:
        """
        Input: name, edges, persist
        Output: True if the service was called correctly, False if not
        ----------
        Adds a place to the graph
        """

    ############ MANIPULATION SERVICES ###############    
        
    def grasp_object(self,object_name:str)->bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Grasp the <object_name>
        """
    
    def leave_object(self,object_name:str)->bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Leave the <object_name>
        """
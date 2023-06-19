
#!/usr/bin/env python3
import rospy
import time
import rospkg
import rosservice
import ConsoleFormatter


from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import Trigger, TriggerRequest


class Task_module:

    def __init__(self, perception: bool, speech: bool, manipulation:bool, navigation: bool):
        """Initializer for the Task_module class

        Args:
            perception (bool): Enables or disables perception services
            speech (bool): Enables or disables speech services
            manipulation (bool): Enables or disables manipulation services
            navigation (bool): Enables or disables navigation services
        """


    #################################### SERVICES #######################################

    def initialize_node(self,task_name):
        rospy.init_node('task_'+task_name+'_node')

    ################### PERCEPTION SERVICES ###################

    def turn_camera(self,camera_name:str,command:str,resolution=1,fps=10)->bool:
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

    def start_recognition(self,camera_name:str)->bool:
        """
        Input:
        camera_name: "front_camera" || "bottom_camera"
        Output: True if the service was called correctly, False if not
        ----------
        Starts recognition for <camera_name>, if <camera_name> is empty, it shuts down all recognition
        """

    def look_for_object(self,object_name:str,ignore_already_seen:bool)->bool:
        """
        Input:
        object_name: label of the object to look for
        ignore_already_seen: True->ignore objects already seen || False->don't ignore objects already seen
        Output: True if the service was called correctly, False if not
        ----------
        Looks for object_name in the current get_labels topic, publishes T/F in look_for_object_publisher
        """

    def wait_for_object(self,timeout:float)->bool:
        """
        Input: timeout in seconds || -1 for infinite
        Output: True->Found || False->Timeout/Fail
        ----------
        Waits for object to be found for a max of <timeout> seconds
        """

    def save_face(self,name:str,num_pics=5)->bool:
        """
        Input: name, num_pics
        Output: True if the service was called correctly, False if not
        ----------
        Saves num_pics of the face of the person with name and its encodings
        """

    def recognize_face(self, num_pics=3)->str:
        """
        Input: num_pics of the face to recognize
        Output: name of the recognized person
        ----------
        Recognizes the face of the person with num_pics of person
        """

    def qr_read(self,timeout:float)->str:
        """
        Input: timeout in seconds
        Output: "text" read from qr || "" if timeout or fail
        ----------
        Reads a qr code for a max of <timeout> seconds
        """

    ################### SPEECH SERVICES ###################

    def talk(self,text:str,language:str,wait=True)->str:
        """
        Input: text, language, wait(wait until the robot finishes talking)
        Output: text that indicates what Pepper said.
        ----------
        Allows the robot to say the input of the service.
        """

    def save_audio(self, seconds:int, file_name:str)->bool:
        """
        Input: seconds, file_name
        Output: 'Audio Saved' that indicates that Pepper already saved the audio.
        ----------
        Allows the robot to save audio and saves it to a file.
        """

    def q_a_speech(self, tag:str):
        """
        Input: tag in lowercase ("age", "name", "drink")
        Output: answer.
        ----------
        Returns a specific answer for predefined questions.
        """


    ################### NAVIGATION SERVICES ###################

    def go_to_place(self,place_name:str, graph=1)->bool:
        """
        Input: place_name ("door","living_room"), graph
        Output: True if the service was called correctly, False if not
        ----------
        Goes to place_name
        """

    def set_current_place(self,place_name:str)->bool:
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Sets the current place name to <place_name>
        """

    def go_to_relative_point(self,x:float,y:float,theta:float)->bool:
        """
        Input: x, y, theta
        Output: True if the service was called correctly, False if not
        ----------
        Sends the robot to the coordinates (in meters and relative to the robot)
        specified in the request message.
        """

    def start_random_navigation_srv(self)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Starts random navigation
        """

    def add_place_srv(self, place_name:str, persist:int, edges:list)->bool:
        """
        Input: place_name, persist, edges
        Output: True if the service was called correctly, False if not
        ----------
        Adds place_name to the graph
        """

    def follow_you_srv(self, place_name: str)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Starts following you
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
        Spins the robot
        """

    def go_to_defined_angle_srv(self, degrees:float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Goes to defined angle
        """


    def get_route_guidance_srv(self, place_name: str):
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Gives instructions in steps to get to the <place_name>
        """

    def constant_spin_srv(self,velocity:float)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Starts constant spin at a <velocity>
        """

    def wait_go_to_place(self)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Waits for the robot to reach the place when navigating
        """

    ################### MANIPULATION SERVICES ###################

    def saveState(self, name:str)->bool: 
        """
        Input: robot_joints as name 
        Output: True if the file is created or False if is not
        ---------
        Saves the current state of the robot's joints to a CSV file 
        """
        
    def goToState(self, joint_group_positions: list)->bool: 
        """
        Input: joint_group_position actions for the robot's arms 
        Output: True after the robot moves
        ---------
        Performs a series of actions to move the robot's arms to a desired configuration defined by joint_group_positions.        
        """


#!/usr/bin/env python3
import rospy
import time
import rospkg
import rosservice
import ConsoleFormatter

from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import Trigger, TriggerRequest, SetBool

# All imports from tools

from robot_toolkit_msgs.srv import tablet_service_srv

from manipulation_msgs.srv import GoToState, GoToAction, GraspObject

from speech_utilities_msgs.srv import q_a_speech_srv, talk_speech_srv, speech2text_srv, q_a_speech_srvRequest, talk_speech_srvRequest, speech2text_srvRequest

from perception_msgs.srv import start_recognition_srv, start_recognition_srvRequest, look_for_object_srv, look_for_object_srvRequest, save_face_srv,save_face_srvRequest, recognize_face_srv, recognize_face_srvRequest, save_image_srv,save_image_srvRequest, set_model_recognition_srv,set_model_recognition_srvRequest,read_qr_srv,read_qr_srvRequest,turn_camera_srv,turn_camera_srvRequest,filtered_image_srv,filtered_image_srvRequest

from navigation_msgs.srv import set_current_place_srv, set_current_place_srvRequest, go_to_relative_point_srv, go_to_relative_point_srvRequest, go_to_place_srv, go_to_place_srvRequest, start_random_navigation_srv, start_random_navigation_srvRequest, add_place_srv, add_place_srvRequest, follow_you_srv, follow_you_srvRequest, robot_stop_srv, robot_stop_srvRequest, spin_srv, spin_srvRequest, go_to_defined_angle_srv, go_to_defined_angle_srvRequest, get_absolute_position_srv, get_absolute_position_srvRequest, get_route_guidance_srv, get_route_guidance_srvRequest, correct_position_srv, correct_position_srvRequest, constant_spin_srv, constant_spin_srvRequest
from navigation_msgs.msg import simple_feedback_msg

class Task_module:

    def __init__(self, perception=False, speech=False, manipulation=False, navigation=False, pytoolkit = False):
        """Initializer for the Task_module class

        Args:
            perception (bool): Enables or disables perception services
            speech (bool): Enables or disables speech services
            manipulation (bool): Enables or disables manipulation services
            navigation (bool): Enables or disables navigation services
        """

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        ################### GLOBAL VARIABLES ###################
        self.object_found = False
        self.navigation_status =0

        self.perception= perception
        if perception:
            print(self.consoleFormatter.format("Waiting for PERCEPTION services...","WARNING"))

            print(self.consoleFormatter.format("Waiting for perception_utilities/turn_camera...", "WARNING"))
            rospy.wait_for_service('/perception_utilities/turn_camera_srv')
            self.turn_camera_proxy = rospy.ServiceProxy('/perception_utilities/turn_camera_srv', turn_camera_srv)

            print(self.consoleFormatter.format("Waiting for perception_utilities/start_recognition...", "WARNING"))
            rospy.wait_for_service('/perception_utilities/start_recognition_srv')
            self.start_recognition_proxy = rospy.ServiceProxy('/perception_utilities/start_recognition_srv', start_recognition_srv)

            print(self.consoleFormatter.format("Waiting for perception_utilities/look_for_object...", "WARNING"))
            rospy.wait_for_service('/perception_utilities/look_for_object_srv')
            self.look_for_object_proxy = rospy.ServiceProxy('/perception_utilities/look_for_object_srv', look_for_object_srv)

            print(self.consoleFormatter.format("Waiting for perception_utilities/save_face...", "WARNING"))
            rospy.wait_for_service('/perception_utilities/save_face_srv')
            self.save_face_proxy = rospy.ServiceProxy('/perception_utilities/save_face_srv', save_face_srv)

            print(self.consoleFormatter.format("Waiting for perception_utilities/recognize_face...", "WARNING"))
            rospy.wait_for_service('/perception_utilities/recognize_face_srv')
            self.recognize_face_proxy = rospy.ServiceProxy('/perception_utilities/recognize_face_srv', recognize_face_srv)

            print(self.consoleFormatter.format("Waiting for perception_utilities/read_qr...", "WARNING"))
            rospy.wait_for_service('perception_utilities/read_qr_srv')
            self.qr_read_proxy = rospy.ServiceProxy('/perception_utilities/read_qr_srv', read_qr_srv)

            print(self.consoleFormatter.format("Waiting for perception_utilities/filtered_image...", "WARNING"))
            rospy.wait_for_service("perception_utilities/filtered_image")
            self.filtered_image_proxy = rospy.ServiceProxy("perception_utilities/filtered_image", filtered_image_srv)

            print(self.consoleFormatter.format("PERCEPTION services enabled","OKGREEN"))

        self.speech=speech

        if speech:
            print(self.consoleFormatter.format("Waiting for SPEECH services...","WARNING"))

            print(self.consoleFormatter.format("Waiting for speech_utilities/talk_speech...", "WARNING"))
            rospy.wait_for_service('/speech_utilities/talk_speech_srv')
            self.talk_proxy = rospy.ServiceProxy('/speech_utilities/talk_speech_srv', talk_speech_srv)

            print(self.consoleFormatter.format("Waiting for speech_utilities/speech2text...", "WARNING"))
            rospy.wait_for_service('speech_utilities/speech2text_srv')
            self.speech2text_srv_proxy = rospy.ServiceProxy('speech_utilities/speech2text_srv', speech2text_srv)

            print(self.consoleFormatter.format("Waiting for speech_utilities/q_a_speech...", "WARNING"))
            rospy.wait_for_service('/speech_utilities/q_a_speech_srv')
            self.q_a_proxy = rospy.ServiceProxy('/speech_utilities/q_a_speech_srv', q_a_speech_srv)

            print(self.consoleFormatter.format("SPEECH services enabled","OKGREEN"))

        self.navigation = navigation
        if navigation:
            print(self.consoleFormatter.format("Waiting for NAVIGATION services...","WARNING"))

            print(self.consoleFormatter.format("Waiting for navigation_utilities/set_current_place...", "WARNING"))
            rospy.wait_for_service('/navigation_utilities/set_current_place_srv')
            self.set_current_place_proxy = rospy.ServiceProxy('/navigation_utilities/set_current_place_srv', set_current_place_srv)

            print(self.consoleFormatter.format("Waiting for navigation_utilities/go_to_relative_point...", "WARNING"))
            rospy.wait_for_service('navigation_utilities/go_to_relative_point_srv')
            self.go_to_relative_point_proxy = rospy.ServiceProxy('/navigation_utilities/go_to_relative_point_srv', go_to_relative_point_srv)

            print(self.consoleFormatter.format("Waiting for navigation_utilities/go_to_place...", "WARNING"))
            rospy.wait_for_service('navigation_utilities/go_to_place_srv')
            self.go_to_place_proxy = rospy.ServiceProxy('/navigation_utilities/go_to_place_srv', go_to_place_srv)

            print(self.consoleFormatter.format("Waiting for navigation_utilities/start_random_navigation...", "WARNING"))
            rospy.wait_for_service('navigation_utilities/start_random_navigation_srv')
            self.start_random_navigation_proxy = rospy.ServiceProxy('/navigation_utilities/start_random_navigation_srv', start_random_navigation_srv)

            print(self.consoleFormatter.format("Waiting for navigation_utilities/add_place...", "WARNING"))
            rospy.wait_for_service('navigation_utilities/add_place_srv')
            self.add_place_proxy = rospy.ServiceProxy('/navigation_utilities/add_place_srv', add_place_srv)

            #TODO Follow me
            # print(self.consoleFormatter.format("Waiting for navigation_utilities/follow...", "WARNING"))
            # rospy.wait_for_service('navigation_utilities/follow_you_srv')
            # self.follow_you_proxy = rospy.ServiceProxy('/navigation_utilities/follow_you_srv', follow_you_srv)

            print(self.consoleFormatter.format("Waiting for navigation_utilities/robot_stop_srv...", "WARNING"))
            rospy.wait_for_service('navigation_utilities/robot_stop_srv')
            self.robot_stop_proxy = rospy.ServiceProxy('/navigation_utilities/robot_stop_srv', robot_stop_srv)

            print(self.consoleFormatter.format("Waiting for navigation_utilities/spin_srv...", "WARNING"))
            rospy.wait_for_service('navigation_utilities/spin_srv')
            self.spin_proxy = rospy.ServiceProxy('/navigation_utilities/spin_srv', spin_srv)

            print(self.consoleFormatter.format("Waiting for navigation_utilities/go_to_defined_angle_srv...", "WARNING"))
            rospy.wait_for_service('navigation_utilities/go_to_defined_angle_srv')
            self.go_to_defined_angle_proxy = rospy.ServiceProxy('/navigation_utilities/go_to_defined_angle_srv', go_to_defined_angle_srv)

            print(self.consoleFormatter.format("Waiting for navigation_utilities/get_route_guidance_srv...", "WARNING"))
            rospy.wait_for_service('navigation_utilities/get_route_guidance_srv')
            self.get_route_guidance_proxy = rospy.ServiceProxy('/navigation_utilities/get_route_guidance_srv', get_route_guidance_srv)

            print(self.consoleFormatter.format("Waiting for navigation_utilities/constant_spin_srv...", "WARNING"))
            rospy.wait_for_service('/navigation_utilities/constant_spin_srv')
            self.constant_spin_proxy = rospy.ServiceProxy('/navigation_utilities/constant_spin_srv', constant_spin_srv)
            
            print(self.consoleFormatter.format("NAVIGATION services enabled","OKGREEN"))

        self.manipulation = manipulation
        if manipulation:
            print(self.consoleFormatter.format("Waiting for MANIPULATION services...","WARNING"))
   
            print(self.consoleFormatter.format("Waiting for manipulation_utilitites/goToState"))
            rospy.wait_for_service('manipulation_utilities/goToState')
            self.go_to_pose_proxy = rospy.ServiceProxy('manipulation_utilities/goToState', GoToState)

            print(self.consoleFormatter.format("Waiting for manipulation_utilitites/goToAction"))
            rospy.wait_for_service('manipulation_utilities/goToAction')
            self.go_to_action_proxy = rospy.ServiceProxy('manipulation_utilities/goToAction', GoToAction)

            print(self.consoleFormatter.format("Waiting for manipulation_utilitites/graspObject"))
            rospy.wait_for_service('manipulation_utilities/graspObject')
            self.grasp_object_proxy = rospy.ServiceProxy('manipulation_utilities/graspObject', GraspObject)

            print(self.consoleFormatter.format("MANIPULATION services enabled","OKGREEN"))

        self.pytoolkit = pytoolkit
        if pytoolkit:
            print(self.consoleFormatter.format("Waiting for PYTOOLKIT services...","WARNING"))

            print(self.consoleFormatter.format("Waiting for pytoolkit/awareness...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
            self.awareness_proxy = rospy.ServiceProxy("/pytoolkit/ALBasicAwareness/set_awareness_srv",SetBool)

            print(self.consoleFormatter.format("Waiting for /pytoolkit/ALTabletService/show_image_srv...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
            self.show_image_proxy = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_image_srv",tablet_service_srv)

            print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
            self.show_topic_proxy = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv",tablet_service_srv)

            print(self.consoleFormatter.format("Waiting for pytoolkit/autononumusLife...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALAutonomousLife/set_state_srv")
            self.autonomous_life_proxy = rospy.ServiceProxy("/pytoolkit/ALAutonomousLife/set_state_srv",SetBool)

            print(self.consoleFormatter.format("PYTOOLKIT services enabled","OKGREEN"))


    #################################### SERVICES #######################################

    def initialize_node(self,task_name):
        """
        Initialized the node with the name of the task
        """
        rospy.init_node('task_'+task_name+'_node')

    ################### PERCEPTION SERVICES ###################

    def turn_camera(self,camera_name:str,command="custom",resolution=1,fps=15)->bool:
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
        
    def publish_filtered_image(self,filter_name:str,camera_name:str)->bool:
        """
        Input:
        filter_name: "face" || "qr" 
        camera_name: "front_camera" || "bottom_camera"
        Output: True if the service was called correctly, False if not
        ----------
        Publishes the filtered image from camera_name with filter_name
        """
        if self.perception:
            try:
                approved = self.filtered_image_proxy(filter_name)
                return approved.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("perception as false")
            return False
        
    def start_recognition(self,camera_name:str)->bool:
        """
        Input:
        camera_name: "front_camera" || "bottom_camera"
        Output: True if the service was called correctly, False if not
        ----------
        Starts recognition for <camera_name>, if <camera_name> is empty, it shuts down all recognition
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

    def look_for_object(self,object_name:str,ignore_already_seen:bool)->bool:
        """
        Input:
        object_name: label of the object to look for options -> ("person","bench","backpack","handbag","suitcase","bottle","cup","fork","knife","spoon","bowl","chair","couch","bed","laptop")
        ignore_already_seen: True->ignore objects already seen || False->don't ignore objects already seen
        Output: True if the service was called correctly, False if not
        ----------
        Looks for object_name in the current get_labels topic, publishes T/F in look_for_object_publisher
        """
        if self.perception:
            try:
                approved = self.look_for_object_proxy(object_name,ignore_already_seen)
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
        Input: timeout in seconds || -1 for infinite
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
                subscriber = rospy.Subscriber("/perception_utilities/look_for_object_publisher", Bool, self.callback_look_for_object)
                while not finish:
                    rospy.sleep(0.05)
                    t_now = time.time()
                    if self.object_found:
                        finish=True
                        response = True
                    elif (t_now-t_start>timeout or rospy.is_shutdown()) and timeout>0:
                        finish=True
                        response = False
                return response
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("perception as false")
            return False

    def save_face(self,name:str,num_pics=5)->bool:
        """
        Input: name, num_pics
        Output: True if the service was called correctly, False if not
        ----------
        Saves num_pics of the face of the person with name and its encodings
        """
        if self.perception:
            try:
                approved = self.save_face_proxy(name,num_pics)
                print("approved",approved)
                return approved.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("perception as false")
            return False

    def recognize_face(self, num_pics=3)->str:
        """
        Input: num_pics of the face to recognize
        Output: name of the recognized person
        ----------
        Recognizes the face of the person with <num_pics>
        """
        if self.perception:
            try:
                response = self.recognize_face_proxy(num_pics)
                if response.approved:
                    return response.person
                else:
                    return ""
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return ""
        else:
            print("perception as false")
            return ""

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

    def talk(self,text:str,language="English",wait=True,animated=False)->bool:
        """
        Input: 
        text
        language: English || Spanish
        wait(wait until the robot finishes talking)
        animates: gesture hands
        Output: True if everything ok || False if not
        ----------
        Allows the robot to say the input of the service.
        """
        if self.speech:
            try:
                self.talk_proxy(text,language,wait, animated)
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("speech as false")
            return False

    def speech2text_srv(self, file_name="prueba",seconds=0,transcription=True)->bool:
        """
        Input: 
        seconds: 0 for automatic stop || >0 for seconds to record
        file_name: name of the file
        transcription: True || False
        Output: text that the robot heard
        ----------
        Allows the robot to save audio and saves it to a file.
        """
        if self.speech:
            try:
                text = self.speech2text_srv_proxy(file_name, seconds,transcription)
                return text.answer
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return ""
        else:
            print("speech as false")
            return ""

    def q_a_speech(self, tag:str)->str:
        """
        Input: tag in lowercase: options -> ("age", "name", "drink")
        Output: answer
        ----------
        Returns a specific answer for predefined questions.
        """
        if self.speech:
            try:
                answer = self.q_a_proxy(tag)
                return answer.answer
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return ""
        else:
            print("speech as false")
            return ""

    ################### NAVIGATION SERVICES ###################

    
    def set_current_place(self,place_name:str)->bool:
        """
        Input: place_name 
        Output: True if the service was called correctly, False if not
        ----------
        Sets the place of the robot to the coordinates of the place specified in
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
        Sends the robot to the coordinates (in meters relative to the robot and rotates theta angle (raidans) relative)
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

    def go_to_place(self,place_name:str, graph=1, wait=True)->bool:
        """
        Input: place_name options -> ("door","living_room"), graph
        Output: True if the service was called correctly, False if not
        ----------
        Goes to place_name
        """
        if self.navigation:
            try:
                approved = self.go_to_place_proxy(place_name, graph)
                if wait:
                    self.wait_go_to_place()
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
        
    #TODO
    # def follow_me_srv(self, place_name: str)->bool:
    #     """
    #     Input: place_name
    #     Output: True if the service was called correctly, False if not
    #     ----------
    #     Guides person to <place_name>, when person goes far stops and waits
    #     """
    #     if self.navigation:
    #         try:
    #             approved = self.follow_you_proxy(place_name)
    #             if approved=="approved":
    #                 return True
    #             else:
    #                 return False
    #         except rospy.ServiceException as e:
    #             print("Service call failed: %s"%e)
    #             return False
    #     else:
    #         print("navigation as false")
    #         return False

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
        Spins the robot a number of degrees
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

    def get_route_guidance_srv(self, place_name: str):
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Gives instructions in steps to get to the <place_name>
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

    def constant_spin_srv(self,velocity:float)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Starts constant spin at a <velocity> in degrees/sec (5-25)
        """
        if self.navigation:
            try:
                approved = self.constant_spin_proxy(velocity)
                if approved=="approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def wait_go_to_place(self)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Waits for the robot to reach the place when navigating
        """
        if self.navigation:
            try:
                print("Waiting to reach the place")
                finish=False
                response = False
                subscriber = self.simpleFeedbackSubscriber = rospy.Subscriber('/navigation_utilities/simple_feedback', simple_feedback_msg, self.callback_simple_feedback_subscriber)
                while not finish:
                    rospy.sleep(0.05)
                    if self.navigation_status == 2:
                        finish=True
                        response = True
                    elif rospy.is_shutdown():
                        finish=True
                        response = False
                
                return response
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("perception as false")
            return False
        
    ############ MANIPULATION SERVICES ###############    
    
    def go_to_pose(self,pose:str)->bool:
        """
        Input: pose options ->("bowl","box","cylinder","medium_object", "small_object_left_hand","small_object_right_hand","tray","head_up","head_down","head_default")
        Output: True if the service was called correctly, False if not
        ----------
        Goes to pose with hands or head
        """
        if self.manipulation:
            try:
                approved = self.go_to_pose_proxy(pose)
                if approved=="OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("manipulation as false")
            return False
        
    def execute_trayectory(self,trayectory:str)->bool:
        """
        Input: trayectory options ->("place_both_arms","place_left_arm","place_right_arm")
        Output: True if the service was called correctly, False if not
        ----------
        Executes trayectory with the hands
        """
        if self.manipulation:
            try:
                approved = self.go_to_action_proxy(trayectory)
                if approved=="OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("manipulation as false")
            return False
        
    def grasp_object(self,object_name:str)->bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Grasp the <object_name>
        """
        if self.manipulation:
            try:
                approved = self.grasp_object_proxy(object_name)
                if approved=="OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("manipulation as false")
            return False

    ################ PYTOOLKIT ################

    def show_topic(self, topic:str)->bool:
        """
        Input: topic
        Output: True if the service was called correctly, False if not
        ----------
        Displays the topic on the screen of the robot
        """
        if self.pytoolkit:
            try:
                approved = self.show_topic_proxy(topic)
                if approved=="OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False       
        
    def show_image(self, image_path:str)->bool:
        """
        Input: image_path or allreade saved image options -> ("sinfonia")
        Output: True if the service was called correctly, False if not
        ----------
        Displays the image on the screen of the robot
        """
        images = {"sinfonia":"https://media.discordapp.net/attachments/876543237270163498/1123649957791010939/logo_sinfonia_2.png"}
        if self.pytoolkit:
            try:
                url = image_path
                if image_path in images:
                    url = images[image_path]
                approved = self.show_image_proxy(url)
                if approved=="OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def set_awareness(self, state:bool)->bool:
        """
        Input: True turn on || False turn off
        Output: True if the service was called correctly, False if not
        ----------
        Sets the awareness of the robot
        """
        if self.pytoolkit:
            try:
                approved = self.awareness_proxy(state)
                if approved=="OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def set_autonomous_life(self, state:bool)->bool:
        """
        Input: True turn on || False turn off
        Output: True if the service was called correctly, False if not
        ----------
        Sets the autonomous life of the robot
        """
        if self.pytoolkit:
            try:
                approved = self.autonomous_life_proxy(state)
                if approved=="OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False
     
    ################ SUBSCRIBER CALLBACKS ################

    def callback_look_for_object(self,data):
        self.object_found=data.data
        return data

    def callback_simple_feedback_subscriber(self, msg):
        self.navigation_status = msg.navigation_status


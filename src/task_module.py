#!/usr/bin/env python3
import rospy
import time
import rospkg
import rosservice
import ConsoleFormatter
import json

from std_msgs.msg import Int32, String, Bool
from std_srvs.srv import Trigger, TriggerRequest, SetBool
from geometry_msgs.msg import Twist
from threading import Thread

# All imports from tools

from robot_toolkit_msgs.srv import set_move_arms_enabled_srv,  misc_tools_srv, misc_tools_srvRequest, tablet_service_srv, battery_service_srv , set_security_distance_srv, move_head_srv, go_to_posture_srv, set_security_distance_srv,set_speechrecognition_srv,speech_recognition_srv
from robot_toolkit_msgs.msg import touch_msg

from manipulation_msgs_pytoolkit.srv import GoToState, GoToAction, GraspObject

from speech_msgs.srv import q_a_srv, talk_srv, speech2text_srv , talk_srvRequest, speech2text_srvRequest, answer_srv, calibrate_srv, hot_word_srv

from perception_msgs.srv import start_recognition_srv, get_labels_srv, start_recognition_srvRequest, look_for_object_srv, look_for_object_srvRequest, save_face_srv,save_face_srvRequest, recognize_face_srv, recognize_face_srvRequest, save_image_srv,save_image_srvRequest, set_model_recognition_srv,set_model_recognition_srvRequest,read_qr_srv,read_qr_srvRequest,turn_camera_srv,turn_camera_srvRequest,filtered_image_srv,filtered_image_srvRequest,start_pose_recognition_srv #,get_person_description_srv

from navigation_msgs.srv import set_current_place_srv, set_current_place_srvRequest, go_to_relative_point_srv, go_to_relative_point_srvRequest, go_to_place_srv, go_to_place_srvRequest, start_random_navigation_srv, start_random_navigation_srvRequest, add_place_srv, add_place_srvRequest, follow_you_srv, follow_you_srvRequest, robot_stop_srv, robot_stop_srvRequest, spin_srv, spin_srvRequest, go_to_defined_angle_srv, go_to_defined_angle_srvRequest, get_absolute_position_srv, get_absolute_position_srvRequest, get_route_guidance_srv, get_route_guidance_srvRequest, correct_position_srv, correct_position_srvRequest, constant_spin_srv, constant_spin_srvRequest
from navigation_msgs.msg import simple_feedback_msg
from perception_msgs.msg import get_labels_msg

class Task_module:
    def __init__(
        self,
        perception=False,
        speech=False,
        manipulation=False,
        navigation=False,
        pytoolkit=False,
    ):
        """Initializer for the Task_module class

        Args:
            perception (bool): Enables or disables perception services
            speech (bool): Enables or disables speech services
            manipulation (bool): Enables or disables manipulation services
            navigation (bool): Enables or disables navigation services
        """

        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        ################### GLOBAL VARIABLES ###################
        self.follow_you_active = True
        self.max_tuple = [0,0,0,0,0]
        self.labels = dict()
        self.say_go_ahead = True
        self.object_found = False
        self.isTouched = False
        self.navigation_status = 0
        self.perception = perception
        if perception:
            print(
                self.consoleFormatter.format(
                    "Waiting for PERCEPTION services...", "WARNING"
                )
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/turn_camera...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/turn_camera_srv")
            self.turn_camera_proxy = rospy.ServiceProxy(
                "/perception_utilities/turn_camera_srv", turn_camera_srv
            )


            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/get_labels...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/get_labels_srv")
            self.get_labels_proxy = rospy.ServiceProxy(
                "/perception_utilities/get_labels_srv", get_labels_srv
            )


            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/start_recognition...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/start_recognition_srv")
            self.start_recognition_proxy = rospy.ServiceProxy(
                "/perception_utilities/start_recognition_srv", start_recognition_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/look_for_object...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/look_for_object_srv")
            self.look_for_object_proxy = rospy.ServiceProxy(
                "/perception_utilities/look_for_object_srv", look_for_object_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/save_face...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/save_face_srv")
            self.save_face_proxy = rospy.ServiceProxy(
                "/perception_utilities/save_face_srv", save_face_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/recognize_face...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/recognize_face_srv")
            self.recognize_face_proxy = rospy.ServiceProxy(
                "/perception_utilities/recognize_face_srv", recognize_face_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/read_qr...", "WARNING"
                )
            )
            rospy.wait_for_service("perception_utilities/read_qr_srv")
            self.qr_read_proxy = rospy.ServiceProxy(
                "/perception_utilities/read_qr_srv", read_qr_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/filtered_image...", "WARNING"
                )
            )
            rospy.wait_for_service("perception_utilities/filtered_image")
            self.filtered_image_proxy = rospy.ServiceProxy(
                "perception_utilities/filtered_image", filtered_image_srv
            )

            # print(self.consoleFormatter.format("Waiting for perception_utilities/get_person_description...", "WARNING"))
            # print(self.consoleFormatter.format("Waiting for perception_utilities/get_person_description...", "WARNING"))
            # rospy.wait_for_service("/perception_utilities/get_person_description_srv")
            # self.get_person_description_proxy = rospy.ServiceProxy("/perception_utilities/get_person_description_srv", get_person_description_srv)

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/pose_srv...", "WARNING"
                )
            )
            rospy.wait_for_service("/perception_utilities/pose_srv")
            self.pose_srv_proxy = rospy.ServiceProxy(
                "/perception_utilities/pose_srv", start_pose_recognition_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for perception_utilities/set_model_recognition...",
                    "WARNING",
                )
            )
            rospy.wait_for_service("perception_utilities/set_model_recognition_srv")
            self.set_model_proxy = rospy.ServiceProxy(
                "perception_utilities/set_model_recognition_srv",
                set_model_recognition_srv,
            )

            print(
                self.consoleFormatter.format("PERCEPTION services enabled", "OKGREEN")
            )

        self.speech = speech

        if speech:
            print(self.consoleFormatter.format("Waiting for SPEECH services...","WARNING"))

            print(self.consoleFormatter.format("Waiting for speech_utilities/talk_speech...", "WARNING"))
            #rospy.wait_for_service('/speech_utilities/talk_speech_srv')
            #self.talk_proxy = rospy.ServiceProxy('/speech_utilities/talk_speech_srv', talk_speech_srv)

            print(self.consoleFormatter.format("Waiting for speech_utilities/speech2text...", "WARNING"))
            rospy.wait_for_service('speech_utilities/speech2text_srv')
            self.speech2text_srv_proxy = rospy.ServiceProxy('speech_utilities/speech2text_srv', speech2text_srv)

            print(self.consoleFormatter.format("Waiting for speech_utilities/q_a_speech...", "WARNING"))
            #rospy.wait_for_service('/speech_utilities/q_a_speech_srv')
            #self.q_a_proxy = rospy.ServiceProxy('/speech_utilities/q_a_speech_srv', q_a_speech_srv)

            print(self.consoleFormatter.format("Waiting for speech_utilities/answer...", "WARNING"))
            rospy.wait_for_service('/speech_utilities/answers_srv')
            self.answer_proxy = rospy.ServiceProxy('/speech_utilities/answers_srv', answer_srv)

            print(self.consoleFormatter.format("Waiting for speech_utilities/live_transcription...", "WARNING"))
            rospy.wait_for_service('/speech_utilities/live_transcription_srv')
            self.live_transcription_proxy = rospy.ServiceProxy('/speech_utilities/live_transcription_srv', live_transcription_srv)

            print(self.consoleFormatter.format("SPEECH services enabled","OKGREEN"))

        self.navigation = navigation
        if navigation:
            print(
                self.consoleFormatter.format(
                    "Waiting for NAVIGATION services...", "WARNING"
                )
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/set_current_place...", "WARNING"
                )
            )
            rospy.wait_for_service("/navigation_utilities/set_current_place_srv")
            self.set_current_place_proxy = rospy.ServiceProxy(
                "/navigation_utilities/set_current_place_srv", set_current_place_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/go_to_relative_point...",
                    "WARNING",
                )
            )
            rospy.wait_for_service("navigation_utilities/go_to_relative_point_srv")
            self.go_to_relative_point_proxy = rospy.ServiceProxy(
                "/navigation_utilities/go_to_relative_point_srv",
                go_to_relative_point_srv,
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/go_to_place...", "WARNING"
                )
            )
            rospy.wait_for_service("navigation_utilities/go_to_place_srv")
            self.go_to_place_proxy = rospy.ServiceProxy(
                "/navigation_utilities/go_to_place_srv", go_to_place_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/start_random_navigation...",
                    "WARNING",
                )
            )
            rospy.wait_for_service("navigation_utilities/start_random_navigation_srv")
            self.start_random_navigation_proxy = rospy.ServiceProxy(
                "/navigation_utilities/start_random_navigation_srv",
                start_random_navigation_srv,
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/add_place...", "WARNING"
                )
            )
            rospy.wait_for_service("navigation_utilities/add_place_srv")
            self.add_place_proxy = rospy.ServiceProxy(
                "/navigation_utilities/add_place_srv", add_place_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/follow...", "WARNING"
                )
            )
            rospy.wait_for_service("/navigation_utilities/follow_you_srv")
            self.follow_you_proxy = rospy.ServiceProxy(
                "/navigation_utilities/follow_you_srv", follow_you_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/robot_stop_srv...", "WARNING"
                )
            )
            rospy.wait_for_service("navigation_utilities/robot_stop_srv")
            self.robot_stop_proxy = rospy.ServiceProxy(
                "/navigation_utilities/robot_stop_srv", robot_stop_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/spin_srv...", "WARNING"
                )
            )
            rospy.wait_for_service("navigation_utilities/spin_srv")
            self.spin_proxy = rospy.ServiceProxy(
                "/navigation_utilities/spin_srv", spin_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/go_to_defined_angle_srv...",
                    "WARNING",
                )
            )
            rospy.wait_for_service("navigation_utilities/go_to_defined_angle_srv")
            self.go_to_defined_angle_proxy = rospy.ServiceProxy(
                "/navigation_utilities/go_to_defined_angle_srv", go_to_defined_angle_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/get_route_guidance_srv...",
                    "WARNING",
                )
            )
            rospy.wait_for_service("navigation_utilities/get_route_guidance_srv")
            self.get_route_guidance_proxy = rospy.ServiceProxy(
                "/navigation_utilities/get_route_guidance_srv", get_route_guidance_srv
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for navigation_utilities/constant_spin_srv...", "WARNING"
                )
            )
            rospy.wait_for_service("/navigation_utilities/constant_spin_srv")
            self.constant_spin_proxy = rospy.ServiceProxy(
                "/navigation_utilities/constant_spin_srv", constant_spin_srv
            )

            print(
                self.consoleFormatter.format("NAVIGATION services enabled", "OKGREEN")
            )

        self.manipulation = manipulation
        if manipulation:
            print(
                self.consoleFormatter.format(
                    "Waiting for MANIPULATION services...", "WARNING"
                )
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for manipulation_utilitites/goToState", "WARNING"
                )
            )
            rospy.wait_for_service("manipulation_utilities/goToState")
            self.go_to_pose_proxy = rospy.ServiceProxy(
                "manipulation_utilities/goToState", GoToState
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for manipulation_utilitites/goToAction", "WARNING"
                )
            )
            rospy.wait_for_service("manipulation_utilities/goToAction")
            self.go_to_action_proxy = rospy.ServiceProxy(
                "manipulation_utilities/goToAction", GoToAction
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for manipulation_utilitites/graspObject", "WARNING"
                )
            )
            rospy.wait_for_service("manipulation_utilities/graspObject")
            self.grasp_object_proxy = rospy.ServiceProxy(
                "manipulation_utilities/graspObject", GraspObject
            )

            print(
                self.consoleFormatter.format("MANIPULATION services enabled", "OKGREEN")
            )

        self.pytoolkit = pytoolkit
        if pytoolkit:
            print(
                self.consoleFormatter.format(
                    "Waiting for PYTOOLKIT services...", "WARNING"
                )
            )

            self.setMoveHead_srv = rospy.ServiceProxy(
                "/pytoolkit/ALMotion/move_head_srv",
                move_head_srv
            )
            
            self.setDistance_srv = rospy.ServiceProxy(
                "/pytoolkit/ALMotion/set_security_distance_srv",
                set_security_distance_srv
            )

            self.setRPosture_srv = rospy.ServiceProxy(
                "/pytoolkit/ALRobotPosture/go_to_posture_srv",
                go_to_posture_srv
            )

            self.setMoveArms_srv = rospy.ServiceProxy(
                "pytoolkit/ALMotion/set_move_arms_enabled_srv",
                set_move_arms_enabled_srv
            )
            print(
                self.consoleFormatter.format(
                    "Waiting for pytoolkit/awareness...", "WARNING"
                )
            )
            rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
            self.awareness_proxy = rospy.ServiceProxy(
                "/pytoolkit/ALBasicAwareness/set_awareness_srv", SetBool
            )

            print(
                self.consoleFormatter.format(
                    "Waiting for /pytoolkit/ALTabletService/show_image_srv...",
                    "WARNING",
                )
            )

            print(self.consoleFormatter.format("Waiting for pytoolkit/ALSpeechRecognition/set_speechrecognition_srv...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALSpeechRecognition/set_speechrecognition_srv")
            self.speech_recognition = rospy.ServiceProxy("/pytoolkit/ALSpeechRecognition/set_speechrecognition_srv", set_speechrecognition_srv)

            print(self.consoleFormatter.format("Waiting for pytoolkit/ALSpeechRecognition/set_words_srv...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALSpeechRecognition/set_words_srv")
            self.set_words = rospy.ServiceProxy("/pytoolkit/ALSpeechRecognition/set_words_srv", speech_recognition_srv)

            rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
            self.awareness_proxy = rospy.ServiceProxy(
                "/pytoolkit/ALBasicAwareness/set_awareness_srv", SetBool
            )


            rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
            self.show_image_proxy = rospy.ServiceProxy(
                "/pytoolkit/ALTabletService/show_image_srv", tablet_service_srv
            )

            print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
            self.show_topic_proxy = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv", tablet_service_srv)

            print(self.consoleFormatter.format("Waiting for pytoolkit/autononumusLife...", "WARNING"))
            
            rospy.wait_for_service("/pytoolkit/ALMotion/set_security_distance_srv")
            self.set_security_distance_proxy = rospy.ServiceProxy("/pytoolkit/ALMotion/set_security_distance_srv", set_security_distance_srv)
            print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/set_security_distance_srv...", "WARNING"))

            rospy.wait_for_service("/pytoolkit/ALAutonomousLife/set_state_srv")
            self.autonomous_life_proxy = rospy.ServiceProxy("/pytoolkit/ALAutonomousLife/set_state_srv",SetBool)

            print(self.consoleFormatter.format("Waiting for pytoolkit/stop_tracker...", "WARNING"))
            rospy.wait_for_service("/pytoolkit/ALTracker/stop_tracker_srv")
            self.stop_tracker_proxy = rospy.ServiceProxy("/pytoolkit/ALTracker/stop_tracker_srv",battery_service_srv)

            print(self.consoleFormatter.format("PYTOOLKIT services enabled","OKGREEN"))

            print(self.consoleFormatter.format("PYTOOLKIT services enabled", "OKGREEN"))

    #################################### SERVICES #######################################

    def initialize_node(self, task_name):
        """
        Initialized the node with the name of the task
        """
        rospy.init_node("task_" + task_name + "_node")

    def initialize_pepper(self):
        """
        Initializes the pepper robot with default parameteres
        """
        if self.pytoolkit and self.perception:
            self.turn_camera("front_camera","custom",1,15)
            self.start_recognition("front_camera")
            self.calibrate_srv(5)
            # quedarse como un coco
        else:
            print("pytoolkit or perception as false")

    ################### PERCEPTION SERVICES ###################

    def turn_camera(
        self, camera_name: str, command="custom", resolution=1, fps=15
    ) -> bool:
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
                approved = self.turn_camera_proxy(camera_name, command, resolution, fps)
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def publish_filtered_image(self, filter_name: str, camera_name: str) -> bool:
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
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def start_recognition(self, camera_name: str) -> bool:
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
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def get_labels(self, bool):
        if self.perception:
    
            try:
                return self.get_labels_proxy.call(bool)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
            return ""
            
    def look_for_object(self, object_name: str, ignore_already_seen=False) -> bool:
        """
        Input:
        object_name: label of the object to look for options -> classes names depends of the actual model (see set_model service)
        ignore_already_seen: True->ignore objects already seen || False->don't ignore objects already seen
        Output: True if the service was called correctly, False if not
        ----------
        Looks for object_name in the current get_labels topic, publishes T/F in look_for_object_publisher
        """
        if self.perception:
            try:
                approved = self.look_for_object_proxy(object_name, ignore_already_seen)
                if approved == "approved":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def wait_for_object(self, timeout: float) -> bool:
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
                finish = False
                response = False
                subscriber = rospy.Subscriber(
                    "/perception_utilities/look_for_object_publisher",
                    Bool,
                    self.callback_look_for_object,
                )
                while not finish:
                    rospy.sleep(0.0666)
                    t_now = time.time()
                    if self.object_found:
                        finish = True
                        response = True
                    elif (
                        t_now - t_start > timeout or rospy.is_shutdown()
                    ) and timeout > 0:
                        finish = True
                        response = False
                return response
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def find_object(self, object_name: str, timeout=25) -> bool:
        """
        Input:
        object_name: label of the object to look for options -> classes names depends of the actual model (see set_model service)
        timeout: time in seconds to look for the object while spinning
        Output: True if the object was found, False if not
        ----------
        Spins while looking for <object_name> for <timeout> seconds while spinning at 15 deg/s
        """
        # spins until the object is found or timeout
        if self.perception and self.navigation:
            try:
                # self.go_to_pose("default_head")
                self.look_for_object(object_name)
                self.constant_spin_srv(15)
                found = self.wait_for_object(timeout)
                self.robot_stop_srv()
                return found
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception, manipulation or navigation as false")

    def count_objects(self, object_name: str) -> int:
        """
        Input: object_name, classes names depends of the actual model (see set_model service)
        Output: Number of objects seen when rotating 360
        ----------
        Spins 360 degrees and then returns the number of objects of <object_name> seen
        """
        if self.perception and self.navigation and self.manipulation:
            try:
                counter = 0
                self.go_to_pose("default_head")
                self.look_for_object(object_name, ignore_already_seen=True)
                self.constant_spin_srv(15)
                t1 = time.time()
                while time.time() - t1 < 22:
                    if self.object_found:
                        counter += 1
                self.robot_stop_srv()
                return counter
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception, manipulation or navigation as false")

    def save_face(self, name: str, num_pics=5) -> bool:
        """
        Input: name, num_pics
        Output: True if the service was called correctly, False if not
        ----------
        Saves num_pics of the face of the person with name and its encodings
        """
        if self.perception:
            try:
                approved = self.save_face_proxy(name, num_pics)
                print("approved", approved)
                return approved.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def recognize_face(self, num_pics=3) -> str:
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
                print("Service call failed: %s" % e)
                return ""
        else:
            print("perception as false")
            return ""

    def get_person_description(self) -> dict:
        """
        Input:
        Output: dict with the description of the person {"status":(happy,neutral,sad,angry),"gender":("Man","Woman"),"age":(int),"race":(race of the person)}
        ----------
        Returns a dict with the description of the person
        """
        attributes = {}
        if self.perception:
            try:
                response = self.get_person_description_proxy()
                attributes = {
                    "status": response.status,
                    "gender": response.gender,
                    "age": int(response.age),
                    "race": response.attributes,
                }
                return attributes
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return {}
        else:
            print("perception as false")
            return {}

    def pose_srv(self, camera_name: str, start: bool) -> bool:
        """
        Input: camera_name
        Output: True if the service was called correctly, False if not
        ----------
        Saves the pose of the person in front of the camera
        """
        if self.perception:
            try:
                approved = self.pose_srv_proxy(camera_name, start)
                return approved.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def qr_read(self, timeout: float) -> str:
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
                print("Service call failed: %s" % e)
                return ""
        else:
            print("perception as false")
            return ""

    def set_model(self, model_name: str) -> bool:
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
        if self.perception:
            try:
                approved = self.set_model_proxy(model_name)
                # wait for the model to be set
                rospy.sleep(3)
                return approved.approved
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False
    
    def get_closer_person(self):
        while self.follow_you_active:
            labels_actuales = self.labels
            for label in labels_actuales:
                if label == "person":
                    self.max_tuple = max(labels_actuales[label], key=lambda x: x[3])
                    self.i = 0

    ################### SPEECH SERVICES ###################

    def talk(self, text: str, language="English", wait=True, animated=False) -> bool:
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
                self.talk_proxy(text, language, wait, animated)
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("speech as false")
            return False
        
    def hot_word(self, words: list, noise = False, eyes = False, threshold = 0.4):
        """
        Input:
        words -> list of words to detect
        noise -> True || False if there is a noise when the hot worrd is detected
        eyes -> True || False if the eyes shine 
        threshold -> Threshold to detect a word
        Output: True if everything ok || False if not
        ----------
        Allows the robot to detect hot words
        """
        if self.speech:
            try:
                self.hot_word_srv(words, noise, eyes, threshold)
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("speech as false")
            return False

    def calibrate_srv(self, duration = 5)->bool:
        """
        Input: duration in seconds
        Output: True if the service was called correctly, False if not
        ----------
        Calibrates the robot's microphone for <duration> seconds
        """
        if self.speech:
            try:
                threshold = self.calibrate_speech_proxy(duration)
                if threshold:
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("speech as false")
            return False

    def speech2text_srv(
        self, seconds=0
    ) -> bool:
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
                text = self.speech2text_srv_proxy(seconds)
                return text.transcription
            except rospy.ServiceException as e  :
                print("Service call failed: %s" % e)
                return ""
        else:
            print("speech as false")
            return ""
        
    def hot_word(self, hot_words:list,threshold = 0.4):
        """
        Input: hot_words
        Output:
        ----------
        Activates the hot word detection
        """
        if self.pytoolkit:
            try:
                if hot_words == []:
                    self.speech_recognition(False,False,False)
                else:
                    print("speech_recogntion")
                    self.speech_recognition(True,False,False)
                    print("hot_words")
                    self.set_words(hot_words,threshold)
                return True                
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("speech as false")
            return False
        
    def live_transcription_proxy_thread(self):
        self.live_transcription_proxy(True)
        rospy.time(1000)
        
    def answer_question(self, question:str, temperature = 0.5, save_conversation = True, fill_time = True)->str:
        """
        Input: 
        question: String question to respond 
        temperature: scale from 0 to 1 being 0 deterministic and 1 creative
        save_conversation: Save history of the conversation or not
        fill_time: Fill with speaking 
        Output: 
        answer: A string with the answer to the question
        """
        if self.speech:
            try:
                response = self.answer_proxy(question,save_conversation, temperature, "").answer
            except rospy.ServiceException as e:
                print("Service call failedL %s"%e)
                response = "I could not find relevant results for your question "
        return response
    
    def q_a(self, tag:str)->str:
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
                print("Service call failed: %s" % e)
                return ""
        else:
            print("speech as false")
            return ""

    ################### NAVIGATION SERVICES ###################

    def set_current_place(self, place_name: str) -> bool:
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Sets the place of the robot to the coordinates of the place specified in
        the request message, then it clears the global costmap.
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.set_current_place_proxy(place_name)
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

    def go_to_relative_point(self, x: float, y: float, theta: float) -> bool:
        """
        Input: x, y, theta
        Output: True if the service was called correctly, False if not
        ----------
        Sends the robot to the coordinates (in meters relative to the robot and rotates theta angle (raidans) relative)
        specified in the request message.
        """
        if self.navigation:
            approved=False
            try:
                self.set_move_arms_enabled(False)
                if self.pytoolkit:
                    approved = self.go_to_relative_point_proxy(x,y,theta)
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

    def go_to_place(self, place_name: str, graph=1, wait=True) -> bool:
        """
        Input:
        place_name: options -> ("door","living_room")
        graph: 0 no graph || 1 graph
        wait: True (waits until the robot reaches) || False (doesn't wait)
        Output: True if the service was called correctly, False if not
        ----------
        Goes to place_name
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                if self.pytoolkit:
                    self.stop_tracker_proxy()
                    self.stop_tracker_proxy()
                    self.stop_tracker_proxy()
                approved = self.go_to_place_proxy(place_name, graph)
                if self.pytoolkit:
                    self.stop_tracker_proxy()
                    self.stop_tracker_proxy()
                    self.stop_tracker_proxy()
                if wait:
                    if self.pytoolkit:
                        self.stop_tracker_proxy()
                        self.stop_tracker_proxy()
                        self.stop_tracker_proxy()
                    self.wait_go_to_place()
                    if self.pytoolkit:
                        self.stop_tracker_proxy()
                        self.stop_tracker_proxy()
                        self.stop_tracker_proxy()
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

    def follow_you(self, command: bool) -> bool:
        """
        Input:
        start: Start to follow a person
        Output: True if the service was called correctly, False if not
        ----------
        Follows the person in front of the robot until the person touches the head of the robot
        """
        rospy.Subscriber('/perception_utilities/get_labels_publisher', get_labels_msg, self.callback_get_labels_subscriber)
        self.cmd_velPublisher = rospy.Publisher('/pytoolkit/ALMotion/move', Twist, queue_size=10)
        
        if self.navigation and self.pytoolkit:
            try:
                if  command:
                    self.set_move_arms_enabled(False)
                    self.follow_you_active = command
                    self.setDistance_srv.call(0.3)
                    service_thread = Thread(target=self.follow_you_srv_thread)
                    service_thread.start()
                    head_thread = Thread(target=self.head_srv_thread)
                    head_thread.start()
                    closer_thread = Thread(target=self.get_closer_person)
                    closer_thread.start()
                    self.follow_you_proxy(command)
                else:
                    self.set_move_arms_enabled(True)
                    self.follow_you_active = command
                    self.follow_you_proxy(command)
                    cmd_vel_msg = Twist() 
                    cmd_vel_msg.linear.x = 0
                    cmd_vel_msg.angular.z = 0
                    self.cmd_velPublisher.publish(cmd_vel_msg)
                    return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False
    
    def head_srv_thread(self):
        while self.follow_you_active: 
            self.setMoveHead_srv.call("up")
            rospy.sleep(8)
    
    def follow_you_srv_thread(self):
        moviendose = False
        cerca = False
        self.i=0
        target_x = 0
        center_x = 320 / 2
        linear_vel = 0.3
        error_x = target_x - center_x
        angular_vel = 0
        id_max_tuple = self.max_tuple[0]
        while self.follow_you_active: 
            cmd_vel_msg = Twist()
            id_max_tuple = self.max_tuple[0]
            if "person" in self.labels:
                if self.labels["person"][0][0] == id_max_tuple:
                    result = self.labels["person"][0]
                else:
                    continue
                if len(result) == 0 and moviendose:
                    moviendose = False
                    cmd_vel_msg.linear.x = 0
                    cmd_vel_msg.angular.z = 0
                    self.cmd_velPublisher.publish(cmd_vel_msg)
                    self.i+=1
                    continue
                else:
                    self.i=0
                if len(result) != 0:
                    target_x = result[1]
                    error_x = target_x - center_x
                    angular_vel = 0.004 * error_x
                if self.max_tuple[3]>=170:
                    cerca = True
                    if moviendose:
                        moviendose = False
                        cmd_vel_msg.linear.x = 0
                        cmd_vel_msg.angular.z = 0
                        self.cmd_velPublisher.publish(cmd_vel_msg)
                else:
                    cerca = False
                if (abs(angular_vel) > 0.25 or not moviendose) and not cerca:
                    moviendose = True
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.linear.x = linear_vel
                    cmd_vel_msg.angular.z = angular_vel
                    self.cmd_velPublisher.publish(cmd_vel_msg)
                    rospy.sleep(0.2)
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.linear.x = linear_vel
                    cmd_vel_msg.angular.z = 0
                    self.cmd_velPublisher.publish(cmd_vel_msg)
            else:
                self.i+=1
            if self.i >= 20:
                moviendose = False
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = 0
                self.cmd_velPublisher.publish(cmd_vel_msg)
                self.setMoveHead_srv.call("up")
                self.talk_proxy("I have lost you, please stand in front of me, I will tell you when you can continue", "English", True, False)
                print("I have lost you, please stand in front of me, I will tell you when you can continue")
                rospy.sleep(3)
                id_max_tuple = self.max_tuple[0]
                self.talk_proxy("GO AHEAD!", "English", True, False)
                print("GO AHEAD!")

    def robot_stop_srv(self) -> bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Stops the robot
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.robot_stop_proxy()
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

    def spin_srv(self, degrees: float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Spins the robot a number of degrees
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.spin_proxy(degrees)
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

    def go_to_defined_angle_srv(self, degrees: float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Goes to defined angle
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.go_to_defined_angle_proxy(degrees)
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

    def get_route_guidance_srv(self, place_name: str):
        """
        Input: place_name
        Output: True if the service was called correctly, False if not
        ----------
        Gives instructions in steps to get to the <place_name>
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                instructions = self.get_route_guidance_proxy(place_name)
                return instructions
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("navigation as false")
            return False

    def constant_spin_srv(self, velocity: float) -> bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Starts constant spin at a <velocity> in degrees/sec (5-25)
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.constant_spin_proxy(velocity)
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

    def wait_go_to_place(self) -> bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Waits for the robot to reach the place when navigating
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                print("Waiting to reach the place")
                finish = False
                response = False
                self.simpleFeedbackSubscriber = rospy.Subscriber(
                    "/navigation_utilities/simple_feedback",
                    simple_feedback_msg,
                    self.callback_simple_feedback_subscriber,
                )
                while not finish:
                    rospy.sleep(0.05)
                    if self.navigation_status == 2:
                        finish = True
                        response = True
                    elif rospy.is_shutdown():
                        finish = True
                        response = False

                return response
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("perception as false")
            return False

    def add_place(self, name: str, persist=0, edges=[]) -> bool:
        """
        Input: name, edges, persist
        Output: True if the service was called correctly, False if not
        ----------
        Adds a place to the graph
        """
        if self.navigation:
            try:
                self.set_move_arms_enabled(False)
                approved = self.add_place_proxy(name, persist, edges)
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

    ############ MANIPULATION SERVICES ###############

    def go_to_pose(self, pose: str, velocity=0.05) -> bool:
        """
        Input: pose options ->("bowl","box","cylinder","medium_object", "small_object_left_hand","small_object_right_hand","tray","head_up","head_down","head_default")
        Output: True if the service was called correctly, False if not
        ----------
        Goes to pose with hands or head
        """
        if self.manipulation:
            try:
                approved = self.go_to_pose_proxy(pose, velocity)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("manipulation as false")
            return False

    ############ MANIPULATION SERVICES ###############

    def saveState(self, name: str) -> bool:
        """
        Input: state as name
        Output: True if the state was created or False if is not
        ---------
        Saves the current state of the robot's joints to a CSV file
        """
        if self.manipulation:
            try:
                name_msg = String()
                name_msg.data = name
                save_state = self.saveState_proxy(name_msg)
                if save_state:
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("manipulation as false")
            return False

    def execute_trayectory(self, trayectory: str) -> bool:
        """
        Input: trayectory options ->("place_both_arms","place_left_arm","place_right_arm")
        Output: True if the service was called correctly, False if not
        ----------
        Executes trayectory with the hands
        """
        if self.manipulation:
            try:
                approved = self.go_to_action_proxy(trayectory)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("manipulation as false")
            return False

    def goToState(self, name: str) -> bool:
        """
        Input: state to be executed.
        Output: True after the robot moves.
        ---------
        Performs a series of actions to move the robot's arms to a desired configuration defined by joint_group_positions.
        """
        if self.manipulation:
            try:
                name_msg = String()
                name_msg.data = name
                go_to_state = self.goToState_proxy(name_msg)
                if go_to_state:
                    return True
                else:
                    print("ccannot move")
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("manipulation as false")
            return False

    def grasp_object(self, object_name: str) -> bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Grasp the <object_name>
        """
        if self.manipulation:
            try:
                self.setMoveArms_srv.call(False, False)
                self.execute_trayectory("request_help_both_arms")
                self.talk("Could you place the "+object_name+" in my hands, please?","English",wait=True)
                rospy.sleep(9)
                self.go_to_pose("almost_open_both_hands")
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("manipulation as false")
            return False

    def leave_object(self, object_name: str) -> bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Leave the <object_name>
        """
        if self.manipulation:
            try:
                self.talk("Please pick up the "+object_name,"English",wait=True)
                rospy.sleep(7)
                self.execute_trayectory("place_both_arms") 
                self.setMoveArms_srv.call(True, True)           
                return True
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("manipulation as false")
            return False

    ################ PYTOOLKIT ################

    def show_topic(self, topic: str) -> bool:
        """
        Input: topic
        Output: True if the service was called correctly, False if not
        ----------
        Displays the topic on the screen of the robot
        """
        if self.pytoolkit:
            try:
                approved = self.show_topic_proxy(topic)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def show_image(self, image_path: str) -> bool:
        """
        Input: image_path or allreade saved image options -> ("sinfonia")
        Output: True if the service was called correctly, False if not
        ----------
        Displays the image on the screen of the robot
        """
        images = {
            "sinfonia": "https://media.discordapp.net/attachments/876543237270163498/1123649957791010939/logo_sinfonia_2.png",
            "cereal_pose": "https://cdn.discordapp.com/attachments/754111872399048707/1126798434070970408/WhatsApp_Image_2023-07-07_at_10.52.20_AM.jpg",
        }
        if self.pytoolkit:
            try:
                url = image_path
                if image_path in images:
                    url = images[image_path]
                approved = self.show_image_proxy(url)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def set_awareness(self, state: bool) -> bool:
        """
        Input: True turn on || False turn off
        Output: True if the service was called correctly, False if not
        ----------
        Sets the awareness of the robot
        """
        if self.pytoolkit:
            try:
                approved = self.awareness_proxy(state)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False

    def set_autonomous_life(self, state: bool) -> bool:
        """
        Input: True turn on || False turn off
        Output: True if the service was called correctly, False if not
        ----------  
        Sets the autonomous life of the robot
        """
        if self.pytoolkit:
            try:
                approved = self.autonomous_life_proxy(state)
                if approved == "OK":
                    return True
                else:
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False
        
    def set_security_distance(self, state:bool) -> bool: 
        if self.pytoolkit: 
            try: 
                approved = self.set_security_distance_proxy(state)
                if approved == "OK": 
                    return True
                else: 
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False
        else:
            print("pytoolkit as false")
            return False
    
    def set_move_arms_enabled(self, state:bool)->bool: 
        if self.pytoolkit: 
            try: 
                approved = self.setMoveArms_srv(state,state)
                if approved == "OK": 
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

    def callback_look_for_object(self, data):
        self.object_found = data.data
        return data

    def callback_simple_feedback_subscriber(self, msg):
        self.navigation_status = msg.navigation_status

    def callback_head_sensor_subscriber(self, msg):
        if "head" in msg.name:
            self.isTouched = msg.state
            
    def callback_get_labels_subscriber(self, msg):
        self.labels = {}
        labels_msg = msg.labels
        x_coordinates_msg = msg.x_coordinates
        y_coordinates_msg = msg.y_coordinates
        widths = msg.widths
        heights = msg.heights
        ids = msg.ids
        for label in range(len(labels_msg)):
            if labels_msg[label] not in self.labels:
                self.labels[labels_msg[label]] = [(ids[label], x_coordinates_msg[label], y_coordinates_msg[label], widths[label], heights[label])]
            else:
                self.labels[labels_msg[label]].append((ids[label], x_coordinates_msg[label], y_coordinates_msg[label], widths[label], heights[label]))
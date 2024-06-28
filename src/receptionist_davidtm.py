#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
import ConsoleFormatter
import time
import random
import threading
import rospy
import math
import os
from std_srvs.srv import SetBool

from perception_msgs.msg import get_labels_msg
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srv
from robot_toolkit_msgs.msg import animation_msg


class RECEPTIONIST:
    def __init__(self):
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.task_name = "Receptionist"
        self.labels_info_dict = {}
        self.is_saving_1st_guest = True
        self.is_introducing_first_old_guest = True
        self.get_person_description_thread = None
        self.get_clothes_and_hair_color_thread = None
        self.clothes_color = ""
        self.hair_color = ""
        # TODO update the places before the task
        self.initial_place = "init_receptionist"
        self.greeting_place = "house_door"
        self.first_guest_name = ""
        self.living_room = "living_room"
        # TODO hard code the host's attributes before the task
        self.all_guests_dict = {
            "Charlie": {
                "name": "Charlie",
                "age": self.categorize_age(21),
                "drink": "Milk",
                "gender": "Man",
                "pronoun": "he",
            }
        }
        self.current_guest = {}
        self.not_introduced_guests_list = list(self.all_guests_dict.keys())
        # TODO hard code the chair's angles before the task
        self.CHAIR_ANGLES = [0, -45, -70, 45, 70]
        self.checked_chair_angles = []
        self.not_checked_chair_angles = self.CHAIR_ANGLES.copy()
        self.empty_chairs = self.CHAIR_ANGLES.copy()
        self.chair_angles_index = 0
        self.current_angle = 0
        self.people_from_current_angle = []
        
        self.tm = tm(
            perception=True,
            speech=True,
            manipulation=True,
            navigation=True,
            pytoolkit=True,
        )
        self.tm.initialize_node(self.task_name)
        states = [
            "INIT",
            "WAITING4GUEST",
            "QA",
            "SAVE_FACE",
            "INTRODUCE_NEW",
            "INTRODUCE_OLD",
            "GO2LIVING",
            "GO2DOOR",
            "LOOK4PERSON",
            "LOOK4CHAIR",
            "ACCOMODATE_GUEST",
        ]
        transitions = [
            {"trigger": "start", "source": "RECEPTIONIST", "dest": "INIT"},
            {"trigger": "beggining", "source": "INIT", "dest": "WAITING4GUEST"},
            {"trigger": "person_arrived", "source": "WAITING4GUEST", "dest": "QA"},
            {"trigger": "met_first_guest", "source": "QA", "dest": "SAVE_FACE"},
            {"trigger": "met_second_guest", "source": "QA", "dest": "GO2LIVING"},
            {
                "trigger": "save_face_succeded",
                "source": "SAVE_FACE",
                "dest": "GO2LIVING",
            },
            {
                "trigger": "arrived_to_place",
                "source": "GO2LIVING",
                "dest": "INTRODUCE_NEW",
            },
            {
                "trigger": "introduced_new_guest",
                "source": "INTRODUCE_NEW",
                "dest": "LOOK4PERSON",
            },
            {
                "trigger": "person_not_found",
                "source": "LOOK4PERSON",
                "dest": "LOOK4PERSON",
            },
            {
                "trigger": "person_found",
                "source": "LOOK4PERSON",
                "dest": "INTRODUCE_OLD",
            },
            {
                "trigger": "introduced_old_guest",
                "source": "INTRODUCE_OLD",
                "dest": "LOOK4PERSON",
            },
            {
                "trigger": "introduced_everyone",
                "source": "LOOK4PERSON",
                "dest": "LOOK4CHAIR",
            },
            {
                "trigger": "chair_found",
                "source": "LOOK4CHAIR",
                "dest": "ACCOMODATE_GUEST",
            },
            {
                "trigger": "person_accomodated",
                "source": "ACCOMODATE_GUEST",
                "dest": "GO2DOOR",
            },
            {"trigger": "wait_for_new_guest", "source": "GO2DOOR", "dest": "WAITING4GUEST"},
        ]
        self.machine = Machine(
            model=self, states=states, transitions=transitions, initial="RECEPTIONIST"
        )

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        print(
            self.consoleFormatter.format(
                "Waiting for pytoolkit/awareness...", "WARNING"
            )
        )
        rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        self.set_awareness_srv = rospy.ServiceProxy(
            "/pytoolkit/ALBasicAwareness/set_awareness_srv", SetBool
        )

        print(
            self.consoleFormatter.format(
                "Waiting for pytoolkit/ALMotion/move_head...", "WARNING"
            )
        )
        rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        self.move_head_srv = rospy.ServiceProxy(
            "/pytoolkit/ALMotion/move_head_srv", move_head_srv
        )

        print(
            self.consoleFormatter.format(
                "Waiting for /pytoolkit/ALTabletService/show_image_srv...", "WARNING"
            )
        )
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_image_srv")
        self.show_image_srv = rospy.ServiceProxy(
            "/pytoolkit/ALTabletService/show_image_srv", tablet_service_srv
        )

        print(
            self.consoleFormatter.format(
                "Waiting for pytoolkit/show_topic...", "WARNING"
            )
        )
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
        self.show_topic_srv = rospy.ServiceProxy(
            "/pytoolkit/ALTabletService/show_topic_srv", tablet_service_srv
        )

        print(
            self.consoleFormatter.format(
                "Waiting for pytoolkit/autononumusLife...", "WARNING"
            )
        )
        rospy.wait_for_service("/pytoolkit/ALAutonomousLife/set_state_srv")
        self.autonomous_life_srv = rospy.ServiceProxy(
            "/pytoolkit/ALAutonomousLife/set_state_srv", SetBool
        )

        # Subscribers
        print(
            self.consoleFormatter.format(
                "Waiting for /perception_utilities/get_labels_publisher", "WARNING"
            )
        )
        self.get_labels_publisher = rospy.Subscriber(
            "/perception_utilities/get_labels_publisher",
            get_labels_msg,
            self.callback_get_labels,
        )

        # Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher(
            "/animations", animation_msg, queue_size=1
        )

    # Callbacks
    def callback_get_labels(self, msg):
        for i in len(msg.labels):
            detections_of_label = self.labels_info_dict.get(msg.labels[i], [])
            detections_of_label.append(
                {
                    "label": msg.labels[i],
                    "x": msg.x[i],
                    "y": msg.y[i],
                    "w": msg.w[i],
                    "h": msg.h[i],
                    "id" : msg.id[i]
                }
            )
            self.labels_info_dict[msg.labels[i]] = detections_of_label

    # Helper methods
    def categorize_age(self, age):
        if age < 18:
            category = "a teenager"
        elif age < 25:
            category = "a young adult"
        elif age < 35:
            category = "an adult"
        elif age < 50:
            category = "a middle aged adult"
        elif age < 65:
            category = "a senior"
        else:
            category = "an elder"
        return category
    
    def check_rospy(self):
        # Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()

    def get_clothes_and_hair_color(self):
        gpt_vision_prompt = "Answer about the person centered in the image: What color is the person weating? What is the hair color of the person? Answer only with the color's names separated by a comma as follows: 'blue, black'"
        self.clothes_color, self.hair_color = self.tm.img_description(
            gpt_vision_prompt
        )["message"].split(", ")
        
    def introduce_old_guest(self, guest_name):
        self.tm.show_words_proxy()

        if guest_name in self.not_introduced_guests_list:
            self.not_introduced_guests_list.remove(guest_name)
            guest_being_introduced = self.all_guests_dict[guest_name]
            
            if guest_name == self.first_guest_name:
                clothes_color_str = f"is wearing {guest_being_introduced['clothes_color']}"
                hair_color_str = f"has {guest_being_introduced['hair_color']} hair"
                self.tm.talk(
                    f"{self.current_guest['name']}, I introduce to you {guest_name}. {guest_being_introduced['pronoun']} is a {guest_being_introduced['gender']}. {guest_being_introduced['pronoun']} is {guest_being_introduced['age']}, and {guest_being_introduced['pronoun']} likes to drink {guest_being_introduced['drink']}. {guest_being_introduced['pronoun']} {clothes_color_str}, and {hair_color_str}.",
                    "English",
                )
            else:
                self.tm.talk(
                    f'{self.current_guest["name"]}, I introduce to you {guest_name}. {guest_being_introduced["pronoun"]} is a {guest_being_introduced["gender"]}. {guest_being_introduced["pronoun"]} is {guest_being_introduced["age"]}, and {guest_being_introduced["pronoun"]} likes to drink {guest_being_introduced["drink"]}.'
                )
        else:
            print(f"Already introduced guest {guest_name}")

    ############## TASK STATES ##############
    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.initialize_pepper()
        self.tm.turn_camera("front_camera", "custom", 1, 20)
        self.tm.turn_camera("depth_camera", "custom", 1, 20)
        self.tm.toggle_filter_by_distance(True, 1.8, ["person"])
        self.tm.set_current_place(self.initial_place)
        self.tm.talk(f"I am going to do the {self.task_name} task", "English", wait=False)
        print(self.consoleFormatter.format(f"Starting {self.task_name} task", "HEADER"))
        self.tm.go_to_place(self.greeting_place)
        self.beggining()

    def on_enter_WAITING4GUEST(self):
        print(self.consoleFormatter.format("WAITING4GUEST", "HEADER"))
        self.show_topic_srv("/perception_utilities/yolo_publisher")
        self.tm.talk("Waiting for guests", "English", wait=False)
        self.tm.look_for_object("person", ignore_already_seen=False)
        self.tm.wait_for_object(-1)
        self.tm.look_for_object("", ignore_already_seen=True)
        self.person_arrived()

    def on_enter_QA(self):
        print(self.consoleFormatter.format("QA", "HEADER"))
        # make eye contact
        self.move_head_srv("up")
        self.tm.talk(
            "Hello, when you are going to talk to me, please wait until my eyes turn the color blue.",
            wait=False,
        )
        # turns off object recognition when it is not needed
        self.tm.start_recognition("")
        # ask for name and drink of every guest
        name = self.tm.q_a("name").title()
        drink = self.tm.q_a("drink").lower()
        self.current_guest = {"name": name, "drink": drink}
        # Only get extra attributes for the first guest
        if self.is_saving_1st_guest:
            # calculate the first guest's attributes in parallel
            self.first_guest_name = name
            self.get_clothes_and_hair_color_thread = threading.Thread(
                target=self.get_clothes_and_hair_color
            )
            self.get_clothes_and_hair_color_thread.start()
            self.met_first_guest()
        else:
            self.met_second_guest()

    def on_enter_SAVE_FACE(self):
        print(self.consoleFormatter.format("SAVE_FACE", "HEADER"))
        # prepare the face filter for the guest to position his/her face
        self.tm.publish_filtered_image("face", "front_camera")
        self.show_topic_srv("/perception_utilities/filtered_image")
        self.tm.talk(
            f"Hey {self.current_guest['name']}, I will take some pictures of your face to recognize you in future occasions, please place your face inside the green circle."
            "English",
            wait=True
        )
        success = False
        save_face_attempts = 0
        # try to save the face 3 times
        while (not success) and (save_face_attempts < 3):
            success = self.tm.save_face(self.current_guest["name"])
            save_face_attempts += 1
            if not success:
                self.tm.talk(
                f"I am sorry {self.current_guest['name']}, I was not able to save your face, can you please fit your face in the green circle on my tablet?",
                "English",
                wait=True
            )
        
        # calculate the first guest's attributes in parallel
        self.get_person_description_thread = threading.Thread(
            target=self.tm.get_person_description
        )
        self.get_person_description_thread.start()
        
        self.tm.talk("Thank you, I already took your pictures.", wait=False)
        print(f"Saved face? {'yes' if success else 'no'}.")
        # continue with the task, even if the face was not saved
        self.tm.show_words_proxy()
        self.save_face_succeded()
    
    def on_enter_GO2LIVING(self):
        print(self.consoleFormatter.format("GO2LIVING", "HEADER"))
        self.tm.talk(
            f"Please {self.current_guest["name"]}, follow me to the living room.",
            "English",
            wait=False,
        )
        self.tm.go_to_place(self.living_room)
        self.arrived_to_place()

    def on_enter_INTRODUCE_NEW(self):
        print(self.consoleFormatter.format("INTRODUCE_NEW", "HEADER"))
        self.move_head_srv("default")
        
        if self.is_saving_1st_guest:
            # collect the first guest's attributes
            self.get_clothes_and_hair_color_thread.join()
            self.get_person_description_thread.join()
            guest_attributes = self.tm.person_attributes
            self.current_guest["age"] = self.categorize_age(guest_attributes["age"])
            self.current_guest["gender"] = guest_attributes["gender"]
            self.current_guest["pronoun"] = "he" if guest_attributes["gender"] == "Man" else "she"
            self.current_guest["clothes_color"] = self.clothes_color
            self.current_guest["hair_color"] = self.hair_color
            self.is_saving_1st_guest = False
        
        print(f"Guest attributes: {self.current_guest}")
        
        self.move_head_srv("up")
        
        self.tm.talk(
            f"Please {self.current_guest['name']}, stand besides me", "English", wait=False
        )
        self.tm.show_words_proxy()
        
        self.tm.talk(
            f"Hello everyone, this is {self.current_guest['name']}, {self.current_guest['pronoun']} is a {self.current_guest['gender']}.  {self.current_guest['pronoun']} is {self.current_guest['age']}, and {self.current_guest['pronoun']} likes to drink {self.current_guest['drink']}.",
            "English",
            animated=True,
        )
        
        self.not_introduced_guests_list = list(self.all_guests_dict.keys())
        self.all_guests_dict[self.current_guest["name"]] = self.current_guest
        
        # turns on object recognition to search for guests to introduce
        self.tm.start_recognition("front_camera")
        
        # Reset the chair angles related variables to introduce old guests to the new one
        self.checked_chair_angles = []
        self.not_checked_chair_angles = self.CHAIR_ANGLES.copy()
        self.empty_chairs = self.CHAIR_ANGLES.copy()
        self.chair_angles_index = 0
        
        self.show_topic_srv("/perception_utilities/yolo_publisher")
        self.introduced_new_guest()
        
    def on_enter_LOOK4PERSON(self):
        print(self.consoleFormatter.format("LOOK4PERSON", "HEADER"))
        self.move_head_srv("default")
        
        
        if (self.chair_angles_index < len(self.CHAIR_ANGLES)) and (len(self.not_introduced_guests_list) > 0):
            self.current_angle = self.CHAIR_ANGLES[self.chair_angles_index]
            self.chair_angles_index += 1
            self.checked_chair_angles.append(self.current_angle)
            self.not_checked_chair_angles.remove(self.current_angle)
            
            self.tm.set_angles_srv(
                ["HeadYaw", "HeadPitch"],
                [math.radians(self.current_angle), -0.3],
                0.1,
            )
            t0 = time.time()
            # reset detected labels
            self.labels_info_dict = {}
            # 1 second to recognize a person in the current chair angle
            while (time.time() - t0) < 1:
                if "person" in self.labels_info_dict:
                    self.empty_chairs.remove(self.current_angle)
                    self.people_from_current_angle = self.labels_info_dict["person"]
                    self.person_found()
                    break
            self.person_not_found()
        elif len(self.not_introduced_guests_list) == 0:
            print(self.consoleFormatter.format("INTRODUCED_EVERYONE", "OKGREEN"))
            self.introduced_everyone()
        else:
            print(self.consoleFormatter.format("NO_CHAIRS_LEFT", "FAIL"))
            for guest_name in self.not_introduced_guests_list:
                self.tm.talk(
                    f"{guest_name}, I am sorry, I was not able to recognize you, please introduce yourself to {self.current_guest['name']}."
                    "English",
                    wait=True,
                )
                self.introduced_everyone()

    def on_enter_INTRODUCE_OLD(self):
        print(self.consoleFormatter.format("INTRODUCE_OLD", "HEADER"))
        # If the robot hasn't introduced any guest to the new guest, it will identify the first guest to introduce
        for person_info in self.people_from_current_angle:
            guest_name = ""
            self.tm.center_head_with_label(person_info)
            if self.is_introducing_first_old_guest:
                print("Finding the first old guest to introduce...")
                # Identify the first old guest the robot saw
                recognize_attempts = 0
                while (recognize_attempts < 3) and guest_name == "":
                    guest_name = self.tm.recognize_face(3)
                    print(f"Saw guest: {guest_name}")
                    recognize_attempts += 1
                if guest_name != "":
                    self.is_introducing_first_old_guest = False
                    self.introduce_old_guest(guest_name)
                else:
                    print(self.consoleFormatter.format("Guest not recognized", "FAIL"))
            else:
                # If the robot introduced one guest, the other guest not introduced is the only guest left
                print("Introducing the other guest...")
                guest_name = self.not_introduced_guests_list[0]
                self.introduce_old_guest(guest_name)

            # Re center head to look for the next guest   
            self.tm.set_angles_srv(
                ["HeadYaw", "HeadPitch"],
                [math.radians(self.current_angle), -0.3],
                0.1,
            )
                
        self.show_topic_srv("/perception_utilities/yolo_publisher")
        self.introduced_old_guest()
        
    def on_enter_LOOK4CHAIR(self):
        print(self.consoleFormatter.format("LOOK4CHAIR", "HEADER"))
        self.move_head_srv("default")
        if len(self.empty_chairs) > 0:
            self.tm.set_angles_srv(
                ["HeadYaw", "HeadPitch"],
                [math.radians(self.empty_chairs[0]), -0.3],
                0.1,
            )
        else:
            self.move_head_srv("default")
        self.chair_found()
    
    def on_enter_ACCOMODATE_GUEST(self):
        print(self.consoleFormatter.format("ACCOMODATE_GUEST", "HEADER"))
        self.tm.go_to_pose("point_there")
        self.tm.talk(
            f"Please, take a seat {self.current_guest["name"]}", "English", wait=True
        )
        self.person_accomodated()
        
    def on_enter_GO2DOOR(self):
        print(self.consoleFormatter.format("GO2DOOR", "HEADER"))
        self.tm.talk("Waiting for other guests to come", "English", wait=False)
        self.tm.go_to_place(self.greeting_place)
        self.wait_for_new_guest()

if __name__ == "__main__":
    sm = RECEPTIONIST()
    sm.run()
    rospy.spin()

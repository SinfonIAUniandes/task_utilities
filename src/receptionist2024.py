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


class RECEPTIONIST(object):
    def __init__(self):
        self.consoleFormatter = ConsoleFormatter.ConsoleFormatter()
        self.task_name = "receptionist"
        states = [
            "INIT",
            "WAIT4GUEST",
            "QA",
            "SAVE_FACE",
            "INTRODUCE_NEW",
            "INTRODUCE_OLD",
            "GO2LIVING",
            "GO2DOOR",
            "LOOK4PERSON",
            "LOOK4CHAIR",
            "SIGNAL_SOMETHING",
        ]
        self.tm = tm(
            perception=True,
            speech=True,
            manipulation=False,
            navigation=True,
            pytoolkit=True,
        )
        self.tm.initialize_node(self.task_name)
        transitions = [
            {"trigger": "start", "source": "RECEPTIONIST", "dest": "INIT"},
            {"trigger": "beggining", "source": "INIT", "dest": "WAIT4GUEST"},
            {"trigger": "person_arrived", "source": "WAIT4GUEST", "dest": "QA"},
            {"trigger": "person_met", "source": "QA", "dest": "SAVE_FACE"},
            {"trigger": "save_face_failed", "source": "SAVE_FACE", "dest": "SAVE_FACE"},
            {
                "trigger": "save_face_succeded",
                "source": "SAVE_FACE",
                "dest": "GO2LIVING",
            },
            {
                "trigger": "arrived_to_point",
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
                "dest": "SIGNAL_SOMETHING",
            },
            {
                "trigger": "person_accomodated",
                "source": "SIGNAL_SOMETHING",
                "dest": "GO2DOOR",
            },
            {"trigger": "wait_new_guest", "source": "GO2DOOR", "dest": "WAIT4GUEST"},
        ]

        # Crear la máquina de estados
        self.machine = Machine(
            model=self, states=states, transitions=transitions, initial="RECEPTIONIST"
        )

        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        ##################### ROS CALLBACK VARIABLES #####################
        self.labels = {}
        ##################### GLOBAL VARIABLES #####################
        self.person_description_thread = None
        self.get_clothes_and_hair_color_thread = None
        self.clothes_color = ""
        self.hair_color = ""
        self.initial_place = "init_receptionist"
        self.greeting_place = "house_door"
        self.guests_place = "living_room"
        self.recognized_guests_counter = 0
        self.all_guests = {
            "Charlie": {
                "name": "Charlie",
                "age": self.categorize_age(21),
                "drink": "Milk",
                "gender": "Man",
                "pronoun": "he",
            }
        }
        self.introduced_guests = []
        self.current_guest = {}
        self.old_person = ""
        self.failed_saving_face = False
        self.angle_index = 0
        self.chair_angles = [0, 20, -30]
        self.checked_chair_angles = []
        self.empty_chair_angles = []
        self.is_first_guest = True
        self.introducing_2nd_guest = False
        self.first_guest = {}
        self.host_name = "Charlie"

        # ROS Services (PyToolkit)
        print(
            self.consoleFormatter.format(
                "Waiting for pytoolkit/awareness...", "WARNING"
            )
        )
        rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        self.awareness_srv = rospy.ServiceProxy(
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

        # ROS subscribers (perception)
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

        # ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher(
            "/animations", animation_msg, queue_size=1
        )

    def callback_get_labels(self, data):
        self.labels = data.labels

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
    
    def get_clothes_and_hair_color(self):
        self.consoleFormatter.format("get_clothes_and_hair_color called", "WARNING")
        gpt_vision_prompt = "Answer about the person centered in the image: What color is the person weating? What is the hair color of the person? Answer only with the color's names separated by a comma as follows: 'blue, black'"
        answer = self.tm.img_description(gpt_vision_prompt)["message"].split(", ")
        self.consoleFormatter.format("get_clothes_color executed", "OKGREEN")
        self.clothes_color = answer[0]
        self.hair_color = answer[1]
        return answer

    ############## TASK STATES ##############

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.initialize_pepper()
        self.tm.turn_camera("front_camera", "custom", 1, 20)
        self.tm.turn_camera("depth_camera", "custom", 1, 20)
        self.tm.toggle_filter_by_distance(True, 2, ["person"])
        self.tm.set_current_place(self.initial_place)
        self.tm.talk("I am going to do the  " + self.task_name + " task", "English")
        print(
            self.consoleFormatter.format(
                "Inicializacion del task: " + self.task_name, "HEADER"
            )
        )
        self.tm.go_to_place(self.greeting_place)
        self.beggining()

    def on_enter_WAIT4GUEST(self):
        print(self.consoleFormatter.format("WAIT4GUEST", "HEADER"))
        self.show_topic_srv("/perception_utilities/yolo_publisher")
        time.sleep(0.2)
        self.tm.talk("Waiting for guests", "English")
        self.tm.look_for_object("person", False)
        self.tm.wait_for_object(-1)
        self.tm.look_for_object("", True)
        self.person_arrived()

    def on_enter_QA(self):
        print(self.consoleFormatter.format("QA", "HEADER"))
        self.move_head_srv("up")
        self.tm.talk(
            "Hello, when you are going to talk to me, please wait until my eyes turn blue.",
            wait=False,
        )
        if self.is_first_guest:
            self.get_clothes_and_hair_color_thread = threading.Thread(target=self.get_clothes_and_hair_color)
        rospy.sleep(4)
        name = self.tm.q_a("name")
        drink = self.tm.q_a("drink")
        self.current_guest = {"name": name, "drink": drink}
        self.tm.start_recognition("")
        self.person_met()

    def on_enter_SAVE_FACE(self):

        print(self.consoleFormatter.format("SAVE_FACE", "HEADER"))
        # self.move_head_srv("up")
        if not self.failed_saving_face:
            self.tm.publish_filtered_image("face", "front_camera")
            self.show_topic_srv("/perception_utilities/filtered_image")
            self.tm.talk(
                "Hey {}, I will take some pictures of your face to recognize you in future occasions, please place your face inside the green circle.".format(
                    self.current_guest["name"]
                ),
                "English",
            )
        success = self.tm.save_face(self.current_guest["name"], 3)
        self.person_description_thread = threading.Thread(
            target=self.tm.get_person_description
        )
        self.person_description_thread.start()
        self.tm.talk("Thank you, I already took your pictures.")

        print("success: ", success)
        if success:
            self.save_face_succeded()
        else:
            self.failed_saving_face = True
            self.tm.talk(
                "I am sorry {}, I was not able to save your face, can you please see my tablet and fit your face".format(
                    self.current_guest["name"]
                ),
                "English",
            )
            self.save_face_failed()

    def on_enter_GO2LIVING(self):
        print(self.consoleFormatter.format("GO2LIVING", "HEADER"))
        self.tm.talk(
            "Please {}, follow me to the living room".format(
                self.current_guest["name"]
            ),
            "English",
            wait=False,
        )
        self.tm.go_to_place(self.guests_place)
        self.arrived_to_point()

    def on_enter_INTRODUCE_NEW(self):
        self.person_description_thread.join()
        attributes = self.tm.person_attributes
        print("attributes: ", attributes)
        self.current_guest["age"] = self.categorize_age(attributes["age"])
        self.current_guest["gender"] = attributes["gender"]
        self.current_guest["pronoun"] = "he" if attributes["gender"] == "Man" else "she"
        self.current_guest["color"] = self.clothes_color
        self.all_guests[self.current_guest["name"]] = self.current_guest
        if self.is_first_guest:
            self.get_clothes_and_hair_color_thread.join()
            print("1st guest is wearing ", self.clothes_color)
            self.is_first_guest = False
            self.first_guest["name"] = self.current_guest["name"]
            self.first_guest["drink"] = self.current_guest["drink"]
            self.first_guest["age"] = self.categorize_age(attributes["age"])
            self.first_guest["gender"] = attributes["gender"]
            self.first_guest["pronoun"] = (
                "he" if attributes["gender"] == "Man" else "she"
            )
            self.first_guest["color"] = self.clothes_color
            self.first_guest["id"] = self.id_counter

        if len(self.all_guests) == 3:
            self.introducing_2nd_guest = True
        self.move_head_srv("default")
        self.failed_saving_face = False
        self.tm.show_words_proxy()
        self.saving_face = False

        self.move_head_srv("up")
        print(self.consoleFormatter.format("INTRODUCE_NEW", "HEADER"))
        self.tm.talk(
            "Please {}, stand besides me".format(self.current_guest["name"]), "English"
        )
        
        self.tm.talk(
            f'Hello everyone, this is {self.current_guest["name"]}, {self.current_guest["pronoun"]} is a {self.current_guest["gender"]}.  {self.current_guest["pronoun"]} is {self.current_guest["age"]}, and {self.current_guest["pronoun"]} likes to drink {self.current_guest["drink"]}.',
            "English",
            animated=True,
        )
        # Turns on recognition and looks for  person
        self.tm.start_recognition("front_camera")
        # Reiniciar las variables de presentacion de personas y sillas
        self.introduced_guests = []
        self.introduced_guests.append(self.current_guest["name"])
        self.checked_chair_angles = []
        self.empty_chair_angles = self.chair_angles.copy()
        self.angle_index = 0
        self.show_topic_srv("/perception_utilities/yolo_publisher")
        self.introduced_new_guest()

    def on_enter_LOOK4PERSON(self):
        print(self.consoleFormatter.format("LOOK4PERSON", "HEADER"))
        # El robot ya fue a todas las posiciones de personas o introdujo a todas las personas
        if len(self.checked_chair_angles) == len(self.chair_angles):
            if len(self.introduced_guests) == len(self.all_guests):
                print(self.consoleFormatter.format("INTRODUCED_EVERYONE", "OKGREEN"))
                self.introduced_everyone()
            else:
                print(self.consoleFormatter.format("FAILED_INTRODUCING", "FAIL"))
                not_introduced = []
                for person in self.all_guests:
                    if person not in self.introduced_guests:
                        not_introduced.append(person)
                not_introduced_persons = ", ".join(not_introduced)
                self.tm.talk(
                    "{} I am sorry I was not able to recognize you, please introduce yourself to {}".format(
                        not_introduced_persons, self.current_guest["name"]
                    ),
                    "English",
                    wait=True,
                )
                self.introduced_everyone()
        # El robot busca una persona
        else:
            self.move_head_srv("default")
            self.tm.set_angles_srv(
                ["HeadYaw", "HeadPitch"],
                [math.radians(self.chair_angles[self.angle_index]), -0.3],
                0.1,
            )
            self.checked_chair_angles.append(self.chair_angles[self.angle_index])
            self.angle_index += 1
            t1 = time.time()
            self.labels = {}
            while time.time() - t1 < 1:
                if "person" in self.labels:
                    self.tm.talk("Recognizing person", "English")
                    self.person_found()
                    break
            self.person_not_found()

    def on_enter_INTRODUCE_OLD(self):
        print(self.consoleFormatter.format("INTRODUCE_OLD", "HEADER"))
        guest_name = ""
        while guest_name == "" and self.recognized_guests_counter < 3:
            guest_name = self.tm.recognize_face(3)
            print("saw: " + guest_name)
            self.recognized_guests_counter += 1
            self.recognized_guests_counter = 0
        if guest_name not in self.introduced_guests and guest_name in self.all_guests:
            self.empty_chair_angles.remove(self.checked_chair_angles[-1])
            guest_being_introduced = self.all_guests[guest_name]
            print("Introducing person: ", guest_being_introduced)
            # TODO manipulacion animations/poses
            self.animations_publisher.publish("animations", "Gestures/TakePlace_2")
            self.introduced_guests.append(guest_name)
            if (guest_name == self.first_guest["name"]) and self.introducing_2nd_guest:
                clothes_color_str = f"is wearing {self.first_guest['color']}"
                hair_color_str = f"has {self.first_guest['color']} hair"
                self.tm.talk(
                    f'{self.current_guest["name"]}, I introduce to you {guest_name} is a {guest_being_introduced["gender"]}. {guest_being_introduced["pronoun"]} is {guest_being_introduced["age"]}, and {guest_being_introduced["pronoun"]} likes to drink {guest_being_introduced["drink"]}. {guest_being_introduced["pronoun"]} {clothes_color_str}, and {hair_color_str}.',
                    "English",
                )
            else:
                self.tm.talk(
                    f'{self.current_guest["name"]}, I introduce to you {guest_name}. {guest_being_introduced["pronoun"]} is a {guest_being_introduced["gender"]}. {guest_being_introduced["pronoun"]} is {guest_being_introduced["age"]}, and {guest_being_introduced["pronoun"]} likes to drink {guest_being_introduced["drink"]}.'
                )

        self.introduced_old_guest()

    def on_enter_LOOK4CHAIR(self):
        print(self.consoleFormatter.format("LOOK4CHAIR", "HEADER"))
        self.tm.show_words_proxy()
        if len(self.empty_chair_angles) != 0:
            chair_angle = random.choice(self.empty_chair_angles)
            print("chair_angle ", chair_angle)
        else:
            chair_angle = 0
        self.tm.set_angles_srv(
            ["HeadYaw", "HeadPitch"], [math.radians(chair_angle), -0.3], 0.1
        )
        self.chair_found()

    def on_enter_SIGNAL_SOMETHING(self):
        print(self.consoleFormatter.format("SIGNAL_SOMETHING", "HEADER"))
        self.animations_publisher.publish("animations", "Gestures/TakePlace_2")
        self.tm.talk(
            "Please, take a seat {}".format(self.current_guest["name"]), "English"
        )
        time.sleep(0.5)
        self.person_accomodated()

    def on_enter_GO2DOOR(self):
        print(self.consoleFormatter.format("GO2DOOR", "HEADER"))
        self.tm.talk("Waiting for other guests to come", "English", wait=False)
        self.tm.go_to_place(self.greeting_place)
        self.wait_new_guest()

    def check_rospy(self):
        # Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()


# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = RECEPTIONIST()
    sm.run()
    rospy.spin()

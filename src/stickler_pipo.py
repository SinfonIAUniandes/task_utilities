#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from std_msgs.msg import Bool
import ConsoleFormatter
import time
import random
import threading
import sys
import rospy
import math
import os
import numpy as np
from std_srvs.srv import SetBool

from navigation_msgs.srv import constant_spin_srv, get_absolute_position_srv, get_absolute_position_srvRequest
from navigation_msgs.msg import simple_feedback_msg
from perception_msgs.msg import get_labels_msg
from perception_msgs.srv import start_recognition_srv, start_recognition_srvRequest
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srv
from robot_toolkit_msgs.msg import animation_msg,touch_msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class STICKLER_RULES(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles 
        self.task_name = "stickler_for_the_rules"
        states = ['INIT', 'SFTR', 'GO_TO_PLACE','CHECK_RULE','SOLVE']
        self.tm = tm(perception = True , speech=True, manipulation=True, navigation=True,pytoolkit=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'SFTR', 'dest': 'INIT'},
            {'trigger': 'beggining', 'source': 'INIT', 'dest': 'GO_TO_PLACE'},
            {'trigger': 'check', 'source': 'GO_TO_PLACE', 'dest': 'CHECK_RULE'},
            {'trigger': 'solution', 'source': 'CHECK_RULE', 'dest': 'SOLVE'},
            {'trigger': 'continue_checking', 'source': 'SOLVE', 'dest': 'CHECK_RULE'},
            {'trigger': 'next_location', 'source': 'CHECK_RULE', 'dest': 'GO_TO_PLACE'},
            {'trigger': 'good_person', 'source': 'CHECK_RULE', 'dest': 'CHECK_RULE'},
        ]
        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='SFTR')

        # Inicializa el thread SFTR
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        # ROS Callback
        print(self.consoleFormatter.format("Waiting for /look_for_object_publisher", "WARNING"))
        self.subscriber_look_for_object = rospy.Subscriber("/perception_utilities/look_for_object_publisher", Bool, self.callback_look_for_object_default)

        #Navigation angle
        print(self.consoleFormatter.format("Waiting for /amcl_pose", "WARNING"))
        self.subscriber_odom = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_spin_until)

        # ROS Services (PyToolkit)
        print(self.consoleFormatter.format("Waiting for pytoolkit/awareness...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        self.awareness_srv = rospy.ServiceProxy("/pytoolkit/ALBasicAwareness/set_awareness_srv",SetBool)

        print(self.consoleFormatter.format("Waiting for pytoolkit/ALMotion/move_head...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        self.move_head_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/move_head_srv",move_head_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
        self.show_topic_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv",tablet_service_srv)

        # ROS Services (Navigation)
        print(self.consoleFormatter.format("Waiting for /navigation_utilities/get_absolute_position_srv...", "WARNING"))
        rospy.wait_for_service('/navigation_utilities/get_absolute_position_srv')
        self.get_position_proxy = rospy.ServiceProxy('/navigation_utilities/get_absolute_position_srv', get_absolute_position_srv)

        # ROS Services (Perception - shoes)
        # rospy.wait_for_service('/perception_utilities_shoes/start_recognition_srv')
        # self.start_recognition_shoes_proxy = rospy.ServiceProxy('/perception_utilities_shoes/start_recognition_srv', start_recognition_srv)

        # ROS Services (Perception - garbage)
        rospy.wait_for_service('/perception_utilities_garbage/start_recognition_srv')
        self.start_recognition_garbage_proxy = rospy.ServiceProxy('/perception_utilities_garbage/start_recognition_srv', start_recognition_srv)

        self.headSensorSubscriber = rospy.Subscriber('/touch', touch_msg, self.callback_head_sensor_subscriber)


        # ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        #Subscriber:
        self.get_labels_publisher = rospy.Subscriber("/perception_utilities/get_labels_publisher", get_labels_msg, self.callback_get_labels_default)
        # self.get_labels_shoes_publisher = rospy.Subscriber("/perception_utilities_shoes/get_labels_publisher", get_labels_msg, self.callback_get_labels_shoes)
        self.get_labels_garbage_publisher = rospy.Subscriber("/perception_utilities_garbage/get_labels_publisher", get_labels_msg, self.callback_get_labels_garbage)

        ##################### ROS CALLBACK VARIABLES #####################
        
        self.angle = 0
        self.recognize_person_counter = 0
        self.angle_stop_looking_person = 359
        self.stop_rotation = False
        self.broken_rule = None
        self.isTouched=False
        self.time_to_rotate = 0

        # Callbacks for the perception
        self.default_labels = []
        self.shoes_labels = []
        self.garbage_labels = []
        
        self.look_for_object_default = False
        self.look_for_object_shoes = False
        self.look_for_object_garbage = False
        
        ##################### GLOBAL VARIABLES #####################

        self.places_available = ["bedroom","kitchen", "living_room", "study"]

        self.places_i = 0

        self.places_instructions = {
            "living_room":{"angle": 360, "width": 0, "is_forbidden": False},
            "kitchen":{"angle": 360, "width": 0, "is_forbidden": False},
            "bedroom":{"angle": 180, "width": 0, "is_forbidden": True},
            "study":{"angle": 180, "width": 0, "is_forbidden": False}
        }

        self.rules = {
            "drink":{"action": " May I have a moment of your attention, please?  In this house we are compulsory hydratation, all guests are required to have a drink in the hand while inside the house. It seems you don't currently have a drink.", "suggestion": "To comply with this policy, please go to the kitchen area and grab yourself a drink. Thank you for your understanding and cooperation.","not_accomplish":"Even after the reminder, I notice that you still don't have a drink. Please go to the kitchen area and obtain a drink. Thank you for your understanding and cooperation."},
            "shoes":{"action": "Excuse me, I would like to bring your attention to one of our house rules. Rule number 1 states that no shoes are allowed inside the house. I noticed that you still have your shoes on.", "suggestion": "To rectify the situation, I kindly request that you take off your shoes and leave them at the entrance or in the exit. Thank you for your understanding and cooperation.","not_accomplish":"Even after being reminded, I notice that you still have your shoes on. To comply with our policy, please remove your shoes at the entrance. Thank you for your understanding and cooperation."},
            "garbage":{"action": "Pardon me, but it appears that you have left some garbage on the floor. Our house rule number 3 prohibits littering.", "suggestion": "Kindly pick up the garbage and dispose of it in the designated bin. We appreciate your cooperation in keeping our space clean. Thank you.","not_accomplish":"I see that you have not picked pud yet the garbage on the floor. To rectify the situation, please pick up the garbage and dispose of it in the designated bin. Thank you for your understanding and cooperation in keeping our space clean."},
            "is_forbidden":{"action": "I'm sorry to interrupt, but it seems you have entered the Forbidden Room, which goes against our house rules. Rule number 2 clearly states that no guests are allowed in that particular room", "suggestion": "To ensure compliance, I kindly ask you to return to the main area where the other party guests are located. Please refrain from entering the Forbidden Room again. Thank you for your understanding.","not_accomplish":" Despite the clarification, you have once again entered the room. To maintain order, I kindly ask you to return to the main area and refrain from entering the Forbidden Room. Thank you for your understanding and cooperation."}
        }

    def check_stop_angle(self):
        place = self.places_available[self.places_i]
        if place in ["bedroom","study"]:
            if ( 250 <self.angle <280):
                return True
            else:
                return False
        else:
            if (  100 <self.angle <110):
                return True
            else:
                return False
            

    def on_enter_INIT(self):
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.tm.talk("I am going to do the "+self.task_name+" task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",2,15) 
        self.tm.turn_camera("bottom_camera","custom",2,15)
        self.beggining()

    # ============================== LOOK4 STATES ==============================
    
    def on_enter_GO_TO_PLACE(self):
        print(self.consoleFormatter.format("GO_TO_PLACE", "HEADER"))
        print("Place: ",self.places_available[self.places_i])
        # --- Show logo
        self.tm.show_image('sinfonia')
        # --- End Show logo
        if self.places_instructions[self.places_available[self.places_i]]["is_forbidden"]:
            self.tm.talk("I am going to the forbidden room","English")
        else:
            self.tm.talk("I am going to the "+self.places_available[self.places_i],"English")
        self.tm.set_model("yolo_gpu")

        self.tm.go_to_place(self.places_available[self.places_i]) 
        self.check()

    def on_enter_CHECK_RULE(self):
        print(self.consoleFormatter.format("CHECK_RULE", "HEADER"))
        # --- Show topic
        self.tm.show_topic('/perception_utilities/yolo_publisher')
        ## --- Show Topic end
        self.tm.talk("Looking for people breaking the rules",wait=False)
        self.tm.set_awareness(False)
        self.move_head_srv("default")
        self.tm.start_recognition("front_camera")
        # self.start_recognition_shoes_proxy("bottom_camera")
        self.start_recognition_garbage_proxy("bottom_camera")
        # Extraer el angulo que se necesita girar
        place = self.places_available[self.places_i]
        rotating_angle = self.places_instructions[place]["angle"]
        angular_speed = 15
        self.tm.constant_spin_srv(-15)
        # Si es la primera vez sacar cuanto tiempo se debe girar
        if self.time_to_rotate is None: #PIPE NO ENTIENDE ESTA
            self.time_to_rotate = rotating_angle/angular_speed
        t1 = time.time()
        # Wait to see person or all the angle
        print("Looking for person breaking any rule")
        self.tm.look_for_object("person",ignore_already_seen=True)

        while not self.check_stop_angle():
        # Check person
            if self.look_for_object_default:
                print("Saw person")
                break
        self.tm.robot_stop_srv()
        # All angled covered
        # snow-julian-juanjo(el marica) TODO: SE ASUME QUE SOLO HAY UNA PERSONA EN CADA ROOM? - DEBERÍA SER >?
        
        if self.check_stop_angle():
            self.time_to_rotate = None
            self.places_i += 1
            self.next_location()
            
        self.time_to_rotate = self.time_to_rotate - (time.time()-t1)

        if(self.places_instructions[self.places_available[self.places_i]]["is_forbidden"] == False):
            t2 = time.time()
            drink_broken = True
            shoes_broken = False
            gargabe_broken = False
            while(time.time()-t2 < 5):
                for label in self.default_labels:
                    if((label["label"] == "bottle") and (640*0.2<label["x"]<640*0.8) or (label["label"] == "cup") and (640*0.2<label["x"]<640*0.8) or (label["label"] == "wine glass") and (640*0.2<label["x"]<640*0.8)):
                        drink_broken = False
                        break
                
                # for label in self.shoes_labels:
                #     if( (label["label"] == "shoes") and (640*0.2<label["x"]<640*0.8) or (label["label"] == "socks") and (640*0.2<label["x"]<640*0.8)):
                #         print("shoes found")
                #         shoes_broken = True
                #         break

                not_garbage_labels = ["footwear", "bag", "chair", "cloth", "dress", "jacket", "jeans", "person", "pitcher base", "shirt"]
                try:
                    for label in self.garbage_labels:
                        if(640*0.2<label["x"]<640*0.8) and (label["label"] not in not_garbage_labels):
                            print("garbage found")
                            gargabe_broken = True
                            break
                except:
                    pass
                
            if not drink_broken and not gargabe_broken:
                shoes_broken = True
        else:
            print("Rule broken: Forbidden room")
            self.broken_rule = "is_forbidden"
            self.solution()

        # No drink found, rule broken
        if drink_broken:
            self.broken_rule = "drink"
            self.solution()

        # garbage on the floor
        elif gargabe_broken:
            self.broken_rule = "garbage"
            self.solution()
        
        # Shoes not found
        elif shoes_broken:
            self.broken_rule = "shoes"
            self.solution()

        # TODO: DEBERIA TENER EN CUENTA EL ID DE LA PERSONA PARA NO VOLVER A VERLA
        
        # Buena people
        self.tm.set_awareness(False)
        self.tm.talk("Guest, thank you for following the rules of the house")
        self.tm.spin_srv(-40)
        self.good_person()
                
    def on_enter_SOLVE(self):
        # Check drink 
        print(self.consoleFormatter.format("SOLVE", "HEADER"))
        self.tm.set_awareness(True)
        # self.move_head_srv("up")
        self.tm.talk(self.rules[self.broken_rule]["action"])
        self.tm.talk(self.rules[self.broken_rule]["suggestion"])
        if self.broken_rule == "is_forbidden":
            rospy.sleep(6)
            self.tm.look_for_object("person",ignore_already_seen=False)
            start_time = time.time()
            end_time = start_time + 2
            esUnaMamaHuevo = False
            while time.time() < end_time:
                if self.look_for_object_default:
                    esUnaMamaHuevo = True
                    break
            self.tm.talk("I am checking if you really left the room", wait = False)
            self.tm.execute_trayectory("spin_head")
            if not esUnaMamaHuevo:
                self.tm.talk("Now there is not anyone in the forbidden room")
            else:
                self.tm.talk(self.rules[self.broken_rule]["not_accomplish"])

        else:        
            self.tm.talk("When you comeback please touch my head and I will confirm that you are following the rule")
            t1 = time.time()
            while not self.isTouched and time.time()-t1 < 15:
                rospy.sleep(0.1)
            if time.time()-t1 >= 15:
                self.tm.talk("As you did not touch my head I suppose you are not following the rule")
                self.broken_rule = None    
                self.tm.spin_srv(-40)
                self.time_to_rotate-=2
                self.continue_checking()

            if self.broken_rule == "drink":
                self.tm.talk("I am going to check that you are not breaking the rule anymore. Look at my tablet and show me your drink.")
            else:
                self.tm.talk("I am going to check that you are not breaking the rule anymore")
            rospy.sleep(3)
            # TODO: Check drink again
            # Tiempo que nos damos para verificar
            isAccomplish = False
            t0 = time.time()
            while(time.time()-t0 < 5):
                if self.broken_rule == "drink":
                    for label in self.default_labels:
                        if((label["label"] in ["bottle","cup"]) and (640*0.2<label["x"]<640*0.8)):
                            print("drink found")
                            isAccomplish = True
                            break
                elif self.broken_rule == "garbage":
                    isAccomplish = True
                    for label in self.garbage_labels:
                        if(640*0.2<label["x"]<640*0.8) and (label["label"] != "person"):
                            print("garbage found")
                            isAccomplish = False
                            break
                # elif self.broken_rule == "shoes":
                #     isAccomplish = True
                #     for label in self.shoes_labels:
                #         if((label["label"] == "shoes") and (640*0.2<label["x"]<640*0.8)):
                #             print("shoes found")
                #             isAccomplish = False
                #             break
                else:
                    isAccomplish = True
                    break
            if isAccomplish:
                self.tm.talk("Thank you for following the rules")
            else:
                self.tm.talk(self.rules[self.broken_rule]["not_accomplish"])
        self.broken_rule = None    
        self.tm.spin_srv(-40)
        self.time_to_rotate-=2
        self.continue_checking()

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            time.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        print("run")
        self.start()

    def callback_spin_until(self,data):
        self.angle = int(np.degrees(euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2] % (2 * math.pi)))
        return self.angle

    def callback_get_labels_default(self,data):
        labels = data.labels
        ids = data.ids
        widths = data.widths
        heights = data.heights
        x=data.x_coordinates
        y=data.y_coordinates
        self.default_labels =[]
        for i in range(0, len(data.labels)):
            label = {"label": labels[i], "id": ids[i], "width": widths[i], "height": heights[i], "x": x[i], "y": y[i]}
            self.default_labels.append(label)

    #TODO incluir la subsccripcion al topico como tal
    def callback_get_labels_shoes(self,data):
        labels = data.labels
        ids = data.ids
        widths = data.widths
        heights = data.heights
        x=data.x_coordinates
        y=data.y_coordinates
        self.shoes_labels =[]
        for i in range(0, len(heights)):
            label = {"label": labels[i], "id": ids[i], "width": widths[i], "height": heights[i], "x": x[i], "y": y[i]}
            self.shoes_labels.append(label)

    #TODO incluir la subiscrcion al topico como tal
    def callback_get_labels_garbage(self,data):
        labels = data.labels
        ids = data.ids
        widths = data.widths
        heights = data.heights
        x=data.x_coordinates
        y=data.y_coordinates
        self.garbage_labels =[]
        for i in range(0, len(heights)):
            label = {"label": labels[i], "id": ids[i], "width": widths[i], "height": heights[i], "x": x[i], "y": y[i]}
            self.garbage_labels.append(label)

    def callback_look_for_object_default(self,data):
        self.look_for_object_default = data.data

    def callback_head_sensor_subscriber(self, msg):
        if msg.name in["head_rear","head_middle","head_front"]:
            self.isTouched=msg.state

# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = STICKLER_RULES()
    sm.run()
    rospy.spin()

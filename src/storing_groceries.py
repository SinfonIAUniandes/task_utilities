#!/usr/bin/env python3
from transitions import Machine
from task_module import Task_module as tm
from perception_msgs.msg import get_labels_msg
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from robot_toolkit_msgs.msg import touch_msg, animation_msg
from robot_toolkit_msgs.srv import point_at_srv, get_segmentation3D_srv, point_at_srvRequest
from robot_toolkit_msgs.srv import tablet_service_srv, move_head_srvss
import ConsoleFormatter
import rospy
import os
import time
import threading
import numpy as np
import random
import sys

class STORING_GROCERIES(object):
    def __init__(self):

        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        # Definir los estados posibles del semáforo
        self.task_name = "storing_groceries"
        states = ['INIT', 'GO2TABLE', 'LOOK4OBJECT', 'REQHELPGRAB', 'GO2CABINET', 'RECOGCABINETCATEGORIES', 'REQHELPSTORE', 'END']
        self.tm = tm(perception=True, speech=True, manipulation=False, navigation=True)
        self.tm.initialize_node(self.task_name)
        # Definir las transiciones permitidas entre los estados
        transitions = [
            {'trigger': 'start', 'source': 'STORING_GROCERIES', 'dest': 'INIT'},
            {'trigger': 'init_go2table', 'source': 'INIT', 'dest': 'GO2TABLE'},
            {'trigger': 'arrived_table', 'source': 'GO2TABLE', 'dest': 'LOOK4OBJECT'},
            {'trigger': 'object_found_categorized', 'source': 'LOOK4OBJECT', 'dest': 'REQHELPGRAB'},
            {'trigger': 'object_grabbed', 'source': 'REQHELPGRAB', 'dest': 'GO2CABINET'},
            {'trigger': 'cabinet_categories_recognized', 'source': 'RECOGCABINETCATEGORIES', 'dest': 'REQHELPSTORE'},
            {'trigger': 'arrived_cabinet', 'source': 'GO2CABINET', 'dest': 'REQHELPSTORE'},
            {'trigger': 'object_stored', 'source': 'REQHELPSTORE', 'dest': 'GO2TABLE'},
        ]



        # Crear la máquina de estados
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='STORING_GROCERIES')
        # {'trigger': 'arrived_cabinet_first_time', 'source': 'GO2CABINET', 'dest': 'RECOGCABINETCATEGORIES'},
        self.machine.add_transition(trigger='arrived_cabinet_first_time', source='GO2CABINET', dest='RECOGCABINETCATEGORIES', after='arrived_cabinet', conditions=['is_first_time'])

        self.machine.add_transition(trigger='object_stored', source='REQHELPSTORE', dest='END', conditions=['all_objects_stored'])
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()

        # ROS Callbacks

        # ROS Services (PyToolkit)
        print(self.consoleFormatter.format("Waiting for pytoolkit/awareness...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALBasicAwareness/set_awareness_srv")
        self.awareness_srv = rospy.ServiceProxy("/pytoolkit/ALBasicAwareness/set_awareness_srv",SetBool)

        rospy.wait_for_service("/pytoolkit/ALMotion/move_head_srv")
        self.move_head_srv = rospy.ServiceProxy("/pytoolkit/ALMotion/move_head_srv",move_head_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/show_topic...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALTabletService/show_topic_srv")
        self.show_topic_srv = rospy.ServiceProxy("/pytoolkit/ALTabletService/show_topic_srv",tablet_service_srv)

        print(self.consoleFormatter.format("Waiting for pytoolkit/autononumusLife...", "WARNING"))
        rospy.wait_for_service("/pytoolkit/ALAutonomousLife/set_state_srv")
        self.autonomous_life_srv = rospy.ServiceProxy("/pytoolkit/ALAutonomousLife/set_state_srv",SetBool)

        self.headSensorSubscriber = rospy.Subscriber('/touch', touch_msg, self.callback_head_sensor_subscriber)
        
        # ROS subscribers (perception)
        print(self.consoleFormatter.format("Waiting for /perception_utilities/get_labels_publisher", "WARNING"))
        self.get_labels_publisher = rospy.Subscriber("/perception_utilities/get_labels_publisher", get_labels_msg, self.callback_get_labels)

        # ROS Publishers
        print(self.consoleFormatter.format("Waiting for /animations", "WARNING"))
        self.animations_publisher = rospy.Publisher("/animations", animation_msg, queue_size = 1)

        ##################### ROS CALLBACK VARIABLES #####################
        self.labels ={}

        ##################### GLOBAL VARIABLES #####################
        self.sinfonia_url_img="https://cdn.discordapp.com/attachments/876543237270163498/1123367466102427708/logo_sinfonia.png"
        self.img_dimensions = (320,240)

        self.selected_object = ""
        self.actual_obj_cat = "Uncategorized"
        self.num_sections = 5
        self.isTouched = False
        self.sensorFront = False
        self.sensorMiddle = False
        self.sensorRear = False

        self.cabinet_sections = {'Packaged Dry Goods':-1, 'Canned Goods':-1, 'Fresh Fruits':-1,'Dairy':-1,'Uncategorized':-1}
        self.objects_stored = {k:[] for k in range(self.num_sections)}

    def callback_get_labels(self,data):
        labels = data.labels
        x_coordinates = data.x_coordinates
        y_coordinates = data.y_coordinates
        widths = data.widths
        heights = data.heights
        ids = data.ids
        for i in range(len(labels)):
            if self.img_dimensions[0]//3<x_coordinates[i]<int(self.img_dimensions[0]*2/3):
                self.labels[labels[i]] = {"x":x_coordinates[i],"y":y_coordinates[i],"w":widths[i],"h":heights[i],"id":ids[i]}

    def callback_head_sensor_subscriber(self, msg:touch_msg):
        if msg.name == "head_rear":
            self.sensorRear = msg.state
        elif msg.state == "head_middle":
            self.sensorMiddle = msg.state
        elif msg.state == "head_front":
            self.sensorFront = msg.state
        if self.sensorFront or self.sensorMiddle or self.sensorRear:
            self.isTouched=True
        else:
            self.isTouched=False


    def categorize_object(self, object_name):
        packaged_dry_goods = ['cracker box', 'sugar box', 'pudding box', 'gelatin box', 'cereal box']
        #box and cylinder for cereal box


        canned_goods = ['meat can', 'coffee can', 'fish can', 'chips can']
        fresh_fruits = ['banana', 'strawberry', 'apple', 'lemon', 'peach', 'pear', 'orange', 'plum']
        dairy = ['milk']
        
        if object_name in packaged_dry_goods:
            return "Packaged Dry Goods"
        elif object_name in canned_goods:
            return "Canned Goods"
        elif object_name in fresh_fruits:
            return "Fresh Fruits"
        elif object_name in dairy:
            return "Dairy"
        else:
            return "Uncategorized"

    def categorize_sections(self):
        print(self.consoleFormatter.format("Categorizing sections...", "WARNING"))
        range_section = self.img_dimensions[0]//self.num_sections
        y_ranges = [(range_section*i, range_section*(i+1)) for i in range(self.num_sections)]
        for label in self.labels:
            category = self.categorize_object(label)
            if category != "Uncategorized":
                y = self.labels[label]["y"]
                for i in range(len(y_ranges)):
                    if y_ranges[i][0] <= y <= y_ranges[i][1]:
                        if self.cabinet_sections[category] == -1:
                            self.cabinet_sections[category] = i
                        break
        # assign empty sections to the first empty section
        empty_sections = [i for i in self.cabinet_sections if self.cabinet_sections[i] == -1]
        for category in self.cabinet_sections:
            if self.cabinet_sections[category] == -1:
                self.cabinet_sections[category] = empty_sections[0]
                empty_sections.pop(0)
                break
        print(self.consoleFormatter.format("Sections categorized", "OKGREEN"))
        print(self.consoleFormatter.format("Sections: "+str(self.cabinet_sections), "OKGREEN"))
        self.cabinet_sections_recognized()


    def all_objects_stored(self):
        if len(self.objects_stored) >= 10:
            return True
        return False

    def is_first_time(self):
        if len (self.objects_stored) == 0:
            return True
        return False

    def on_enter_INIT(self):
        self.autonomous_life_srv(False)
        self.tm.talk("I am going to do the storing groceries task","English")
        print(self.consoleFormatter.format("Inicializacion del task: "+self.task_name, "HEADER"))
        self.tm.turn_camera("front_camera","custom",1,15) 
        self.awareness_srv(False)
        # TODO: Add the perception: set_model_recognition srv to the tm module
        self.init_go2table()

    def on_enter_GO2TABLE(self):
        print(self.consoleFormatter.format("GO2TABLE", "HEADER"))
        self.tm.talk("I am going to the table position","English",wait=False)
        self.tm.go_to_place("table")
        self.tm.talk("I'm in the table position","English",wait=False)
        self.arrived_table()

    def on_enter_LOOK4OBJECT(self):
        print(self.consoleFormatter.format("LOOK4OBJECT", "HEADER"))
        self.tm.talk("I am looking for an object","English",wait=False)
        posible_objects = ['cracker box', 'sugar box', 'pudding box', 'gelatin box', 'meat can', 'coffe can', 'fish can', 'chips can', 'banana', 'strawberry', 'apple', 'lemon', 'peach', 'pear', 'orange', 'plum', 'milk', 'cereal box']
        self.selected_object = random.choice([obj for obj in self.labels["labels"] if obj in posible_objects])
        self.actual_obj_cat = self.categorize_object(self.selected_object)
        self.tm.talk("I found a "+self.selected_object,"English",wait=False)
        self.object_found_categorized()
    
    def on_enter_REQHELPGRAB(self):
        print(self.consoleFormatter.format("REQHELPGRAB", "HEADER"))
        if self.selected_object == "cereal box":
            self.tm.go_to_pose('box', 0.2)
            rospy.sleep(2)
            self.tm.talk("Could you place the "+self.selected_object+" between my hands, please?, when you are ready touch my head","English",wait=False)
            while not self.isTouched:
                rospy.sleep(0.1)
            self.tm.go_to_pose('cylinder', 0.1)
            rospy.sleep(2)
            self.tm.go_to_pose('close_both_hands', 0.2)
            rospy.sleep(1)
        elif self.selected_object == "fruit":
            self.tm.go_to_pose('small_object_right_hand', 0.2)
            self.tm.go_to_pose('open_right_hand', 0.2)
            rospy.sleep(2)
            self.tm.talk("Could you place the "+self.selected_object+" in my right hand, please?, when you are ready touch my head","English",wait=False)
            while not self.isTouched:
                rospy.sleep(0.1)
            self.tm.go_to_pose('close_right_hand', 0.2)
            rospy.sleep(1)
        else:
            self.tm.go_to_pose('bottle', 0.2)
            rospy.sleep(2)
            self.tm.talk("Could you place the "+self.selected_object+" between my hands, please?, when you are ready touch my head","English",wait=False)
            while not self.isTouched:
                rospy.sleep(0.1)
            self.tm.go_to_pose('take_bottle', 0.2)
            rospy.sleep(1)

        self.object_grabbed()

    def on_enter_GO2CABINET(self):
        print(self.consoleFormatter.format("GO2CABINET", "HEADER"))
        self.tm.talk("I am going to the cabinet position","English",wait=False)
        self.tm.go_to_place("cabinet")
        self.tm.talk("I'm in the cabinet position","English",wait=False)
        self.arrived_cabinet()

    def on_enter_REQHELPSTORE(self):
        print(self.consoleFormatter.format("REQHELPSTORE", "HEADER"))
        destine_section = self.cabinet_sections[self.actual_obj_cat]
        postion_section = ''
        if destine_section == 0:
            postion_section = 'first'
        elif destine_section == 1:
            postion_section = 'second'
        elif destine_section == 2:
            postion_section = 'third'
        elif destine_section == 3:
            postion_section = 'fourth'
        elif destine_section == 4:
            postion_section = 'fifth'

        if self.selected_object == "cereal box":
            self.tm.talk("Could you take the "+self.selected_object+" from my hands?", "English",wait=False)
            self.tm.talk("When you are ready to grab the "+ self.selected_object + " touch my head","English",wait=False)
            while not self.isTouched:
                rospy.sleep(0.1)
            self.tm.go_to_pose('box', 0.05)
            rospy.sleep(2)
            self.tm.go_to_pose('standard', 0.2)
            rospy.sleep(1)
        elif self.selected_object == "fruit":
            self.tm.talk("Could you take the "+self.selected_object+" from my right hand?", "English",wait=False)
            self.tm.talk("When you are ready to grab the "+ self.selected_object + " touch my head","English",wait=False)
            while not self.isTouched:
                rospy.sleep(0.1)
            self.tm.go_to_pose('open_right_hand', 0.2)
            self.tm.go_to_pose('standard', 0.2)
            rospy.sleep(1)
        else:
            self.tm.talk("Could you take the "+self.selected_object+" from my hands?", "English",wait=False)
            self.tm.talk("When you are ready to grab the "+ self.selected_object + " touch my head","English",wait=False)
            while not self.isTouched:
                rospy.sleep(0.1)
            self.tm.go_to_pose('open_both_hands', 0.2)
            self.tm.go_to_pose('standard', 0.2)

        self.tm.talk("Can you please put the"+ self.selected_object+" inside the"+ postion_section + " from the top to the bottom", "English",wait=False)
        rospy.sleep(3)
        self.tm.talk("Thank you for your help","English",wait=False)
        self.object_stored()

    def on_enter_RECOGCABINETCATEGORIES(self):
        print(self.consoleFormatter.format("RECOGCABINETCATEGORIES", "HEADER"))
        self.tm.talk("Recognizing cabinet categories","English",wait=False)


    def on_enter_END(self):
        print(self.consoleFormatter.format("END", "HEADER"))
        sys.exit(0)

    def check_rospy(self):
        #Termina todos los procesos al cerrar el nodo
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(self.consoleFormatter.format("Shutting down", "FAIL"))
        os._exit(os.EX_OK)

    def run(self):
        while not rospy.is_shutdown():
            self.start()

# Crear una instancia de la maquina de estados
if __name__ == "__main__":
    sm = STORING_GROCERIES()
    sm.run()
    rospy.spin()

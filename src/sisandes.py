#!/usr/bin/env python3
import task_module as tm
import rospy
from perception_msgs.srv import start_recognition_srv, start_recognition_srvRequest, look_for_object_srv, look_for_object_srvRequest, save_face_srv,save_face_srvRequest, recognize_face_srv, recognize_face_srvRequest, save_image_srv,save_image_srvRequest, set_model_recognition_srv,set_model_recognition_srvRequest
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Int32, String


tm = tm.Task_module()
#Esperar 2 segunods

tm.initialize_node()

print("Voy a hacer el task de colsubsidio")
while True:
    rospy.sleep(2)
    tm.start_recognition("")    
    rospy.sleep(2)

    tm.start_recognition("front_camera")
    rospy.sleep(2)
    tm.turn_camera("front_camera","enable")

    tm.talk("Esperando persona",2,"Spanish")

    tm.look_for_object("person")
    tm.wait_for_object(10)

    tm.talk("Hola invitado, presentame tu QR",3,"Spanish")

    print("Hola invitado, presentame tu QR")
    texto_qr = tm.qr_read(10)

    tm.talk("Bienvenido al evento "+texto_qr,3,"Spanish")

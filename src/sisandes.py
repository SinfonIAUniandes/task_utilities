#!/usr/bin/env python3
import task_module as tm
import rospy
import time
from perception_msgs.srv import start_recognition_srv, start_recognition_srvRequest, look_for_object_srv, look_for_object_srvRequest, save_face_srv,save_face_srvRequest, recognize_face_srv, recognize_face_srvRequest, save_image_srv,save_image_srvRequest, set_model_recognition_srv,set_model_recognition_srvRequest
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Int32, String


tm = tm.Task_module(perception = True,speech=True,manipulation=False, navigation=False)
#Esperar 2 segunods
time.sleep(2)
tm.initialize_node()
rospy.sleep(1)
tm.turn_camera("front_camera","custom",1,15)   
rospy.sleep(1)
tm.start_recognition("front_camera")
rospy.sleep(1)
tm.look_for_object("person")
tm.talk("Hola, soy el robot de recepcion, voy a iniciar el reconocimiento de rostros","Spanish")
while True:
    tm.talk("Esperando persona",2,"Spanish")
    print("Esperando persona")
    #Esperar a una persona de manera indefinida
    tm.wait_for_object(-1)
    tm.talk("Hola invitado, presentame tu QR","Spanish")
    print("Hola invitado, presentame tu QR")
    rospy.sleep(1)
    texto_qr = tm.qr_read(10)
    tm.talk("Bienvenido al evento "+texto_qr,"Spanish")
    print("Bienvenido al evento "+texto_qr)
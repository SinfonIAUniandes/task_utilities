#!/usr/bin/env python3
import os  # Módulo para interactuar con el sistema operativo
# import rospy  # Módulo para manejar el tiempo
import rospy  # Módulo para manejar nodos ROS
import threading  # Módulo para manejar hilos
import ConsoleFormatter  # Módulo para formatear la consola

from transitions import Machine  # Módulo para manejar máquinas de estados
from task_module import Task_module as TaskModule  # Importa y renombra el módulo de tareas
from robot_toolkit_msgs.srv import set_angle__srv, set_angle_srvRequest

class ServeBreakfast(object):
    def __init__(self) -> None:
        # Inicializa el formateador de consola para mensajes
        self.console_formatter = ConsoleFormatter.ConsoleFormatter()
        
        # Inicializa el módulo de tareas con las herramientas necesarias
        self.task_module = TaskModule(navigation=True, manipulation=True, speech=True, perception=True, pytoolkit=True)
        self.task_module.initialize_node('SERVE_BREAKFAST')
        
        # Define los estados de la máquina de estados
        self.states = ['INIT', 'GO_TO_CUPBOARD', 'SCAN_FOR_OBJECTS', 'GRAB_OBJECT', 'GO_TO_DROP_PLACE', 'DROP', 'MAKE_BREAKFAST', 'END']
        
        # Define las transiciones entre los estados
        self.transitions = [
            {'trigger': 'zero', 'source': 'SERVE_BREAKFAST', 'dest': 'INIT'},  # Transición inicial
            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_TO_CUPBOARD'},  # De INIT a GO_TO_CUPBOARD
            {'trigger': 'scan_for_objects', 'source': 'GO_TO_CUPBOARD', 'dest': 'SCAN_FOR_OBJECTS'},
            {'trigger': 'grab_ingredient', 'source': 'SCAN_FOR_OBJECTS', 'dest': 'GRAB_OBJECT'},  # De GO_TO_CUPBOARD a GRAB_OBJECT
            {'trigger': 'go_to_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_TO_DROP_PLACE'},  # De GRAB_OBJECT a GO_TO_DROP_PLACE
            {'trigger': 'drop_object', 'source': 'GO_TO_DROP_PLACE', 'dest': 'DROP'},  # De GO_TO_DROP_PLACE a DROP
            {'trigger': 'serve', 'source': 'DROP', 'dest': 'MAKE_BREAKFAST'},  # De GO_TO_DROP_PLACE a DROP_
            {'trigger': 'again', 'source': 'DROP', 'dest': 'GO_TO_CUPBOARD'},  # Repetir el ciclo
            {'trigger': 'again2', 'source': 'MAKE_BREAKFAST', 'dest': 'GO_TO_CUPBOARD'}, 
            {'trigger': 'end', 'source': 'DROP', 'dest': 'END'}  # Finalizar el ciclo
        ]
        
        # Inicializa la máquina de estados con los estados y transiciones definidos
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial='SERVE_BREAKFAST')
        
        # Inicia un hilo para comprobar el estado de rospy
        rospy_check = threading.Thread(target=self.check_rospy)
        rospy_check.start()
        
        # Parámetros generales de movimiento y objetos
        self.fast_movement = 0.15  # Velocidad de movimiento rápido
        self.normal_movement = 0.1  # Velocidad de movimiento normal
        self.slow_movement = 0.05  # Velocidad de movimiento lento
        
        # Lista de objetos a recoger
        self.items = ["bowl", "cereal_box", "milk_carton"]
        self.items_collected = []  # Lista de objetos recogidos

        # Lista de objetos a escanear
        self.scan_items = ["bowl", "cereal_box", "milk_carton"]

        # CAMBIAR en ROBOCUP: Angulos en grados donde Nova escanea y encuentra los items
        self.bowl_angle = 50
        self.cereal_angle = 90
        self.milk_angle = 130
        
        # CAMBIAR en ROBOCUP: Grados de inclinación de NOVA para recoger y dejar los objetos sobre la mesa
        self.bowl_angle = 50
        
        # Instrucciones para agarrar los objetos
        self.grab_items_poses = {
            "bowl": ["mid_arms_bowl", "bowl_hands", "close_arms_bowl", "raise_arms_bowl"],
            "cereal_box": ["open_both_hands", "both_arms_cereal", "close_arms_cereal","bowl_hands" , "raise_arms_cereal"],
            "milk_carton": ["open_both_hands", "both_arms_milk", "close_arms_milk", "bowl_hands" ,"raise_arms_milk"]
        }
        
        # Posiciones finales relativas para dejar los objetos
        # TODO Ajustar posiciones relativas
        self.drop_and_serve_position = {
            "bowl": -0.8,  # Posición para dejar el bowl
            "milk_carton": 0.6,  # Posición para dejar el cartón de leche
            "cereal_box": -1.0  # Posición para dejar la caja de cereal
        }
        
        # Instrucciones para dejar los objetos
        self.drop_and_serve_items_poses = {
            "bowl": ["close_arms_bowl","open_both_hands", "both_arms_bowl"],
            "milk_carton": ["close_arms_milk", "both_arms_milk", "close_arms_milk"],
            "cereal_box": ["close_arms_cereal", "both_arms_cereal", "close_arms_cereal"]
        }
        
        self.serve_items_poses = {
            "milk_carton": [], # TODO Terminar poses para servir y soltar (en ese orden)
            "cereal_box": [] # TODO Terminar poses para servir y soltar (en ese orden)
        }
        
        self.item_counter=0
        
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        
        self.relative_drop_position=0.3
        
        
        
    # --------------------------------------------------------------------------
    #                           SERVICIOS DE APOYO
    # --------------------------------------------------------------------------
    
    self.setAngles_srv = rospy.ServiceProxy('pytoolkit/ALMotion/set_angles_srv', set_angles_enabled_srv)
    set_angles_request = set_angles_enabled_srvRequest()
    
    def tilt_hip(self, angle):
        
        # Max angle: 1.03 rads and Min angle: -0.41 rads
        
        self.setAngles_srv.data.name = "HipPitch"
        self.setAngles_srv.data.angle = angle
        self.setAngles_srv.data.speed = 0.1
        self.setAngles_srv.call(self.setAngles_srv.data)

    # ---------------------------------------------------------------------------
    #                       ESTADOS / TRANSICIONES
    # ---------------------------------------------------------------------------

    def on_enter_INIT(self):
        """Acciones al entrar en el estado INIT."""
        self.task_module.go_to_pose("standard", self.slow_movement)
        print(self.consoleFormatter.format("INIT", "HEADER"))
        self.task_module.set_current_place("entrance")  # Establece la posición inicial del robot
        self.task_module.go_to_pose("standard", self.fast_movement)  # Pone al robot en la posición estándar
        self.task_module.initialize_pepper()  # Inicializa los módulos necesarios
        self.task_module.talk("Good morning! Today I would like to help you with the preparation of a delicious breakfast", "English", wait=False)  # Mensaje de saludo
        self.start()  # Transición de INIT a GO_TO_CUPBOARD



    def on_enter_GO_TO_CUPBOARD(self):
        """Acciones al entrar en el estado GO_TO_CUPBOARD."""
        print(self.consoleFormatter.format("GO_TO_CUPBOARD", "HEADER"))
        self.task_module.talk("I will navigate to the kitchen door to look for the ingredients.", "English", wait=False)  # Mensaje informativo
        self.task_module.go_to_place("kitchen", lower_arms=False)  # Dirige al robot a la cocina
        self.scan_for_objects()  # Transición de GO_TO_CUPBOARD a SCAN_FOR_OBJECTS

    
    
    def on_enter_SCAN_FOR_OBJECTS(self):
        """Acciones al entrar a escanear los objetos que va a tomar de la cocina"""
        print(self.consoleFormatter.format("SCAN_FOR_OBJECTS", "HEADER"))

        current_scanning_object = self.scan_items[0]

        self.task_module.talk("I will look for the " + current_scanning_object + " in this moment, please give me a second.", "English", wait=False)

        if current_scanning_object == "bowl":

            self.task_module.go_to_relative_point(0,0,self.bowl_angle)
            response = self.task_module.img_description("Describe the bowl you see in front of you, add some details about the size and color of the bowl. Then describe the place where it is located naming objects around it. Don't add any other words to your response. Give a short answer of maximum 2 lines. If you don't see a bowl, answer just with a "" and don't say sorry or anything more.", "front_camera")

            if response["status"] and response["message"]:
                self.task_module.go_to_pose('point_there_right', 0.1)
                self.task_module.go_to_pose('open_right_hand', 0.1)
                self.task_module.talk("Oh, I see the bowl is right there!", "English", wait=False)
                self.task_module.talk(response["message"], language="English", wait=False)
                self.task_module.talk("May you please follow my instructions to help me grab it? I will tell you what to do", "English", wait=False)

            elif response["message"] == "":
                self.task_module.go_to_relative_point(0,0,self.bowl_angle)
                self.task_module.go_to_pose('point_there_right', 0.1)
                self.task_module.go_to_pose('open_right_hand', 0.1)
                self.task_module.talk("Oh, I see the bowl is right there!", "English", wait=False)
                self.task_module.talk("May you please follow my instructions to help me grab it? I will tell you what to do", "English", wait=False)

            self.scan_items.remove(current_scanning_object)

        elif current_scanning_object == "cereal_box":
            
            self.task_module.go_to_relative_point(0,0,self.cereal_angle)
            response = self.task_module.img_description("Describe the cereal box you see in front of you, add some details about the size and color of the box. Then describe the place where it is located naming objects around it. Don't add any other words to your response. Give a short answer of maximum 2 lines. If you don't see a cereal box, answer just with a "" and don't say sorry or anything more.", "front_camera")

            if response["status"] and response["message"]:
                self.task_module.go_to_pose('point_there_right', 0.1)
                self.task_module.go_to_pose('open_right_hand', 0.1)
                self.task_module.talk("Oh, I see the cereal box is right there!", "English", wait=False)
                self.task_module.talk(response["message"], language="English", wait=False)
                self.task_module.talk("May you please follow my instructions to help me grab it? I will tell you what to do", "English", wait=False)
                
            elif response["message"]:
                self.task_module.go_to_pose('point_there_right', 0.1)
                self.task_module.go_to_pose('open_right_hand', 0.1)
                
                self.task_module.talk("Oh, I see the cereal box is right there!", language="English", wait=False)
                self.task_module.talk("May you please follow my instructions to help me grab it? I will tell you what to do", "English", wait=False)
            
            self.scan_items.remove(current_scanning_object)

                
        elif current_scanning_object == "milk_carton":
            self.task_module.go_to_relative_point(0,0,self.milk_angle)
            response = self.task_module.img_description("Describe the milk carton you see in front of you, add some details about the size and color of the carton. Then describe the place where it is located naming objects around it. Don't add any other words to your response. Give a short answer of maximum 2 lines. If you don't see a milk carton, answer just with a "" and don't say sorry or anything more.", "front_camera")

            if response["status"] and response["message"]:
                self.task_module.go_to_pose('point_there_right', 0.1)
                self.task_module.go_to_pose('open_right_hand', 0.1)
                self.task_module.talk("Oh, I see the milk carton is right there!", "English", wait=False)
                
                self.task_module.talk(response["message"], language="English", wait=False)

                self.task_module.talk("May you please follow my instructions to help me grab it? I will tell you what to do", "English", wait=False)

            elif response["message"] == "":
                self.task_module.go_to_pose('point_there_right', 0.1)
                self.task_module.go_to_pose('open_right_hand', 0.1)
                self.task_module.talk("Oh, I see the milk carton is right there!", "English", wait=False)
                self.task_module.talk("May you please follow my instructions to help me grab it? I will tell you what to do", "English", wait=False)

            self.scan_items.remove(current_scanning_object)

        self.grab_ingredient()


    def on_enter_GRAB_OBJECT(self):
        """Acciones al entrar en el estado GRAB_OBJECT."""
        print(self.consoleFormatter.format("GRAB_OBJECT", "HEADER"))

        # Determina el siguiente objeto a recoger
        if "bowl" not in self.items_collected:
            self.actual_item = "bowl"
        elif "cereal_box" not in self.items_collected:
            self.actual_item = "cereal_box"
        elif "milk_carton" not in self.items_collected:
            self.actual_item = "milk_carton"
        
        actions = self.grab_items_poses[self.actual_item]  # Obtiene las acciones necesarias para recoger el objeto

        if self.actual_item == "bowl":
            self.task_module.show_image("https://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/grab_bowl.jpeg")
            self.task_module.talk("Could you please put the spoon in the bowl?", "English", wait=False)  # Solicita colocar la cuchara en el bowl       
            rospy.sleep(4)
            self.task_module.talk("Thank you!", "English", wait=False) 
        self.carry_to_serve = True
        self.task_module.go_to_pose("both_arms_cereal")

        for action in actions:  # Ejecuta las acciones para recoger el objeto
            if action == "mid_arms_bowl" :
                self.task_module.talk("Now please place the bowl in my hands just like the image in my tablet shows, so I can take it to the table safely! I will tell you when to stop holding it", wait=False)

            if action == "both_arms_cereal" or action == "both_arms_milk":
                self.task_module.show_image(f"https://raw.githubusercontent.com/SinfonIAUniandes/Image_repository/main/grab_{self.actual_item}.jpeg")
                self.task_module.talk(f"Now please hold the {self.actual_item} in middle of my hands like the image in my tablet shows! I will tell you when to stop holding it.", "English", wait=False)   
            
            self.task_module.go_to_pose(action, self.slow_movement)  # Ejecuta la pose
            rospy.sleep(3)
            
            if action  == "raise_arms_cereal" or action  == "raise_arms_milk" or action  == "raise_arms_bowl":
                carry_thread = threading.Thread(target=self.carry_thread,args=[action])
                carry_thread.start()

        self.task_module.show_words_proxy()
        self.task_module.talk(f"That is a perfect position, thanks a lot for your help", "English", wait=False)  # Informa que se dirige al comedor
        self.go_to_drop_place()  # Transición a GO_TO_DROP_PLACE
        


    def on_enter_GO_TO_DROP_PLACE(self):
        """Acciones al entrar en el estado GO_TO_DROP_PLACE."""
        print(self.consoleFormatter.format("GO_TO_DROP_PLACE", "HEADER"))
        self.task_module.talk("On my way to the dining room", "English", wait=False)  # Informa que se dirige al comedor
        self.task_module.set_move_arms_enabled(False)
        self.task_module.go_to_place("dining_table", lower_arms=False)  # Mueve el robot al comedor
        rospy.sleep(1)  # Espera 1 segundo
        self.carry_to_serve = False
        self.drop_object()  # Transición a DROP
        


    def on_enter_DROP(self):
        """Acciones al entrar en el estado DROP_OBJECT."""
        print(self.consoleFormatter.format("DROP", "HEADER"))
        if self.actual_item == "bowl":
            self.task_module.talk("Now, I will leave the bowl and the spoon on the table", "English", wait=False)  # Informa que dejará el bowl y la cuchara
        else:
            self.task_module.talk(f"Now, I will place the {self.actual_item} on the table and then grab it again", "English", wait=False)  # Informa que servirá y dejará el objeto
        
        drop_relative_point = self.drop_and_serve_position[self.actual_item]  # Obtiene la posición relativa para dejar el objeto
        self.task_module.go_to_relative_point(0.0, drop_relative_point, 0.0)  # Mueve el robot a la posición relativa
        actions = self.drop_and_serve_items_poses[self.actual_item]  # Obtiene las acciones para dejar el objeto
        self.task_module.go_to_relative_point(self.relative_drop_position, 0.0, 0.0)  # Mueve el robot a la posición relativa cerca na a a mesa
        
        for action in actions:  # Ejecuta las acciones para dejar el objeto
            self.task_module.go_to_pose(action, self.slow_movement)  # Ejecuta la pose
            rospy.sleep(1)  # Espera 2 segundos
        rospy.sleep(2)
        self.task_module.go_to_relative_point(-(self.relative_drop_position), 0.0, 0.0)  # Aleja el robot de la posición relativa a la mesa
        self.task_module.go_to_pose("standard")  # Vuelve a la pose estándar
        self.items_collected.append(self.actual_item)  # Añade el objeto a la lista de recogidos
        
        if self.actual_item == "cereal_box" or self.actual_item == "milk_carton":
            self.serve()
        
        elif self.actual_item == "bowl":
            self.again()

            
    def on_enter_MAKE_BREAKFAST(self):
        """Acciones para servir el cereal."""
        print(self.consoleFormatter.format("MAKE_BREAKFAST", "HEADER"))
        self.task_module.talk(f"I will serve the {self.actual_item}", "English", wait=False) 
        actions = self.serve_items_poses[self.actual_item]  # Obtiene las acciones para dejar el objeto
        for action in actions:  # Ejecuta las acciones para dejar el objeto
            self.task_module.go_to_pose(action, self.slow_movement)  # Ejecuta la pose
            rospy.sleep(1)  # Espera 2 segundos
        
        if len(self.items_collected) == len(self.items):  # Verifica si todos los objetos han sido recogidos
            self.end()  # Transición a END
        
        else:
            self.again2()
        

    def on_enter_END(self):
        """Acciones al entrar en el estado END."""
        print(self.consoleFormatter.format("END", "HEADER"))
        self.task_module.talk("I finished serving breakfast, enjoy it", "English", wait=False)  # Mensaje de finalización
        return


    def check_rospy(self):
        """Verifica el estado de rospy y termina el programa si se cierra."""
        while not rospy.is_shutdown():  # Mientras rospy no se haya cerrado
            rospy.sleep(0.1)  # Duerme por 0.1 segundos
        print(self.console_formatter.format("Shutting down", "FAIL"))  # Imprime mensaje de apagado
        os._exit(os.EX_OK)  # Sale del programa

    def carry_thread(self, pose):
        while self.carry_to_serve:
            self.task_module.go_to_pose(pose)
            rospy.sleep(1)
        


    def run(self):
        """Ejecuta la máquina de estados."""
        while not rospy.is_shutdown():  # Mientras rospy no se haya cerrado
            self.zero()  # Transición inicial

    # ---------------------------------------------------------------------------
    #                       FUNCIÓN PRINCIPAL
    # ---------------------------------------------------------------------------
    
if __name__ == "__main__":
    serve_breakfast = ServeBreakfast()  # Crea una instancia de la clase ServeBreakfast
    serve_breakfast.run()  # Ejecuta la máquina de estados
    rospy.spin()  # Mantiene el nodo activo

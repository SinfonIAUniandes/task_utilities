#!/usr/bin/env python3
import os  # Módulo para interactuar con el sistema operativo
import time  # Módulo para manejar el tiempo
import rospy  # Módulo para manejar nodos ROS
import threading  # Módulo para manejar hilos
import ConsoleFormatter  # Módulo para formatear la consola

from transitions import Machine  # Módulo para manejar máquinas de estados
from task_module import Task_module as TaskModule  # Importa y renombra el módulo de tareas

class ServeBreakfast(object):
    def __init__(self) -> None:
        # Inicializa el formateador de consola para mensajes
        self.console_formatter = ConsoleFormatter.ConsoleFormatter()
        
        # Inicializa el módulo de tareas con las herramientas necesarias
        self.task_module = TaskModule(navigation=True, manipulation=True, speech=True, perception=False, pytoolkit=True)
        self.task_module.initialize_node('SERVE_BREAKFAST')
        
        # Define los estados de la máquina de estados
        self.states = ['INIT', 'GO_TO_CUPBOARD', 'GRAB_OBJECT', 'GO_TO_DROP_PLACE', 'DROP_AND_SERVE', 'MAKE_BREAKFAST', 'END']
        
        # Define las transiciones entre los estados
        self.transitions = [
            {'trigger': 'zero', 'source': 'SERVE_BREAKFAST', 'dest': 'INIT'},  # Transición inicial
            {'trigger': 'start', 'source': 'INIT', 'dest': 'GO_TO_CUPBOARD'},  # De INIT a GO_TO_CUPBOARD
            {'trigger': 'grab_ingredient', 'source': 'GO_TO_CUPBOARD', 'dest': 'GRAB_OBJECT'},  # De GO_TO_CUPBOARD a GRAB_OBJECT
            {'trigger': 'go_to_drop_place', 'source': 'GRAB_OBJECT', 'dest': 'GO_TO_DROP_PLACE'},  # De GRAB_OBJECT a GO_TO_DROP_PLACE
            {'trigger': 'drop_object', 'source': 'GO_TO_DROP_PLACE', 'dest': 'DROP_AND_SERVE'},  # De GO_TO_DROP_PLACE a DROP_AND_SERVE
            {'trigger': 'again', 'source': 'DROP_AND_SERVE', 'dest': 'GO_TO_CUPBOARD'},  # Repetir el ciclo
            {'trigger': 'end', 'source': 'DROP_AND_SERVE', 'dest': 'END'}  # Finalizar el ciclo
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
        
        # Posiciones iniciales relativas de los objetos
        # TODO Ajustar posiciones relativas
        self.position_items = {
            "bowl": 0.45,  # Posición del bowl
            "milk_carton": -0.5,  # Posición del cartón de leche
            "cereal_box": 0.0  # Posición de la caja de cereal
        }
        
        # Instrucciones para agarrar los objetos
        # TODO Probar poses
        self.grab_items_poses = {
            "bowl": ["mid_arms_bowl", "both_arms_bowl", "close_arms_bowl", "raise_arms_bowl", "finish"],
            "cereal_box": ["open_both_hands", "both_arms_cereal", "close_arms_cereal", "raise_arms_cereal", "finish"],
            "milk_carton": ["open_both_hands", "both_arms_milk", "close_arms_milk", "raise_arms_milk", "finish"]
        }
        
        # Posiciones finales relativas para dejar los objetos
        # TODO Ajustar posiciones relativas
        self.drop_and_serve_position = {
            "bowl": 0,  # Posición para dejar el bowl
            "milk_carton": 0.2,  # Posición para dejar el cartón de leche
            "cereal_box": 0.2  # Posición para dejar la caja de cereal
        }
        
        # Instrucciones para dejar los objetos
        # TODO Crear poses para dejar objetos y servir
        self.drop_and_serve_items_poses = {
            "bowl": ["close_arms_bowl", "both_arms_bowl", "finish"],
            "milk_carton": ["close_arms_milk", "both_arms_milk", "finish"],
            "cereal_box": ["close_arms_cereal", "both_arms_cereal", "finish"]
        }

    # ---------------------------------------------------------------------------
    #                       ESTADOS / TRANSICIONES
    # ---------------------------------------------------------------------------

    def on_enter_INIT(self):
        """Acciones al entrar en el estado INIT."""
        self.task_module.set_current_place("init")  # Establece la posición inicial del robot
        self.task_module.go_to_pose("standard", self.fast_movement)  # Pone al robot en la posición estándar
        self.task_module.initialize_pepper()  # Inicializa los módulos necesarios
        self.task_module.go_to_pose("up_head", self.normal_movement)  # Ajusta la pose de la cabeza del robot
        self.task_module.talk("Good morning! Today I would like to help you with the preparation of a delicious breakfast", "English", wait=False)  # Mensaje de saludo
        self.start()  # Transición de INIT a GO_TO_CUPBOARD



    def on_enter_GO_TO_CUPBOARD(self):
        """Acciones al entrar en el estado GO_TO_CUPBOARD."""
        self.task_module.talk("I will navigate to the kitchen door and look for the ingredients.", "English", wait=False)  # Mensaje informativo
        self.task_module.go_to_place("kitchen", lower_arms=False)  # Dirige al robot a la cocina
        self.grab_ingredient()  # Transición de GO_TO_CUPBOARD a GRAB_OBJECT



    def on_enter_GRAB_OBJECT(self):
        """Acciones al entrar en el estado GRAB_OBJECT."""
        self.task_module.go_to_pose("almost_down_head", self.normal_movement)  # Baja la cabeza para ver los objetos
        self.task_module.talk("I will tell the order of the objects from left to right", "English", wait=False)  # Informa el orden de los objetos
        for item in self.items:  # Itera sobre los objetos
            self.task_module.talk(f"the {item}", "English", wait=False)  # Menciona cada objeto
        
        # Determina el siguiente objeto a recoger
        if "bowl" not in self.items_collected:
            self.actual_item = "bowl"
        elif "milk_carton" not in self.items_collected:
            self.actual_item = "milk_carton"
        elif "cereal_box" not in self.items_collected:
            self.actual_item = "cereal_box"
        
        relative_collect_point = self.position_items[self.actual_item]  # Obtiene la posición relativa del objeto
        self.task_module.go_to_relative_point(0.0, relative_collect_point, 0.0)  # Mueve el robot a la posición relativa en X del objeto
        self.task_module.go_to_pose("both_arms_cereal")
        self.task_module.go_to_relative_point(0.7, 0.0, 0.0)  # Mueve el robot a la posición relativa en Y del objeto
        actions = self.grab_items_poses[self.actual_item]  # Obtiene las acciones necesarias para recoger el objeto
        self.task_module.talk(f"Now, I am in front of the {self.actual_item}, please put it in the middle of my hands, so I can grab it", "English", wait=False)  # Solicita ayuda para colocar el objeto
        if self.actual_item == "bowl":  
                self.task_module.talk("Please, could you put the spoon in the bowl so I can take it to the next table?", "English", wait=False)  # Solicita colocar la cuchara en el bowl
                time.sleep(5)
                self.task_module.talk("Thank you!", "English", wait=False)  # Agradece la ayuda

        for action in actions:  # Ejecuta las acciones para recoger el objeto
              # Espera 5 segundos
            self.task_module.go_to_pose(action, self.slow_movement)  # Ejecuta la pose
            time.sleep(3)  # Espera 3 segundos
        
        self.task_module.talk(f"Now I will go to the dining room to leave the {self.actual_item}", "English", wait=False)  # Informa que se dirige al comedor
        self.go_to_drop_place()  # Transición a GO_TO_DROP_PLACE
        


    def on_enter_GO_TO_DROP_PLACE(self):
        """Acciones al entrar en el estado GO_TO_DROP_PLACE."""
        self.task_module.talk("On my way to the dining room", "English", wait=False)  # Informa que se dirige al comedor
        self.task_module.go_to_place("dining_table", lower_arms=False)  # Mueve el robot al comedor
        time.sleep(1)  # Espera 1 segundo
        self.drop_object()  # Transición a DROP_AND_SERVE
        


    def on_enter_DROP_AND_SERVE(self):
        """Acciones al entrar en el estado DROP_OBJECT."""
        if self.actual_item == "bowl":
            self.task_module.talk("Now, I will leave the bowl and the spoon on the table", "English", wait=False)  # Informa que dejará el bowl y la cuchara
        else:
            self.task_module.talk(f"Now, I will serve the {self.actual_item} and then drop it on the table", "English", wait=False)  # Informa que servirá y dejará el objeto
            self.task_module.go_to_relative_point(0.0, 0.0, 90)  # Gira 90 grados
        
        drop_relative_point = self.drop_and_serve_position[self.actual_item]  # Obtiene la posición relativa para dejar el objeto
        self.task_module.go_to_relative_point(0.0, drop_relative_point, 0.0)  # Mueve el robot a la posición relativa
        actions = self.drop_and_serve_items_poses[self.actual_item]  # Obtiene las acciones para dejar el objeto
        
        for action in actions:  # Ejecuta las acciones para dejar el objeto
            if self.actual_item == "bowl":
                self.task_module.talk("Please, could you take off the spoon?", "English", wait=False)  # Solicita retirar la cuchara
                time.sleep(5)  # Espera 5 segundos
                self.task_module.talk("Thank you!", "English", wait=False)  # Agradece la ayuda
            self.task_module.go_to_pose(action, self.slow_movement)  # Ejecuta la pose
            time.sleep(3)  # Espera 3 segundos
        
        self.task_module.go_to_pose("standard")  # Vuelve a la pose estándar
        self.items_collected.append(self.actual_item)  # Añade el objeto a la lista de recogidos
        
        if len(self.items_collected) == len(self.items):  # Verifica si todos los objetos han sido recogidos
            self.end()  # Transición a END
        else:
            self.again()  # Transición a GO_TO_CUPBOARD
            
            

    def on_enter_END(self):
        """Acciones al entrar en el estado END."""
        self.task_module.talk("I finished serving breakfast, enjoy it", "English", wait=False)  # Mensaje de finalización
        return
    
    

    def check_rospy(self):
        """Verifica el estado de rospy y termina el programa si se cierra."""
        while not rospy.is_shutdown():  # Mientras rospy no se haya cerrado
            rospy.sleep(0.1)  # Duerme por 0.1 segundos
        print(self.console_formatter.format("Shutting down", "FAIL"))  # Imprime mensaje de apagado
        os._exit(os.EX_OK)  # Sale del programa



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

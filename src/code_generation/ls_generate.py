from code_generation.generate_utils import generate_response, load_task_config

# from generate_utils import generate_response, load_task_config
from code_generation.database.models import Model

class LongStringGenerator:

    def __init__(self) -> None:
        self.robot_vars = load_task_config()
        self.place_names = self.robot_vars["place_names"]
        self.objects = self.robot_vars["objects"]
        self.question_tags = self.robot_vars["question_tags"]
        self.task_module_code = f"""
        Perception functions:
        self.tm.find_object(object_name)->bool: Returns True if the object was found, False if not, the only possible objects with their exact syntax are: {self.objects}.
        self.tm.count_objects(object_name)->int: Returns the number of objects found, the only possible objects with their exact syntax are: {self.objects}
        self.tm.search_for_specific_person(class_type,specific_characteristic)->bool: The robot spins in place until it finds a person with a specific characteristic. The options for class_type are: "pointing", "name", "raised_hand". The options for specific_characteristic depend on class_type. If class_type is "name", options are human names like "Paris","Robin","Jaine". If class_type is "pointing" or "raised_hand" options are "right", "left", "center". Returns True if such a person was found, False if not.
        self.tm.find_item_with_characteristic(class_type,characteristic,place)->str: Allows the robot to find the item which has the given "characteristic", the options for class_type are: "color", "size", "weight", "position", "description". the options for characteristic depend on class_type: Color ("red","blue","black with white dots"). Size ("smallest", "largest", "thinnest", "big one", "small one"). Weight ("lightest", "heaviest"). Position ("left", "right"). Description("fragile","container","pourable","two-handed"). "place" Indicates the furniture or item on top of which the robot needs to search for the object.Returns a String with the name of the object with the needed characteristics or "None" if it wasn't found.

        Speech functions:
        self.tm.talk(text): Allows the robot to say the input of the service.
        self.tm.speech2text_srv()->str: Allows the robot to listen to the user and returns the text that the robot heard
        self.tm.q_a_speech(tag)->str: Allows the robot to ASK a question and returns the answer of the user, the list of possible questions with exact syntax is: {self.question_tags}
        self.tm.answer_question(question)->bool: Returns a string with an answer to the question

        Navigation functions:
        self.tm.go_to_place(place_name): Allows the robot to go to a place, the only possible places with their exact syntax are: {self.place_names}
        self.tm.follow_you(command): Allows the robot to follow the user and make him stop by saying "stop", if the command is True the robot follows the user, if it's False it stops following. The robot must tell the user that they can stop the robot by saying "stop".
        self.tm.robot_stop_srv(): Makes the robot to stop
        self.tm.add_place(place_name): Allows the robot to add a place to the map

        Manipulation functions:
        self.tm.ask_for_object(object_name): Allows the robot to grasp an object, the only possible objects with their exact syntax are: {self.objects}
        self.tm.give_object(object_name): Allows the robot to leave an object, the only possible objects with their exact syntax are: {self.objects}
        """

    def generate_exec(self, task:str)-> str:
        print("generate exec")
        system_message = """You are a code generation AI model for a robot called Pepper. You only return code, nothing more."""

        text_prompt = f"""
        You are a Pepper robot, given a task definition and an interface of the codebase (it only describes what each function does). You must generate the python code that completes the task using the codebase interface.

        # Details about the code to generate:
        - Always try to complete the task
        - The task module has allready instantiated as `self.tm = task_module.Task_module(perception = True,speech=True,manipulation=True, navigation=True)` so you should never instantiate it again
        - Use the functions as they are described in the codebase interface, for example `self.tm.talk("Hello")` to talk
        - Do not use classes, just functions
        - If needed you can import libraries for extracting the actual time, date, etc.
        - The code must be written in python and the output will be executed directly
        - Always use self.tm.<function_name> to call the functions of the codebase interface
        - Return only the code, just code, your output is going to be saved in a variable and executed with exec(<your answer>)
        - Make sure to call and execute the functions from the codebase
        - If with the given functions and external libraries you cannot complete the task, please respond with self.tm.talk("I am sorry but I cannot complete this task")
        - MANDATORY: you must talk in between steps so users know what you are doing
        - The only available places are: {self.place_names}, if you need to go to a place that is not listed use the most similar one from the list. Not doing this will result in an error. Use the syntax from the list when calling the codebase functions.
        - If the place you need to go or a similar place is NOT listed above, please respond with self.tm.talk("I cannot go to <place>")
        - Always assume that the robot is at the entrance of the house at the beginning of the task
        - To locate objects, navigate to the place in which is most likely to find the object and then use the `find_object` service
        - Scorting or guiding a person should be done by a `talk` service followed up by a `go_to_place` service

        - The only available objects are: {self.objects}, if you need to recognize an object that is not listed use the most similar one from the list. Not doing this will result in an error. Use the syntax from the list when calling the codebase functions.
        - If the object you need to recognize or a similar object is NOT listed above, please respond with self.tm.talk("I cannot recognize <object>")
        - For recognizing people just use "person" instead of a specific name

        - The only available default questions are: {self.question_tags}
        - If you need to ask a question that is not listed just use the `talk` method to say the question and the `speech2text_srv` followed to save the answer. Use the syntax from the list when calling the codebase functions.

        # Output Format:
        - Your output needs to be formatted in markdown as a python code snippet, do not add anything else to the output (don't add any exec calls or "Here's your code" statements), just the code.
        - DO NOT INCLUDE ANY AI CHAT RESPONSES IN THE OUTPUT, JUST GENERATE THE CODE
        - DO NOT INCLUDE ANY COMMENTS IN THE CODE
        - The response must be formatted as follows: ```python\n<CODE> ```

        For example, if the task is "Grab a bottle, and bring it to the living room" you should return:
        ```python
        self.tm.talk("I am going to grab a bottle")
        self.tm.go_to_place("kitchen")
        found_bottle = self.tm.find_object("bottle")
        if found_bottle:
            self.tm.grasp_object("bottle")
        else:
            self.tm.talk("I cannot find the bottle")'
        self.tm.talk("I am going to the living room")
        self.tm.go_to_place("living_room")
        if found_bottle:
            self.tm.leave_object("bottle")
        else:
            self.tm.talk("I am sorry, I did not find the bottle")
        ```
        Without the asterisks

        # Task description:
        {task}

        # Codebase Interface (functions you can use):

        {self.task_module_code}

        # Code to generate:
        """
        return generate_response(text_prompt, system_message=system_message, is_code=True, model_type=self.model)

    def generate_code(self, task:str, model: Model)->str:
        self.model = model
        code = self.generate_exec(task)
        return code
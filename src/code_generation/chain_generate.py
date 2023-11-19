from generate_utils import generate_response, get_task_module_code, load_task_config
from database.models import Model

class ChainGenerator:

    def __init__(self) -> None:
        self.robot_vars = load_task_config()
        self.place_names = ", ".join(['"'+elemento+'"' for elemento in self.robot_vars["place_names"]])
        self.objects = ", ".join(['"'+elemento+'"' for elemento in self.robot_vars["objects"]])
        self.question_tags = ", ".join(['"'+elemento+'"' for elemento in self.robot_vars["question_tags"]])
        self.task_module_code = f"""
        Perception functions:
        self.tm.find_object(object_name)->bool: Returns True if the object was found, False if not, the only possible objects with their exact syntax are: {self.objects}
        self.tm.count_objects(object_name)->int: Returns the number of objects found, the only possible objects with their exact syntax are: {self.objects}

        Speech functions:
        self.tm.talk(text): Allows the robot to say the input of the service.
        self.tm.speech2text_srv()->str: Allows the robot to listen to the user and returns the text that the robot heard
        self.tm.q_a_speech(tag)->str: Allows the robot to ask a question and returns the answer of the user, the list of possible questions with exact syntax is: {self.question_tags}
        self.tm.answer_question(question)->str: Allows the robot to answer a question, the input should be extracted with the speech2text_srv() function and to say the answer use the talk(text) function

        Navigation functions:
        self.tm.go_to_place(place_name): Allows the robot to go to a place, the only possible places with their exact syntax are: {self.place_names}
        self.tm.follow_you(): Allows the robot to follow the user
        self.tm.robot_stop_srv(): Allows the robot to stop
        self.tm.add_place(place_name): Allows the robot to add a place to the map

        Manipulation functions:
        self.tm.grasp_object(object_name): Allows the robot to grasp an object, the only possible objects with their exact syntax are: {self.objects}
        self.tm.leave_object(object_name): Allows the robot to leave an object, the only possible objects with their exact syntax are: {self.objects}
        """

    def extract_entities(self, task:str)->str:
        text_prompt = f"""
        # Instructions:
        Extract all entities from the following task description, understand entities as places, objects, people, etc.
        For example, in the task description "Look for a person in the dining table, place a plate on the dining table, and guide Jacob from the corridor to the apartment." the entities are: person, dining table, plate, corridor, apartment, Jacob.
        Requirements:
        - Please format the output as a python list of strings, for example: ["person", "dining table", "plate", "corridor", "apartment", "Jacob"]
        # Task description:
        {task}
        """
        return generate_response(text_prompt, is_code=False, model_type=self.model)

    def replace_semantic_entities(self, input_entities:list)->str:
        text_prompt = f"""
        # Instructions:
        Replace all entities from the following list with equivalent places, objects or q_a from the valid entities list.
        For example: replace "toilet" with "bathroom".
        Understand entities as places, objects, people, etc.

        Requirements:
        - Please format the output as a python list of strings, for example: ["person", "dining_room", "plate", "corridor", "apartment", "Jacob"]
        - If you think there is a synonym included in the valid entities list, please replace it in the output.
        - ONLY replace entities from the input list with entities in the valid entities list. Do not add new entities.

        # Input entities:
        {input_entities}

        # Valid entities:
        - Valid Places: {self.place_names}
        - Valid Objects: {self.objects}
        - Valid Q_A: {self.question_tags}
        """
        return generate_response(text_prompt, is_code=False, model_type=self.model)

    def replace_entities_in_task(self, task_input:str, input_entities:list, valid_entities:list)->str:
        text_input = f"""
        # Instructions:
        Replace all given entities in the following task description with their equivalent synonyms from the valid entities list and return a task description with the replaced values.
        For example: replace "toilet" with "bathroom".
        Do not replace person names with "person".
        If there isn't anything to replace, return the original task description.

        # Input entities:
        {input_entities}

        # Valid entities:
        {valid_entities}

        # Task description:
        {task_input}
        """
        try:
            return generate_response(text_input, is_code=False, model_type=self.model)
        except KeyError:
            return task_input

    def generate_task_steps(self, task_input:str)->str:
        system_message = """You serve as a professional planner for Pepper, a versatile general-purpose service robot. Your role involves providing detailed instructions on how to accomplish a specific task changing places and object to known places and objects for Pepper."""
        text_prompt = f"""
        Output constraints:
        - MANDATORY: Answer in a paragraph describing the process to complete the task.
        - Just give me the answer, do not include anything else in your answer.
        - Try to do the most simple solution possible.
        - Think as a robot, not as a human. You have access to internet, time, date, etc.
        - Do not generate code, just a detailed short description.
        - Do not exceed 80 words in your description.
        - Do not use specific steps
        - Change all places and locations to the ones Pepper know, for example, change kitchen (unknown for Pepper) to kitchen_table (know for Pepper)
        - Change all objects to the ones Pepper know, for example, change water bottle (unknown for Pepper) to bottle (know for Pepper)

        Navigation Constraints:
        - **Available places to navigate**: {self.place_names}
        - If the place you need to go is not listed above, you may decide if going to an above place is enough or if you are not able to do the task
        - When you give the description make sure to only use places listed here with the exact syntax of the next list: {self.place_names}
        - Assume you are never in the place where you need to go or at the starting place (If you need to go somewhere)
        - If you need to go back to a place you already visited it is important for you to save that place and this had to be included in the description.

        Perception constraints:
        - For object recognition include both cases where the object is found and where it is not found
        - **Available objects to recognize**: {self.objects}
        - If the object you need to recognize is not listed, you may decide if recognizing an above object is enough or if you are not able to do the task
        - For recognizing specific persons just recognize "person", using personal names will result in an error
        - When you give the description make sure to only use object listed here with the exact syntax used in the next list: {self.objects}
        - To recognize special persons you may just recognize "person" instead of looking for a specific person

        Speech constraints:
        - You can ask as much you want to the people if the task is not clear or if you need to know something, for example peoples name, age, drink, etc
        - Meeting a person means greeting the person

        # Task Description:

        {task_input}

        """
        return generate_response(text_prompt, system_message=system_message, is_code=False, model_type=self.model)

    def classify_task(self, task:str)->(bool, str):
        system_message = """Your role is to classify tasks as approved or not approved for Pepper, a versatile general-purpose service robot. Your role involves providing if it's possible for Pepper to complete a task and the reason."""
        text_prompt = f"""
        Output constraints:
        - MANDATORY: Answer in a formatted way using ; to separate the answer and the reason, for example: "True; I can do the task because I can go to the kitchen and I can recognize a bottle" or "False; I cannot do the task because I cannot go to the car wash"
        - Just give me the answer, do not include anything else in your answer.
        - Think as a robot, not as a human. You have access to internet, time, date, etc.
        - The only cases in which a task is not approved is when one of the following constraints is not met:

        Navigation Constraints:
        - **Available places to navigate**: {self.place_names}
        - You can only go the places listed above. If you need to go to a different place, your answer should be "False; I cannot do the task because I cannot go to <place1>"
        - If the place is in the list, then you can go to that place with no problem.

        Perception constraints:
        - **Available things to recognize**: {self.objects}
        - You can only recognize objects or things listed above. If you need to recognize something not listed above, the answer should be "False; I cannot do the task because I cannot recognize <object1>"
        - If the thing is in the list, then you can recognize that thing with no problem.
        - Special people can be recognized as "person" instead of a specific name

        Speech constraints:
        - **Available questions to ask**: {self.question_tags}
        - If the question you need to ask is not listed, you cannot do the task. Your answer should be "False; I cannot do the task because I cannot ask <question1>"

        # Task Description:

        {task}

        """
        answer = generate_response(text_prompt, system_message=system_message, is_code=False, model_type=self.model)
        try:
            approved, reason = answer.split(";")
            if approved == "True":
                approved = True
            else:
                approved = False
            return (approved, reason)
        except:
            return (True, "I cannot understand the answer, please try again")

    def generate_exec(self, task:str)-> str:

        system_message = """You are a code generation AI model for a robot called Pepper. You only return code, nothing more."""

        text_prompt = f"""
        You are a Pepper robot, given a task definition and an interface of the codebase (it only describes what each function does). You must generate the python code that completes the task using the codebase interface.

        # Details about the code to generate:
        - Always try to complete the task
        - If you need any extrenal libraries, you can import them
        - The task module has allready instantiated as `self.tm = task_module.Task_module(perception = True,speech=True,manipulation=True, navigation=True)` so you should never instantiate it again
        - Use the functions as they are described in the codebase interface, for example `self.tm.talk("Hello")` to talk
        - Do not use classes, just functions
        - The code must be written in python and the output will be executed directly
        - Always use self.tm.<function_name> to call the functions of the codebase interface
        - Return only the code, just code, your output is going to be saved in a variable and executed with exec(<your answer>)
        - Make sure to call and execute the functions from the codebase
        - MANDATORY: you must talk in between steps so users know what you are doing.

        # Output Format:
        - Your output needs to be formatted in markdown as a python code snippet, do not add anything else to the output (don't add any exec calls or "Here's your code" statements), just the code.
        - The response must be formatted as follows: ```python\n<CODE> ```
        - DO NOT INCLUDE ANY AI CHAT RESPONSES IN THE OUTPUT, JUST GENERATE THE CODE

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


        - The only available places are: {self.place_names}, if you need to go to a place that is not listed use the most similar one from the list. Not doing this will result in an error. Use the syntax from the list when calling the codebase functions.
        - If the place you need to go or a similar place is NOT listed above, please respond with self.tm.talk("I cannot go to <place>")
        - Scorting or guiding a person should be done by a `talk` service followed up by a `go_to_place` service

        - The only available objects are: {self.objects}, if you need to recognize an object that is not listed use the most similar one from the list. Not doing this will result in an error. Use the syntax from the list when calling the codebase functions.
        - If the object you need to recognize or a similar object is NOT listed above, please respond with self.tm.talk("I cannot recognize <object>")
        - For recognizing people just use "person" instead of a specific name

        - The only available default questions are: {self.question_tags}, if you need to ask a question that is not listed just use the `talk` method to day the question and the `speech2text_srv` followed to save the answer. Use the syntax from the list when calling the codebase functions.

        # Task description:
        {task}

        # Codebase Interface:

        {self.task_module_code}

        # Code to generate:

        """
        return generate_response(text_prompt, system_message=system_message, is_code=True, model_type=self.model)

    def generate_code(self, task:str, model: Model)->str:
        self.model = model
        steps = self.generate_task_steps(task)
        entities = self.extract_entities(steps)
        new_entities = self.replace_semantic_entities(entities)
        new_task = self.replace_entities_in_task(steps, entities, new_entities)
        approved, reason = self.classify_task(new_task)
        if approved:
            code = self.generate_exec(new_task)
            return (entities,new_entities,new_task,steps,code)
        else:
            return (entities,new_entities,new_task,steps,f'self.tm.talk(\"\"\"I am sorry but {reason}\"\"\")')
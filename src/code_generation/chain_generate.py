from generate_utils import generate_gpt, get_task_module_code, load_task_config
from database.models import Model
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from database.config import init_db
from dotenv import load_dotenv
from generate_utils import load_code_gen_config

load_dotenv()
init_db()
load_code_gen_config()

from database.models import Model

class ChainGenerator:

    def __init__(self) -> None:
        self.robot_vars = load_task_config()
        self.place_names = self.robot_vars["place_names"]
        self.objects = self.robot_vars["objects"]
        self.question_tags = self.robot_vars["question_tags"]

    def extract_entities_gpt(self, task:str)->str:
        text_prompt = f"""
        # Instructions:
        Extract all entities from the following task description, understand entities as places, objects, people, etc.
        For example, in the task description "Look for a person in the dining table, place a plate on the dining table, and guide Jacob from the corridor to the apartment." the entities are: person, dining table, plate, corridor, apartment, Jacob.
        Requirements:
        - Please format the output as a python list of strings, for example: ["person", "dining table", "plate", "corridor", "apartment", "Jacob"]
        # Task description:
        {task}
        """
        return generate_gpt(text_prompt, is_code=False)

    def replace_semantic_entities_gpt(self, input_entities:list)->str:
        text_prompt = f"""
        # Instructions:
        Replace all entities from the following list with equivalent places, objects or q_a from the valid entities list.
        For example: Replace "Jacob" with "person" or "toilet" with "bathroom".
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
        return generate_gpt(text_prompt, is_code=False)

    def replace_entities_in_task(self, task_input:str, input_entities:list, valid_entities:list)->str:
        text_input = f"""
        # Instructions:
        Replace all given entities in the following task description with their equivalent synonyms from the valid entities list and return a task description with the replaced values.
        If there isn't anything to replace, return the original task description.

        # Input entities:
        {input_entities}

        # Valid entities:
        {valid_entities}

        # Task description:
        {task_input}
        """
        try:
            return generate_gpt(text_input, is_code=False)
        except KeyError:
            return task_input

    def generate_task_steps_gpt(self, task_input:str)->str:
        system_message = """You serve as a professional planner for Pepper, a versatile general-purpose service robot. Your role involves providing detailed instructions on how to accomplish a specific task changing places and object to known places and objects for Pepper."""
        text_prompt = f"""
        Output constraints:
        - MANDATORY: Answer in a paragraph describing the process to complete the task.
        - Just give me the answer, do not include anything else in your answer.
        - If you believe you cannot accomplish the task, just say "Pepper should say: I cannot do the task because <reason>"
        - Try to do the most simple solution possible.
        - Think as a robot, not as a human. You have access to internet, time, date, etc.
        - Do not generate code, just a detailed short description.
        - Do not exceed 80 words in your description.
        - Do not use specific steps
        - Change all places and locations to the ones Pepper know, for example, change kitchen (unknown for Pepper) to kitchen_table (know for Pepper)
        - - Change all objects to the ones Pepper know, for example, change water bottle (unknown for Pepper) to bottle (know for Pepper)

        Navigation Constraints:
        - **Available places to navigate**: {self.place_names}
        - If the place you need to go is not listed above, you may decide if going to an above place is enough or if you are not able to do the task
        - When you give the description make sure to only use places listed here with the exact syntax of the next list: {self.place_names}
        - Asume you are never in the place where you need to go (If you need to go somewhere)
        - If you need to go back to a place you already visited it is important for you to save that place and this had to be included in the description.

        Perception constraints:
        - For object recognition include both cases where the object is found and where it is not found
        - **Available objects to recognize**: {self.objects}
        - If the object you need to recognize is not listed, you may decide if recognizing an above object is enough or if you are not able to do the task
        - When you give the description make sure to only use object listed here with the exact syntax used in the next list: {self.objects}
        - To recognize special persons you may just recognize "person" instead of looking for a specific person

        Speech constraints:
        - You can ask as much you want to the people if the task is not clear or if you need to know something, for example peoples name, age, drink, etc
        - Meeting a person means greeting the person

        # Task Description:

        {task_input}

        """
        return generate_gpt(text_prompt, system_message=system_message, is_code=False)

    def generate_exec_gpt(self, task:str)-> str:

        task_module_code = get_task_module_code()

        system_message = """You are a code generation AI model for a robot called Pepper."""

        text_prompt = f"""
        You are a Pepper robot, given a task definition and an interface of the codebase (it only describes what each function does). You must generate the python code that completes the task using the codebase interface.

        # Details about the code to generate:
        - Always try to complete the task
        - The task module has allready instantiated as `self.tm = task_module.Task_module(perception = True,speech=True,manipulation=True, navigation=True)` so you should never instantiate it again
        - Do not use classes, just functions
        - The code must be written in python and the output will be executed directly
        - Always use self.tm.<function_name> to call the functions of the codebase interface
        - Return only the code, just code, your output is going to be saved in a variable and executed with exec(<your answer>)
        - Make sure to call and execute the functions from the codebase
        - MANDATORY: you must speak in between steps so users know what you are doing
        - The only available places are: {self.place_names}, if you need to go to a place that is not listed use the most similar one from the list. Not doing this will result in an error. Use exactly the sintax from the list
        - The only available objects are: {self.objects}, if you need to recognize an object that is not listed use the most similar one from the list. Not doing this will result in an error. Use exactly the sintax from the list
        - Special people could be considered as "person"

        # Task description:
        {task}

        # Codebase Interface:

        {task_module_code}

        # Code to generate:

        """
        return generate_gpt(text_prompt, system_message=system_message, is_code=True)

    def generate_code(self, task:str, model: Model)->str:
        if model == Model.GPT35:
            entities = self.extract_entities_gpt(task)
            new_entities = self.replace_semantic_entities_gpt(entities)
            new_task = self.replace_entities_in_task(task, entities, new_entities)
            steps = self.generate_task_steps_gpt(new_task)
            code = self.generate_exec_gpt(steps)
            return (entities,new_entities,new_task,steps,code)
        else:
            pass
            #TODO: Add llama model code generation
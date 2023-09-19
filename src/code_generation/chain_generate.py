from generate_utils import generate_gpt, get_task_module_code, count_tokens, load_task_config
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

import time
import json
from traceback import format_exception
from generate import generate_code as gen
from database.models import Model, PromptingType, ExecutionResults, PepperTest
from database.crud import get_non_executed_tests, update_test, create_test
from task_generation.generate import generate_category_tasks

def generate_task_steps_gpt(task_input:str,config:dict)->str:
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
    - **Available places to navigate**: {", ".join(config["place_names"])}
    - If the place you need to go is not listed above, you may decide if going to an above place is enough or if you are not able to do the task
    - When you give the description make sure to only use places listed here with the exact syntax of the next list: {", ".join(config["place_names"])}
    - Asume you are never in the place where you need to go (If you need to go somewhere)
    - If you need to go back to a place you already visited it is important for you to save that place and this had to be included in the description.

    Perception constraints:
    - For object recognition include both cases where the object is found and where it is not found
    - **Available objects to recognize**: {", ".join(config["objects"])}
    - If the object you need to recognize is not listed, you may decide if recognizing an above object is enough or if you are not able to do the task
    - When you give the description make sure to only use object listed here with the exact syntax used in the next list: {", ".join(config["objects"])}
    - To recognize special persons you may just recognize "person" instead of looking for a specific person

    Speech constraints:
    - You can ask as much you want to the people if the task is not clear or if you need to know something, for example peoples name, age, drink, etc
    - Meeting a person means greeting the person

    # Task Description:

    {task_input}

    """
    print(f"Tokens used (Steps): {count_tokens(system_message+text_prompt)}")
    return generate_gpt(text_prompt, system_message=system_message, is_code=False)

def generate_exec_gpt(task:str,config:dict)-> str:

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
    - MANDATOY: you must speak in between steps so users know what you are doing
    - The only available places are: {", ".join(config["place_names"])}, if you need to go to a place that is not listed use the most similar one from the list. Not doing this will result in an error. Use exactly the sintax from the list
    - The only available objects are: {", ".join(config["objects"])}, if you need to recognize an object that is not listed use the most similar one from the list. Not doing this will result in an error. Use exactly the sintax from the list
    - Special people could be considered as "person"

    # Task description:
    {task}

    # Codebase Interface:

    {task_module_code}

    # Code to generate:

    """
    print(f"Tokens used (Code): {count_tokens(system_message+text_prompt)}")
    return generate_gpt(text_prompt, system_message=system_message, is_code=True)

def generate_code(task:str, model: Model)->str:
    robot_vars = load_task_config()
    if model == Model.GPT35:
        steps = generate_task_steps_gpt(task,robot_vars)
        code = generate_exec_gpt(steps,robot_vars)
        return (steps,code)
    else:
        pass
        #TODO: Add llama model code generation

# print(generate_code("Look for a person in the dining table, place a plate on the dining table, and guide Jacob from the corridor to the apartment.",Model.GPT35))
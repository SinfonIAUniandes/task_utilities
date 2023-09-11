import openai
import os
from generate_utils import generate_openai, generate_azure, get_task_module_code, count_tokens, load_code_gen_config, load_task_config


def generate_task_steps(task_input:str,config:dict)->str:
    system_message = """You serve as a professional planner for Pepper, a versatile general-purpose service robot. Your role involves providing detailed instructions on how to accomplish a specific task."""
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

    Navigation Constraints:
    - **Available places to navigate**: {",".join(config["place_names"])}
    - If the place you need to go is not listed above, you may decide if going to an above place is enough or if you are not able to do the task
    - Asume you are never in the place where you need to go (If you need to go somewhere)
    - If you need to go back to a place you already visited it is important for you to save that place and this had to be included in the description.

    Perception constraints:
    - For object recognition include both cases where the object is found and where it is not found
    - **Available objects to recognize**: {",".join(config["objects"])}
    - If the object you need to recognize is not listed, you may decide if recognizing an above object is enough or if you are not able to do the task
    - To recognize special persons you may just recognize "person" instead of looking for a specific person

    Speech constraints:
    - You can ask as much you want to the people if the task is not clear or if you need to know something, for example peoples name, age, drink, etc
    - Meeting a person means greeting the person

    # Task Description:

    {task_input}

    """
    print(f"Tokens used (Steps): {count_tokens(system_message+text_prompt)}")
    if openai.api_version is None:
        return generate_openai(text_prompt, system_message=system_message, is_code=False)
    else:
        return generate_azure(text_prompt, system_message=system_message, deployment_name=os.getenv("OPENAI_DEPLOYMENT_NAME", "gpt-35-turbo"), is_code=False)

def generate_exec(task:str,config:dict)-> str:

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
    - The only available places are: {",".join(config["place_names"])}, if you need to go to a place that is not listed, you may decide if going to a listed above place is enough or if you are not able to do the task
    - The only available objects are: {",".join(config["objects"])}, if you need to recognize an object that is not listed, you may decide if recognizing a listed object is enough or if you are not able to do the task. Special people could be considered as "person"
    - Special people could be considered as "person"

    # Task description:
    {task}

    # Codebase Interface:

    {task_module_code}

    # Code to generate:

    """
    print(f"Tokens used (Code): {count_tokens(system_message+text_prompt)}")
    if openai.api_version is None:
        return generate_openai(text_prompt, system_message=system_message)
    else:
        return generate_azure(text_prompt, system_message=system_message, deployment_name=os.getenv("OPENAI_DEPLOYMENT_NAME", "gpt-35-turbo"))

def generate_code(task:str)->str:
    robot_vars = load_task_config()
    steps = generate_task_steps(task,robot_vars)
    print(steps)
    code = generate_exec(steps,robot_vars)
    return code

if __name__ == "__main__":
    load_code_gen_config()

    task = "What time is it"
    #task = input("Write the task: ")
    print(task)
    print(generate_code(task))
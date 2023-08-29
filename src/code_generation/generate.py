import os
import openai
from generate_utils import generate_openai, generate_azure, get_task_module_code, load_code_gen_config

## TODO: Get a working OpenAI API key

def generate_code(task_input: str)-> str:

    ## TODO: Get the codebase from the task module
    task_module_code = get_task_module_code()

    system_message = """You are a code generation AI model for a robot called Pepper."""

    text_prompt = f"""
    You are a Pepper robot, given a task description and an interface of the codebase (it only describes what each function does). You must generate the code that completes the task using the codebase interface.
    For the given task, you must generate the code that completes the task using the codebase interface.
    Think step by step what you need to do to complete the task before you generate code. Then generate the code that completes the task using the codebase interface.

    # Details about the code to generate:
    - The code must be written in python and the output will be executed directly
    - The code is going to be executed in a ROS node, so there is no need to initialize ROS
    - The Task_module class is allready instantiated as `self.tm = task_module.Task_module(perception = True,speech=True,manipulation=True, navigation=True)`
    - Do not instantiate the Task_module class again
    - Use only self.tm.<function_name> to call the functions of the codebase interface
    - Only use the functions of the codebase interface that are needed to complete the task and callbacks of the given ros topics
    - The initialize_node function is already called for you, you cannot call it again
    - The code cannot include the original codebase interface, it is only for using its functions
    - Remember to initialize and dispose of every sensor in case you need them, for example calling `self.tm.turn_camera("front_camera","custom",1,15)`
    - Return only the code, just code, your output is going to be saved in a variable and executed with exec(<your answer>)
    - You can ask as much you want to the people, for example peoples name, age, drink, etc
    - You are in the "door_living_room", for every different location you have to navigate to the location first
    - You are allowed to do ros topic callbacks of the given topics
    - For object recognition include both cases where the object is found and where it is not found
    - The robot start always in the "door" spot and it must go and move to other places if needed
    - Meeting a person means greeting the person
    - Talking in every step to the user is mandatory
    - Make sure to call and execute the functions created
    - **Available places to navigate**: "bed","dishwasher","kitchen_table","dining_room","sink","desk","entrance","cleaning_stuff","bedside_table","shelf_bedroom","trashbin","pantry","refrigerator",cabinet","tv_stand","storage_rack","side_table","sofa","bookshelf"
    - You cannot go to places that aren't listed above.
    - One location at a time, first the robot will navigate to one location, then the rest of the task
    - Speak with the speech module when you start doing something and when you finish doing something
    - MANDATORY: Use the speech module to tell the user your current action
    - MANDATORY: If you believe the given task cannot be done, just return code where the robot speaks: "I cannot do that task"

    # Task Description:

    {task_input}

    # Codebase Interface:

    {task_module_code}

    # Code to generate:
    """

    if openai.api_version is None:
        return generate_openai(text_prompt, system_message=system_message)
    else:
        return generate_azure(text_prompt, system_message=system_message, deployment_name=os.getenv("OPENAI_DEPLOYMENT_NAME", "gpt-35-turbo"))

if __name__ == "__main__":
    load_code_gen_config()
    task = "Could you enter to the living room, locate the fruits, and hand it to Elizabeth at the dining table. Fruits: apple"
    #task = input("Write the task: ")
    print(task)
    generate_code(task)

import openai
from generate_utils import generate_openai, get_task_module_code, count_tokens

def generate_task_steps(task_input:str)->str:
    system_message = """You are a professional scheduler and planner for a general purpose service robot called Pepper. Your task will consist of indicating what steps should a robot take to complete a given task."""
    text_prompt = f"""
    Think step by step what you need to do to complete the task. Then generate the steps that the robot must take to complete it.
    - If you believe you cannot accomplish the task, just say "Step #: I cannot do that task". Where # is the number of the step you cannot do. After that, the robot should not have any more steps.
    - Do not try to solve the aforementioned impossible task.
    - Do not generate code, just steps.

    # Details about how the planning must look like:
    - Step #: <task description>

    # Task Description:

    {task_input}

    """
    print(f"Tokens used (Steps): {count_tokens(system_message+text_prompt)}")
    return generate_openai(text_prompt, system_message=system_message, code=False)

def generate_exec(steps:str)-> str:

    task_module_code = get_task_module_code()

    system_message = """You are a code generation AI model for a robot called Pepper."""

    text_prompt = f"""
    You are a Pepper robot, given a step definition and an interface of the codebase (it only describes what each function does). You must generate the code that completes the specified steps using the codebase interface.
    For the given task, you must generate the code that completes the task using the codebase interface.

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
    - Make sure to call and execute the functions from the codebase
    - **Available places to navigate**: "bed","dishwasher","kitchen_table","dining_room","sink","desk","entrance","cleaning_stuff","bedside_table","shelf_bedroom","trashbin","pantry","refrigerator",cabinet","tv_stand","storage_rack","side_table","sofa","bookshelf"
    - You cannot go to places that aren't listed above.
    - One location at a time, first the robot will navigate to one location, then the rest of the task
    - MANDATORY: Use the speech module to tell the user your current action

    # Task steps:
    {steps}

    # Codebase Interface:

    {task_module_code}

    # Code to generate:

    """
    print(f"Tokens used (Code): {count_tokens(system_message+text_prompt)}")
    return generate_openai(text_prompt, system_message=system_message)

def generate_code(task:str)->str:
    steps = generate_task_steps(task)
    print(steps)
    code = generate_exec(steps)
    return code

if __name__ == "__main__":
    openai.api_key = "YOUR_API_KEY"

    task = "Could you bring me a glass of water?"
    #task = input("Write the task: ")
    print(task)
    print(generate_code(task))
import openai
import os
import re

## TODO: Get a working OpenAI API key

codebase = """"""

def generate_text(text_prompt, system_message=None, model="gpt-3.5-turbo-16k", temperature=0):

    messages = [
        {"role": "user", "content": text_prompt}
    ]

    if system_message:
        messages = [
            {"role": "system", "content": system_message},
        ] + messages

    prediction = openai.ChatCompletion.create(
        model=model,
        temperature=temperature,
        messages=messages
    )
    answer =prediction['choices'][0]['message']['content']
    pattern = r'```python(.*?)\n```'
    try:
        code = ((re.search(pattern, answer, re.DOTALL)).group(1)).strip()
    except:
        code = """self.tm.talk("I cannot do this command")"""
    #print answer without code
    print(answer)
    return code

def get_task_module_code()-> str:
    global codebase
    if codebase == "":
        with open(os.path.join(os.path.dirname(__file__), "task_module_interface.py"), "r") as f:
            codebase = f.read()
    return codebase

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
    - The robot start always in the "door" spot and it must go and move to other places if needed
    - Meeting a person means greeting the person
    - TALKING IN EVERY STEP TO THE USER IS MANDATORY
    - Make sure to call and execute the functions created
    - AVAILABLE PLACES TO NAVIGATE: "bed","dishwasher","kitchen_table","dining_room","sink","desk","entrance","cleaning_stuff","bedside_table","shelf_bedroom","trashbin","pantry","refrigerator",cabinet","tv_stand","storage_rack","side_table","sofa","bookshelf"
    - PLACES THAR ARE NOT PART OF THE AVAILABLE PLACES TO NAVIGATE ARE NOT ALLOWED IN FOLLOW YOU
    - FIRST THE ROBOT WILL NAVIGATE TO THE LOCATION OF THE TASK, THEN THE REST OF THE TASK
    - One location at a time, first the robot will navigate to one location, then the rest of the task
    - REMEMBER TO THINK STEP BY STEP WHAT YOU NEED TO DO TO COMPLETE THE TASK BEFORE YOU GENERATE CODE
    - THEN GENERATE THE CODE THAT COMPLETES THE TASK USING THE CODEBASE INTERFACE. JUST ONE CODEBOX IS NEEDED
    - SAY WITH SPEECH MODULE WHEN YOU START DOING SOMETHING AND WHEN YOU FINISH DOING SOMETHING
    - MANDATORY: USE THE SPEECH MODULE TO SAY WHAT YOU ARE DOING
    - MANDATORY: IF YOU BELIEVE THE GIVEN TASK CANNOT BE DONE, JUST RETURN A PYTHON CODE SPEAKING "I CANNOT DO THAT TASK"

    # Task Description:

    {task_input}

    # Codebase Interface:

    {task_module_code}

    # Code to generate:
    """

    return generate_text(text_prompt, system_message=system_message)

if __name__ == "__main__":
    openai.api_key = "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"

    task = "Could you enter to the living room, locate the fruits, and hand it to Elizabeth at the dining table. Fruits: apple"
    #task = input("Write the task: ")
    print(task)
    code = generate_code(task)


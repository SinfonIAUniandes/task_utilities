import openai
import os

## TODO: Get a working OpenAI API key

codebase = """"""

def generate_text(text_prompt, system_message=None, model="gpt-3.5-turbo", temperature=0):

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
    return prediction['choices'][0]['message']['content']


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

    # Details about the code to generate:
    - The code must be written in python and the output will be executed directly
    - The code is going to be executed in a ROS node, so there is no need to initialize ROS
    - The Task_module class is allready instantiated as `self.tm = task_module.Task_module(perception = True,speech=True,manipulation=True, navigation=True)`
    - Use only self.tm.<function_name> to call the functions of the codebase interface
    - Only use the functions of the codebase interface that are needed to complete the task and callbacks of the given ros topics
    - The initialize_node function is allready called for you, you cannot call it again
    - The code cannot include the original codebase interface, it is only for using its functions
    - Remember to initialize and dispose of every sensor in case you need them, for example calling `self.tm.turn_camera("front_camera","custom",1,15)`
    - Return only the code, just code, your output is going to be saved in a variable and executed with exec(<your answer>)

    # Task Description:

    {task_input}

    # Codebase Interface:

    {task_module_code}

    # Code to generate:
    """

    return generate_text(text_prompt, system_message=system_message)

if __name__ == "__main__":
    openai.api_key = os.environ["OPENAI_API_KEY"]
    print(generate_code("Go to the living room and ask the person his name, then go to the door and say something funny of the person name"))
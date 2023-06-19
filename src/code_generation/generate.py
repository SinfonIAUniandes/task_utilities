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
    - The code utilizes the rospy library to communicate with the robot
    - The interface is already given as an importable python module, the import sentence is: `import task_module`
    - The Task_module class must be instantiated in order to use the codebase interface only once `tm = task_module.Task_module(perception = True,speech=True,manipulation=True, navigation=True)`
    - The initialize_node function must be called before any other function in the codebase interface, this turns on a ros node
    - The code cannot include the original codebase interface, it is given that the codebase interface is instantiated as the `tm` instance
    - No calls to rospy are allowed with the exception of rospy.sleep() calls, everything must be called through the codebase interface
    - Do not use the time library to sleep, use rospy.sleep() instead
    - Remember to initialize and dispose of every sensor in case you need them

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
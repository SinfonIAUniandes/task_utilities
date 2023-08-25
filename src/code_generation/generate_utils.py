import openai
import re
import os

codebase = """"""

def get_task_module_code()-> str:
    global codebase
    if codebase == "":
        with open(os.path.join(os.path.dirname(__file__), "task_module_interface.py"), "r") as f:
            codebase = f.read()
    return codebase

def generate_openai(text_prompt, system_message=None, model="gpt-3.5-turbo-16k", temperature=0):

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
    return code

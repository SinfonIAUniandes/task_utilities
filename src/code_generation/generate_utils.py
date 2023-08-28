import openai
import tiktoken
import re
import os

codebase = """"""

def get_task_module_code()-> str:
    global codebase
    if codebase == "":
        with open(os.path.join(os.path.dirname(__file__), "task_module_interface.py"), "r") as f:
            codebase = f.read()
    return codebase

def count_tokens(string: str, encoding_name: str = "cl100k_base") -> int:
    """Returns the number of tokens in a text string."""
    encoding = tiktoken.get_encoding(encoding_name)
    num_tokens = len(encoding.encode(string))
    return num_tokens

def generate_openai(text_prompt, system_message=None, code=True, model="gpt-3.5-turbo-16k", temperature=0):

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
    answer = prediction['choices'][0]['message']['content']
    if code:
        pattern = r'```python(.*?)\n```'
        try:
            code = (re.search(pattern, answer, re.DOTALL).group(1)).strip()
        except AttributeError:
            code = """self.tm.talk("I cannot do this command")"""
        return code
    else:
        return answer

def generate_azure(text_prompt, system_message=None, deployment_name=None, temperature=0):
    messages = [
        {"role": "user", "content": text_prompt}
    ]

    if system_message:
        messages = [
            {"role": "system", "content": system_message},
        ] + messages

    prediction = openai.ChatCompletion.create(
        engine=deployment_name,
        temperature=temperature,
        messages=messages
    )
    answer =prediction['choices'][0]['message']['content']
    pattern = r'```python(.*?)\n```'
    try:
        code = (re.search(pattern, answer, re.DOTALL).group(1)).strip()
    except AttributeError:
        code = """self.tm.talk("I cannot do this command")"""
    return code
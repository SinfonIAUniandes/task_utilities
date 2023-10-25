import openai
import tiktoken
import re
import os
import config as cfg
from database.models import Model
from requests.exceptions import JSONDecodeError
import requests
codebase = """"""

CONFIG_PATH = os.path.join(cfg.CONFIG_PATH, "robot_vars.csv")
TASK_VARS = None

def load_task_config()->dict:
    global TASK_VARS
    if TASK_VARS is None:
        config = {}
        with open(CONFIG_PATH,"r") as f:
            for line in f:
                splitted_line = line.split(":")
                key = splitted_line[0]
                value = splitted_line[1].strip().split(",")
                config[key] = value
        TASK_VARS = config
    return TASK_VARS

def load_code_gen_config():
    openai.api_type = os.getenv("OPENAI_API_TYPE", "open_ai")
    openai.api_version = os.getenv("OPENAI_API_VERSION", "2023-05-15" if openai.api_type in ("azure", "azure_ad", "azuread") else None)

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

def generate_response(text_prompt, system_message=None, is_code=True, model="gpt-3.5-turbo-16k", model_type= Model.GPT35, temperature=0):
    messages = [
        {"role": "user", "content": text_prompt}
    ]

    if system_message:
        messages = [
            {"role": "system", "content": system_message},
        ] + messages
    if model_type == Model.LLAMA2:
        response = requests.post("http://localhost:6969/llama2", json={"messages": messages})
        try:
            answer = response.json()[0]["generation"]["content"].rstrip()
        except JSONDecodeError:
            print("Error decoding json response from LLAMA2, please check that the server is running and that the model is loaded correctly")
            return None

    elif openai.api_version is None:
        prediction = openai.ChatCompletion.create(
            model=model,
            temperature=temperature,
            messages=messages
        )
    else:
        api_key = os.getenv("OPENAI3_KEY") if model_type == Model.GPT35 else os.getenv("OPENAI4_KEY")
        api_base = os.getenv("OPENAI3_API_BASE", "https://api.openai.com/v1") if model_type == Model.GPT35 else os.getenv("OPENAI4_API_BASE", "https://api.openai.com/v1")
        deployment_name=os.getenv("OPENAI3_DEPLOYMENT_NAME", "gpt-35-turbo") if model_type == Model.GPT35 else os.getenv("OPENAI4_DEPLOYMENT_NAME", "gpt-4")
        prediction = openai.ChatCompletion.create(
            api_key=api_key,
            api_base=api_base,
            engine=deployment_name,
            temperature=temperature,
            messages=messages
        )
    if model_type != Model.LLAMA2:
        answer = prediction['choices'][0]['message']['content']
    if is_code:
        pattern = r'```python(.*?)```'
        try:
            return (re.search(pattern, answer, re.DOTALL).group(1)).strip()
        except AttributeError:
            pass
        try:
            pattern = r'```(.*?)```'
            return (re.search(pattern, answer, re.DOTALL).group(1)).strip()
        except AttributeError:
            return f"""#Error while getting response from model {model_type.value}. Response had an incorrect format.\n#The response was:\n{f'{answer}'}"""
    else:
        return answer
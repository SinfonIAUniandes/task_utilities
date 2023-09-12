import openai
import tiktoken
import re
import os

codebase = """"""

dirname, _ = os.path.split(__file__)
CONFIG_PATH = os.path.join(dirname, "configs/robot_vars.csv")
TASK_VARS = None

def load_task_config()->dict:
    if TASK_VARS is None:
        config = {}
        with open(CONFIG_PATH,"r") as f:
            for line in f:
                splitted_line = line.split(":")
                key = splitted_line[0]
                value = splitted_line[1].strip().split(",")
                config[key] = value
        global TASK_VARS
        TASK_VARS = config
    return TASK_VARS

def load_code_gen_config():
    openai.api_key = os.getenv("OPENAI_KEY")
    openai.api_base = os.getenv("OPENAI_API_BASE", "https://api.openai.com/v1")
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

def generate_gpt(text_prompt, system_message=None, is_code=True, model="gpt-3.5-turbo-16k", temperature=0):

    messages = [
        {"role": "user", "content": text_prompt}
    ]

    if system_message:
        messages = [
            {"role": "system", "content": system_message},
        ] + messages

    if openai.api_version is None:
        prediction = openai.ChatCompletion.create(
            model=model,
            temperature=temperature,
            messages=messages
        )
    else:
        deployment_name=os.getenv("OPENAI_DEPLOYMENT_NAME", "gpt-35-turbo")
        prediction = openai.ChatCompletion.create(
            engine=deployment_name,
            temperature=temperature,
            messages=messages
        )
    answer = prediction['choices'][0]['message']['content']
    if is_code:
        pattern = r'```python(.*?)\n```'
        try:
            code = (re.search(pattern, answer, re.DOTALL).group(1)).strip()
        except AttributeError:
            code = """self.tm.talk("I cannot do this command")"""
        return code
    else:
        return answer
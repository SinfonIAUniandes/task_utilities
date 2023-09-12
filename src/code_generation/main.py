from database.config import init_db
from dotenv import load_dotenv
from generate_utils import load_code_gen_config

load_dotenv()
init_db()
load_code_gen_config()

import time
import json
from traceback import format_exception
from chain_generate import generate_code as chain_gen
from generate import generate_code as gen
from database.models import Model, PromptingType, ExecutionResults
from database.crud import get_non_executed_tests, update_test

def evaluate_automated_tests(num_tests:int=10, prompting_type = PromptingType.CHAINING, model=Model.GPT35):
    tasks = get_non_executed_tests(num_tests)
    for task in tasks:
        description = task.task
        model_response = None
        t1 = time.time()
        if prompting_type == PromptingType.CHAINING:
            steps, code = chain_gen(description, model)
            model_response = json.dumps({"steps": steps, "code": code})
        else:
            code = gen(description, model)
            model_response = json.dumps({"code": code})
        t2 = time.time()

        generation_time = t2-t1
        exception_message = None
        execution = ExecutionResults.NOT_EXECUTED
        try:
            exec(code)
            execution = ExecutionResults.EXECUTED
        except Exception as e:
            execution = ExecutionResults.EXECUTED_BUT_FAILED
            exception_message = format_exception(type(e), e, e.__traceback__)

        task.model_response = model_response
        task.task_execution_result = execution
        task.raised_error = exception_message
        task.generation_time_ms = generation_time
        update_test(task)

if __name__ == "__main__":
    evaluate_automated_tests()

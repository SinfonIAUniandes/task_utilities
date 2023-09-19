import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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
from database.models import Model, PromptingType, ExecutionResults, PepperTest
from database.crud import get_non_executed_tests, update_test, create_test
from task_generation.generate import generate_category_tasks

import test_module

class CodeGeneration:

    def __init__(self):
        self.tm = test_module.Task_module(perception = True, speech = True, manipulation = True, navigation = True)
        
    def evaluate_automated_tests(self,num_tests:int=10, prompting_type = PromptingType.CHAINING, model=Model.GPT35):
        print("Fetching non executed tests...")
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
            execution = ExecutionResults.NOT_EXECUTED
            try:
                exec(code)
                execution = ExecutionResults.PASSED_AUTOMATIC_EXECUTION
            except Exception as e:
                execution = ExecutionResults.EXECUTED_BUT_FAILED
                task.exception_traceback = "".join(format_exception(type(e), e, e.__traceback__))
                task.exception_type = type(e).__name__

            task.model_name = model.value
            task.prompting_type = prompting_type.value
            task.model_response = model_response
            task.task_execution_result = execution
            task.generation_time_ms = generation_time
            update_test(task)
        print("Finished evaluating tests")

    def create_new_tasks(self):
        
        print("Creating new tasks...")
        tasks = generate_category_tasks(10)
        for task_category in tasks:
            for task in task_category[0]:
                description = task
                category = task_category[1]
                task = PepperTest(task=description, task_category=category.value)
                create_test(task)
            print("Created tasks for category: ", category.value)
        print("Created new tasks successfully")

if __name__ == "__main__":
    cg = CodeGeneration()
    cg.evaluate_automated_tests(num_tests=5)
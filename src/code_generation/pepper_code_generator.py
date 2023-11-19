import time
import json
from tqdm import tqdm
from traceback import format_exception
from code_generation.ls_generate import LongStringGenerator
from chain_generate import ChainGenerator
from database.models import Model, PromptingType, ExecutionResults, PepperTest
from database.crud import get_non_executed_tests, update_test, create_test
from task_generation.generate import generate_category_tasks

import test_module

class CodeGenerator:

    def __init__(self):
        self.tm = test_module.Task_module(perception = True, speech = True, manipulation = True, navigation = True)

    def evaluate_automated_tests(self,num_tests:int=10, model=Model.GPT35):
        print("Fetching non executed tests...")
        with tqdm(total=num_tests*len(list(PromptingType)), desc="Evaluating tests: ") as pbar:
            count = 0
            total_time1 = time.time()
            for prompting_type in list(PromptingType):
                tasks = get_non_executed_tests(model, prompting_type, limit=num_tests)
                if prompting_type == PromptingType.CHAINING:
                    cg = ChainGenerator()
                else:
                    cg = LongStringGenerator()
                for i in range(len(tasks)):
                    task = tasks[i]
                    description = task.task
                    model_response = None
                    t1 = time.time()
                    if prompting_type == PromptingType.CHAINING:
                        entities,new_entities,new_task,steps,code = cg.generate_code(description, model)
                        model_response = json.dumps({"entities": entities, "new_entities": new_entities, "new_task": new_task, "steps": steps, "code": code})
                    else:
                        code = cg.generate_code(description, model)
                        model_response = json.dumps({"code": code})
                    t2 = time.time()

                    generation_time = t2-t1
                    execution = ExecutionResults.NOT_EXECUTED
                    try:
                        exec(code)
                        execution = ExecutionResults.PASSED_AUTOMATIC_EXECUTION
                    except IndentationError:
                        try:
                            code = code.replace('    '*2, '')
                            exec(code)
                            execution = ExecutionResults.PASSED_AUTOMATIC_EXECUTION
                        except Exception as e:
                            execution = ExecutionResults.EXECUTED_BUT_FAILED
                            task.exception_traceback = "".join(format_exception(type(e), e, e.__traceback__))
                            task.exception_type = type(e).__name__
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
                    count += 1
                    pbar.update(1)
            if count != num_tests*len(list(PromptingType)):
                pbar.update(num_tests*len(list(PromptingType))-count)
            total_time2 = time.time()
            time_elapsed = total_time2 - total_time1
            input(f"\nFinished evaluating tests. Took {time_elapsed} seconds or {time_elapsed/60} minutes in total.\nPress any key to go back...")

    def create_new_tasks(self):
        inp = input("Are you sure you want to create new tasks? This will populate the database (Y for yes, anything else to quit): ")
        if inp.lower() == "y":
            count = 0
            print("Creating new tasks...")
            tasks_tuple = generate_category_tasks(15)
            for tuple in tasks_tuple:
                for task in tuple[0]:
                    category = tuple[1]
                    for prompting_type in list(PromptingType):
                        for model in list(Model):
                            description = task
                            task_gpt = PepperTest(task=description, task_category=category.value, model_name=model.value, prompting_type=prompting_type.value)
                            count += 1
                            create_test(task_gpt)
                print("Created tasks for category: ", category.value)
            input(f"Created {count} new tasks succesfully. Press any key to continue...")
        else:
            input("Did not create new tasks. Press any key to continue...")
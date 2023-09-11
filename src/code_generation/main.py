from database.config import init_db
from dotenv import load_dotenv
import time
import json
from chain_generate import generate_code as chain_gen
from generate import generate_code as gen 
from database.models import Model, PromptingType, ExecutionResults

def test(num_tests:int=10, prompting_type = PromptingType.CHAINING, model=Model.GPT35):
    tasks = ["",""] #Traer todas las vaicas con un limit (<num_test>) TODO
    for task in tasks:
        id = task.id
        description = task.task
        model_response = None
        if prompting_type == PromptingType.CHAINING:
            t1 = time.time()
            steps, code = chain_gen(description) #TODO add model name to function and load model
            model_response = json.dumps({"steps": steps, "code": code})
            t2 = time.time()
        else:
            t1 = time.time()
            code = gen(description) #TODO add model name to function and load model 
            model_response = json.dumps({"code": code})
            t2 = time.time()
        generation_time = t2-t1
        exception = None
        execution = ExecutionResults.NOT_EXECUTED
        try:
            exec(code)
            execution = ExecutionResults.EXECUTED
        except Exception as e:
            execution = ExecutionResults.EXECUTED_BUT_FAILED
            exception = e
        results = {"id": id, "model_name": model, "prompting_type": prompting_type, "task": description, "model_response": model_response, "task_execution_result": execution, "raised_error": exception, "generation_time_ms": generation_time}
        # save results in database TODO



if __name__ == "__main__":
    load_dotenv()
    init_db()
    from database.crud import get_test
    print("Database initialized successfully!")
    get_test("1cc583ad-3595-4079-8d60-3fd76035f727")

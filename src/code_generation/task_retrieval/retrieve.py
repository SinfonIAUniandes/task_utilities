from database.crud import get_succesfully_executed_tests
import json
import os

def write_tasks_to_file():
    tasks = get_succesfully_executed_tests()
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "tasks.txt"), "w") as f:
        for task in tasks:
            task_code = json.loads(task.model_response)["code"]
            f.write(f"Task ID: {task.id}" + "\n")
            f.write(f"Task: {task.task}" + "\n")
            f.write(f"Model: {task.model_name}" + "\n")
            f.write(f"{task_code}" + "\n")
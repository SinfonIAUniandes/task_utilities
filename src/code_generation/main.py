import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from consolemenu import ConsoleMenu
from consolemenu.items import FunctionItem

from database.config import init_db
from dotenv import load_dotenv
from generate_utils import load_code_gen_config
from database.models import Model, ExecutionResults
from view.menu import *
import json

load_dotenv()
init_db()
load_code_gen_config()
from database.crud import get_test, update_test
from pepper_code_generator import CodeGenerator

cg = CodeGenerator()

def automatically_evaluate_tests():
    options = ["GPT3.5", "GPT4", "LLAMA2"]
    model = get_select_menu(options, "Select an LLM model")

    if model == 0:
        model = "GPT35"
    elif model == 1:
        model = "GPT4"
    elif model == 2:
        model = "LLAMA2"
    else:
        return
    done = False
    while not done:
        num_tests = input("How many tests do you want to evaluate? (Default 10): ")
        if num_tests == "":
            num_tests = 10
        else:
            try:
                num_tests = int(num_tests)
                if num_tests <= 0 or num_tests > 100:
                    raise TypeError
            except TypeError:
                print("Please enter a valid number of tests")
                continue
        cg.evaluate_automated_tests(num_tests=num_tests, model=Model[model])
        done = True

def manually_evaluate_tasks():
    options = ["Evaluate random task", "Evaluate specific task by ID"]
    selection = get_select_menu(options, "Select an option to evaluate")

    if selection == 0:
        #TODO Implement random task evaluation
        input("Not implemented yet, press any key to go back to the main menu...")
        return
    elif selection == 1:
        task_id = None
        while True:
            task_id = input("Enter the task UUID (q to go back): ")
            if task_id == "q":
                return
            try:
                print(f"Getting task with id: {task_id}\n")
                task = get_test(task_id)
                if task.task_execution_result != ExecutionResults.PASSED_AUTOMATIC_EXECUTION:
                    input("This task has not been successfully executed automatically yet. Press any key to go back to the main menu...")
                    return
                else:
                    break
            except Exception as e:
                print(e)


        eval_options = ["Executed but failed", "Partially completed task","Completed task successfully", "SAVE TASK TO FILE!"]
        while True:
            code = json.loads(task.model_response)['code']
            print(f"Task:\n{task.task}\n")
            print(f"Code:\n\n{code}")

            selection_index = get_select_menu(eval_options, "Select an option to evaluate", clear_screen=False)
            if selection_index == len(eval_options):
                return
            elif selection_index == 3:
                save_code_to_file(code, task.task, task.id)
                del eval_options[-1]
                continue
            selection = eval_options[selection_index]
            confirm = ""

            while confirm.lower() != "y" and confirm.lower() != "n":
                confirm = input(f"\nAre you sure to evaluate the task as {selection}? (y/n): ")
                if confirm == "y":
                    print(f"Updating task with id: {task_id} as {selection}")
                    task.task_execution_result = list(ExecutionResults)[selection_index+2]
                    update_test(task)
                    print("Successfully updated task\n")

            if confirm == "n":
                continue
            input("Press any key to go back to the main menu...")
            return

if __name__ == "__main__":
    # Create the menu
    menu = ConsoleMenu("Pepper Code Generation", "Subtitle")
    evaluate_tests = FunctionItem("Automatically Evaluate Tests", automatically_evaluate_tests)
    create_new_tests = FunctionItem("Create New Tests", cg.create_new_tasks)
    manually_evaluate = FunctionItem("Manually Evaluate Tests", manually_evaluate_tasks)

    # Once we're done creating them, we just add the items to the menu
    menu.append_item(evaluate_tests)
    menu.append_item(manually_evaluate)
    menu.append_item(create_new_tests)

    # Finally, we call show to show the menu and allow the user to interact
    menu.show()
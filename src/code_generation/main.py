import sys
import os
import pickle
import config as cfg
from getpass import getpass
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from consolemenu import ConsoleMenu
from consolemenu.items import FunctionItem

from database.config import init_db
from dotenv import load_dotenv
from generate_utils import load_code_gen_config
from database.models import Model, ExecutionResults

load_dotenv()
init_db()
load_code_gen_config()

from database.crud import get_test, get_random_passed_auto_test, update_test, sign_in
from pepper_code_generator import CodeGenerator
from view.menu import get_select_menu, evaluate_task

cg = CodeGenerator()

def automatically_evaluate_tasks_menu():
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
        num_tests = input("How many tests do you want to evaluate (Per prompting type)? (Default 10, q to quit): ")
        if num_tests == "":
            num_tests = 10
        elif num_tests == "q":
            return
        else:
            try:
                num_tests = int(num_tests)
                if num_tests <= 0 or num_tests > 1000:
                    raise TypeError
            except (TypeError, ValueError):
                print("Please enter a valid number of tests")
                continue
        cg.evaluate_automated_tests(num_tests=num_tests, model=Model[model])
        done = True


def evaluate_manually():
    print("Getting a random task to evaluate...\n")
    task = get_random_passed_auto_test()
    if task is None:
        input("No tasks found to evaluate. Press any key to go back to the main menu...")
        return None
    evaluate_task(task, update_test)
    return True

def manually_evaluate_tasks_menu():
    try:
        path = os.path.join(cfg.CONFIG_PATH, "config.pkl")
        with open(path, "rb") as f:
            user = pickle.load(f)
        cfg.CURRENT_USER = sign_in(user.username, user.password, dump=False)
    except FileNotFoundError:
        print("You need to log in first to evaluate tasks manually")
        while True:
            username = input("Enter your username (q to quit): ")
            if username == "q":
                return
            password = getpass("Enter your password (q to quit) [Input will be invisible]: ")
            if password == "q":
                return
            try:
                cfg.CURRENT_USER = sign_in(username, password)
                break
            except Exception as e:
                print(e)
                continue
        input(f"Logged in as {cfg.CURRENT_USER.username} successfully. Press enter to continue...")

    options = ["Evaluate random task", "Evaluate specific task by ID", "Log out/Switch user"]
    selection = get_select_menu(options, "Select an option to evaluate")

    if selection == 0:
        task_found = evaluate_manually()
        if task_found is None:
            return
        continue_evaluating = "y"
        while continue_evaluating == "y":
            continue_evaluating = input("Do you want to continue evaluating tasks? (y/n): ")
            if continue_evaluating == "y":
                task_found = evaluate_manually()
            elif continue_evaluating == "":
                print("Please enter y or n.")
                continue
            elif continue_evaluating != "y" and continue_evaluating != "n":
                print(f"{continue_evaluating} is an invalid option. Please enter y or n")
                continue
            if task_found is None:
                return
        input("Press any key to go back to the main menu...")
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
            evaluate_task(task)
    elif selection == 2:
        cfg.CURRENT_USER = None
        try:
            path = os.path.join(cfg.CONFIG_PATH, "config.pkl")
            os.remove(path)
        except FileNotFoundError:
            pass
        return
if __name__ == "__main__":
    #Create the menu
    menu = ConsoleMenu("Pepper Code Generation", "Console Menu to evaluate and create new tests")
    evaluate_tests = FunctionItem("Automatically Evaluate Tests", automatically_evaluate_tasks_menu)
    create_new_tests = FunctionItem("Create New Tests", cg.create_new_tasks)
    manually_evaluate = FunctionItem("Manually Evaluate Tests", manually_evaluate_tasks_menu)

    # Once we're done creating them, we just add the items to the menu
    menu.append_item(evaluate_tests)
    menu.append_item(manually_evaluate)
    menu.append_item(create_new_tests)

    # Finally, we call show to show the menu and allow the user to interact
    menu.show()
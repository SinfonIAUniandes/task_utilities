import os
import json
from consolemenu import SelectionMenu
from code_generation.database.models import ExecutionResults, PepperTest

def get_select_menu(options, title, clear_screen=True):
    selection_menu = SelectionMenu(options, title=title, clear_screen=clear_screen)
    selection_menu.show()
    selection_menu.join()
    return selection_menu.selected_option

def save_code_to_file(code, task, task_id, path=os.getcwd()):
    filepath = path+f"/task_{str(task_id)}.txt"
    with open(filepath, "w") as f:
        f.write(f"Task: {task}\nCode:\n\n")
        f.write(code)
    print(f"Successfully saved task to file to: \n{filepath}\n")
    input("Press any key to evaluate...")

def evaluate_task(task: PepperTest, update_test: callable):
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
                print(f"Updating task with id: {task.id} as {selection}")
                task.task_execution_result = list(ExecutionResults)[selection_index+2].value
                update_test(task)
                print("Successfully updated task\n")
            elif confirm != "y" and confirm != "n":
                print(f"{confirm} is an invalid option. Please enter y or n")
                continue

        if confirm == "n":
            continue
        input("Press any key to go back or continue evaluating...")
        return
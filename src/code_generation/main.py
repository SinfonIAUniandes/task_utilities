import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from consolemenu import ConsoleMenu, SelectionMenu
from consolemenu.items import FunctionItem

from database.config import init_db
from dotenv import load_dotenv
from generate_utils import load_code_gen_config
from database.models import Model

load_dotenv()
init_db()
load_code_gen_config()
from pepper_code_generator import CodeGenerator

cg = CodeGenerator()

def automatically_evaluate_tests():
    options = ["GPT3.5", "GPT4", "LLAMA2"]
    selection_menu = SelectionMenu(options, title="Select an LLM model")
    selection_menu.show()
    selection_menu.join()
    model = selection_menu.selected_option
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



if __name__ == "__main__":

    # Create the menu
    menu = ConsoleMenu("Pepper Code Generation", "Subtitle")
    evaluate_tests = FunctionItem("Automatically Evaluate Tests", automatically_evaluate_tests)
    create_new_tests = FunctionItem("Create New Tests", cg.create_new_tasks)
    #TODO: Implement manual test evaluation
    #manually_evaluate = FunctionItem("Manually Evaluate Tests", cg.evaluate_manually)

    # Once we're done creating them, we just add the items to the menu
    menu.append_item(evaluate_tests)
    menu.append_item(create_new_tests)

    # Finally, we call show to show the menu and allow the user to interact
    menu.show()
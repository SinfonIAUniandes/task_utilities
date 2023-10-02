import os
from consolemenu import SelectionMenu

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
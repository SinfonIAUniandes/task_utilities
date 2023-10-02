from consolemenu import SelectionMenu

def get_select_menu(options, title, clear_screen=True):
    selection_menu = SelectionMenu(options, title=title, clear_screen=clear_screen)
    selection_menu.show()
    selection_menu.join()
    return selection_menu.selected_option
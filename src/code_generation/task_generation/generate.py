import re
import os
import subprocess
from code_generation.database.models import TaskCategory

GENERATE_PATH = os.path.dirname(os.path.abspath(__file__))

def extract_tasks_from_category(path:str):
    with open(path, 'r') as f:
        tasks_file = f.read()
    search = re.findall(r'\$(.*?)\$', tasks_file)

    search = [s.strip() for s in search if s.strip() != '']
    return search

def generate_category_tasks(n_tasks: int):
    global GENERATE_PATH
    gpsr_proc = subprocess.Popen([os.path.join(GENERATE_PATH, 'GPSRGenerator/GPSRCmdGen.exe'), '--bulk', f'{n_tasks}'], cwd=GENERATE_PATH)
    egpsr_proc = subprocess.Popen([os.path.join(GENERATE_PATH, 'GPSRGenerator/EEGPSRCmdGen.exe'), '--bulk', f'{n_tasks}'], cwd=GENERATE_PATH)
    gpsr_proc.wait()
    egpsr_proc.wait()

    categories = [cat for cat in TaskCategory]

    tasks = list()

    for category in categories:
        category_no = category.value[-1]
        if "EGSPR" in category.value:
            task_path = os.path.join(GENERATE_PATH, f'EEGPSR Examples/EEGPSR Examples Cat {category_no}.txt')
        else:
            task_path = os.path.join(GENERATE_PATH, f'GPSR Examples/GPSR Examples Cat {category_no}.txt')

        extracted_tasks = extract_tasks_from_category(task_path)
        tasks.append((extracted_tasks, category))

    return tasks
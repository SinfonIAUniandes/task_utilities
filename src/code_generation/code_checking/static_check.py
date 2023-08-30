import os
import subprocess

def static_check(file_content: str):
    """Check if the generated code is valid according to python syntax."""
    # Getting the path of the temp folder
    dirname, _ = os.path.split(__file__)
    dirname = os.path.join(dirname, "temp")
    # Creating a temporary file with the generated code
    path = dirname + "/code.py"
    with open(path, mode="w") as f:
        f.write(file_content)
    # Checking if the code is valid with pyflakes
    process = subprocess.run(["pyflakes", path], capture_output=True)
    result = process.stdout.decode("utf-8")
    os.remove(path)
    if result == "":
        return True
    else:
        return False

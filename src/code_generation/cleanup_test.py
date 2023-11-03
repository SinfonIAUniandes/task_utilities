import sys
import os
import json
from sqlalchemy import cast, String
from sqlalchemy.orm import Session
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from database.config import init_db
from dotenv import load_dotenv
from generate_utils import load_code_gen_config
from database.models import PepperTest

load_dotenv()
init_db()
load_code_gen_config()

from database.config import engine
import traceback

import test_module

class CodeGenerator:

    def __init__(self):
        self.tm = test_module.Task_module(perception = True, speech = True, manipulation = True, navigation = True)

    def start(self):
        with Session(bind=engine) as session:
            test = session.query(PepperTest).filter(PepperTest.model_name == "LLAMA2", cast(PepperTest.model_response, String).not_like("%#Error%"), cast(PepperTest.model_response, String).not_like("%I am sorry%"),).first()
            original_string = json.loads(test.model_response)["code"]
            try:
                exec(original_string)
                print("Success")
            except IndentationError:
                code = code.replace('    '*2, '')
                exec(code)
                print("Success")
            except Exception as e:
                traceback.print_exc()

if __name__ == "__main__":
    cg = CodeGenerator()
    cg.start()
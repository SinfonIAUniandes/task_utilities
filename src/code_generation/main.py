import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from database.config import init_db
from dotenv import load_dotenv
from generate_utils import load_code_gen_config

load_dotenv()
init_db()
load_code_gen_config()
from pepper_code_generator import CodeGenerator


if __name__ == "__main__":
    # Test evaluation
    cg = CodeGenerator()
    cg.evaluate_automated_tests(num_tests=2)
from sqlalchemy.orm import Session
from .config import engine
from .models import PepperTest
from .utils.validators import is_valid_uuid

def get_test(test_id: str):
    if not is_valid_uuid(test_id):
        raise Exception(f"Test id {test_id} is not a valid UUID")
    with Session(bind=engine) as session:
        test = session.query(PepperTest).filter(PepperTest.id == test_id).first()
        if not test:
            raise Exception(f"Test with id {test_id} was not found")
        return test

def get_random_passed_auto_test():
    with Session(bind=engine) as session:
        test = session.query(PepperTest).filter(PepperTest.task_execution_result == "PASSED_AUTOMATIC_EXECUTION").first()
        return test

def get_non_executed_tests(model, prompting_type, limit=10):
    with Session(bind=engine) as session:
        tests = session.query(PepperTest).filter(PepperTest.task_execution_result == "NOT_EXECUTED", PepperTest.model_name == model.value, PepperTest.prompting_type == prompting_type.value).limit(limit).all()
        return tests

def get_succesfully_executed_tests(limit=10):
    with Session(bind=engine) as session:
        tests = session.query(PepperTest).filter(PepperTest.task_execution_result == "PASSED_AUTOMATIC_EXECUTION").limit(limit).all()
        return tests

def create_test(test: PepperTest):
    with Session(bind=engine) as session:
        session.add(test)
        session.commit()
        session.refresh(test)
        return test

def update_test(test: PepperTest):
    with Session(bind=engine) as session:
        session.add(test)
        session.commit()
        return test


def delete_test(test_id: str):
    if not is_valid_uuid(test_id):
        raise Exception(f"Test id {test_id} is not a valid UUID")
    with Session(bind=engine) as session:
        test = session.query(PepperTest).filter(PepperTest.id == test_id).first()
        if not test:
            raise Exception(f"Test with id {test_id} was not found")
        test.delete()
        session.commit()
        return test
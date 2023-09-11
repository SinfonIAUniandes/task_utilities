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

def get_non_executed_tests(limit=10):
    with Session(bind=engine) as session:
        tests = session.query(PepperTest).filter(PepperTest.model_response == None).limit(limit).all()
        return tests

def create_test(test: PepperTest):
    with Session(bind=engine) as session:
        session.add(test)
        session.commit()
        session.refresh(test)
        return test


def update_test(test: PepperTest):
    with Session(bind=engine) as session:
        test.update()
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
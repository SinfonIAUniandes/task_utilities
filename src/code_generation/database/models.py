import uuid
from sqlalchemy import Column, String, Float
from sqlalchemy.orm import DeclarativeBase
from sqlalchemy.dialects.postgresql import UUID, ENUM, JSON
from enum import Enum

class Base(DeclarativeBase):
    pass

class PromptingType(Enum):
    LONG_STRING = 'LONG_STRING'
    CHAINING = 'CHAINING'

class ExecutionResults(Enum):
    NOT_EXECUTED = 'NOT_EXECUTED'
    PASSED_AUTOMATIC_EXECUTION = 'PASSED_AUTOMATIC_EXECUTION'
    EXECUTED_BUT_FAILED = 'EXECUTED_BUT_FAILED'
    PARTIALLY_COMPLETED = 'PARTIALLY_COMPLETED'
    SUCCESFULLY_COMPLETED = 'SUCCESFULLY_COMPLETED'

class Model(Enum):
    GPT35 = 'GPT35'
    LLAMA = 'LLAMA2'

class TaskCategory(Enum):
    GSPR1 = 'GSPR1'
    GSPR2 = 'GSPR2'
    GSPR3 = 'GSPR3'
    EGSPR1 = 'EGSPR1'
    EGSPR2 = 'EGSPR2'
    EGSPR3 = 'EGSPR3'
    EGSPR4 = 'EGSPR4'
    EGSPR5 = 'EGSPR5'
    EGSPR6 = 'EGSPR6'

class PepperTest(Base):
    __tablename__ = 'pepper_tests'
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    model_name = Column(ENUM(Model), nullable=False, default=Model.GPT35.value)
    prompting_type = Column(ENUM(PromptingType), nullable=True, default=PromptingType.CHAINING.value)
    task = Column(String(255), nullable=False)
    task_category = Column(ENUM(TaskCategory), nullable=False)
    model_response = Column(JSON, nullable=True)
    task_execution_result = Column(ENUM(ExecutionResults), nullable=False, default=ExecutionResults.NOT_EXECUTED.value)
    exception_traceback = Column(String(255), nullable=True)
    exception_type = Column(String(255), nullable=True)
    generation_time_ms = Column(Float, nullable=True)
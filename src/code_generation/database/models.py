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
    EXECUTED = 'PASSED_AUTOMATIC_EXECUTION'
    EXECUTED_BUT_FAILED = 'EXECUTED_BUT_FAILED'
    PARTIALLY_COMPLETED = 'PARTIALLY_COMPLETED'
    SUCCESFULLY_COMPLETED = 'SUCCESFULLY_COMPLETED'

class Model(Enum):
    GPT35 = 'GPT3.5'
    LLAMA = 'LLAMA2'

class PepperTest(Base):
    __tablename__ = 'pepper_tests'
    id = Column(UUID(as_uuid=True), primary_key=True)
    model_name = Column(ENUM(Model), nullable=False)
    prompting_type = Column(ENUM(PromptingType), nullable=True)
    task = Column(String(255), nullable=False)
    model_response = Column(JSON, nullable=True)
    task_execution_result = Column(ENUM(ExecutionResults), nullable=False)
    raised_error = Column(String(255), nullable=True)
    generation_time_ms = Column(Float, nullable=True)
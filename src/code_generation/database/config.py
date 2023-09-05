import os
from sqlalchemy import create_engine
from .models import Base

def init_db():
    DATABASE_URL = f'postgresql+psycopg2://{os.getenv("POSTGRES_USER")}:{os.getenv("POSTGRES_PASSWORD")}@{os.getenv("POSTGRES_HOST")}/{os.getenv("POSTGRES_DBNAME")}'
    engine = create_engine(DATABASE_URL)
    Base.metadata.create_all(engine)
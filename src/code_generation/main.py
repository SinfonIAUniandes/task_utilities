from database.config import init_db
from dotenv import load_dotenv

if __name__ == "__main__":
    load_dotenv()
    init_db()
    print("Database initialized successfully!")
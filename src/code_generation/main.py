from database.config import init_db
from dotenv import load_dotenv


if __name__ == "__main__":
    load_dotenv()
    init_db()
    from database.crud import get_test
    print("Database initialized successfully!")
    get_test("1cc583ad-3595-4079-8d60-3fd76035f727")
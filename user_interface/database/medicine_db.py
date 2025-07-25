from sqlmodel import Session, SQLModel, create_engine

DATABASE_URL = "sqlite:///./hospital.db"  # 共用資料庫
engine = create_engine(DATABASE_URL, echo=True)

def init_db():
    SQLModel.metadata.create_all(engine)  # OK！現在所有表都建在同一個 DB

def get_session():
    return Session(engine)

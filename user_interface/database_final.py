"""
醫院藥物管理系統 - 最終版資料庫配置
完全乾淨版本：無測試資料、無自動模擬
"""

from sqlalchemy import create_engine, Column, Integer, String, Text, Float, DateTime, Boolean, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from datetime import datetime

 最終版資料庫配置
DATABASE_URL = "sqlite:///./hospital_medicine_final.db"

engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

 基本藥物表
class Medicine(Base):
    __tablename__ = "medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(), nullable=False, index=True)
    amount = Column(Integer, default=)
    position = Column(String(), nullable=False)
    manufacturer = Column(String())
    dosage = Column(String())
    is_active = Column(Boolean, default=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
     關聯到詳細資料
    detail = relationship("MedicineDetail", back_populates="medicine", uselist=False)

 詳細藥物表
class MedicineDetail(Base):
    __tablename__ = "medicine_detailed"
    
    id = Column(Integer, primary_key=True, index=True)
    medicine_id = Column(Integer, ForeignKey("medicine.id"), nullable=False)
    description = Column(Text)
    ingredient = Column(String())
    category = Column(String())
    usage_method = Column(String())
    unit_dose = Column(Float, default=.)
    side_effects = Column(Text)
    storage_conditions = Column(String())
    expiry_date = Column(String())
    barcode = Column(String(), unique=True)
    appearance_type = Column(String())
    notes = Column(Text)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
     關聯到基本資料
    medicine = relationship("Medicine", back_populates="detail")

 處方籤表
class Prescription(Base):
    __tablename__ = "prescriptions"
    
    id = Column(Integer, primary_key=True, index=True)
    patient_name = Column(String(), nullable=False)
    patient_id = Column(String(), nullable=False)
    doctor_name = Column(String(), nullable=False)
    diagnosis = Column(String())
    status = Column(String(), default="pending")   pending, processing, completed, cancelled
    created_at = Column(DateTime, default=datetime.utcnow)
    prescription_date = Column(DateTime, default=datetime.utcnow)
    
     關聯到處方籤藥物
    medicines = relationship("PrescriptionMedicine", back_populates="prescription")

 處方籤藥物關聯表
class PrescriptionMedicine(Base):
    __tablename__ = "prescription_medicines"
    
    id = Column(Integer, primary_key=True, index=True)
    prescription_id = Column(Integer, ForeignKey("prescriptions.id"), nullable=False)
    medicine_id = Column(Integer, ForeignKey("medicine.id"), nullable=False)
    dosage = Column(String())
    frequency = Column(String())
    duration = Column(String())
    quantity = Column(Integer, default=)
    instructions = Column(Text)
    created_at = Column(DateTime, default=datetime.utcnow)
    
     關聯
    prescription = relationship("Prescription", back_populates="medicines")
    medicine = relationship("Medicine")

def get_db():
    """獲取資料庫連接"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def create_tables():
    """創建所有表格"""
    Base.metadata.create_all(bind=engine)
    print(" 最終版資料庫表格創建完成")

def init_final_database():
    """初始化最終版資料庫 - 完全乾淨，無任何資料"""
    print(" 初始化最終版醫院藥物管理系統資料庫...")
    create_tables()
    print(" 最終版資料庫初始化完成")
    print(" 系統已準備就緒，完全乾淨無資料")
    print(" 所有接口已準備，等待您的整合")

if __name__ == "__main__":
    init_final_database()
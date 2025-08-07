"""
醫院藥物管理系統 - 正式版資料庫配置
不含任何測試資料的純淨版本
"""

from sqlalchemy import create_engine, Column, Integer, String, Text, Float, DateTime, Boolean, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from datetime import datetime

# 正式版資料庫配置
DATABASE_URL = "sqlite:///./hospital_medicine_production.db"

engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# 基本藥物表
class Medicine(Base):
    __tablename__ = "medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(100), nullable=False, index=True)
    amount = Column(Integer, default=0)
    position = Column(String(50), nullable=False)
    manufacturer = Column(String(100))
    dosage = Column(String(50))
    is_active = Column(Boolean, default=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # 關聯到詳細資料
    detail = relationship("MedicineDetail", back_populates="medicine", uselist=False)

# 詳細藥物表
class MedicineDetail(Base):
    __tablename__ = "medicine_detailed"
    
    id = Column(Integer, primary_key=True, index=True)
    medicine_id = Column(Integer, ForeignKey("medicine.id"), nullable=False)
    description = Column(Text)
    ingredient = Column(String(200))
    category = Column(String(100))
    usage_method = Column(String(100))
    unit_dose = Column(Float, default=0.0)
    side_effects = Column(Text)
    storage_conditions = Column(String(200))
    expiry_date = Column(String(20))
    barcode = Column(String(50), unique=True)
    appearance_type = Column(String(100))
    notes = Column(Text)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # 關聯到基本資料
    medicine = relationship("Medicine", back_populates="detail")

# 處方籤表
class Prescription(Base):
    __tablename__ = "prescriptions"
    
    id = Column(Integer, primary_key=True, index=True)
    patient_name = Column(String(100), nullable=False)
    patient_id = Column(String(50), nullable=False)
    doctor_name = Column(String(100), nullable=False)
    diagnosis = Column(String(200))
    status = Column(String(20), default="pending")  # pending, processing, completed, cancelled
    created_at = Column(DateTime, default=datetime.utcnow)
    prescription_date = Column(DateTime, default=datetime.utcnow)
    
    # 關聯到處方籤藥物
    medicines = relationship("PrescriptionMedicine", back_populates="prescription")

# 處方籤藥物關聯表
class PrescriptionMedicine(Base):
    __tablename__ = "prescription_medicines"
    
    id = Column(Integer, primary_key=True, index=True)
    prescription_id = Column(Integer, ForeignKey("prescriptions.id"), nullable=False)
    medicine_id = Column(Integer, ForeignKey("medicine.id"), nullable=False)
    dosage = Column(String(50))
    frequency = Column(String(50))
    duration = Column(String(50))
    quantity = Column(Integer, default=1)
    instructions = Column(Text)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # 關聯
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
    print("✅ 正式版資料庫表格創建完成")

def init_production_database():
    """初始化正式版資料庫 - 不添加任何測試資料"""
    print("🏥 初始化正式版醫院藥物管理系統資料庫...")
    create_tables()
    print("✅ 正式版資料庫初始化完成 - 無測試資料")
    print("📊 系統已準備就緒，等待實際資料輸入")

if __name__ == "__main__":
    init_production_database()
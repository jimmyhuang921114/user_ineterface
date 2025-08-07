#!/usr/bin/env python3
"""
Hospital Medicine Management Database (Clean Version)
醫院藥物管理資料庫
"""

from sqlalchemy import create_engine, Column, Integer, String, Boolean, DateTime, Text, ForeignKey, Float
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from datetime import datetime
import os

# 資料庫設定
DATABASE_URL = "sqlite:///./hospital_medicine_clean.db"

engine = create_engine(DATABASE_URL, connect_args={"check_same_thread": False})
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()

class Medicine(Base):
    """基本藥物資料表"""
    __tablename__ = "medicine_basic"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, index=True, nullable=False)
    amount = Column(Integer, default=0)
    position = Column(String)
    manufacturer = Column(String)
    dosage = Column(String)
    is_active = Column(Boolean, default=True)
    created_at = Column(DateTime, default=datetime.now)
    updated_at = Column(DateTime, default=datetime.now, onupdate=datetime.now)
    
    # 關聯到詳細資料
    detailed = relationship("MedicineDetail", back_populates="medicine")
    
    # 關聯到處方籤
    prescriptions = relationship("PrescriptionMedicine", back_populates="medicine")

class MedicineDetail(Base):
    """詳細藥物資料表"""
    __tablename__ = "medicine_detailed"
    
    id = Column(Integer, primary_key=True, index=True)
    medicine_id = Column(Integer, ForeignKey("medicine_basic.id"), nullable=False)
    description = Column(Text)
    ingredient = Column(String)
    category = Column(String)
    usage_method = Column(String)
    unit_dose = Column(Float)
    side_effects = Column(Text)
    storage_conditions = Column(String)
    expiry_date = Column(String)
    barcode = Column(String, unique=True)
    appearance_type = Column(String)
    notes = Column(Text)
    created_at = Column(DateTime, default=datetime.now)
    updated_at = Column(DateTime, default=datetime.now, onupdate=datetime.now)
    
    # 關聯到基本資料
    medicine = relationship("Medicine", back_populates="detailed")

class Prescription(Base):
    """處方籤資料表"""
    __tablename__ = "prescriptions"
    
    id = Column(Integer, primary_key=True, index=True)
    patient_name = Column(String, nullable=False)
    patient_id = Column(String, nullable=False)
    doctor_name = Column(String, nullable=False)
    diagnosis = Column(Text)
    status = Column(String, default="active")  # active, pending, processing, completed, cancelled
    created_at = Column(DateTime, default=datetime.now)
    updated_at = Column(DateTime, default=datetime.now, onupdate=datetime.now)
    
    # 關聯到處方籤藥物
    medicines = relationship("PrescriptionMedicine", back_populates="prescription")

class PrescriptionMedicine(Base):
    """處方籤藥物關聯表"""
    __tablename__ = "prescription_medicines"
    
    id = Column(Integer, primary_key=True, index=True)
    prescription_id = Column(Integer, ForeignKey("prescriptions.id"), nullable=False)
    medicine_id = Column(Integer, ForeignKey("medicine_basic.id"), nullable=False)
    dosage = Column(String)
    frequency = Column(String)
    duration = Column(String)
    instructions = Column(Text)
    quantity = Column(Integer, default=1)
    created_at = Column(DateTime, default=datetime.now)
    
    # 關聯
    prescription = relationship("Prescription", back_populates="medicines")
    medicine = relationship("MedicineBasic", back_populates="prescriptions")

def get_db():
    """獲取資料庫會話"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def init_database():
    """初始化資料庫"""
    try:
        # 創建所有表格
        Base.metadata.create_all(bind=engine)
        print("資料庫表格創建完成")
        
    except Exception as e:
        print(f"初始化資料庫時發生錯誤: {e}")

if __name__ == "__main__":
    init_database()
#!/usr/bin/env python3
"""
SQL Database Models and Configuration
SQL資料庫模型和配置
"""

from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text, Float, Boolean, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from sqlalchemy.sql import func
from datetime import datetime
import os

# 資料庫配置
DATABASE_URL = "sqlite:///./data/hospital_management.db"

# 創建資料庫引擎
engine = create_engine(
    DATABASE_URL, 
    connect_args={"check_same_thread": False},  # SQLite特定配置
    echo=True  # 開發時顯示SQL查詢
)

# 創建Session
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# 基礎模型類
Base = declarative_base()

class MedicineBasic(Base):
    """基本藥物資料表"""
    __tablename__ = "medicine_basic"
    
    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    name = Column(String(200), unique=True, index=True, nullable=False)
    amount = Column(Integer, nullable=False, default=0)
    usage_days = Column(Integer, nullable=True)
    position = Column(String(100), nullable=False)
    manufacturer = Column(String(200), nullable=True)
    dosage = Column(String(100), nullable=True)
    prompt = Column(Text, nullable=True)  # AI提示詞
    created_time = Column(DateTime, default=func.now(), nullable=False)
    updated_time = Column(DateTime, default=func.now(), onupdate=func.now(), nullable=False)
    is_active = Column(Boolean, default=True, nullable=False)
    
    # 關聯關係
    detailed_info = relationship("MedicineDetailed", back_populates="basic_medicine", uselist=False)
    prescription_items = relationship("PrescriptionMedicine", back_populates="medicine")
    stock_logs = relationship("StockLog", back_populates="medicine")

class MedicineDetailed(Base):
    """詳細藥物資料表"""
    __tablename__ = "medicine_detailed"
    
    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    medicine_id = Column(Integer, ForeignKey("medicine_basic.id"), nullable=False, unique=True)
    description = Column(Text, nullable=True)
    ingredient = Column(String(500), nullable=True)
    category = Column(String(100), nullable=True)
    usage_method = Column(String(200), nullable=True)
    unit_dose = Column(String(100), nullable=True)
    side_effects = Column(Text, nullable=True)
    storage_conditions = Column(String(200), nullable=True)
    expiry_date = Column(DateTime, nullable=True)
    barcode = Column(String(100), nullable=True, unique=True)
    appearance_type = Column(String(200), nullable=True)
    notes = Column(Text, nullable=True)
    created_time = Column(DateTime, default=func.now(), nullable=False)
    updated_time = Column(DateTime, default=func.now(), onupdate=func.now(), nullable=False)
    
    # 關聯關係
    basic_medicine = relationship("MedicineBasic", back_populates="detailed_info")

class Prescription(Base):
    """處方籤主表"""
    __tablename__ = "prescriptions"
    
    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    patient_name = Column(String(100), nullable=False)
    patient_id = Column(String(20), nullable=False)  # 身份證號或病患編號
    doctor_name = Column(String(100), nullable=False)
    diagnosis = Column(Text, nullable=True)  # 診斷結果
    status = Column(String(50), default="active", nullable=False)  # active, completed, cancelled
    created_at = Column(DateTime, default=func.now(), nullable=False)
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now(), nullable=False)
    
    # 關聯關係
    medicines = relationship("PrescriptionMedicine", back_populates="prescription", cascade="all, delete-orphan")

class PrescriptionMedicine(Base):
    """處方籤藥物明細表"""
    __tablename__ = "prescription_medicines"
    
    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    prescription_id = Column(Integer, ForeignKey("prescriptions.id"), nullable=False)
    medicine_id = Column(Integer, ForeignKey("medicine_basic.id"), nullable=False)
    dosage = Column(String(100), nullable=False)  # 劑量
    frequency = Column(String(100), nullable=False)  # 頻率
    duration = Column(String(100), nullable=False)  # 療程
    instructions = Column(Text, nullable=True)  # 特殊說明
    quantity = Column(Integer, nullable=False, default=1)  # 數量
    
    # 關聯關係
    prescription = relationship("Prescription", back_populates="medicines")
    medicine = relationship("MedicineBasic", back_populates="prescription_items")

class StockLog(Base):
    """庫存異動記錄表"""
    __tablename__ = "stock_logs"
    
    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    medicine_id = Column(Integer, ForeignKey("medicine_basic.id"), nullable=False)
    action = Column(String(50), nullable=False)  # add, subtract, adjust
    quantity_before = Column(Integer, nullable=False)
    quantity_change = Column(Integer, nullable=False)  # 正數為增加，負數為減少
    quantity_after = Column(Integer, nullable=False)
    reason = Column(String(200), nullable=True)
    operator = Column(String(100), nullable=True)  # 操作人員
    created_at = Column(DateTime, default=func.now(), nullable=False)
    
    # 關聯關係
    medicine = relationship("MedicineBasic", back_populates="stock_logs")

class SystemLog(Base):
    """系統操作記錄表"""
    __tablename__ = "system_logs"
    
    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    action = Column(String(100), nullable=False)
    table_name = Column(String(50), nullable=True)
    record_id = Column(Integer, nullable=True)
    details = Column(Text, nullable=True)
    ip_address = Column(String(50), nullable=True)
    user_agent = Column(String(500), nullable=True)
    created_at = Column(DateTime, default=func.now(), nullable=False)

# 資料庫初始化函數
def create_tables():
    """創建所有資料表"""
    # 確保data目錄存在
    os.makedirs("data", exist_ok=True)
    
    # 創建所有資料表
    Base.metadata.create_all(bind=engine)
    print("✅ 資料庫表格創建完成")

def get_db():
    """獲取資料庫session"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# 資料庫工具函數
def init_database():
    """初始化資料庫"""
    create_tables()
    
    # 可以在這裡添加初始數據
    db = SessionLocal()
    try:
        # 檢查是否有資料，如果沒有則添加示例資料
        if db.query(MedicineBasic).count() == 0:
            sample_medicines = [
                MedicineBasic(
                    name="普拿疼",
                    amount=100,
                    usage_days=7,
                    position="A1-01",
                    manufacturer="Johnson & Johnson",
                    dosage="500mg",
                    prompt="適用於發燒、頭痛，成人每次1-2錠，每日最多8錠"
                ),
                MedicineBasic(
                    name="阿斯匹靈",
                    amount=50,
                    usage_days=5,
                    position="A1-02",
                    manufacturer="Bayer",
                    dosage="100mg",
                    prompt="用於心血管保護，成人每日1錠，餐後服用"
                )
            ]
            
            for medicine in sample_medicines:
                db.add(medicine)
            
            db.commit()
            print("✅ 示例資料添加完成")
            
    except Exception as e:
        print(f"❌ 初始化資料時發生錯誤: {e}")
        db.rollback()
    finally:
        db.close()

if __name__ == "__main__":
    init_database()
#!/usr/bin/env python3
"""
Final database configuration for hospital medicine management system
Clean production version without test data
"""

from sqlalchemy import create_engine, Column, Integer, String, Float, DateTime, ForeignKey, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from datetime import datetime

# Final database configuration
DATABASE_URL = "sqlite:///./hospital_medicine_final.db"

engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# Basic medicine table
class Medicine(Base):
    __tablename__ = "medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(255), nullable=False, index=True)
    amount = Column(Integer, default=0)
    position = Column(String(100), nullable=False)
    manufacturer = Column(String(255))
    dosage = Column(String(100))
    expiry_date = Column(DateTime)

# Medicine detail table
class MedicineDetail(Base):
    __tablename__ = "medicine_detail"
    
    id = Column(Integer, primary_key=True, index=True)
    medicine_id = Column(Integer, ForeignKey("medicine.id"), nullable=False)
    description = Column(Text)
    category = Column(String(100))
    unit_dose = Column(Float, default=1.0)
    side_effects = Column(Text)
    instructions = Column(Text)
    
    medicine = relationship("Medicine", back_populates="details")

# Update Medicine to include reverse relationship
Medicine.details = relationship("MedicineDetail", back_populates="medicine", cascade="all, delete-orphan")

# Prescription table
class Prescription(Base):
    __tablename__ = "prescription"
    
    id = Column(Integer, primary_key=True, index=True)
    patient_name = Column(String(255), nullable=False)
    doctor_name = Column(String(255))
    diagnosis = Column(Text)
    created_at = Column(DateTime, default=datetime.utcnow)
    status = Column(String(50), default="pending")  # pending, processing, completed, cancelled

# Prescription medicine junction table
class PrescriptionMedicine(Base):
    __tablename__ = "prescription_medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    prescription_id = Column(Integer, ForeignKey("prescription.id"), nullable=False)
    medicine_id = Column(Integer, ForeignKey("medicine.id"), nullable=False)
    amount = Column(Integer, nullable=False)
    notes = Column(Text)
    
    prescription = relationship("Prescription")
    medicine = relationship("Medicine")

def get_db():
    """Get database connection"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def init_final_database():
    """Create all tables"""
    Base.metadata.create_all(bind=engine)
    print("Final database tables created")

if __name__ == "__main__":
    init_final_database()
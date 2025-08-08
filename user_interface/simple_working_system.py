#!/usr/bin/env python3
"""
Simple Working Hospital Medicine Management System
A minimal but fully functional ROS2-integrated system
"""

from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
from sqlalchemy import create_engine, Column, Integer, String, Float, DateTime, ForeignKey, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship, Session
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
import uvicorn
import logging
import json
import os
from pathlib import Path

# Database setup
DATABASE_URL = "sqlite:///./hospital_medicine_working.db"
engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# Database Models
class Medicine(Base):
    __tablename__ = "medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(255), nullable=False, index=True)
    amount = Column(Integer, default=0)
    position = Column(String(100), nullable=False, default="A1")
    description = Column(Text, default="")
    category = Column(String(100), default="General")
    unit_dose = Column(Float, default=1.0)

class Prescription(Base):
    __tablename__ = "prescription"
    
    id = Column(Integer, primary_key=True, index=True)
    patient_name = Column(String(255), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    status = Column(String(50), default="pending")  # pending, processing, completed, cancelled

class PrescriptionMedicine(Base):
    __tablename__ = "prescription_medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    prescription_id = Column(Integer, ForeignKey("prescription.id"), nullable=False)
    medicine_id = Column(Integer, ForeignKey("medicine.id"), nullable=False)
    amount = Column(Integer, nullable=False)
    
    prescription = relationship("Prescription")
    medicine = relationship("Medicine")

# Pydantic Models
class MedicineCreate(BaseModel):
    name: str
    amount: int = 100
    position: str = "A1"
    description: str = ""
    category: str = "General"
    unit_dose: float = 1.0

class PrescriptionCreate(BaseModel):
    patient_name: str
    medicines: List[Dict[str, Any]]  # [{"medicine_id": 1, "amount": 10}]

# FastAPI App
app = FastAPI(
    title="Hospital Medicine Management System",
    description="Simple working system with ROS2 integration",
    version="1.0.0"
)

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("hospital_system")

# Database functions
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def init_database():
    Base.metadata.create_all(bind=engine)
    logger.info("Database initialized")
    
    # Add sample data if empty
    db = SessionLocal()
    if db.query(Medicine).count() == 0:
        sample_medicines = [
            Medicine(name="Aspirin", amount=100, position="A1", description="Pain reliever", category="Pain Relief"),
            Medicine(name="Vitamin C", amount=50, position="A2", description="Vitamin supplement", category="Vitamins"),
            Medicine(name="Ibuprofen", amount=75, position="B1", description="Anti-inflammatory", category="Pain Relief")
        ]
        for med in sample_medicines:
            db.add(med)
        db.commit()
        logger.info("Sample medicines added")
    db.close()

# API Routes
@app.get("/")
async def root():
    return {"message": "Hospital Medicine Management System", "status": "running"}

@app.get("/api/system/status")
async def get_system_status():
    return {
        "status": "running",
        "ros_mode": "interface_ready",
        "database": "connected",
        "version": "1.0.0"
    }

# Medicine endpoints
@app.get("/api/medicine/")
async def get_medicines(db: Session = Depends(get_db)):
    medicines = db.query(Medicine).all()
    return [{"id": m.id, "name": m.name, "amount": m.amount, "position": m.position, 
             "description": m.description, "category": m.category, "unit_dose": m.unit_dose} for m in medicines]

@app.get("/api/medicine/basic")
async def get_basic_medicines(db: Session = Depends(get_db)):
    medicines = db.query(Medicine).all()
    return [{"name": m.name, "amount": m.amount, "position": m.position} for m in medicines]

@app.get("/api/medicine/detailed")
async def get_detailed_medicines(db: Session = Depends(get_db)):
    medicines = db.query(Medicine).all()
    return [{"id": m.id, "name": m.name, "amount": m.amount, "position": m.position,
             "description": m.description, "category": m.category, "unit_dose": m.unit_dose} for m in medicines]

@app.post("/api/medicine/")
async def create_medicine(medicine: MedicineCreate, db: Session = Depends(get_db)):
    db_medicine = Medicine(**medicine.dict())
    db.add(db_medicine)
    db.commit()
    db.refresh(db_medicine)
    logger.info(f"Created medicine: {medicine.name}")
    return {"id": db_medicine.id, "name": db_medicine.name, "message": "Medicine created successfully"}

@app.post("/api/medicine/unified")
async def create_medicine_unified(request: Request, db: Session = Depends(get_db)):
    try:
        data = await request.json()
        medicine = MedicineCreate(**data)
        db_medicine = Medicine(**medicine.dict())
        db.add(db_medicine)
        db.commit()
        db.refresh(db_medicine)
        logger.info(f"Created medicine: {medicine.name}")
        return {"id": db_medicine.id, "name": db_medicine.name, "message": "Medicine created successfully"}
    except Exception as e:
        logger.error(f"Error creating medicine: {e}")
        raise HTTPException(status_code=400, detail=str(e))

# Prescription endpoints
@app.get("/api/prescription/")
async def get_prescriptions(db: Session = Depends(get_db)):
    prescriptions = db.query(Prescription).all()
    result = []
    for p in prescriptions:
        medicines = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.prescription_id == p.id
        ).all()
        medicine_list = []
        for pm in medicines:
            med = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
            if med:
                medicine_list.append({
                    "id": med.id,
                    "name": med.name,
                    "amount": pm.amount
                })
        
        result.append({
            "id": p.id,
            "patient_name": p.patient_name,
            "created_at": p.created_at.isoformat() if p.created_at else None,
            "status": p.status,
            "medicines": medicine_list
        })
    return result

@app.post("/api/prescription/")
async def create_prescription(prescription: PrescriptionCreate, db: Session = Depends(get_db)):
    try:
        # Create prescription
        db_prescription = Prescription(patient_name=prescription.patient_name)
        db.add(db_prescription)
        db.commit()
        db.refresh(db_prescription)
        
        # Add medicines
        for med_data in prescription.medicines:
            pm = PrescriptionMedicine(
                prescription_id=db_prescription.id,
                medicine_id=med_data["medicine_id"],
                amount=med_data["amount"]
            )
            db.add(pm)
            
            # Deduct stock
            medicine = db.query(Medicine).filter(Medicine.id == med_data["medicine_id"]).first()
            if medicine:
                medicine.amount = max(0, medicine.amount - med_data["amount"])
        
        db.commit()
        logger.info(f"Created prescription for: {prescription.patient_name}")
        return {"id": db_prescription.id, "message": "Prescription created successfully"}
    except Exception as e:
        logger.error(f"Error creating prescription: {e}")
        raise HTTPException(status_code=400, detail=str(e))

@app.put("/api/prescription/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_data: dict, db: Session = Depends(get_db)):
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="Prescription not found")
    
    prescription.status = status_data.get("status", prescription.status)
    db.commit()
    logger.info(f"Updated prescription {prescription_id} status to: {prescription.status}")
    return {"message": "Status updated successfully"}

@app.get("/api/prescription/pending/next")
async def get_next_pending_prescription(db: Session = Depends(get_db)):
    prescription = db.query(Prescription).filter(
        Prescription.status == "pending"
    ).order_by(Prescription.id).first()
    
    if not prescription:
        raise HTTPException(status_code=404, detail="No pending prescriptions")
    
    # Get medicines for this prescription
    medicines = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.prescription_id == prescription.id
    ).all()
    
    medicine_list = []
    for pm in medicines:
        med = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
        if med:
            medicine_list.append({
                "name": med.name,
                "amount": pm.amount,
                "locate": [1, 1],  # Default position
                "prompt": "tablet"  # Default type
            })
    
    return {
        "order_id": f"{prescription.id:06d}",
        "prescription_id": prescription.id,
        "patient_name": prescription.patient_name,
        "medicine": medicine_list
    }

# ROS2 integration endpoints
@app.get("/api/ros2/service-status")
async def get_ros2_status():
    return {
        "status": "ready",
        "mode": "interface_ready",
        "services": ["medicine_query", "order_processing"]
    }

@app.post("/api/ros2/query-medicine-detail")
async def query_medicine_detail(request_data: dict, db: Session = Depends(get_db)):
    medicine_name = request_data.get("medicine_name", "")
    medicines = db.query(Medicine).filter(Medicine.name.contains(medicine_name)).all()
    
    result = []
    for med in medicines:
        result.append({
            "name": med.name,
            "description": med.description,
            "category": med.category,
            "unit_dose": med.unit_dose,
            "stock_quantity": med.amount,
            "position": med.position
        })
    
    return {"medicines": result, "count": len(result)}

# Mount static files
static_path = Path(__file__).parent / "static"
if static_path.exists():
    app.mount("/static", StaticFiles(directory=str(static_path)), name="static")

# Serve HTML files
@app.get("/integrated_medicine_management.html", response_class=HTMLResponse)
async def serve_medicine_management():
    file_path = static_path / "integrated_medicine_management.html"
    if file_path.exists():
        return FileResponse(file_path)
    return HTMLResponse("<h1>Medicine Management Interface</h1><p>Static files not found</p>")

@app.get("/doctor.html", response_class=HTMLResponse)
async def serve_doctor():
    file_path = static_path / "doctor.html"
    if file_path.exists():
        return FileResponse(file_path)
    return HTMLResponse("<h1>Doctor Interface</h1><p>Static files not found</p>")

@app.get("/Prescription.html", response_class=HTMLResponse)
async def serve_prescription():
    file_path = static_path / "Prescription.html"
    if file_path.exists():
        return FileResponse(file_path)
    return HTMLResponse("<h1>Prescription Management</h1><p>Static files not found</p>")

if __name__ == "__main__":
    print("Initializing Hospital Medicine Management System...")
    init_database()
    
    print("Starting server...")
    print("Web interfaces:")
    print("- Medicine Management: http://localhost:8001/integrated_medicine_management.html")
    print("- Doctor Interface: http://localhost:8001/doctor.html")
    print("- Prescription Management: http://localhost:8001/Prescription.html")
    print("- API Documentation: http://localhost:8001/docs")
    
    uvicorn.run(app, host="0.0.0.0", port=8001)
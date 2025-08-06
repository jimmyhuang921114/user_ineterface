#!/usr/bin/env python3
"""
Simple Hospital Medicine Management System Server
簡化醫院藥物管理系統伺服器
"""

from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel
from sqlalchemy.orm import Session
from typing import List, Optional
import uvicorn
from pathlib import Path
import os

from database import (
    get_db, init_database, 
    MedicineBasic, MedicineDetailed, Prescription, PrescriptionMedicine
)

# Initialize FastAPI app
app = FastAPI(title="Simple Medicine Management API", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models for API
class MedicineBasicCreate(BaseModel):
    name: str
    amount: int
    position: str
    manufacturer: Optional[str] = None
    dosage: Optional[str] = None
    prompt: Optional[str] = None

class MedicineBasicResponse(BaseModel):
    id: int
    name: str
    amount: int
    position: str
    manufacturer: Optional[str] = None
    dosage: Optional[str] = None
    prompt: Optional[str] = None
    
    class Config:
        from_attributes = True

class PrescriptionCreate(BaseModel):
    patient_name: str
    patient_id: str
    doctor_name: str
    diagnosis: Optional[str] = None

class PrescriptionResponse(BaseModel):
    id: int
    patient_name: str
    patient_id: str
    doctor_name: str
    diagnosis: Optional[str] = None
    status: str
    
    class Config:
        from_attributes = True

# Initialize database on startup
@app.on_event("startup")
async def startup_event():
    init_database()

# Health check endpoint
@app.get("/api/health")
async def health_check():
    return {"status": "healthy", "message": "Simple Medicine Management System is running"}

# Medicine Basic CRUD endpoints
@app.get("/api/medicine/basic", response_model=List[MedicineBasicResponse])
async def get_basic_medicines(db: Session = Depends(get_db)):
    """獲取基本藥物列表"""
    medicines = db.query(MedicineBasic).filter(MedicineBasic.is_active == True).all()
    return medicines

@app.post("/api/medicine/basic", response_model=MedicineBasicResponse)
async def create_basic_medicine(medicine: MedicineBasicCreate, db: Session = Depends(get_db)):
    """創建基本藥物"""
    # Check if medicine already exists
    existing = db.query(MedicineBasic).filter(MedicineBasic.name == medicine.name).first()
    if existing:
        raise HTTPException(status_code=400, detail="Medicine already exists")
    
    db_medicine = MedicineBasic(**medicine.dict())
    db.add(db_medicine)
    db.commit()
    db.refresh(db_medicine)
    return db_medicine

@app.get("/api/medicine/basic/{medicine_id}", response_model=MedicineBasicResponse)
async def get_basic_medicine(medicine_id: int, db: Session = Depends(get_db)):
    """獲取特定藥物資料"""
    medicine = db.query(MedicineBasic).filter(MedicineBasic.id == medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="Medicine not found")
    return medicine

@app.put("/api/medicine/basic/{medicine_id}", response_model=MedicineBasicResponse)
async def update_basic_medicine(medicine_id: int, medicine: MedicineBasicCreate, db: Session = Depends(get_db)):
    """更新藥物資料"""
    db_medicine = db.query(MedicineBasic).filter(MedicineBasic.id == medicine_id).first()
    if not db_medicine:
        raise HTTPException(status_code=404, detail="Medicine not found")
    
    for key, value in medicine.dict().items():
        setattr(db_medicine, key, value)
    
    db.commit()
    db.refresh(db_medicine)
    return db_medicine

@app.delete("/api/medicine/basic/{medicine_id}")
async def delete_basic_medicine(medicine_id: int, db: Session = Depends(get_db)):
    """刪除藥物（軟刪除）"""
    db_medicine = db.query(MedicineBasic).filter(MedicineBasic.id == medicine_id).first()
    if not db_medicine:
        raise HTTPException(status_code=404, detail="Medicine not found")
    
    db_medicine.is_active = False
    db.commit()
    return {"message": "Medicine deleted successfully"}

# Prescription CRUD endpoints
@app.get("/api/prescription/", response_model=List[PrescriptionResponse])
async def get_prescriptions(db: Session = Depends(get_db)):
    """獲取處方籤列表"""
    prescriptions = db.query(Prescription).all()
    return prescriptions

@app.post("/api/prescription/", response_model=PrescriptionResponse)
async def create_prescription(prescription: PrescriptionCreate, db: Session = Depends(get_db)):
    """創建處方籤"""
    db_prescription = Prescription(**prescription.dict())
    db.add(db_prescription)
    db.commit()
    db.refresh(db_prescription)
    return db_prescription

@app.get("/api/prescription/{prescription_id}", response_model=PrescriptionResponse)
async def get_prescription(prescription_id: int, db: Session = Depends(get_db)):
    """獲取特定處方籤"""
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="Prescription not found")
    return prescription

# Static files
static_dir = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

# HTML pages
@app.get("/")
async def read_root():
    html_path = Path(__file__).parent / "static" / "html" / "Medicine.html"
    if not html_path.exists():
        raise HTTPException(status_code=404, detail=f"File not found: {html_path}")
    return FileResponse(html_path)

@app.get("/debug")
async def debug_paths():
    current_dir = Path(__file__).parent
    html_path = current_dir / "static" / "html" / "Medicine.html"
    return {
        "current_dir": str(current_dir),
        "html_path": str(html_path),
        "html_exists": html_path.exists(),
        "static_dir": str(current_dir / "static"),
        "static_exists": (current_dir / "static").exists()
    }

@app.get("/Medicine.html")
async def medicine_page():
    html_path = Path(__file__).parent / "static" / "html" / "Medicine.html"
    if not html_path.exists():
        raise HTTPException(status_code=404, detail=f"File not found: {html_path}")
    return FileResponse(html_path)

@app.get("/Prescription.html")
async def prescription_page():
    html_path = Path(__file__).parent / "static" / "html" / "Prescription.html"
    if not html_path.exists():
        raise HTTPException(status_code=404, detail=f"File not found: {html_path}")
    return FileResponse(html_path)

@app.get("/doctor.html")
async def doctor_page():
    html_path = Path(__file__).parent / "static" / "html" / "doctor.html"
    if not html_path.exists():
        raise HTTPException(status_code=404, detail=f"File not found: {html_path}")
    return FileResponse(html_path)

@app.get("/integrated_medicine_management.html")
async def integrated_page():
    html_path = Path(__file__).parent / "static" / "html" / "integrated_medicine_management.html"
    if not html_path.exists():
        raise HTTPException(status_code=404, detail=f"File not found: {html_path}")
    return FileResponse(html_path)

if __name__ == "__main__":
    print("🏥 簡化醫院藥物管理系統")
    print("=" * 50)
    print("🌐 藥物管理: http://localhost:8000/Medicine.html")
    print("📋 處方籤管理: http://localhost:8000/Prescription.html")
    print("👨‍⚕️ 醫生界面: http://localhost:8000/doctor.html")
    print("📖 API文檔: http://localhost:8000/docs")
    print("=" * 50)
    
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
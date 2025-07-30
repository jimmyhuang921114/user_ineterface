#!/usr/bin/env python3
"""
演示伺服器 - 包含病例管理功能
Demo Server with Medical Record Management
"""

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
import uvicorn

# 創建FastAPI應用
app = FastAPI(title="醫療系統演示", version="1.0.0")

# 數據模型
class MedicineBasic(BaseModel):
    name: str
    amount: int
    usage_days: int
    position: str

class MedicineDetailed(BaseModel):
    medicine_name: str
    medicine_data: Dict[str, Any]

class MedicalRecordCreate(BaseModel):
    patient_id: str
    patient_name: str
    doctor_name: str
    diagnosis: str
    symptoms: str
    treatment_plan: str
    prescribed_medicines: List[Dict[str, Any]]
    vital_signs: Optional[Dict[str, str]] = {}
    medical_history: Optional[str] = ""
    notes: Optional[str] = ""
    visit_date: Optional[str] = ""

# 記憶體內資料庫
medicines_db = []
detailed_medicines_db = {}
medical_records_db = []
next_medicine_id = 1
next_record_id = 1

# === 系統狀態 ===
@app.get("/api/system/status")
async def system_status():
    return {
        "system": "醫療系統演示",
        "status": "運行中",
        "total_medicines": len(medicines_db),
        "total_records": len(medical_records_db)
    }

# === 藥物API ===
@app.post("/api/medicine/")
async def create_medicine(medicine: MedicineBasic):
    global next_medicine_id
    new_medicine = {
        "id": next_medicine_id,
        "name": medicine.name,
        "amount": medicine.amount,
        "usage_days": medicine.usage_days,
        "position": medicine.position,
        "created_time": datetime.now().isoformat()
    }
    medicines_db.append(new_medicine)
    next_medicine_id += 1
    return {"message": "藥物創建成功", "medicine": new_medicine}

@app.post("/api/medicine/detailed/")
async def create_detailed_medicine(detailed: MedicineDetailed):
    detailed_medicines_db[detailed.medicine_name] = detailed.medicine_data
    return {"message": "詳細藥物資訊創建成功"}

@app.get("/api/medicine/integrated/{medicine_name}")
async def get_integrated_medicine(medicine_name: str):
    basic_info = None
    for med in medicines_db:
        if med["name"] == medicine_name:
            basic_info = med
            break
    
    detailed_info = detailed_medicines_db.get(medicine_name)
    
    return {
        "medicine_name": medicine_name,
        "basic_info": basic_info,
        "detailed_info": detailed_info,
        "has_complete_info": basic_info is not None and detailed_info is not None
    }

# === 病例API ===
@app.post("/api/medical_record/")
async def create_medical_record(record: MedicalRecordCreate):
    global next_record_id
    
    visit_date = record.visit_date or datetime.now().date().isoformat()
    
    new_record = {
        "id": next_record_id,
        "patient_id": record.patient_id,
        "patient_name": record.patient_name,
        "doctor_name": record.doctor_name,
        "diagnosis": record.diagnosis,
        "symptoms": record.symptoms,
        "treatment_plan": record.treatment_plan,
        "prescribed_medicines": record.prescribed_medicines,
        "vital_signs": record.vital_signs,
        "medical_history": record.medical_history,
        "notes": record.notes,
        "visit_date": visit_date,
        "status": "active",
        "created_time": datetime.now().isoformat(),
        "updated_time": datetime.now().isoformat()
    }
    
    medical_records_db.append(new_record)
    next_record_id += 1
    
    return {"message": "病例創建成功", "record": new_record}

@app.get("/api/medical_record/{record_id}")
async def get_medical_record(record_id: int):
    for record in medical_records_db:
        if record["id"] == record_id:
            return record
    raise HTTPException(status_code=404, detail="病例未找到")

@app.get("/api/medical_record/patient/{patient_id}")
async def get_records_by_patient(patient_id: str):
    records = [r for r in medical_records_db if r["patient_id"] == patient_id]
    if not records:
        raise HTTPException(status_code=404, detail="該病患沒有病例記錄")
    return records

@app.get("/api/medical_record/stats/summary")
async def get_stats():
    doctor_stats = {}
    for record in medical_records_db:
        doctor = record["doctor_name"]
        doctor_stats[doctor] = doctor_stats.get(doctor, 0) + 1
    
    return {
        "total_records": len(medical_records_db),
        "doctor_statistics": doctor_stats
    }

if __name__ == "__main__":
    print("啟動醫療系統演示伺服器...")
    uvicorn.run(app, host="0.0.0.0", port=8000)
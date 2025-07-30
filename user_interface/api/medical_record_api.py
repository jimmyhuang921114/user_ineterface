#!/usr/bin/env python3
"""
病例管理API模組
Medical Record Management API Module
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime

# 病例管理API路由器
router = APIRouter(prefix="/api/medical_record", tags=["病例管理"])

# Pydantic數據模型
class MedicalRecordCreate(BaseModel):
    patient_id: str
    patient_name: str
    doctor_name: str
    diagnosis: str
    symptoms: str
    treatment_plan: str
    prescribed_medicines: List[Dict[str, Any]]  # 處方藥物清單
    vital_signs: Optional[Dict[str, str]] = {}  # 生命徵象
    medical_history: Optional[str] = ""
    notes: Optional[str] = ""
    visit_date: Optional[str] = ""

class MedicalRecordUpdate(BaseModel):
    diagnosis: Optional[str] = None
    symptoms: Optional[str] = None
    treatment_plan: Optional[str] = None
    prescribed_medicines: Optional[List[Dict[str, Any]]] = None
    vital_signs: Optional[Dict[str, str]] = None
    medical_history: Optional[str] = None
    notes: Optional[str] = None

# 記憶體內資料庫
medical_records_db = []
next_record_id = 1

# === 病例管理API端點 ===
@router.post("/")
async def create_medical_record(record: MedicalRecordCreate):
    """創建新病例"""
    global next_record_id
    
    # 使用當前日期如果沒有提供日期
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
    
    return {
        "message": "病例創建成功",
        "record": new_record
    }

@router.get("/")
async def get_all_medical_records():
    """獲取所有病例"""
    return medical_records_db

@router.get("/{record_id}")
async def get_medical_record(record_id: int):
    """獲取特定病例"""
    for record in medical_records_db:
        if record["id"] == record_id:
            return record
    
    raise HTTPException(status_code=404, detail="病例未找到")

@router.get("/patient/{patient_id}")
async def get_medical_records_by_patient(patient_id: str):
    """按病患ID獲取病例"""
    records = [r for r in medical_records_db if r["patient_id"] == patient_id]
    if not records:
        raise HTTPException(status_code=404, detail="該病患沒有病例記錄")
    return records

@router.get("/doctor/{doctor_name}")
async def get_medical_records_by_doctor(doctor_name: str):
    """按醫師獲取病例"""
    records = [r for r in medical_records_db if r["doctor_name"] == doctor_name]
    return records

@router.put("/{record_id}")
async def update_medical_record(record_id: int, update_data: MedicalRecordUpdate):
    """更新病例"""
    for record in medical_records_db:
        if record["id"] == record_id:
            # 更新提供的字段
            update_dict = update_data.dict(exclude_unset=True)
            record.update(update_dict)
            record["updated_time"] = datetime.now().isoformat()
            
            return {
                "message": "病例更新成功",
                "record": record
            }
    
    raise HTTPException(status_code=404, detail="病例未找到")

@router.delete("/{record_id}")
async def delete_medical_record(record_id: int):
    """刪除病例"""
    for i, record in enumerate(medical_records_db):
        if record["id"] == record_id:
            deleted_record = medical_records_db.pop(i)
            return {
                "message": "病例刪除成功",
                "record": deleted_record
            }
    
    raise HTTPException(status_code=404, detail="病例未找到")

@router.get("/search/diagnosis/{diagnosis}")
async def search_by_diagnosis(diagnosis: str):
    """按診斷搜索病例"""
    records = [r for r in medical_records_db if diagnosis.lower() in r["diagnosis"].lower()]
    return {
        "search_term": diagnosis,
        "results": records,
        "count": len(records)
    }

@router.get("/stats/summary")
async def get_medical_records_summary():
    """獲取病例統計摘要"""
    total_records = len(medical_records_db)
    
    # 按醫師統計
    doctor_stats = {}
    for record in medical_records_db:
        doctor = record["doctor_name"]
        doctor_stats[doctor] = doctor_stats.get(doctor, 0) + 1
    
    # 按診斷統計
    diagnosis_stats = {}
    for record in medical_records_db:
        diagnosis = record["diagnosis"]
        diagnosis_stats[diagnosis] = diagnosis_stats.get(diagnosis, 0) + 1
    
    # 最近一週的病例
    recent_records = []
    current_date = datetime.now().date()
    for record in medical_records_db:
        try:
            record_date = datetime.fromisoformat(record["visit_date"]).date()
            days_diff = (current_date - record_date).days
            if days_diff <= 7:
                recent_records.append(record)
        except:
            continue
    
    return {
        "total_records": total_records,
        "doctor_statistics": doctor_stats,
        "diagnosis_statistics": diagnosis_stats,
        "recent_records_count": len(recent_records),
        "recent_records": recent_records
    }
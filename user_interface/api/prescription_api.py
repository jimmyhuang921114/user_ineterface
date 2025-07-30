#!/usr/bin/env python3
"""
處方管理API模組
Prescription Management API Module
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime

# 處方管理API路由器
router = APIRouter(prefix="/api/prescription", tags=["處方管理"])

# Pydantic數據模型
class PrescriptionMedicine(BaseModel):
    medicine_name: str
    dosage: str
    frequency: str
    duration: str
    instructions: Optional[str] = ""

class PrescriptionCreate(BaseModel):
    patient_name: str
    doctor_name: str
    medicines: List[PrescriptionMedicine]
    diagnosis: Optional[str] = ""
    instructions: Optional[str] = ""
    priority: Optional[str] = "normal"
    prescription_date: Optional[str] = ""

class PrescriptionStatus(BaseModel):
    status: str
    notes: Optional[str] = ""

# 記憶體內資料庫
prescriptions_db = []
prescription_status_db = []
next_prescription_id = 1

# === 處方管理API端點 ===
@router.post("/")
async def create_prescription(prescription: PrescriptionCreate):
    """創建新處方"""
    global next_prescription_id
    
    # 使用當前日期如果沒有提供日期
    prescription_date = prescription.prescription_date or datetime.now().date().isoformat()
    
    new_prescription = {
        "id": next_prescription_id,
        "patient_name": prescription.patient_name,
        "doctor_name": prescription.doctor_name,
        "medicines": [med.dict() for med in prescription.medicines],
        "diagnosis": prescription.diagnosis,
        "instructions": prescription.instructions,
        "priority": prescription.priority,
        "prescription_date": prescription_date,
        "status": "pending",
        "created_time": datetime.now().isoformat()
    }
    
    prescriptions_db.append(new_prescription)
    
    # 添加初始狀態記錄
    status_record = {
        "prescription_id": next_prescription_id,
        "status": "pending",
        "notes": "處方已開立",
        "timestamp": datetime.now().isoformat()
    }
    prescription_status_db.append(status_record)
    
    next_prescription_id += 1
    
    return {
        "message": "處方創建成功",
        "prescription": new_prescription
    }

@router.get("/")
async def get_all_prescriptions():
    """獲取所有處方"""
    return prescriptions_db

@router.get("/{prescription_id}")
async def get_prescription(prescription_id: int):
    """獲取特定處方"""
    for prescription in prescriptions_db:
        if prescription["id"] == prescription_id:
            return prescription
    
    raise HTTPException(status_code=404, detail="處方未找到")

@router.get("/status/{status}")
async def get_prescriptions_by_status(status: str):
    """按狀態篩選處方"""
    filtered = [p for p in prescriptions_db if p["status"] == status]
    return filtered

@router.get("/doctor/{doctor_name}")
async def get_prescriptions_by_doctor(doctor_name: str):
    """按醫師篩選處方"""
    filtered = [p for p in prescriptions_db if p["doctor_name"] == doctor_name]
    return filtered

@router.put("/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_update: PrescriptionStatus):
    """更新處方狀態"""
    # 查找處方
    prescription = None
    for p in prescriptions_db:
        if p["id"] == prescription_id:
            prescription = p
            break
    
    if not prescription:
        raise HTTPException(status_code=404, detail="處方未找到")
    
    # 更新狀態
    old_status = prescription["status"]
    prescription["status"] = status_update.status
    prescription["updated_time"] = datetime.now().isoformat()
    
    # 添加狀態變更記錄
    status_record = {
        "prescription_id": prescription_id,
        "old_status": old_status,
        "new_status": status_update.status,
        "notes": status_update.notes,
        "timestamp": datetime.now().isoformat()
    }
    prescription_status_db.append(status_record)
    
    return {
        "message": "處方狀態更新成功",
        "prescription": prescription,
        "status_change": status_record
    }

@router.delete("/{prescription_id}")
async def delete_prescription(prescription_id: int):
    """刪除處方"""
    for i, prescription in enumerate(prescriptions_db):
        if prescription["id"] == prescription_id:
            deleted_prescription = prescriptions_db.pop(i)
            
            # 添加刪除記錄
            status_record = {
                "prescription_id": prescription_id,
                "status": "deleted",
                "notes": "處方已刪除",
                "timestamp": datetime.now().isoformat()
            }
            prescription_status_db.append(status_record)
            
            return {
                "message": "處方刪除成功",
                "prescription": deleted_prescription
            }
    
    raise HTTPException(status_code=404, detail="處方未找到")

@router.get("/status/history/{prescription_id}")
async def get_prescription_status_history(prescription_id: int):
    """獲取處方狀態歷史"""
    history = [s for s in prescription_status_db if s["prescription_id"] == prescription_id]
    if not history:
        raise HTTPException(status_code=404, detail="處方狀態歷史未找到")
    
    return {
        "prescription_id": prescription_id,
        "status_history": history
    }
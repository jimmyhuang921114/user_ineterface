#!/usr/bin/env python3
"""
處方管理API模組
Prescription Management API Module
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime

# 創建API路由器
router = APIRouter(prefix="/api/prescription", tags=["處方管理"])

# Pydantic模型
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
    prescription_id: int
    status: str  # pending, processing, completed, cancelled
    updated_by: str
    notes: Optional[str] = ""

# 資料儲存 (這些變數將被主伺服器導入)
prescriptions_db = []
prescription_status_db = []
next_prescription_id = 1

# === 處方管理API ===
@router.post("/")
async def create_prescription(prescription: PrescriptionCreate):
    """醫生開立新處方"""
    global next_prescription_id
    
    new_prescription = {
        "id": next_prescription_id,
        "patient_name": prescription.patient_name,
        "doctor_name": prescription.doctor_name,
        "medicines": [med.dict() for med in prescription.medicines],
        "diagnosis": prescription.diagnosis,
        "instructions": prescription.instructions,
        "priority": prescription.priority,
        "prescription_date": prescription.prescription_date or datetime.now().date().isoformat(),
        "status": "pending",
        "created_time": datetime.now().isoformat(),
        "updated_time": datetime.now().isoformat()
    }
    
    prescriptions_db.append(new_prescription)
    next_prescription_id += 1
    
    # 添加初始狀態記錄
    status_record = {
        "prescription_id": new_prescription["id"],
        "status": "pending",
        "updated_by": prescription.doctor_name,
        "updated_time": datetime.now().isoformat(),
        "notes": "處方已開立，等待處理"
    }
    prescription_status_db.append(status_record)
    
    return new_prescription

@router.get("/")
async def get_all_prescriptions():
    """獲取所有處方"""
    return prescriptions_db

@router.get("/{prescription_id}")
async def get_prescription(prescription_id: int):
    """獲取特定處方詳情"""
    prescription = next((p for p in prescriptions_db if p["id"] == prescription_id), None)
    if not prescription:
        raise HTTPException(status_code=404, detail="處方未找到")
    
    # 獲取狀態歷史
    status_history = [s for s in prescription_status_db if s["prescription_id"] == prescription_id]
    
    return {
        "prescription": prescription,
        "status_history": status_history
    }

@router.put("/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_data: dict):
    """更新處方狀態"""
    prescription = next((p for p in prescriptions_db if p["id"] == prescription_id), None)
    if not prescription:
        raise HTTPException(status_code=404, detail="處方未找到")
    
    new_status = status_data.get("status", "pending")
    updated_by = status_data.get("updated_by", "系統")
    notes = status_data.get("notes", f"狀態更新為: {new_status}")
    
    # 更新處方狀態
    prescription["status"] = new_status
    prescription["updated_time"] = datetime.now().isoformat()
    
    # 添加狀態記錄
    status_record = {
        "prescription_id": prescription_id,
        "status": new_status,
        "updated_by": updated_by,
        "updated_time": datetime.now().isoformat(),
        "notes": notes
    }
    prescription_status_db.append(status_record)
    
    return {"success": True, "message": "處方狀態已更新", "new_status": new_status}

@router.get("/status/{status}")
async def get_prescriptions_by_status(status: str):
    """根據狀態獲取處方"""
    filtered_prescriptions = [p for p in prescriptions_db if p["status"] == status]
    return filtered_prescriptions

@router.get("/doctor/{doctor_name}")
async def get_prescriptions_by_doctor(doctor_name: str):
    """獲取特定醫生的處方"""
    doctor_prescriptions = [p for p in prescriptions_db if p["doctor_name"] == doctor_name]
    return doctor_prescriptions

@router.delete("/{prescription_id}")
async def delete_prescription(prescription_id: int):
    """刪除處方"""
    global prescriptions_db
    original_length = len(prescriptions_db)
    prescriptions_db = [p for p in prescriptions_db if p["id"] != prescription_id]
    if len(prescriptions_db) == original_length:
        raise HTTPException(status_code=404, detail="處方未找到")
    return {"success": True, "message": "處方刪除成功", "id": prescription_id}
#!/usr/bin/env python3
"""
API
Prescription Management API Module
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime

# API
router = APIRouter(prefix="/api/prescription", tags=[""])

# Pydantic
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

#  ()
prescriptions_db = []
prescription_status_db = []
next_prescription_id = 1

# === API ===
@router.post("/")
async def create_prescription(prescription: PrescriptionCreate):
    """"""
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

    #
    status_record = {
        "prescription_id": new_prescription["id"],
        "status": "pending",
        "updated_by": prescription.doctor_name,
        "updated_time": datetime.now().isoformat(),
        "notes": ""
    }
    prescription_status_db.append(status_record)

    return new_prescription

@router.get("/")
async def get_all_prescriptions():
    """"""
    return prescriptions_db

@router.get("/{prescription_id}")
async def get_prescription(prescription_id: int):
    """"""
    prescription = next((p for p in prescriptions_db if p["id"] == prescription_id), None)
    if not prescription:
        raise HTTPException(status_code=404, detail="")

    #
    status_history = [s for s in prescription_status_db if s["prescription_id"] == prescription_id]

    return {
        "prescription": prescription,
        "status_history": status_history
    }

@router.put("/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_data: dict):
    """"""
    prescription = next((p for p in prescriptions_db if p["id"] == prescription_id), None)
    if not prescription:
        raise HTTPException(status_code=404, detail="")

    new_status = status_data.get("status", "pending")
    updated_by = status_data.get("updated_by", "")
    notes = status_data.get("notes", f": {new_status}")

    #
    prescription["status"] = new_status
    prescription["updated_time"] = datetime.now().isoformat()

    #
    status_record = {
        "prescription_id": prescription_id,
        "status": new_status,
        "updated_by": updated_by,
        "updated_time": datetime.now().isoformat(),
        "notes": notes
    }
    prescription_status_db.append(status_record)

    return {"success": True, "message": "", "new_status": new_status}

@router.get("/status/{status}")
async def get_prescriptions_by_status(status: str):
    """"""
    filtered_prescriptions = [p for p in prescriptions_db if p["status"] == status]
    return filtered_prescriptions

@router.get("/doctor/{doctor_name}")
async def get_prescriptions_by_doctor(doctor_name: str):
    """"""
    doctor_prescriptions = [p for p in prescriptions_db if p["doctor_name"] == doctor_name]
    return doctor_prescriptions

@router.delete("/{prescription_id}")
async def delete_prescription(prescription_id: int):
    """"""
    global prescriptions_db
    original_length = len(prescriptions_db)
    prescriptions_db = [p for p in prescriptions_db if p["id"] != prescription_id]
    if len(prescriptions_db) == original_length:
        raise HTTPException(status_code=404, detail="")
    return {"success": True, "message": "", "id": prescription_id}
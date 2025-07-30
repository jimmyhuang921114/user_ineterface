#!/usr/bin/env python3
"""
API
Medicine Management API Module
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime

# API
router = APIRouter(prefix="/api/medicine", tags=[""])

# Pydantic
class MedicineBasic(BaseModel):
    name: str
    amount: int
    usage_days: int
    position: str

class MedicineDetailed(BaseModel):
    : Dict[str, str]
    : Dict[str, str]
    : Optional[Dict[str, str]] = {}
    : Dict[str, str]
    : str
    : str
    : Optional[str] = ""
    : Optional[str] = ""
    : Optional[str] = ""
    : Optional[str] = ""
    : Optional[str] = ""

#  ()
medicines_db = []
detailed_medicines_db = {}
next_medicine_id = 1

# === API ===
@router.get("/")
async def get_all_medicines():
    """"""
    return medicines_db

@router.post("/")
async def create_medicine(medicine: MedicineBasic):
    """"""
    global next_medicine_id

    new_medicine = {
        "id": next_medicine_id,
        "name": medicine.name,
        "amount": medicine.amount,
        "usage_days": medicine.usage_days,
        "position": medicine.position,
        "create_time": datetime.now().isoformat()
    }
    medicines_db.append(new_medicine)
    next_medicine_id += 1
    return new_medicine

@router.put("/{medicine_id}")
async def update_medicine(medicine_id: int, medicine: MedicineBasic):
    """"""
    for i, existing_medicine in enumerate(medicines_db):
        if existing_medicine["id"] == medicine_id:
            medicines_db[i].update({
                "name": medicine.name,
                "amount": medicine.amount,
                "usage_days": medicine.usage_days,
                "position": medicine.position,
                "updated_time": datetime.now().isoformat()
            })
            return medicines_db[i]
    raise HTTPException(status_code=404, detail="")

@router.delete("/{medicine_id}")
async def delete_medicine(medicine_id: int):
    """"""
    global medicines_db
    original_length = len(medicines_db)
    medicines_db = [m for m in medicines_db if m["id"] != medicine_id]
    if len(medicines_db) == original_length:
        raise HTTPException(status_code=404, detail="")
    return {"success": True, "message": "", "id": medicine_id}

# === API ===
@router.post("/detailed/")
async def create_detailed_medicine(data: dict):
    """"""
    medicine_name = data.get("medicine_name")
    medicine_data = data.get("medicine_data")

    if not medicine_name or not medicine_data:
        raise HTTPException(status_code=400, detail="")

    detailed_medicines_db[medicine_name] = {
        **medicine_data,
        "created_time": datetime.now().isoformat()
    }
    return {"success": True, "message": f": {medicine_name}"}

@router.get("/detailed/{medicine_name}")
async def get_detailed_medicine(medicine_name: str):
    """"""
    if medicine_name not in detailed_medicines_db:
        raise HTTPException(status_code=404, detail="")
    return detailed_medicines_db[medicine_name]

@router.get("/detailed/")
async def get_all_detailed_medicines():
    """"""
    return detailed_medicines_db

@router.get("/search/code/{code}")
async def search_medicine_by_code(code: str):
    """"""
    results = {}
    code_upper = code.upper()

    for name, data in detailed_medicines_db.items():
        packaging_codes = data.get("", {})
        #
        for code_key, code_value in packaging_codes.items():
            if code_upper in str(code_value).upper():
                results[name] = {
                    **data,
                    "matched_code": {
                        "type": code_key,
                        "value": code_value,
                        "search_term": code
                    }
                }
                break

    if not results:
        raise HTTPException(status_code=404, detail=f" '{code}' ")
    return results

# === API ===
@router.get("/integrated/{medicine_name}")
async def get_integrated_medicine_info(medicine_name: str):
    """ ( + )"""
    result = {
        "medicine_name": medicine_name,
        "basic_info": None,
        "detailed_info": None,
        "status": "not_found"
    }

    #
    for medicine in medicines_db:
        if medicine["name"].lower() == medicine_name.lower():
            result["basic_info"] = medicine
            break

    #
    if medicine_name in detailed_medicines_db:
        result["detailed_info"] = detailed_medicines_db[medicine_name]

    #
    if result["basic_info"] and result["detailed_info"]:
        result["status"] = "complete"  #
    elif result["basic_info"]:
        result["status"] = "basic_only"  #
    elif result["detailed_info"]:
        result["status"] = "detailed_only"  #
    else:
        raise HTTPException(status_code=404, detail="")

    return result
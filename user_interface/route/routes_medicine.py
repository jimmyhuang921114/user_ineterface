from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session
from database.database import get_db
from model import medicine_models
from services import crud_medicine
from schemas.medicine_schema import MedicineCreate, MedicineUpdate, MedicineRead
from typing import List
import json
from datetime import datetime

router = APIRouter()

# add medicine
@router.post("/medicine/", response_model=MedicineRead)
def add_medicine(medicine: MedicineCreate, db: Session = Depends(get_db)):
    return crud_medicine.create_medicine(db, medicine)

# get all medicines from medicine database
@router.get("/medicine/", response_model=List[MedicineRead])
def list_all(db: Session = Depends(get_db)):
    return crud_medicine.get_all_medicines(db)

# get all medicines in JSON format (for export)
@router.get("/medicine/export/json")
def export_medicines_json(db: Session = Depends(get_db)):
    medicines = crud_medicine.get_all_medicines(db)
    json_data = []

    for medicine in medicines:
        json_data.append({
            "id": medicine.id,
            "name": medicine.name,
            "amount": medicine.amount,
            "usage_days": medicine.usage_days,
            "position": medicine.position,
            "create_time": medicine.create_time.isoformat() if medicine.create_time else None
        })

    return JSONResponse(
        content={
            "total_medicines": len(json_data),
            "export_date": datetime.utcnow().isoformat(),
            "medicines": json_data
        },
        headers={
            "Content-Disposition": "attachment; filename=medicines_export.json"
        }
    )

# search medicines by name (supports partial matching)
@router.get("/medicine/search/{query}")
def search_medicines(query: str, db: Session = Depends(get_db)):
    medicines = crud_medicine.search_medicines_by_name(db, query)
    if not medicines:
        raise HTTPException(status_code=404, detail="No medicines found matching the query")
    return medicines

# get the medicine by name from medicine database
@router.get("/medicine/{name}", response_model=MedicineRead)
def get_by_name(name: str, db: Session = Depends(get_db)):
    medicine = crud_medicine.get_medicine_by_name(db, name)
    if medicine is None:
        raise HTTPException(status_code=404, detail="Medicine not found")
    return medicine

# update medicine by id
@router.put("/medicine/{medicine_id}", response_model=MedicineRead)
def update_medicine(medicine_id: int, medicine_update: MedicineUpdate, db: Session = Depends(get_db)):
    medicine = crud_medicine.update_medicine(db, medicine_id, medicine_update)
    if medicine is None:
        raise HTTPException(status_code=404, detail="Medicine not found")
    return medicine

# delete the medicine by id
@router.delete("/medicine/{medicine_id}")
def delete_by_id(medicine_id: int, db: Session = Depends(get_db)):
    success = crud_medicine.delete_medicine_by_id(db, medicine_id)
    if not success:
        raise HTTPException(status_code=404, detail="Medicine not found")
    return {"success": success, "message": "Medicine deleted successfully"}

# batch operations
@router.post("/medicine/batch/")
def batch_create_medicines(medicines: List[MedicineCreate], db: Session = Depends(get_db)):
    created_medicines = []
    for medicine in medicines:
        created_medicine = crud_medicine.create_medicine(db, medicine)
        created_medicines.append(created_medicine)
    return {
        "success": True,
        "created_count": len(created_medicines),
        "medicines": created_medicines
    }

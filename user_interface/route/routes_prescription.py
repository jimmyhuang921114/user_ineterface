from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from database.database import get_db
from model import prescription_models
from services import crud_prescription
from schemas.prescription_schemas import PatientCreate, PrescriptionCreate, PatientRead, PrescriptionRead
from typing import List

router = APIRouter()

# 新增處方
@router.post("/prescription/", response_model=PrescriptionRead)
def add_prescription(prescription: PrescriptionCreate, db: Session = Depends(get_db)):
    return crud_prescription.create_prescription(db, prescription)

# 取得所有處方
@router.get("/prescription/", response_model=List[PrescriptionRead])
def list_all_prescriptions(db: Session = Depends(get_db)):
    return crud_prescription.get_all_prescriptions(db)

# 根據病患姓名查找處方
@router.get("/prescription/by_patient_name/{name}", response_model=List[PrescriptionRead])
def get_by_patient_name(name: str, db: Session = Depends(get_db)):
    return crud_prescription.get_prescription_by_patient_name(db, name)

# 根據病患 ID 查找處方
@router.get("/prescription/by_patient_id/{patient_id}", response_model=List[PrescriptionRead])
def get_by_patient_id(patient_id: int, db: Session = Depends(get_db)):
    return crud_prescription.get_prescriptions_by_patient_id(db, patient_id)

# 根據 ID 刪除處方
@router.delete("/prescription/{id}")
def delete_by_id(id: int, db: Session = Depends(get_db)):
    success = crud_prescription.delete_prescription_by_id(db, id)
    return {"success": success}

# 同時建立病患與處方
@router.post("/prescription/register", response_model=PatientRead)
def register_patient_and_prescriptions(data: PatientCreate, db: Session = Depends(get_db)):
    return crud_prescription.create_patient_with_prescriptions(db, data)

# 額外：取得所有病患與處方
@router.get("/patients_with_prescriptions", response_model=List[PatientRead])
def get_patients_with_prescriptions(db: Session = Depends(get_db)):
    return crud_prescription.get_patients_with_prescriptions(db)
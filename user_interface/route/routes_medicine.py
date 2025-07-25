from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from database.database import get_db  
from model import medicine_models
from services import crud_medicine
from schemas.medicine_schema import MedicineCreate  # 如果有 schema 分層

router = APIRouter()

# 新增藥物
@router.post("/medicine/")
def add_medicine(medicine: MedicineCreate, db: Session = Depends(get_db)):
    return crud_medicine.create_medicine(db, medicine)

# 取得所有藥物
@router.get("/medicine/")
def list_all(db: Session = Depends(get_db)):
    return crud_medicine.get_all_medicines(db)

# 根據藥品名稱查詢
@router.get("/medicine/{name}")
def get_by_name(name: str, db: Session = Depends(get_db)):
    return crud_medicine.get_medicine_by_name(db, name)

# 根據 ID 刪除藥品
@router.delete("/medicine/{id}")
def delete_by_id(id: int, db: Session = Depends(get_db)):
    success = crud_medicine.delete_medicine_by_id(db, id)
    return {"success": success}

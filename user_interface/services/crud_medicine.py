# hospital_datebase/med_sys/crud_medicine.py

from sqlalchemy.orm import Session
from model import medicine_models
from schemas.medicine_schema import MedicineCreate, MedicineUpdate

#create medicine
def create_medicine(db: Session, medicine: MedicineCreate):
    db_medicine = medicine_models.Medicine(**medicine.dict())
    db.add(db_medicine)
    db.commit()
    db.refresh(db_medicine)
    return db_medicine

#get the medicine by name
def get_medicine_by_name(db: Session, name: str):
    return db.query(medicine_models.Medicine).filter(medicine_models.Medicine.name == name).first()

#search medicines by name (partial matching)
def search_medicines_by_name(db: Session, query: str):
    return db.query(medicine_models.Medicine).filter(
        medicine_models.Medicine.name.ilike(f"%{query}%")
    ).all()

#get medicine by id
def get_medicine_by_id(db: Session, medicine_id: int):
    return db.query(medicine_models.Medicine).filter(medicine_models.Medicine.id == medicine_id).first()

#get all the medicine in medicine database
def get_all_medicines(db: Session):
    return db.query(medicine_models.Medicine).all()

#update medicine by id
def update_medicine(db: Session, medicine_id: int, medicine_update: MedicineUpdate):
    medicine = get_medicine_by_id(db, medicine_id)
    if medicine:
        update_data = medicine_update.dict(exclude_unset=True)
        for field, value in update_data.items():
            setattr(medicine, field, value)
        db.commit()
        db.refresh(medicine)
        return medicine
    return None

#delete the medicine by id
def delete_medicine_by_id(db: Session, medicine_id: int):
    medicine = get_medicine_by_id(db, medicine_id)
    if medicine:
        db.delete(medicine)
        db.commit()
        return True
    return False

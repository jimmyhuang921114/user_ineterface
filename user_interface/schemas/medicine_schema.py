from sqlmodel import SQLModel
from typing import Optional
from datetime import datetime


# POST body
class MedicineCreate(SQLModel):
    name: str
    amount: int
    usage_days: int
    position: str


# /GET
class MedicineRead(SQLModel):
    id: int
    name: str
    amount: int
    usage_days: int
    position: str
    create_time: datetime


# PATCH / PUT
class MedicineUpdate(SQLModel):
    name: Optional[str] = None
    amount: Optional[int] = None
    usage_days: Optional[int] = None
    position: Optional[str] = None

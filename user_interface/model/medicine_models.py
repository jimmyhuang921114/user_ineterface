from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class Medicine(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    amount: int
    usage_days: int
    position: str
    create_time: datetime = Field(default_factory=datetime.utcnow)

class InventoryLog(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    medicine_id: int
    change: int
    timestamp: datetime = Field(default_factory=datetime.utcnow)

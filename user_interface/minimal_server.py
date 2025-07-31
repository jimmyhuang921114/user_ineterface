#!/usr/bin/env python3
"""
最簡單的醫療系統伺服器
Minimal Medical System Server
"""

from fastapi import FastAPI
from pydantic import BaseModel
from typing import List, Dict, Any
from datetime import datetime
import uvicorn

# 創建FastAPI應用
app = FastAPI()

# 簡單的數據模型
class MedicalRecord(BaseModel):
    patient_id: str
    patient_name: str
    doctor_name: str
    diagnosis: str
    symptoms: str

# 內存數據庫
records = []
medicines = []
next_id = 1

@app.get("/")
async def root():
    return {"message": "醫療系統正在運行", "status": "OK"}

@app.get("/test")
async def test():
    return {"test": "success", "chinese": "中文測試正常"}

@app.post("/api/medical_record/")
async def create_record(record: MedicalRecord):
    global next_id
    new_record = {
        "id": next_id,
        "patient_id": record.patient_id,
        "patient_name": record.patient_name,
        "doctor_name": record.doctor_name,
        "diagnosis": record.diagnosis,
        "symptoms": record.symptoms,
        "created_time": datetime.now().isoformat()
    }
    records.append(new_record)
    next_id += 1
    return {"message": "病例創建成功", "record": new_record}

@app.get("/api/medical_record/{record_id}")
async def get_record(record_id: int):
    for record in records:
        if record["id"] == record_id:
            return record
    return {"error": "病例未找到"}

@app.get("/api/records/all")
async def get_all_records():
    return {"records": records, "count": len(records)}

@app.get("/api/status")
async def status():
    return {
        "server": "minimal_medical_server",
        "status": "running",
        "records_count": len(records),
        "medicines_count": len(medicines)
    }

if __name__ == "__main__":
    print("正在啟動最簡單的醫療系統伺服器...")
    print("伺服器將在 http://0.0.0.0:8000 啟動")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
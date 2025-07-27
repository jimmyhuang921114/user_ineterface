#!/usr/bin/env python3
"""
醫院藥物管理系統 - 簡化工作版本
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from datetime import datetime
from typing import List
import uvicorn

# 數據模型
class Medicine(BaseModel):
    name: str
    amount: int
    usage_days: int
    position: str

# 建立應用
app = FastAPI(title="醫院藥物管理系統", description="簡化工作版本")

# 啟用 CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 內存數據庫
medicines = []
next_id = 1

# 根路由
@app.get("/")
def read_root():
    return {
        "message": "🏥 醫院藥物管理系統",
        "status": "✅ 運行中",
        "version": "1.0",
        "medicine_count": len(medicines),
        "endpoints": {
            "test": "/api/test",
            "medicines": "/api/medicine/",
            "docs": "/docs"
        }
    }

# 測試端點
@app.get("/api/test")
def test_endpoint():
    return {
        "status": "success",
        "message": "✅ API連接正常",
        "time": datetime.now().isoformat(),
        "medicines_count": len(medicines)
    }

# 獲取所有藥物
@app.get("/api/medicine/")
def get_all_medicines():
    return medicines

# 新增藥物
@app.post("/api/medicine/")
def create_medicine(medicine: Medicine):
    global next_id
    
    new_medicine = {
        "id": next_id,
        "name": medicine.name,
        "amount": medicine.amount,
        "usage_days": medicine.usage_days,
        "position": medicine.position,
        "create_time": datetime.now().isoformat()
    }
    
    medicines.append(new_medicine)
    next_id += 1
    
    return new_medicine

# 獲取特定藥物
@app.get("/api/medicine/{medicine_id}")
def get_medicine(medicine_id: int):
    for med in medicines:
        if med["id"] == medicine_id:
            return med
    raise HTTPException(status_code=404, detail="藥物未找到")

# 更新藥物
@app.put("/api/medicine/{medicine_id}")
def update_medicine(medicine_id: int, medicine: Medicine):
    for i, med in enumerate(medicines):
        if med["id"] == medicine_id:
            medicines[i].update({
                "name": medicine.name,
                "amount": medicine.amount,
                "usage_days": medicine.usage_days,
                "position": medicine.position
            })
            return medicines[i]
    raise HTTPException(status_code=404, detail="藥物未找到")

# 刪除藥物
@app.delete("/api/medicine/{medicine_id}")
def delete_medicine(medicine_id: int):
    global medicines
    original_count = len(medicines)
    medicines = [med for med in medicines if med["id"] != medicine_id]
    
    if len(medicines) == original_count:
        raise HTTPException(status_code=404, detail="藥物未找到")
    
    return {"message": "藥物刪除成功", "id": medicine_id}

# JSON導出
@app.get("/api/medicine/export/json")
def export_json():
    export_data = {
        "total": len(medicines),
        "export_time": datetime.now().isoformat(),
        "data": medicines
    }
    return JSONResponse(
        content=export_data,
        headers={"Content-Disposition": "attachment; filename=medicines.json"}
    )

# 初始化測試數據
def init_data():
    global medicines, next_id
    
    test_medicines = [
        {
            "id": 1,
            "name": "阿斯匹靈",
            "amount": 100,
            "usage_days": 30,
            "position": "A1-01",
            "create_time": datetime.now().isoformat()
        },
        {
            "id": 2,
            "name": "維他命C",
            "amount": 50,
            "usage_days": 60,
            "position": "A1-02",
            "create_time": datetime.now().isoformat()
        }
    ]
    
    medicines.extend(test_medicines)
    next_id = 3

if __name__ == "__main__":
    print("🏥 醫院藥物管理系統 - 簡化版")
    print("=" * 50)
    
    # 初始化數據
    init_data()
    print(f"✅ 載入了 {len(medicines)} 個測試藥物")
    
    print("🚀 啟動伺服器...")
    print("🌐 URL: http://localhost:8000")
    print("📋 API文檔: http://localhost:8000/docs")
    print("🧪 測試: http://localhost:8000/api/test")
    print("=" * 50)
    
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    except KeyboardInterrupt:
        print("\n👋 伺服器已停止")
    except Exception as e:
        print(f"❌ 錯誤: {e}")
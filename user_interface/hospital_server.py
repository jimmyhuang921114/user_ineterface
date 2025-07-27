#!/usr/bin/env python3
"""
醫院藥物管理系統 - 完整版本
Hospital Medicine Management System - Complete Version
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from pydantic import BaseModel
from pathlib import Path
from datetime import datetime
from typing import List, Optional
import uvicorn

# Pydantic 資料模型
class MedicineCreate(BaseModel):
    name: str
    amount: int
    usage_days: int
    position: str

class MedicineUpdate(BaseModel):
    name: Optional[str] = None
    amount: Optional[int] = None
    usage_days: Optional[int] = None
    position: Optional[str] = None

class MedicineResponse(BaseModel):
    id: int
    name: str
    amount: int
    usage_days: int
    position: str
    create_time: str

# 建立 FastAPI 應用
app = FastAPI(
    title="醫院藥物管理系統",
    description="Hospital Medicine Management System",
    version="1.0.0"
)

# CORS 設定
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 靜態文件設定
base_dir = Path(__file__).resolve().parent
static_dir = base_dir / "static"

# 掛載靜態文件
try:
    if (static_dir / "css").exists():
        app.mount("/css", StaticFiles(directory=static_dir / "css"), name="css")
    if (static_dir / "js").exists():
        app.mount("/js", StaticFiles(directory=static_dir / "js"), name="js")
    if (static_dir / "html").exists():
        app.mount("/html", StaticFiles(directory=static_dir / "html"), name="html")
except Exception as e:
    print(f"警告: 靜態文件掛載失敗: {e}")

# 內存資料庫 (用於演示)
medicines_db = []
next_id = 1

# 根路由
@app.get("/")
async def root():
    return {
        "message": "🏥 醫院藥物管理系統",
        "status": "✅ 運行中",
        "version": "1.0.0",
        "medicine_count": len(medicines_db),
        "endpoints": {
            "medicines": "/api/medicine/",
            "export": "/api/medicine/export/json",
            "test": "/api/test",
            "docs": "/docs",
            "medicine_page": "/Medicine.html"
        }
    }

# 測試端點
@app.get("/api/test")
async def test_api():
    return {
        "status": "success",
        "message": "✅ API連接正常",
        "time": datetime.now().isoformat(),
        "medicine_count": len(medicines_db)
    }

# 藥物管理 API
@app.get("/api/medicine/", response_model=List[MedicineResponse])
async def get_all_medicines():
    """獲取所有藥物"""
    return medicines_db

@app.post("/api/medicine/", response_model=MedicineResponse)
async def create_medicine(medicine: MedicineCreate):
    """新增藥物"""
    global next_id
    
    new_medicine = {
        "id": next_id,
        "name": medicine.name,
        "amount": medicine.amount,
        "usage_days": medicine.usage_days,
        "position": medicine.position,
        "create_time": datetime.now().isoformat()
    }
    
    medicines_db.append(new_medicine)
    next_id += 1
    
    return new_medicine

@app.get("/api/medicine/{medicine_id}", response_model=MedicineResponse)
async def get_medicine(medicine_id: int):
    """根據ID獲取藥物"""
    for medicine in medicines_db:
        if medicine["id"] == medicine_id:
            return medicine
    raise HTTPException(status_code=404, detail="藥物未找到")

@app.get("/api/medicine/search/{name}")
async def search_medicine_by_name(name: str):
    """根據名稱搜尋藥物"""
    results = []
    for medicine in medicines_db:
        if name.lower() in medicine["name"].lower():
            results.append(medicine)
    
    if not results:
        raise HTTPException(status_code=404, detail="找不到匹配的藥物")
    
    return results

@app.put("/api/medicine/{medicine_id}", response_model=MedicineResponse)
async def update_medicine(medicine_id: int, medicine_update: MedicineUpdate):
    """更新藥物資訊"""
    for i, medicine in enumerate(medicines_db):
        if medicine["id"] == medicine_id:
            update_data = medicine_update.dict(exclude_unset=True)
            for key, value in update_data.items():
                if key in medicine and key not in ["id", "create_time"]:
                    medicine[key] = value
            medicines_db[i] = medicine
            return medicine
    
    raise HTTPException(status_code=404, detail="藥物未找到")

@app.delete("/api/medicine/{medicine_id}")
async def delete_medicine(medicine_id: int):
    """刪除藥物"""
    global medicines_db
    original_length = len(medicines_db)
    medicines_db = [m for m in medicines_db if m["id"] != medicine_id]
    
    if len(medicines_db) == original_length:
        raise HTTPException(status_code=404, detail="藥物未找到")
    
    return {"success": True, "message": "藥物刪除成功", "id": medicine_id}

# JSON 導出功能
@app.get("/api/medicine/export/json")
async def export_medicines_json():
    """導出所有藥物為JSON格式"""
    export_data = {
        "total_medicines": len(medicines_db),
        "export_date": datetime.now().isoformat(),
        "system": "醫院藥物管理系統",
        "version": "1.0.0",
        "medicines": medicines_db
    }
    
    return JSONResponse(
        content=export_data,
        headers={
            "Content-Disposition": "attachment; filename=medicines_export.json"
        }
    )

# 批量操作
@app.post("/api/medicine/batch/")
async def batch_create_medicines(medicines: List[MedicineCreate]):
    """批量新增藥物"""
    global next_id
    created_medicines = []
    
    for medicine in medicines:
        new_medicine = {
            "id": next_id,
            "name": medicine.name,
            "amount": medicine.amount,
            "usage_days": medicine.usage_days,
            "position": medicine.position,
            "create_time": datetime.now().isoformat()
        }
        medicines_db.append(new_medicine)
        created_medicines.append(new_medicine)
        next_id += 1
    
    return {
        "success": True,
        "created_count": len(created_medicines),
        "medicines": created_medicines
    }

# 前端頁面路由
@app.get("/Medicine.html")
async def serve_medicine_page():
    """提供藥物管理頁面"""
    html_file = static_dir / "html" / "Medicine.html"
    if html_file.exists():
        return FileResponse(html_file, media_type='text/html')
    else:
        return JSONResponse(
            content={
                "error": "Medicine.html not found",
                "message": "請確保 static/html/Medicine.html 文件存在"
            },
            status_code=404
        )

@app.get("/Prescription.html")
async def serve_prescription_page():
    """提供處方籤頁面"""
    html_file = static_dir / "html" / "Prescription.html"
    if html_file.exists():
        return FileResponse(html_file, media_type='text/html')
    else:
        return JSONResponse(content={"error": "Prescription.html not found"}, status_code=404)

@app.get("/doctor.html")
async def serve_doctor_page():
    """提供醫生頁面"""
    html_file = static_dir / "html" / "doctor.html"
    if html_file.exists():
        return FileResponse(html_file, media_type='text/html')
    else:
        return JSONResponse(content={"error": "doctor.html not found"}, status_code=404)

# 初始化測試資料
def init_test_data():
    """初始化測試資料"""
    global medicines_db, next_id
    
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
        },
        {
            "id": 3,
            "name": "感冒糖漿",
            "amount": 25,
            "usage_days": 14,
            "position": "B2-03",
            "create_time": datetime.now().isoformat()
        }
    ]
    
    medicines_db.extend(test_medicines)
    next_id = 4

if __name__ == "__main__":
    print("🏥 醫院藥物管理系統")
    print("=" * 60)
    print("🚀 正在啟動伺服器...")
    print(f"📁 靜態文件目錄: {static_dir}")
    print("🌐 伺服器地址: http://localhost:8000")
    print("📱 API文檔: http://localhost:8000/docs")
    print("💊 藥物管理頁面: http://localhost:8000/Medicine.html")
    print("📋 處方籤頁面: http://localhost:8000/Prescription.html")
    print("👨‍⚕️ 醫生頁面: http://localhost:8000/doctor.html")
    print("🧪 API測試: http://localhost:8000/api/test")
    print("📦 JSON導出: http://localhost:8000/api/medicine/export/json")
    print("=" * 60)
    
    # 初始化測試資料
    init_test_data()
    print(f"✅ 已載入 {len(medicines_db)} 個測試藥物")
    
    try:
        uvicorn.run(
            app,
            host="0.0.0.0",
            port=8000,
            log_level="info"
        )
    except KeyboardInterrupt:
        print("\n👋 伺服器已停止")
    except Exception as e:
        print(f"❌ 伺服器啟動失敗: {e}")
        exit(1)
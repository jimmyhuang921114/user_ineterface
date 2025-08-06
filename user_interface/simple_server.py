#!/usr/bin/env python3
"""
Simple Hospital Medicine Management System with ROS2 Integration
簡化醫院藥物管理系統 - 整合ROS2
"""

from fastapi import FastAPI, HTTPException, Depends
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
from sqlalchemy.orm import Session
from typing import List, Optional, Dict
import json
import os
import threading
import time
from datetime import datetime

from database import get_db, MedicineBasic, MedicineDetailed, Prescription, PrescriptionMedicine, init_database

# 嘗試導入ROS2模組（如果可用）
try:
    from ros2_integration import init_ros2_node, get_ros2_node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("⚠️ ROS2模組不可用，將使用模擬模式")

# 創建FastAPI應用
app = FastAPI(title="醫院藥物管理系統", version="1.0.0")

# 掛載靜態檔案
app.mount("/static", StaticFiles(directory="static"), name="static")

# 額外掛載CSS和JS路徑
app.mount("/css", StaticFiles(directory="static/css"), name="css")
app.mount("/js", StaticFiles(directory="static/js"), name="js")

# 初始化資料庫
init_database()

# 初始化ROS2節點（如果可用）
ros2_node = None
if ROS2_AVAILABLE:
    ros2_node = init_ros2_node()

@app.get("/")
async def root():
    """根路徑"""
    return {"message": "醫院藥物管理系統", "status": "running", "ros2_available": ROS2_AVAILABLE}

@app.get("/api/health")
async def health_check():
    """健康檢查"""
    ros2_status = "available" if ROS2_AVAILABLE and ros2_node else "unavailable"
    return {
        "status": "healthy", 
        "message": "系統運行正常",
        "ros2_status": ros2_status
    }

# 藥物管理API
@app.post("/api/medicine/")
async def create_medicine(medicine_data: dict, db: Session = Depends(get_db)):
    """創建新藥物"""
    try:
        medicine = MedicineBasic(**medicine_data)
        db.add(medicine)
        db.commit()
        db.refresh(medicine)
        
        result = {"message": "藥物創建成功", "id": medicine.id}
        
        # 如果ROS2可用，發布新藥物資料
        if ROS2_AVAILABLE and ros2_node:
            ros2_node.publish_medicine_data({
                "id": medicine.id,
                "name": medicine.name,
                "amount": medicine.amount,
                "position": medicine.position,
                "manufacturer": medicine.manufacturer,
                "dosage": medicine.dosage,
                "action": "created"
            })
        
        return result
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

@app.get("/api/medicine/basic")
async def get_basic_medicines(db: Session = Depends(get_db)):
    """獲取基本藥物列表"""
    medicines = db.query(MedicineBasic).filter(MedicineBasic.is_active == True).all()
    result = [
        {
            "id": med.id,
            "name": med.name,
            "amount": med.amount,
            "position": med.position,
            "manufacturer": med.manufacturer,
            "dosage": med.dosage
        }
        for med in medicines
    ]
    
    # 如果ROS2可用，發布藥物資料
    if ROS2_AVAILABLE and ros2_node:
        for medicine in result:
            ros2_node.publish_medicine_data(medicine)
    
    return result

@app.get("/api/medicine/detailed")
async def get_detailed_medicines(db: Session = Depends(get_db)):
    """獲取詳細藥物列表"""
    detailed_medicines = db.query(MedicineDetailed).all()
    return [
        {
            "id": med.id,
            "medicine_id": med.medicine_id,
            "description": med.description,
            "ingredient": med.ingredient,
            "category": med.category,
            "usage_method": med.usage_method,
            "unit_dose": med.unit_dose,
            "side_effects": med.side_effects,
            "storage_conditions": med.storage_conditions,
            "expiry_date": med.expiry_date.isoformat() if med.expiry_date else None,
            "barcode": med.barcode,
            "appearance_type": med.appearance_type,
            "notes": med.notes
        }
        for med in detailed_medicines
    ]

@app.get("/api/medicine/{medicine_id}")
async def get_medicine_detail(medicine_id: int, db: Session = Depends(get_db)):
    """獲取藥物詳細資訊"""
    medicine = db.query(MedicineBasic).filter(MedicineBasic.id == medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物不存在")
    
    detailed = db.query(MedicineDetailed).filter(MedicineDetailed.medicine_id == medicine_id).first()
    
    result = {
        "id": medicine.id,
        "name": medicine.name,
        "amount": medicine.amount,
        "position": medicine.position,
        "manufacturer": medicine.manufacturer,
        "dosage": medicine.dosage,
        "prompt": medicine.prompt,
        "detailed": detailed.__dict__ if detailed else None
    }
    
    # 如果ROS2可用，發布完整藥物資料
    if ROS2_AVAILABLE and ros2_node:
        ros2_node.publish_medicine_data(result)
    
    return result

@app.post("/api/medicine/detailed")
async def create_detailed_medicine(medicine_data: dict, db: Session = Depends(get_db)):
    """創建詳細藥物"""
    try:
        detailed_medicine = MedicineDetailed(**medicine_data)
        db.add(detailed_medicine)
        db.commit()
        db.refresh(detailed_medicine)
        return {"message": "詳細藥物創建成功", "id": detailed_medicine.id}
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

@app.post("/api/medicine/unified")
async def create_unified_medicine(medicine_data: dict, db: Session = Depends(get_db)):
    """創建統一藥物（基本+詳細）"""
    try:
        # 分離基本和詳細資料
        basic_data = {
            "name": medicine_data.get("name"),
            "amount": medicine_data.get("amount", 0),
            "position": medicine_data.get("position"),
            "manufacturer": medicine_data.get("manufacturer"),
            "dosage": medicine_data.get("dosage"),
            "prompt": medicine_data.get("prompt")
        }
        
        # 創建基本藥物
        basic_medicine = MedicineBasic(**basic_data)
        db.add(basic_medicine)
        db.flush()  # 獲取ID但不提交
        
        # 創建詳細藥物
        detailed_data = {
            "medicine_id": basic_medicine.id,
            "description": medicine_data.get("description"),
            "ingredient": medicine_data.get("ingredient"),
            "category": medicine_data.get("category"),
            "usage_method": medicine_data.get("usage_method"),
            "unit_dose": medicine_data.get("unit_dose"),
            "side_effects": medicine_data.get("side_effects"),
            "storage_conditions": medicine_data.get("storage_conditions"),
            "expiry_date": medicine_data.get("expiry_date"),
            "barcode": medicine_data.get("barcode"),
            "appearance_type": medicine_data.get("appearance_type"),
            "notes": medicine_data.get("notes")
        }
        
        detailed_medicine = MedicineDetailed(**detailed_data)
        db.add(detailed_medicine)
        
        db.commit()
        
        result = {
            "message": "統一藥物創建成功",
            "basic_id": basic_medicine.id,
            "detailed_id": detailed_medicine.id
        }
        
        # 如果ROS2可用，發布新藥物資料
        if ROS2_AVAILABLE and ros2_node:
            ros2_node.publish_medicine_data({
                "id": basic_medicine.id,
                "name": basic_medicine.name,
                "amount": basic_medicine.amount,
                "position": basic_medicine.position,
                "manufacturer": basic_medicine.manufacturer,
                "dosage": basic_medicine.dosage,
                "action": "created_unified"
            })
        
        return result
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

# 處方籤管理API
@app.get("/api/prescription/")
async def get_prescriptions(db: Session = Depends(get_db)):
    """獲取處方籤列表"""
    prescriptions = db.query(Prescription).all()
    return [
        {
            "id": p.id,
            "patient_name": p.patient_name,
            "patient_id": p.patient_id,
            "doctor_name": p.doctor_name,
            "diagnosis": p.diagnosis,
            "status": p.status,
            "created_at": p.created_at.isoformat()
        }
        for p in prescriptions
    ]

@app.post("/api/prescription/")
async def create_prescription(prescription_data: dict, db: Session = Depends(get_db)):
    """創建新處方籤"""
    try:
        prescription = Prescription(**prescription_data)
        db.add(prescription)
        db.commit()
        db.refresh(prescription)
        
        result = {"message": "處方籤創建成功", "id": prescription.id}
        
        # 如果ROS2可用，將處方籤加入訂單佇列
        if ROS2_AVAILABLE and ros2_node:
            order_data = {
                "order_id": f"ORDER_{prescription.id:04d}",
                "prescription_id": prescription.id,
                "patient_name": prescription.patient_name,
                "patient_id": prescription.patient_id,
                "doctor_name": prescription.doctor_name,
                "diagnosis": prescription.diagnosis,
                "status": "pending",
                "created_at": prescription.created_at.isoformat()
            }
            ros2_node.add_order(order_data)
        
        return result
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

@app.get("/api/prescription/{prescription_id}")
async def get_prescription_detail(prescription_id: int, db: Session = Depends(get_db)):
    """獲取處方籤詳細資訊"""
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="處方籤不存在")
    
    medicines = db.query(PrescriptionMedicine).filter(PrescriptionMedicine.prescription_id == prescription_id).all()
    
    return {
        "id": prescription.id,
        "patient_name": prescription.patient_name,
        "patient_id": prescription.patient_id,
        "doctor_name": prescription.doctor_name,
        "diagnosis": prescription.diagnosis,
        "status": prescription.status,
        "created_at": prescription.created_at.isoformat(),
        "medicines": [
            {
                "id": pm.id,
                "medicine_id": pm.medicine_id,
                "dosage": pm.dosage,
                "frequency": pm.frequency,
                "duration": pm.duration,
                "instructions": pm.instructions,
                "quantity": pm.quantity
            }
            for pm in medicines
        ]
    }

# ROS2相關API
@app.get("/api/ros2/status")
async def get_ros2_status():
    """獲取ROS2狀態"""
    if not ROS2_AVAILABLE:
        return {"status": "unavailable", "message": "ROS2模組未安裝"}
    
    if not ros2_node:
        return {"status": "uninitialized", "message": "ROS2節點未初始化"}
    
    queue_status = ros2_node.get_queue_status()
    return {
        "status": "running",
        "queue_size": queue_status["queue_size"],
        "processing": queue_status["processing"],
        "current_order": queue_status["current_order"]
    }

@app.post("/api/ros2/order")
async def add_ros2_order(order_data: dict):
    """添加ROS2訂單"""
    if not ROS2_AVAILABLE or not ros2_node:
        raise HTTPException(status_code=503, detail="ROS2服務不可用")
    
    try:
        # 生成訂單ID
        order_id = f"ORDER_{int(time.time())}"
        order_data["order_id"] = order_id
        order_data["timestamp"] = time.time()
        
        # 添加到ROS2佇列
        ros2_node.add_order(order_data)
        
        return {
            "message": "訂單已加入佇列",
            "order_id": order_id,
            "queue_position": ros2_node.get_queue_status()["queue_size"]
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"添加訂單失敗: {str(e)}")

@app.get("/api/ros2/queue")
async def get_ros2_queue():
    """獲取ROS2佇列狀態"""
    if not ROS2_AVAILABLE or not ros2_node:
        raise HTTPException(status_code=503, detail="ROS2服務不可用")
    
    return ros2_node.get_queue_status()

# 網頁界面路由
@app.get("/favicon.ico")
async def favicon():
    """網站圖標"""
    return FileResponse("static/favicon.ico", media_type="image/x-icon")



@app.get("/Prescription.html")
async def prescription_page():
    """處方籤管理頁面"""
    return FileResponse("static/Prescription.html")

@app.get("/doctor.html")
async def doctor_page():
    """醫生工作站頁面"""
    return FileResponse("static/doctor.html")

@app.get("/integrated_medicine_management.html")
async def integrated_page():
    """整合管理頁面"""
    return FileResponse("static/integrated_medicine_management.html")

@app.get("/test_all_functions.html")
async def test_page():
    """功能測試頁面"""
    return FileResponse("static/test_all_functions.html")

if __name__ == "__main__":
    import uvicorn
    print("🏥 簡化醫院藥物管理系統")
    print("=" * 50)
    print("🌐 網頁界面: http://localhost:8001/Medicine.html")
    print("📋 處方籤管理: http://localhost:8001/Prescription.html")
    print("👨‍⚕️ 醫生界面: http://localhost:8001/doctor.html")
    print("📖 API文檔: http://localhost:8001/docs")
    print(f"🤖 ROS2狀態: {'可用' if ROS2_AVAILABLE else '不可用'}")
    print("=" * 50)
    
    uvicorn.run(app, host="0.0.0.0", port=8001, log_level="info")
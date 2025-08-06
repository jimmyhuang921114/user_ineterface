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
import logging
from datetime import datetime

# 設置詳細的日誌配置
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('debug.log', encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("hospital_system")

from database import get_db, MedicineBasic, MedicineDetailed, Prescription, PrescriptionMedicine, init_database

# 嘗試導入ROS2模組（如果可用）
try:
    from ros2_integration import init_ros2_node, get_ros2_node
    ROS2_AVAILABLE = True
    ROS2_MODE = "full"
    print("✅ 完整ROS2模組已載入")
except ImportError:
    try:
        from ros2_mock import init_ros2_node, get_ros2_node
        ROS2_AVAILABLE = True
        ROS2_MODE = "mock"
        print("🤖 使用模擬ROS2模式")
    except ImportError:
        ROS2_AVAILABLE = False
        ROS2_MODE = "none"
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

# 添加樣本藥物資料（如果資料庫為空）
def add_sample_medicines():
    """添加樣本藥物資料"""
    from sqlalchemy.orm import sessionmaker
    from database import engine, MedicineBasic, MedicineDetailed
    
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    db = SessionLocal()
    
    try:
        # 檢查是否已有藥物資料
        existing_count = db.query(MedicineBasic).count()
        if existing_count > 0:
            logger.info(f"📊 資料庫已有 {existing_count} 種藥物，跳過樣本資料初始化")
            return
        
        logger.info("📦 正在添加樣本藥物資料...")
        
        # 樣本基本藥物資料
        sample_medicines = [
            {"name": "阿斯匹靈", "amount": 100, "position": "A01", "manufacturer": "台灣製藥", "dosage": "100mg"},
            {"name": "維他命C", "amount": 200, "position": "A02", "manufacturer": "健康製藥", "dosage": "500mg"},
            {"name": "普拿疼", "amount": 150, "position": "A03", "manufacturer": "止痛製藥", "dosage": "500mg"},
            {"name": "感冒糖漿", "amount": 50, "position": "B01", "manufacturer": "感冒製藥", "dosage": "10ml"},
            {"name": "胃藥", "amount": 80, "position": "B02", "manufacturer": "腸胃製藥", "dosage": "200mg"}
        ]
        
        for med_data in sample_medicines:
            medicine = MedicineBasic(**med_data, is_active=True)
            db.add(medicine)
        
        db.commit()
        logger.info(f"✅ 已添加 {len(sample_medicines)} 種樣本藥物")
        
        # 為前3種藥物添加詳細資料
        medicines = db.query(MedicineBasic).limit(3).all()
        detailed_data = [
            {"description": "解熱鎮痛劑", "ingredient": "乙醯水楊酸", "category": "鎮痛劑", "usage_method": "口服"},
            {"description": "維生素補充劑", "ingredient": "維生素C", "category": "維生素", "usage_method": "口服"},
            {"description": "解熱鎮痛劑", "ingredient": "乙醯胺酚", "category": "鎮痛劑", "usage_method": "口服"}
        ]
        
        for i, med in enumerate(medicines):
            if i < len(detailed_data):
                detailed = MedicineDetailed(
                    medicine_id=med.id,
                    **detailed_data[i],
                    unit_dose=med.dosage,
                    side_effects="請依醫師指示使用",
                    storage_conditions="陰涼乾燥處保存",
                    appearance_type="錠劑"
                )
                db.add(detailed)
        
        db.commit()
        logger.info("✅ 樣本詳細資料添加完成")
        
    except Exception as e:
        logger.error(f"❌ 添加樣本資料失敗: {str(e)}")
        db.rollback()
    finally:
        db.close()

add_sample_medicines()

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
    logger.info("💓 系統健康檢查請求")
    ros2_status = "full" if ROS2_MODE == "full" else "mock" if ROS2_MODE == "mock" else "unavailable"
    result = {
        "status": "healthy", 
        "message": "系統運行正常",
        "ros2_status": ros2_status,
        "ros2_mode": ROS2_MODE
    }
    logger.info(f"💓 健康檢查結果: {result}")
    return result

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
    logger.info("🔍 開始獲取基本藥物列表")
    try:
        medicines = db.query(MedicineBasic).filter(MedicineBasic.is_active == True).all()
        logger.debug(f"📊 資料庫查詢結果: 找到 {len(medicines)} 種藥物")
        
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
        
        logger.info(f"✅ 基本藥物列表構建完成，共 {len(result)} 種藥物")
        
        # 如果ROS2可用，發布藥物資料
        if ROS2_AVAILABLE and ros2_node:
            logger.debug("📡 ROS2可用，準備發布藥物資料")
            for medicine in result:
                ros2_node.publish_medicine_data(medicine)
            logger.info("📡 藥物資料已發布到ROS2")
        else:
            logger.debug("❌ ROS2不可用，跳過資料發布")
        
        return result
    except Exception as e:
        logger.error(f"❌ 獲取基本藥物列表失敗: {str(e)}")
        raise HTTPException(status_code=500, detail=f"獲取藥物列表失敗: {str(e)}")

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
    logger.info("📋 獲取處方籤列表")
    prescriptions = db.query(Prescription).all()
    
    result = []
    for p in prescriptions:
        # 獲取每張處方籤的藥物數量
        medicine_count = db.query(PrescriptionMedicine).filter(PrescriptionMedicine.prescription_id == p.id).count()
        
        result.append({
            "id": p.id,
            "patient_name": p.patient_name,
            "patient_id": p.patient_id,
            "doctor_name": p.doctor_name,
            "diagnosis": p.diagnosis,
            "status": p.status,
            "created_at": p.created_at.isoformat(),
            "prescription_date": p.created_at.isoformat(),  # 添加處方日期
            "medicine_count": medicine_count,
            "medicines": [{"length": medicine_count}]  # 兼容前端的 medicines.length
        })
    
    logger.info(f"✅ 返回 {len(result)} 張處方籤")
    return result

@app.post("/api/prescription/")
async def create_prescription(prescription_data: dict, db: Session = Depends(get_db)):
    """創建新處方籤"""
    logger.info("📋 開始創建新處方籤")
    logger.debug(f"📋 接收到的處方數據: {prescription_data}")
    
    try:
        # 提取藥物列表
        medicines_list = prescription_data.pop('medicines', [])
        logger.info(f"💊 處方包含 {len(medicines_list)} 種藥物")
        
        # 創建處方籤
        prescription = Prescription(**prescription_data)
        db.add(prescription)
        db.commit()
        db.refresh(prescription)
        logger.info(f"✅ 處方籤基本資料已創建，ID: {prescription.id}")
        
        # 處理藥物列表
        added_medicines = 0
        for i, medicine_info in enumerate(medicines_list):
            logger.debug(f"💊 處理第 {i+1} 個藥物: {medicine_info}")
            
            if isinstance(medicine_info, list) and len(medicine_info) >= 4:
                # 格式: [藥物名稱, 劑量, 數量, 頻率]
                medicine_name = medicine_info[0]
                dosage = medicine_info[1]
                quantity_str = str(medicine_info[2])
                quantity = int(quantity_str) if quantity_str.isdigit() else 1
                frequency = medicine_info[3]
                
                logger.debug(f"💊 藥物詳情 - 名稱: {medicine_name}, 劑量: {dosage}, 數量: {quantity}, 頻率: {frequency}")
                
                # 查找藥物ID
                medicine = db.query(MedicineBasic).filter(MedicineBasic.name == medicine_name).first()
                if medicine:
                    prescription_medicine = PrescriptionMedicine(
                        prescription_id=prescription.id,
                        medicine_id=medicine.id,
                        dosage=dosage,
                        frequency=frequency,
                        duration="7天",  # 預設療程
                        quantity=quantity,
                        instructions=""
                    )
                    db.add(prescription_medicine)
                    added_medicines += 1
                    logger.debug(f"✅ 藥物 {medicine_name} 已加入處方籤")
                else:
                    logger.warning(f"⚠️ 找不到藥物: {medicine_name}")
            else:
                logger.warning(f"⚠️ 藥物資料格式錯誤: {medicine_info}")
        
        db.commit()
        logger.info(f"✅ 處方籤創建完成，共添加 {added_medicines} 種藥物")
        
        result = {"message": "處方籤創建成功", "id": prescription.id}
        
        # 如果ROS2可用，將處方籤加入訂單佇列
        if ROS2_AVAILABLE and ros2_node:
            logger.info("📡 ROS2可用，準備創建訂單")
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
            logger.info(f"📡 ROS2訂單已創建: {order_data['order_id']}")
        else:
            logger.debug("❌ ROS2不可用，跳過訂單創建")
        
        return result
    except Exception as e:
        logger.error(f"❌ 創建處方籤失敗: {str(e)}")
        db.rollback()
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

@app.get("/api/prescription/{prescription_id}")
async def get_prescription_detail(prescription_id: int, db: Session = Depends(get_db)):
    """獲取處方籤詳細資訊"""
    logger.info(f"📋 獲取處方籤詳細資訊: {prescription_id}")
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="處方籤不存在")
    
    medicines = db.query(PrescriptionMedicine).filter(PrescriptionMedicine.prescription_id == prescription_id).all()
    
    # 獲取藥物詳細資訊
    medicine_details = []
    for pm in medicines:
        basic_medicine = db.query(MedicineBasic).filter(MedicineBasic.id == pm.medicine_id).first()
        medicine_details.append({
            "id": pm.id,
            "medicine_id": pm.medicine_id,
            "medicine_name": basic_medicine.name if basic_medicine else "未知藥物",
            "dosage": pm.dosage,
            "frequency": pm.frequency,
            "duration": pm.duration,
            "instructions": pm.instructions,
            "quantity": pm.quantity
        })
    
    result = {
        "id": prescription.id,
        "patient_name": prescription.patient_name,
        "patient_id": prescription.patient_id,
        "doctor_name": prescription.doctor_name,
        "diagnosis": prescription.diagnosis,
        "status": prescription.status,
        "created_at": prescription.created_at.isoformat(),
        "medicines": medicine_details
    }
    
    logger.info(f"✅ 返回處方籤 {prescription_id} 詳細資訊，包含 {len(medicine_details)} 種藥物")
    return result

@app.put("/api/prescription/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_data: dict, db: Session = Depends(get_db)):
    """更新處方籤狀態"""
    logger.info(f"🔄 更新處方籤 {prescription_id} 狀態")
    
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="處方籤不存在")
    
    new_status = status_data.get('status')
    updated_by = status_data.get('updated_by', '系統')
    notes = status_data.get('notes', '')
    
    # 驗證狀態值
    valid_statuses = ['pending', 'processing', 'completed', 'cancelled']
    if new_status not in valid_statuses:
        raise HTTPException(status_code=400, detail=f"無效的狀態值，必須是: {', '.join(valid_statuses)}")
    
    old_status = prescription.status
    prescription.status = new_status
    
    try:
        db.commit()
        logger.info(f"✅ 處方籤 {prescription_id} 狀態已更新: {old_status} → {new_status}")
        
        # 如果狀態變為 processing，且 ROS2 可用，可以觸發處理
        if new_status == 'processing' and ROS2_AVAILABLE and ros2_node:
            logger.info(f"📡 觸發 ROS2 處理處方籤 {prescription_id}")
        
        return {
            "message": "狀態更新成功",
            "prescription_id": prescription_id,
            "old_status": old_status,
            "new_status": new_status,
            "updated_by": updated_by
        }
    except Exception as e:
        db.rollback()
        logger.error(f"❌ 更新處方籤狀態失敗: {str(e)}")
        raise HTTPException(status_code=500, detail=f"更新失敗: {str(e)}")

@app.get("/api/prescription/pending/next")
async def get_next_pending_prescription(db: Session = Depends(get_db)):
    """獲取下一個待處理的處方籤（最舊的 pending 狀態）"""
    logger.info("🔍 查找下一個待處理的處方籤")
    
    # 查找最舊的 pending 狀態處方籤
    prescription = db.query(Prescription)\
        .filter(Prescription.status == 'pending')\
        .order_by(Prescription.created_at.asc())\
        .first()
    
    if not prescription:
        return {"message": "沒有待處理的處方籤", "prescription": None}
    
    # 獲取藥物詳細資訊
    medicines = db.query(PrescriptionMedicine)\
        .filter(PrescriptionMedicine.prescription_id == prescription.id)\
        .all()
    
    medicine_details = []
    for pm in medicines:
        basic_medicine = db.query(MedicineBasic).filter(MedicineBasic.id == pm.medicine_id).first()
        medicine_details.append({
            "medicine_id": pm.medicine_id,
            "medicine_name": basic_medicine.name if basic_medicine else "未知藥物",
            "dosage": pm.dosage,
            "frequency": pm.frequency,
            "duration": pm.duration,
            "quantity": pm.quantity,
            "position": basic_medicine.position if basic_medicine else "未知位置"
        })
    
    result = {
        "message": "找到待處理的處方籤",
        "prescription": {
            "id": prescription.id,
            "patient_name": prescription.patient_name,
            "patient_id": prescription.patient_id,
            "doctor_name": prescription.doctor_name,
            "diagnosis": prescription.diagnosis,
            "status": prescription.status,
            "created_at": prescription.created_at.isoformat(),
            "medicines": medicine_details
        }
    }
    
    logger.info(f"✅ 找到待處理處方籤 {prescription.id}，包含 {len(medicine_details)} 種藥物")
    
    # 如果 ROS2 可用，自動加入處理佇列
    if ROS2_AVAILABLE and ros2_node:
        order_data = {
            "order_id": f"ORDER_{prescription.id:04d}",
            "prescription_id": prescription.id,
            "patient_name": prescription.patient_name,
            "patient_id": prescription.patient_id,
            "doctor_name": prescription.doctor_name,
            "diagnosis": prescription.diagnosis,
            "medicines": medicine_details,
            "status": "queued",
            "priority": "normal"
        }
        ros2_node.add_order(order_data)
        logger.info(f"📡 處方籤 {prescription.id} 已自動加入 ROS2 處理佇列")
    
    return result

@app.post("/api/ros2/request-next-order")
async def request_next_order(request_data: dict, db: Session = Depends(get_db)):
    """ROS2 主控制器請求下一個最舊的待處理訂單"""
    logger.info("🤖 ROS2 主控制器請求下一個訂單")
    
    requester_id = request_data.get("requester_id", "unknown")
    requested_priority = request_data.get("priority", "any")  # "high", "normal", "any"
    
    # 查找最舊的 pending 狀態處方籤
    query = db.query(Prescription).filter(Prescription.status == 'pending')
    
    # 按創建時間排序，優先處理最舊的
    prescription = query.order_by(Prescription.created_at.asc()).first()
    
    if not prescription:
        logger.info("📦 沒有待處理的處方籤")
        return {
            "success": False,
            "message": "沒有待處理的處方籤",
            "order": None
        }
    
    # 獲取藥物詳細資訊
    medicines = db.query(PrescriptionMedicine)\
        .filter(PrescriptionMedicine.prescription_id == prescription.id)\
        .all()
    
    medicine_details = []
    for pm in medicines:
        basic_medicine = db.query(MedicineBasic).filter(MedicineBasic.id == pm.medicine_id).first()
        medicine_details.append({
            "medicine_id": pm.medicine_id,
            "medicine_name": basic_medicine.name if basic_medicine else "未知藥物",
            "dosage": pm.dosage,
            "frequency": pm.frequency,
            "duration": pm.duration,
            "quantity": pm.quantity,
            "position": basic_medicine.position if basic_medicine else "未知位置",
            "manufacturer": basic_medicine.manufacturer if basic_medicine else "未知廠商"
        })
    
    # 將狀態更新為 processing
    prescription.status = 'processing'
    db.commit()
    
    order_data = {
        "order_id": f"ORDER_{prescription.id:04d}",
        "prescription_id": prescription.id,
        "patient_info": {
            "name": prescription.patient_name,
            "id": prescription.patient_id
        },
        "doctor_name": prescription.doctor_name,
        "diagnosis": prescription.diagnosis,
        "medicines": medicine_details,
        "created_at": prescription.created_at.isoformat(),
        "assigned_to": requester_id,
        "priority": "normal",
        "estimated_duration": len(medicine_details) * 60  # 每種藥物估計1分鐘
    }
    
    logger.info(f"📦 分配訂單給 ROS2 主控制器: {order_data['order_id']} (處方籤 {prescription.id})")
    logger.info(f"📦 包含 {len(medicine_details)} 種藥物")
    
    return {
        "success": True,
        "message": f"分配訂單 {order_data['order_id']}",
        "order": order_data
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
    print("🌐 整合管理: http://localhost:8001/integrated_medicine_management.html")
    print("📋 處方籤管理: http://localhost:8001/Prescription.html")
    print("👨‍⚕️ 醫生界面: http://localhost:8001/doctor.html")
    print("📖 API文檔: http://localhost:8001/docs")
    print(f"🤖 ROS2狀態: {'完整模式' if ROS2_MODE == 'full' else '模擬模式' if ROS2_MODE == 'mock' else '不可用'}")
    print("=" * 50)
    
    uvicorn.run(app, host="0.0.0.0", port=8001, log_level="info")
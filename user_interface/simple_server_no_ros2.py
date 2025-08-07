#!/usr/bin/env python3
"""
Hospital Medicine Management System - No ROS2 Version
醫院藥物管理系統 - 無 ROS2 版本
"""

import logging
from datetime import datetime
from fastapi import FastAPI, HTTPException, Depends, File, UploadFile, Form
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from sqlalchemy import create_engine, Column, Integer, String, DateTime, Boolean, Text, Date, Float, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, Session, relationship
from sqlalchemy.sql import func
from typing import List, Dict, Optional
import uvicorn
import os
import json

# 設置日誌
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('hospital_system_no_ros2.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("hospital_system")

# 從 database_clean.py 導入所有資料庫模型
from database_clean import *

# 應用程式實例
app = FastAPI(
    title="Hospital Medicine Management System (No ROS2)",
    description="醫院藥物管理系統 - 無 ROS2 版本",
    version="1.0.0"
)

# 庫存管理函數
def handle_prescription_stock_change(prescription_id: int, old_status: str, new_status: str, db: Session):
    """處理處方籤狀態變化時的庫存變動"""
    logger.info(f"處理處方籤 {prescription_id} 庫存變化: {old_status} → {new_status}")
    
    # 獲取處方籤藥物
    prescription_medicines = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.prescription_id == prescription_id
    ).all()
    
    stock_changes = []
    
    for pm in prescription_medicines:
        medicine = db.query(MedicineBasic).filter(MedicineBasic.id == pm.medicine_id).first()
        if not medicine:
            continue
            
        # 由於創建處方籤時已經扣減庫存，這裡主要處理取消時的庫存恢復
        if old_status in ['active', 'pending', 'processing'] and new_status == 'cancelled':
            # 取消訂單：恢復庫存
            medicine.amount += pm.quantity
            stock_changes.append({
                "medicine_name": medicine.name,
                "action": "restored",
                "quantity": pm.quantity,
                "current": medicine.amount
            })
            logger.info(f"取消訂單恢復庫存: {medicine.name} 增加 {pm.quantity}，現有 {medicine.amount}")
            
        elif old_status == 'cancelled' and new_status in ['pending', 'processing']:
            # 從取消變為處理：重新扣減庫存
            if medicine.amount >= pm.quantity:
                medicine.amount -= pm.quantity
                stock_changes.append({
                    "medicine_name": medicine.name,
                    "action": "deducted",
                    "quantity": pm.quantity,
                    "remaining": medicine.amount
                })
                logger.info(f"重新啟動訂單扣減庫存: {medicine.name} 減少 {pm.quantity}，剩餘 {medicine.amount}")
            else:
                stock_changes.append({
                    "medicine_name": medicine.name,
                    "action": "insufficient",
                    "requested": pm.quantity,
                    "available": medicine.amount
                })
                logger.warning(f"重新啟動失敗，庫存不足: {medicine.name} 需要 {pm.quantity}，只有 {medicine.amount}")
        else:
            # 其他狀態變化（pending → processing → completed）：無需庫存變化
            stock_changes.append({
                "medicine_name": medicine.name,
                "action": "no_change",
                "reason": f"status_change_{old_status}_to_{new_status}"
            })
            logger.info(f"狀態變化無需庫存調整: {medicine.name} ({old_status} → {new_status})")
    
    if stock_changes:
        db.commit()
        logger.info(f"處方籤 {prescription_id} 庫存變化完成，共 {len(stock_changes)} 項藥物")
    
    return stock_changes

def check_stock_availability(prescription_id: int, db: Session):
    """檢查處方籤的庫存可用性"""
    prescription_medicines = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.prescription_id == prescription_id
    ).all()
    
    availability = []
    all_available = True
    
    for pm in prescription_medicines:
        medicine = db.query(MedicineBasic).filter(MedicineBasic.id == pm.medicine_id).first()
        if medicine:
            is_available = medicine.amount >= pm.quantity
            availability.append({
                "medicine_name": medicine.name,
                "required": pm.quantity,
                "available": medicine.amount,
                "sufficient": is_available
            })
            if not is_available:
                all_available = False
        else:
            availability.append({
                "medicine_name": "未知藥物",
                "required": pm.quantity,
                "available": 0,
                "sufficient": False
            })
            all_available = False
    
    return {
        "all_available": all_available,
        "medicines": availability
    }

def _adjust_medicine_stock_impl(medicine_identifier: str, action: str, amount: int, db: Session):
    """調整藥物庫存的實作函數"""
    # 嘗試按 ID 查找
    medicine = None
    if medicine_identifier.isdigit():
        medicine = db.query(MedicineBasic).filter(MedicineBasic.id == int(medicine_identifier)).first()
    
    # 如果按 ID 找不到，則按名稱查找
    if not medicine:
        medicine = db.query(MedicineBasic).filter(MedicineBasic.name == medicine_identifier).first()
    
    if not medicine:
        raise HTTPException(status_code=404, detail=f"找不到藥物: {medicine_identifier}")
    
    old_amount = medicine.amount
    
    if action == "add":
        medicine.amount += amount
    elif action == "subtract":
        if medicine.amount < amount:
            raise HTTPException(status_code=400, detail=f"庫存不足，無法減少 {amount} 個 (目前只有 {medicine.amount} 個)")
        medicine.amount -= amount
    elif action == "set":
        if amount < 0:
            raise HTTPException(status_code=400, detail="庫存數量不能為負數")
        medicine.amount = amount
    else:
        raise HTTPException(status_code=400, detail="無效的操作類型，只支援 add、subtract、set")
    
    medicine.updated_at = datetime.now()
    db.commit()
    
    return {
        "medicine_name": medicine.name,
        "old_amount": old_amount,
        "new_amount": medicine.amount,
        "action": action,
        "change_amount": amount
    }

# 資料庫連接
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# 靜態檔案
app.mount("/static", StaticFiles(directory="static"), name="static")
app.mount("/css", StaticFiles(directory="static/css"), name="css")
app.mount("/js", StaticFiles(directory="static/js"), name="js")

# 前端路由
@app.get("/")
async def root():
    return FileResponse("static/integrated_medicine_management.html")

@app.get("/integrated_medicine_management.html")
async def integrated_medicine_management():
    return FileResponse("static/integrated_medicine_management.html")

@app.get("/Prescription.html")
async def prescription_page():
    return FileResponse("static/Prescription.html")

@app.get("/doctor.html")
async def doctor_page():
    return FileResponse("static/doctor.html")

# 藥物 API
@app.get("/api/medicine/basic")
async def get_basic_medicines(db: Session = Depends(get_db)):
    """獲取基本藥物列表"""
    logger.info("開始獲取基本藥物列表")
    
    medicines = db.query(MedicineBasic).filter(MedicineBasic.is_active == True).all()
    
    result = []
    for medicine in medicines:
        result.append({
            "id": medicine.id,
            "name": medicine.name,
            "amount": medicine.amount,
            "position": medicine.position,
            "manufacturer": medicine.manufacturer,
            "dosage": medicine.dosage,
            "is_active": medicine.is_active,
            "created_at": medicine.created_at.isoformat(),
            "updated_at": medicine.updated_at.isoformat()
        })
    
    logger.info(f"基本藥物列表構建完成，共 {len(result)} 種藥物")
    
    return result

@app.get("/api/medicine/detailed")
async def get_detailed_medicines(db: Session = Depends(get_db)):
    """獲取詳細藥物列表"""
    logger.info("開始獲取詳細藥物列表")
    
    detailed_medicines = db.query(MedicineDetailed).all()
    
    result = []
    for detailed in detailed_medicines:
        # 獲取對應的基本藥物資訊
        basic = db.query(MedicineBasic).filter(MedicineBasic.id == detailed.medicine_id).first()
        if basic:
            result.append({
                "basic_info": {
                    "id": basic.id,
                    "name": basic.name,
                    "amount": basic.amount,
                    "position": basic.position,
                    "manufacturer": basic.manufacturer,
                    "dosage": basic.dosage,
                    "is_active": basic.is_active
                },
                "detailed_info": {
                    "id": detailed.id,
                    "description": detailed.description,
                    "ingredient": detailed.ingredient,
                    "category": detailed.category,
                    "usage_method": detailed.usage_method,
                    "unit_dose": detailed.unit_dose,
                    "side_effects": detailed.side_effects,
                    "storage_conditions": detailed.storage_conditions,
                    "expiry_date": detailed.expiry_date.isoformat() if detailed.expiry_date else None,
                    "barcode": detailed.barcode,
                    "appearance_type": detailed.appearance_type,
                    "notes": detailed.notes
                }
            })
    
    logger.info(f"詳細藥物列表構建完成，共 {len(result)} 種藥物")
    return result

@app.post("/api/medicine/detailed")
async def create_detailed_medicine(medicine_data: dict, db: Session = Depends(get_db)):
    """創建詳細藥物"""
    logger.info("創建詳細藥物")
    logger.debug(f"接收到的藥物資料: {medicine_data}")
    
    try:
        # 檢查是否已存在基本藥物
        basic_medicine = db.query(MedicineBasic).filter(
            MedicineBasic.name == medicine_data.get('name')
        ).first()
        
        if not basic_medicine:
            # 創建基本藥物
            basic_medicine = MedicineBasic(
                name=medicine_data.get('name'),
                amount=medicine_data.get('amount', 0),
                position=medicine_data.get('position', ''),
                manufacturer=medicine_data.get('manufacturer', ''),
                dosage=medicine_data.get('dosage', '')
            )
            db.add(basic_medicine)
            db.commit()
            db.refresh(basic_medicine)
            logger.info(f"基本藥物已創建: {basic_medicine.id}")
        
        # 創建詳細藥物
        detailed_medicine = MedicineDetailed(
            medicine_id=basic_medicine.id,
            description=medicine_data.get('description', ''),
            ingredient=medicine_data.get('ingredient', ''),
            category=medicine_data.get('category', ''),
            usage_method=medicine_data.get('usage_method', ''),
            unit_dose=medicine_data.get('unit_dose', 0.0),
            side_effects=medicine_data.get('side_effects', ''),
            storage_conditions=medicine_data.get('storage_conditions', ''),
            barcode=medicine_data.get('barcode', ''),
            appearance_type=medicine_data.get('appearance_type', ''),
            notes=medicine_data.get('notes', '')
        )
        
        # 處理有效期限
        if medicine_data.get('expiry_date'):
            try:
                detailed_medicine.expiry_date = datetime.fromisoformat(medicine_data['expiry_date']).date()
            except:
                logger.warning(f"無效的有效期限格式: {medicine_data['expiry_date']}")
        
        db.add(detailed_medicine)
        db.commit()
        db.refresh(detailed_medicine)
        
        logger.info(f"詳細藥物已創建: {detailed_medicine.id}")
        
        return {
            "message": "詳細藥物創建成功",
            "basic_id": basic_medicine.id,
            "detailed_id": detailed_medicine.id
        }
        
    except Exception as e:
        logger.error(f"創建詳細藥物失敗: {str(e)}")
        db.rollback()
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

@app.post("/api/medicine/unified")
async def create_unified_medicine(medicine_data: dict, db: Session = Depends(get_db)):
    """創建統一藥物（基本+詳細）"""
    logger.info("創建統一藥物")
    logger.debug(f"接收到的藥物資料: {medicine_data}")
    
    try:
        # 創建基本藥物
        basic_medicine = MedicineBasic(
            name=medicine_data.get('name'),
            amount=medicine_data.get('amount', 0),
            position=medicine_data.get('position', ''),
            manufacturer=medicine_data.get('manufacturer', ''),
            dosage=medicine_data.get('dosage', '')
        )
        db.add(basic_medicine)
        db.flush()  # 確保獲得 ID
        
        # 創建詳細藥物
        detailed_medicine = MedicineDetailed(
            medicine_id=basic_medicine.id,
            description=medicine_data.get('description', ''),
            ingredient=medicine_data.get('ingredient', ''),
            category=medicine_data.get('category', ''),
            usage_method=medicine_data.get('usage_method', ''),
            unit_dose=medicine_data.get('unit_dose', 0.0),
            side_effects=medicine_data.get('side_effects', ''),
            storage_conditions=medicine_data.get('storage_conditions', ''),
            barcode=medicine_data.get('barcode', ''),
            appearance_type=medicine_data.get('appearance_type', ''),
            notes=medicine_data.get('notes', '')
        )
        db.flush()  # 確保獲得 ID
        
        # 處理有效期限
        if medicine_data.get('expiry_date'):
            try:
                detailed_medicine.expiry_date = datetime.fromisoformat(medicine_data['expiry_date']).date()
            except:
                logger.warning(f"無效的有效期限格式: {medicine_data['expiry_date']}")
        
        db.add(detailed_medicine)
        db.commit()
        
        logger.info(f"統一藥物已創建: 基本ID={basic_medicine.id}, 詳細ID={detailed_medicine.id}")
        
        return {
            "message": "統一藥物創建成功",
            "basic_id": basic_medicine.id,
            "detailed_id": detailed_medicine.id,
            "medicine_name": basic_medicine.name
        }
        
    except Exception as e:
        logger.error(f"創建統一藥物失敗: {str(e)}")
        db.rollback()
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

@app.delete("/api/medicine/{medicine_identifier}")
async def delete_medicine(medicine_identifier: str, db: Session = Depends(get_db)):
    """刪除藥物（智能處理：如果被處方籤使用則停用，否則真正刪除）"""
    logger.info(f"請求刪除藥物: {medicine_identifier}")
    
    try:
        # 嘗試按 ID 查找
        medicine = None
        if medicine_identifier.isdigit():
            medicine = db.query(MedicineBasic).filter(MedicineBasic.id == int(medicine_identifier)).first()
        
        # 如果按 ID 找不到，則按名稱查找
        if not medicine:
            medicine = db.query(MedicineBasic).filter(MedicineBasic.name == medicine_identifier).first()
        
        if not medicine:
            raise HTTPException(status_code=404, detail=f"找不到藥物: {medicine_identifier}")
        
        # 檢查是否被處方籤使用
        prescription_count = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.medicine_id == medicine.id
        ).count()
        
        if prescription_count > 0:
            # 如果被處方籤使用，則停用而不是刪除
            medicine.is_active = False
            medicine.updated_at = datetime.now()
            db.commit()
            logger.info(f"藥物 {medicine.name} 已停用（因為被 {prescription_count} 張處方籤使用）")
            return {
                "message": f"藥物已停用（被 {prescription_count} 張處方籤使用）",
                "action": "deactivated",
                "medicine_id": medicine.id,
                "medicine_name": medicine.name
            }
        else:
            # 如果未被使用，則真正刪除
            medicine_name = medicine.name
            medicine_id = medicine.id
            
            # 先刪除詳細資訊
            db.query(MedicineDetailed).filter(MedicineDetailed.medicine_id == medicine.id).delete()
            # 再刪除基本資訊
            db.delete(medicine)
            db.commit()
            
            logger.info(f"藥物 {medicine_name} 已完全刪除")
            return {
                "message": "藥物已完全刪除",
                "action": "deleted",
                "medicine_id": medicine_id,
                "medicine_name": medicine_name
            }
        
    except Exception as e:
        logger.error(f"刪除藥物失敗: {str(e)}")
        db.rollback()
        raise HTTPException(status_code=500, detail=f"刪除失敗: {str(e)}")

@app.put("/api/medicine/{medicine_id}")
async def update_medicine(medicine_id: int, medicine_data: dict, db: Session = Depends(get_db)):
    """更新藥物資訊"""
    logger.info(f"更新藥物: {medicine_id}")
    
    try:
        medicine = db.query(MedicineBasic).filter(MedicineBasic.id == medicine_id).first()
        if not medicine:
            raise HTTPException(status_code=404, detail="找不到藥物")
        
        # 更新基本資訊
        if 'name' in medicine_data:
            medicine.name = medicine_data['name']
        if 'amount' in medicine_data:
            medicine.amount = medicine_data['amount']
        if 'position' in medicine_data:
            medicine.position = medicine_data['position']
        if 'manufacturer' in medicine_data:
            medicine.manufacturer = medicine_data['manufacturer']
        if 'dosage' in medicine_data:
            medicine.dosage = medicine_data['dosage']
        if 'is_active' in medicine_data:
            medicine.is_active = medicine_data['is_active']
        
        medicine.updated_at = datetime.now()
        
        # 更新詳細資訊（如果提供）
        detailed = db.query(MedicineDetailed).filter(MedicineDetailed.medicine_id == medicine_id).first()
        if detailed and any(key.startswith('detailed_') for key in medicine_data.keys()):
            if 'detailed_description' in medicine_data:
                detailed.description = medicine_data['detailed_description']
            if 'detailed_ingredient' in medicine_data:
                detailed.ingredient = medicine_data['detailed_ingredient']
            if 'detailed_category' in medicine_data:
                detailed.category = medicine_data['detailed_category']
            # 可以繼續添加其他詳細資訊欄位
        
        db.commit()
        
        logger.info(f"藥物 {medicine.name} 更新成功")
        return {
            "message": "藥物更新成功",
            "medicine_id": medicine.id,
            "medicine_name": medicine.name
        }
        
    except Exception as e:
        logger.error(f"更新藥物失敗: {str(e)}")
        db.rollback()
        raise HTTPException(status_code=500, detail=f"更新失敗: {str(e)}")

@app.post("/api/medicine/{medicine_id}/adjust-stock")
async def adjust_medicine_stock(medicine_id: int, request_data: dict, db: Session = Depends(get_db)):
    """調整特定藥物庫存"""
    logger.info(f"調整藥物 {medicine_id} 的庫存")
    
    action = request_data.get("action", "add")  # add, subtract, set
    amount = request_data.get("amount", 0)
    
    try:
        result = _adjust_medicine_stock_impl(str(medicine_id), action, amount, db)
        logger.info(f"藥物 {medicine_id} 庫存調整成功: {action} {amount}")
        return result
        
    except Exception as e:
        logger.error(f"調整庫存失敗: {str(e)}")
        raise HTTPException(status_code=500, detail=f"調整失敗: {str(e)}")

@app.post("/api/medicine/adjust-stock")
async def adjust_medicine_stock_by_name_or_id(request_data: dict, db: Session = Depends(get_db)):
    """按名稱或ID調整藥物庫存"""
    logger.info("調整藥物庫存（按名稱或ID）")
    
    medicine_name = request_data.get("medicine_name")
    medicine_id = request_data.get("medicine_id")
    action = request_data.get("action", "add")  # add, subtract, set
    amount = request_data.get("amount", 0)
    
    if not medicine_name and not medicine_id:
        raise HTTPException(status_code=400, detail="必須提供 medicine_name 或 medicine_id")
    
    try:
        medicine_identifier = str(medicine_id) if medicine_id else medicine_name
        result = _adjust_medicine_stock_impl(medicine_identifier, action, amount, db)
        logger.info(f"藥物庫存調整成功: {action} {amount}")
        return result
        
    except Exception as e:
        logger.error(f"調整庫存失敗: {str(e)}")
        raise HTTPException(status_code=500, detail=f"調整失敗: {str(e)}")

@app.get("/api/medicine/search/{medicine_name}")
async def search_medicine_by_name(medicine_name: str, db: Session = Depends(get_db)):
    """根據藥物名稱搜尋完整資訊（基本+詳細）"""
    logger.info(f"搜尋藥物: {medicine_name}")
    
    # 模糊搜尋藥物名稱
    medicines = db.query(MedicineBasic).filter(
        MedicineBasic.name.contains(medicine_name),
        MedicineBasic.is_active == True
    ).all()
    
    if not medicines:
        return {
            "found": False,
            "message": f"找不到包含 '{medicine_name}' 的藥物",
            "medicines": []
        }
    
    result_medicines = []
    for medicine in medicines:
        # 獲取詳細資訊
        detailed = db.query(MedicineDetailed).filter(
            MedicineDetailed.medicine_id == medicine.id
        ).first()
        
        medicine_info = {
            "basic_info": {
                "id": medicine.id,
                "name": medicine.name,
                "amount": medicine.amount,
                "position": medicine.position,
                "manufacturer": medicine.manufacturer,
                "dosage": medicine.dosage,
                "is_active": medicine.is_active,
                "created_at": medicine.created_at.isoformat(),
                "updated_at": medicine.updated_at.isoformat()
            },
            "detailed_info": None
        }
        
        if detailed:
            medicine_info["detailed_info"] = {
                "id": detailed.id,
                "description": detailed.description,
                "ingredient": detailed.ingredient,
                "category": detailed.category,
                "usage_method": detailed.usage_method,
                "unit_dose": detailed.unit_dose,
                "side_effects": detailed.side_effects,
                "storage_conditions": detailed.storage_conditions,
                "expiry_date": detailed.expiry_date.isoformat() if detailed.expiry_date else None,
                "barcode": detailed.barcode,
                "appearance_type": detailed.appearance_type,
                "notes": detailed.notes
            }
        
        result_medicines.append(medicine_info)
    
    logger.info(f"找到 {len(result_medicines)} 種符合的藥物")
    
    return {
        "found": True,
        "total_found": len(result_medicines),
        "search_term": medicine_name,
        "medicines": result_medicines
    }

# 處方籤 API
@app.get("/api/prescription/")
async def get_prescriptions(db: Session = Depends(get_db)):
    """獲取所有處方籤列表"""
    logger.info("獲取處方籤列表")
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
            "prescription_date": p.created_at.isoformat(),
            "medicine_count": medicine_count,
            "medicines": [{"length": medicine_count}]
        })
    
    logger.info(f"返回 {len(result)} 張處方籤")
    return result

@app.post("/api/prescription/")
async def create_prescription(prescription_data: dict, db: Session = Depends(get_db)):
    """創建新處方籤"""
    logger.info("開始創建新處方籤")
    logger.debug(f"接收到的處方數據: {prescription_data}")
    
    try:
        # 提取藥物列表
        medicines_list = prescription_data.pop('medicines', [])
        logger.info(f"處方包含 {len(medicines_list)} 種藥物")
        
        # 創建處方籤
        prescription = Prescription(**prescription_data)
        db.add(prescription)
        db.commit()
        db.refresh(prescription)
        logger.info(f"處方籤基本資料已創建，ID: {prescription.id}")
        
        # 處理藥物列表
        added_medicines = 0
        for i, medicine_info in enumerate(medicines_list):
            logger.debug(f"處理第 {i+1} 個藥物: {medicine_info}")
            
            if isinstance(medicine_info, list) and len(medicine_info) >= 4:
                # 格式: [藥物名稱, 劑量, 數量, 頻率]
                medicine_name = medicine_info[0]
                dosage = medicine_info[1]
                quantity_str = str(medicine_info[2])
                quantity = int(quantity_str) if quantity_str.isdigit() else 1
                frequency = medicine_info[3]
                
                logger.debug(f"藥物詳情 - 名稱: {medicine_name}, 劑量: {dosage}, 數量: {quantity}, 頻率: {frequency}")
                
                # 查找藥物ID
                medicine = db.query(MedicineBasic).filter(MedicineBasic.name == medicine_name).first()
                if medicine:
                    prescription_medicine = PrescriptionMedicine(
                        prescription_id=prescription.id,
                        medicine_id=medicine.id,
                        dosage=dosage,
                        frequency=frequency,
                        duration="7天",
                        quantity=quantity,
                        instructions=""
                    )
                    db.add(prescription_medicine)
                    added_medicines += 1
                    logger.debug(f"藥物 {medicine_name} 已加入處方籤")
                else:
                    logger.warning(f"找不到藥物: {medicine_name}")
            else:
                logger.warning(f"藥物資料格式錯誤: {medicine_info}")
        
        db.commit()
        logger.info(f"處方籤創建完成，共添加 {added_medicines} 種藥物")
        
        # 檢查庫存並立即扣減
        stock_warnings = []
        stock_changes = []
        for pm in db.query(PrescriptionMedicine).filter(PrescriptionMedicine.prescription_id == prescription.id).all():
            medicine = db.query(MedicineBasic).filter(MedicineBasic.id == pm.medicine_id).first()
            if medicine:
                if medicine.amount < pm.quantity:
                    stock_warnings.append(f"{medicine.name}: 庫存不足 (需要:{pm.quantity}, 現有:{medicine.amount})")
                    logger.warning(f"庫存不足: {medicine.name} 需要 {pm.quantity}，只有 {medicine.amount}")
                else:
                    # 立即扣減庫存（創建處方籤時）
                    old_amount = medicine.amount
                    medicine.amount -= pm.quantity
                    stock_changes.append({
                        "medicine_name": medicine.name,
                        "deducted": pm.quantity,
                        "old_amount": old_amount,
                        "new_amount": medicine.amount
                    })
                    logger.info(f"扣減庫存: {medicine.name} 減少 {pm.quantity} (從 {old_amount} 變為 {medicine.amount})")
        
        # 如果有庫存不足，回滾並返回錯誤
        if stock_warnings:
            db.rollback()
            raise HTTPException(status_code=400, detail={
                "error": "庫存不足，無法創建處方籤",
                "warnings": stock_warnings
            })
        
        # 提交庫存變化
        db.commit()
        
        result = {
            "message": "處方籤創建成功", 
            "id": prescription.id,
            "stock_changes": stock_changes,
            "medicines_added": added_medicines
        }
        
        logger.info(f"處方籤 {prescription.id} 創建完成")
        return result
        
    except Exception as e:
        logger.error(f"創建處方籤失敗: {str(e)}")
        db.rollback()
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

@app.get("/api/prescription/{prescription_id}")
async def get_prescription_detail(prescription_id: int, db: Session = Depends(get_db)):
    """獲取處方籤詳情"""
    logger.info(f"獲取處方籤 {prescription_id} 的詳情")
    
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="找不到處方籤")
    
    # 獲取處方籤藥物
    prescription_medicines = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.prescription_id == prescription_id
    ).all()
    
    medicines = []
    for pm in prescription_medicines:
        # 獲取藥物基本資訊
        medicine = db.query(MedicineBasic).filter(MedicineBasic.id == pm.medicine_id).first()
        medicine_name = medicine.name if medicine else "未知藥物"
        
        medicines.append({
            "medicine_id": pm.medicine_id,
            "medicine_name": medicine_name,
            "dosage": pm.dosage,
            "frequency": pm.frequency,
            "duration": pm.duration,
            "quantity": pm.quantity,
            "instructions": pm.instructions
        })
    
    result = {
        "id": prescription.id,
        "patient_name": prescription.patient_name,
        "patient_id": prescription.patient_id,
        "doctor_name": prescription.doctor_name,
        "diagnosis": prescription.diagnosis,
        "status": prescription.status,
        "created_at": prescription.created_at.isoformat(),
        "medicines": medicines
    }
    
    logger.info(f"處方籤 {prescription_id} 詳情獲取完成")
    return result

@app.put("/api/prescription/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_data: dict, db: Session = Depends(get_db)):
    """更新處方籤狀態"""
    logger.info(f"更新處方籤 {prescription_id} 狀態")
    
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="找不到處方籤")
    
    new_status = status_data.get("status")
    updated_by = status_data.get("updated_by", "系統")
    
    if not new_status:
        raise HTTPException(status_code=400, detail="必須提供新狀態")
    
    old_status = prescription.status
    prescription.status = new_status
    
    # 處理庫存變化
    stock_result = handle_prescription_stock_change(prescription_id, old_status, new_status, db)
    
    db.commit()
    
    logger.info(f"處方籤 {prescription_id} 狀態已更新: {old_status} → {new_status}")
    
    return {
        "message": f"處方籤狀態已更新為 {new_status}",
        "prescription_id": prescription_id,
        "old_status": old_status,
        "new_status": new_status,
        "updated_by": updated_by,
        "stock_changes": stock_result
    }

@app.get("/api/prescription/pending/next")
async def get_next_pending_prescription(db: Session = Depends(get_db)):
    """獲取下一個待處理的處方籤"""
    logger.info("獲取下一個待處理的處方籤")
    
    prescription = db.query(Prescription).filter(
        Prescription.status == 'pending'
    ).order_by(Prescription.created_at.asc()).first()
    
    if not prescription:
        return {"message": "沒有待處理的處方籤", "prescription": None}
    
    # 獲取處方籤藥物
    prescription_medicines = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.prescription_id == prescription.id
    ).all()
    
    medicines = []
    for pm in prescription_medicines:
        medicine = db.query(MedicineBasic).filter(MedicineBasic.id == pm.medicine_id).first()
        medicines.append({
            "medicine_name": medicine.name if medicine else "未知藥物",
            "quantity": pm.quantity,
            "dosage": pm.dosage,
            "frequency": pm.frequency
        })
    
    result = {
        "id": prescription.id,
        "patient_name": prescription.patient_name,
        "patient_id": prescription.patient_id,
        "doctor_name": prescription.doctor_name,
        "diagnosis": prescription.diagnosis,
        "created_at": prescription.created_at.isoformat(),
        "medicines": medicines
    }
    
    logger.info(f"找到待處理處方籤: {prescription.id}")
    return {"message": "找到待處理處方籤", "prescription": result}

@app.get("/api/prescription/{prescription_id}/stock-check")
async def check_prescription_stock(prescription_id: int, db: Session = Depends(get_db)):
    """檢查處方籤庫存可用性"""
    logger.info(f"檢查處方籤 {prescription_id} 的庫存可用性")
    
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="找不到處方籤")
    
    availability = check_stock_availability(prescription_id, db)
    
    return {
        "prescription_id": prescription_id,
        "patient_name": prescription.patient_name,
        "stock_availability": availability
    }

# 系統狀態 API
@app.get("/api/system/status")
async def get_system_status():
    """獲取系統狀態 (無 ROS2)"""
    return {
        "status": "running",
        "version": "1.0.0",
        "mode": "no_ros2",
        "ros2_available": False,
        "message": "系統運行正常，ROS2 功能已禁用"
    }

if __name__ == "__main__":
    logger.info("啟動醫院藥物管理系統 (無 ROS2 版本)")
    logger.info("系統模式: 無 ROS2 整合")
    logger.info("=" * 50)
    logger.info("網頁界面: http://localhost:8001/integrated_medicine_management.html")
    logger.info("處方籤管理: http://localhost:8001/Prescription.html")
    logger.info("醫生界面: http://localhost:8001/doctor.html")
    logger.info("API文檔: http://localhost:8001/docs")
    logger.info("=" * 50)
    
    uvicorn.run(app, host="0.0.0.0", port=8001)
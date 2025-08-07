#!/usr/bin/env python3
"""
Simple Hospital Medicine Management System with ROS2 Integration
簡化醫院藥物管理系統 - 整合ROS2 (Clean Version)
"""

from fastapi import FastAPI, HTTPException, Depends
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
from sqlalchemy.orm import Session
from typing import List, Optional, Dict, Any
import json
import os
import threading
import time
import logging
from datetime import datetime
from fastapi.responses import JSONResponse # Added for new_code
from pydantic import BaseModel

# 設置詳細的日誌配置
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('system.log', encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("hospital_system")

from database_clean import get_db, Medicine, MedicineDetail, Prescription, PrescriptionMedicine, init_database

# Pydantic 模型
class MedicineCreate(BaseModel):
    name: str
    amount: int
    position: str
    manufacturer: str
    dosage: str

class MedicineDetailCreate(BaseModel):
    description: str
    ingredient: str
    category: str
    usage_method: str
    unit_dose: float
    side_effects: str
    storage_conditions: str
    expiry_date: str
    barcode: str
    appearance_type: str
    notes: str = ""

class MedicineUpdate(BaseModel):
    name: Optional[str] = None
    amount: Optional[int] = None
    position: Optional[str] = None
    manufacturer: Optional[str] = None
    dosage: Optional[str] = None
    is_active: Optional[bool] = None

class StockAdjustment(BaseModel):
    medicine_id: Optional[int] = None
    medicine_name: Optional[str] = None
    adjustment: int
    reason: str

class PrescriptionCreate(BaseModel):
    patient_name: str
    doctor_name: str
    medicines: List[Dict[str, Any]]

class PrescriptionStatusUpdate(BaseModel):
    status: str

class ROS2OrderRequest(BaseModel):
    prescription_id: int

class ROS2MedicineQuery(BaseModel):
    medicine_name: Optional[str] = None
    medicine_id: Optional[int] = None
    include_detailed: bool = False

class ROS2BatchMedicineQuery(BaseModel):
    medicine_names: Optional[List[str]] = None
    medicine_ids: Optional[List[int]] = None
    include_detailed: bool = False

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
        medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
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
        medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
        if medicine:
            is_available = medicine.amount >= pm.quantity
            availability.append({
                "medicine_id": medicine.id,
                "medicine_name": medicine.name,
                "required": pm.quantity,
                "available": medicine.amount,
                "sufficient": is_available
            })
            if not is_available:
                all_available = False
    
    return {
        "all_available": all_available,
        "medicines": availability
    }

# 嘗試導入ROS2模組（如果可用）
try:
    from ros2_integration import init_ros2_node, get_ros2_node
    ROS2_AVAILABLE = True
    ROS2_MODE = "full"
    print("完整ROS2模組已載入")
except ImportError:
    try:
        from ros2_mock_clean import init_ros2_node, get_ros2_node
        ROS2_AVAILABLE = True
        ROS2_MODE = "mock"
        print("使用模擬ROS2模式")
    except ImportError:
        ROS2_AVAILABLE = False
        ROS2_MODE = "none"
        print("ROS2模組不可用，將使用模擬模式")

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

# 錯誤處理
@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    logger.error(f"HTTP錯誤: {exc.status_code} - {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail}
    )

# 新增：分離的基本藥物 ROS2 服務
@app.post("/api/ros2/service/basic-medicine")
def ros2_basic_medicine_service(query: ROS2MedicineQuery, db: Session = Depends(get_db)):
    """ROS2 服務 - 僅獲取基本藥物資訊"""
    if not ROS2_AVAILABLE or not ros2_node:
        raise HTTPException(status_code=503, detail="ROS2 不可用")
    
    logger.info(f"ROS2 基本藥物服務請求: {query}")
    
    if query.medicine_id:
        medicines = [db.query(Medicine).filter(Medicine.id == query.medicine_id).first()]
    elif query.medicine_name:
        medicines = [db.query(Medicine).filter(Medicine.name == query.medicine_name).first()]
    else:
        # 獲取所有藥物
        medicines = db.query(Medicine).filter(Medicine.is_active == True).all()
    
    # 過濾掉 None 值
    medicines = [m for m in medicines if m is not None]
    
    if not medicines:
        return {
            "success": False,
            "message": "未找到符合條件的藥物",
            "medicines": []
        }
    
    basic_medicines = []
    for medicine in medicines:
        basic_info = {
            "id": medicine.id,
            "name": medicine.name,
            "amount": medicine.amount,
            "position": medicine.position,
            "manufacturer": medicine.manufacturer,
            "dosage": medicine.dosage,
            "is_active": medicine.is_active
        }
        basic_medicines.append(basic_info)
    
    # 發布到 ROS2 (mock)
    if ros2_node:
        ros2_node._send_to_ros2_master({
            "type": "basic_medicine_response",
            "medicines": basic_medicines,
            "count": len(basic_medicines)
        })
    
    logger.info(f"基本藥物服務完成，返回 {len(basic_medicines)} 個藥物")
    return {
        "success": True,
        "message": f"基本藥物資訊獲取成功，共 {len(basic_medicines)} 個",
        "medicines": basic_medicines
    }

# 新增：分離的詳細藥物 ROS2 服務  
@app.post("/api/ros2/service/detailed-medicine")
def ros2_detailed_medicine_service(query: ROS2MedicineQuery, db: Session = Depends(get_db)):
    """ROS2 服務 - 僅獲取詳細藥物資訊"""
    if not ROS2_AVAILABLE or not ros2_node:
        raise HTTPException(status_code=503, detail="ROS2 不可用")
    
    logger.info(f"ROS2 詳細藥物服務請求: {query}")
    
    if query.medicine_id:
        medicines = [db.query(Medicine).filter(Medicine.id == query.medicine_id).first()]
    elif query.medicine_name:
        medicines = [db.query(Medicine).filter(Medicine.name == query.medicine_name).first()]
    else:
        # 獲取所有藥物
        medicines = db.query(Medicine).filter(Medicine.is_active == True).all()
    
    # 過濾掉 None 值
    medicines = [m for m in medicines if m is not None]
    
    if not medicines:
        return {
            "success": False,
            "message": "未找到符合條件的藥物",
            "detailed_medicines": [],
            "basic_medicines": []
        }
    
    detailed_medicines = []
    basic_medicines = []
    
    for medicine in medicines:
        # 基本資訊
        basic_info = {
            "id": medicine.id,
            "name": medicine.name,
            "amount": medicine.amount,
            "position": medicine.position,
            "manufacturer": medicine.manufacturer,
            "dosage": medicine.dosage,
            "is_active": medicine.is_active
        }
        basic_medicines.append(basic_info)
        
        # 詳細資訊
        detail = db.query(MedicineDetail).filter(
            MedicineDetail.medicine_id == medicine.id
        ).first()
        
        if detail:
            detailed_info = {
                "id": detail.id,
                "medicine_id": detail.medicine_id,
                "description": detail.description,
                "ingredient": detail.ingredient,
                "category": detail.category,
                "usage_method": detail.usage_method,
                "unit_dose": float(detail.unit_dose),
                "side_effects": detail.side_effects,
                "storage_conditions": detail.storage_conditions,
                "expiry_date": detail.expiry_date,
                "barcode": detail.barcode,
                "appearance_type": detail.appearance_type,
                "notes": detail.notes
            }
            detailed_medicines.append(detailed_info)
    
    # 發布到 ROS2 (mock)
    if ros2_node:
        ros2_node._send_to_ros2_master({
            "type": "detailed_medicine_response",
            "detailed_medicines": detailed_medicines,
            "basic_medicines": basic_medicines,
            "count": len(detailed_medicines)
        })
    
    logger.info(f"詳細藥物服務完成，返回 {len(detailed_medicines)} 個詳細資訊")
    return {
        "success": True,
        "message": f"詳細藥物資訊獲取成功，共 {len(detailed_medicines)} 個",
        "detailed_medicines": detailed_medicines,
        "basic_medicines": basic_medicines if query.include_detailed else None
    }

# 新增：ROS2 服務狀態檢查
@app.get("/api/ros2/service/status")
def ros2_service_status():
    """檢查 ROS2 服務狀態"""
    return {
        "ros2_available": ROS2_AVAILABLE,
        "node_active": ros2_node is not None,
        "services": {
            "basic_medicine": "/api/ros2/service/basic-medicine",
            "detailed_medicine": "/api/ros2/service/detailed-medicine"
        },
        "message": "ROS2 服務已就緒" if ROS2_AVAILABLE and ros2_node else "ROS2 服務不可用"
    }

@app.get("/")
async def root():
    """根路徑"""
    return {"message": "醫院藥物管理系統", "status": "running", "ros2_available": ROS2_AVAILABLE}

@app.get("/api/health")
async def health_check():
    """健康檢查"""
    logger.info("系統健康檢查請求")
    ros2_status = "full" if ROS2_MODE == "full" else "mock" if ROS2_MODE == "mock" else "unavailable"
    result = {
        "status": "healthy", 
        "message": "系統運行正常",
        "ros2_status": ros2_status,
        "ros2_mode": ROS2_MODE
    }
    logger.info(f"健康檢查結果: {result}")
    return result

# 藥物管理API
@app.post("/api/medicine/")
async def create_medicine(medicine_data: dict, db: Session = Depends(get_db)):
    """創建新藥物"""
    try:
        medicine = Medicine(**medicine_data)
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

@app.post("/api/medicine/detailed")
async def create_detailed_medicine(medicine_data: dict, db: Session = Depends(get_db)):
    """創建詳細藥物"""
    try:
        detailed_medicine = MedicineDetail(**medicine_data)
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
            "is_active": medicine_data.get("is_active", True)
        }
        
        # 創建基本藥物
        basic_medicine = Medicine(**basic_data)
        db.add(basic_medicine)
        db.flush()  # 獲取ID但不提交
        
        # 創建詳細藥物（如果有詳細資料）
        detailed_id = None
        if any(key in medicine_data for key in ["description", "ingredient", "category"]):
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
            
            detailed_medicine = MedicineDetail(**detailed_data)
            db.add(detailed_medicine)
            db.flush()  # 確保獲取ID
            detailed_id = detailed_medicine.id
        
        db.commit()
        
        result = {
            "message": "統一藥物創建成功",
            "basic_id": basic_medicine.id,
            "detailed_id": detailed_id
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

@app.delete("/api/medicine/{medicine_identifier}")
async def delete_medicine(medicine_identifier: str, db: Session = Depends(get_db)):
    """刪除藥物（支援 ID 或名稱）"""
    logger.info(f"嘗試刪除藥物: {medicine_identifier}")
    
    try:
        # 嘗試解析為 ID
        try:
            medicine_id = int(medicine_identifier)
            medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
        except ValueError:
            # 如果不是數字，則按名稱查找
            medicine = db.query(Medicine).filter(Medicine.name == medicine_identifier).first()
        
        if not medicine:
            raise HTTPException(status_code=404, detail=f"找不到藥物: {medicine_identifier}")
        
        medicine_name = medicine.name
        medicine_id = medicine.id
        
        # 檢查是否有相關的處方籤
        prescription_count = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.medicine_id == medicine_id
        ).count()
        
        if prescription_count > 0:
            # 如果有處方籤使用此藥物，只標記為無效而不是真正刪除
            medicine.is_active = False
            db.commit()
            logger.info(f"藥物 {medicine_name} (ID: {medicine_id}) 已標記為無效，因為有 {prescription_count} 張處方籤使用此藥物")
            
            return {
                "message": f"藥物已停用（有 {prescription_count} 張處方籤使用此藥物）",
                "medicine_id": medicine_id,
                "medicine_name": medicine_name,
                "action": "deactivated"
            }
        else:
            # 刪除詳細資料
            detailed = db.query(MedicineDetail).filter(MedicineDetail.medicine_id == medicine_id).first()
            if detailed:
                db.delete(detailed)
                logger.info(f"已刪除藥物詳細資料: {medicine_name}")
            
            # 刪除基本資料
            db.delete(medicine)
            db.commit()
            logger.info(f"已完全刪除藥物: {medicine_name} (ID: {medicine_id})")
            
            # 如果ROS2可用，發布刪除事件
            if ROS2_AVAILABLE and ros2_node:
                ros2_node.publish_medicine_data({
                    "id": medicine_id,
                    "name": medicine_name,
                    "action": "deleted"
                })
            
            return {
                "message": "藥物已成功刪除",
                "medicine_id": medicine_id,
                "medicine_name": medicine_name,
                "action": "deleted"
            }
            
    except HTTPException:
        raise
    except Exception as e:
        db.rollback()
        logger.error(f"刪除藥物失敗: {str(e)}")
        raise HTTPException(status_code=500, detail=f"刪除失敗: {str(e)}")

@app.put("/api/medicine/{medicine_id}")
async def update_medicine(medicine_id: int, medicine_data: dict, db: Session = Depends(get_db)):
    """更新藥物資訊"""
    logger.info(f"更新藥物 ID: {medicine_id}")
    
    medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物不存在")
    
    try:
        # 更新基本資料
        for key, value in medicine_data.items():
            if hasattr(medicine, key) and key not in ['id', 'created_at']:
                setattr(medicine, key, value)
        
        medicine.updated_at = datetime.now()
        db.commit()
        
        logger.info(f"藥物 {medicine.name} (ID: {medicine_id}) 更新成功")
        
        # 如果ROS2可用，發布更新事件
        if ROS2_AVAILABLE and ros2_node:
            ros2_node.publish_medicine_data({
                "id": medicine_id,
                "name": medicine.name,
                "amount": medicine.amount,
                "position": medicine.position,
                "manufacturer": medicine.manufacturer,
                "dosage": medicine.dosage,
                "action": "updated"
            })
        
        return {
            "message": "藥物更新成功",
            "medicine_id": medicine_id,
            "medicine_name": medicine.name
        }
        
    except Exception as e:
        db.rollback()
        logger.error(f"更新藥物失敗: {str(e)}")
        raise HTTPException(status_code=500, detail=f"更新失敗: {str(e)}")

@app.get("/api/medicine/basic")
async def get_basic_medicines(db: Session = Depends(get_db)):
    """獲取基本藥物列表"""
    logger.info("開始獲取基本藥物列表")
    try:
        medicines = db.query(Medicine).filter(Medicine.is_active == True).all()
        logger.debug(f"資料庫查詢結果: 找到 {len(medicines)} 種藥物")
        
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
        
        logger.info(f"基本藥物列表構建完成，共 {len(result)} 種藥物")
        
        # 如果ROS2可用，發布藥物資料
        if ROS2_AVAILABLE and ros2_node:
            logger.debug("ROS2可用，準備發布藥物資料")
            for medicine in result:
                ros2_node.publish_medicine_data(medicine)
            logger.info("藥物資料已發布到ROS2")
        else:
            logger.debug("ROS2不可用，跳過資料發布")
        
        return result
    except Exception as e:
        logger.error(f"獲取基本藥物列表失敗: {str(e)}")
        raise HTTPException(status_code=500, detail=f"獲取藥物列表失敗: {str(e)}")

@app.get("/api/medicine/detailed")
async def get_detailed_medicines(db: Session = Depends(get_db)):
    """獲取詳細藥物列表"""
    detailed_medicines = db.query(MedicineDetail).all()
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
    medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物不存在")
    
    detailed = db.query(MedicineDetail).filter(MedicineDetail.medicine_id == medicine_id).first()
    
    result = {
        "id": medicine.id,
        "name": medicine.name,
        "amount": medicine.amount,
        "position": medicine.position,
        "manufacturer": medicine.manufacturer,
        "dosage": medicine.dosage,
        "is_active": medicine.is_active
    }
    
    if detailed:
        result.update({
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
        })
    
    return result

# 處方籤管理API
@app.get("/api/prescription/")
async def get_prescriptions(db: Session = Depends(get_db)):
    """獲取處方籤列表"""
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
        
        # 創建處方籤，預設狀態為 pending
        prescription_data.setdefault('status', 'pending')
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
                medicine = db.query(Medicine).filter(Medicine.name == medicine_name).first()
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
            medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
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
        
        # 如果ROS2可用，將處方籤加入訂單佇列
        if ROS2_AVAILABLE and ros2_node:
            logger.info("ROS2可用，準備創建訂單")
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
            logger.info(f"ROS2訂單已創建: {order_data['order_id']}")
        else:
            logger.debug("ROS2不可用，跳過訂單創建")
        
        return result
    except Exception as e:
        logger.error(f"創建處方籤失敗: {str(e)}")
        db.rollback()
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

@app.get("/api/prescription/{prescription_id}")
async def get_prescription_detail(prescription_id: int, db: Session = Depends(get_db)):
    """獲取處方籤詳細資訊"""
    logger.info(f"獲取處方籤詳細資訊: {prescription_id}")
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="處方籤不存在")
    
    medicines = db.query(PrescriptionMedicine).filter(PrescriptionMedicine.prescription_id == prescription_id).all()
    
    # 獲取藥物詳細資訊
    medicine_details = []
    for pm in medicines:
        basic_medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
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
    
    logger.info(f"返回處方籤 {prescription_id} 詳細資訊，包含 {len(medicine_details)} 種藥物")
    return result

@app.put("/api/prescription/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_data: dict, db: Session = Depends(get_db)):
    """更新處方籤狀態"""
    logger.info(f"更新處方籤 {prescription_id} 狀態")
    
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
        logger.info(f"處方籤 {prescription_id} 狀態已更新: {old_status} → {new_status}")
        
        # 根據狀態變化處理庫存
        stock_result = handle_prescription_stock_change(prescription_id, old_status, new_status, db)
        
        # 如果狀態變為 processing，且 ROS2 可用，可以觸發處理
        if new_status == 'processing' and ROS2_AVAILABLE and ros2_node:
            logger.info(f"觸發 ROS2 處理處方籤 {prescription_id}")
        
        return {
            "message": "狀態更新成功",
            "prescription_id": prescription_id,
            "old_status": old_status,
            "new_status": new_status,
            "updated_by": updated_by,
            "stock_changes": stock_result
        }
    except Exception as e:
        db.rollback()
        logger.error(f"更新處方籤狀態失敗: {str(e)}")
        raise HTTPException(status_code=500, detail=f"更新失敗: {str(e)}")

@app.get("/api/prescription/pending/next")
async def get_next_pending_prescription(db: Session = Depends(get_db)):
    """獲取下一個待處理的處方籤（最舊的 pending 狀態）"""
    logger.info("查找下一個待處理的處方籤")
    
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
        basic_medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
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
    
    logger.info(f"找到待處理處方籤 {prescription.id}，包含 {len(medicine_details)} 種藥物")
    
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
        logger.info(f"處方籤 {prescription.id} 已自動加入 ROS2 處理佇列")
    
    return result

@app.get("/api/prescription/{prescription_id}/stock-check")
async def check_prescription_stock(prescription_id: int, db: Session = Depends(get_db)):
    """檢查處方籤庫存可用性"""
    logger.info(f"檢查處方籤 {prescription_id} 庫存可用性")
    
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="處方籤不存在")
    
    availability = check_stock_availability(prescription_id, db)
    
    result = {
        "prescription_id": prescription_id,
        "patient_name": prescription.patient_name,
        "status": prescription.status,
        "stock_check": availability,
        "can_process": availability["all_available"]
    }
    
    logger.info(f"處方籤 {prescription_id} 庫存檢查完成，可處理: {availability['all_available']}")
    return result

@app.post("/api/medicine/{medicine_id}/adjust-stock")
async def adjust_medicine_stock(medicine_id: int, adjustment_data: dict, db: Session = Depends(get_db)):
    """調整藥物庫存（按ID）"""
    return await _adjust_medicine_stock_impl(medicine_id, adjustment_data, db)

@app.post("/api/medicine/adjust-stock")
async def adjust_medicine_stock_by_name(adjustment_data: dict, db: Session = Depends(get_db)):
    """調整藥物庫存（按名稱或ID）"""
    medicine_identifier = adjustment_data.get("medicine_id") or adjustment_data.get("medicine_name")
    if not medicine_identifier:
        raise HTTPException(status_code=400, detail="必須提供 medicine_id 或 medicine_name")
    
    # 嘗試按ID查找
    try:
        medicine_id = int(medicine_identifier)
        medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    except ValueError:
        # 按名稱查找
        medicine = db.query(Medicine).filter(Medicine.name == medicine_identifier).first()
        if medicine:
            medicine_id = medicine.id
        else:
            raise HTTPException(status_code=404, detail=f"找不到藥物: {medicine_identifier}")
    
    if not medicine:
        raise HTTPException(status_code=404, detail=f"找不到藥物: {medicine_identifier}")
    
    return await _adjust_medicine_stock_impl(medicine_id, adjustment_data, db)

async def _adjust_medicine_stock_impl(medicine_id: int, adjustment_data: dict, db: Session = Depends(get_db)):
    """調整藥物庫存"""
    logger.info(f"調整藥物 {medicine_id} 庫存")
    
    medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物不存在")
    
    try:
        adjustment_type = adjustment_data.get("type")  # "add", "subtract", "set"
        quantity = adjustment_data.get("quantity", 0)
        reason = adjustment_data.get("reason", "手動調整")
        operator = adjustment_data.get("operator", "系統")
        
        old_amount = medicine.amount
        
        if adjustment_type == "add":
            medicine.amount += quantity
        elif adjustment_type == "subtract":
            if medicine.amount >= quantity:
                medicine.amount -= quantity
            else:
                raise HTTPException(status_code=400, detail=f"庫存不足，現有: {medicine.amount}，嘗試減少: {quantity}")
        elif adjustment_type == "set":
            medicine.amount = quantity
        else:
            raise HTTPException(status_code=400, detail="無效的調整類型，請使用: add, subtract, set")
        
        medicine.updated_at = datetime.now()
        db.commit()
        
        logger.info(f"藥物 {medicine.name} 庫存調整: {old_amount} → {medicine.amount} (原因: {reason})")
        
        # 如果ROS2可用，發布庫存更新
        if ROS2_AVAILABLE and ros2_node:
            ros2_node.publish_medicine_data({
                "id": medicine_id,
                "name": medicine.name,
                "amount": medicine.amount,
                "old_amount": old_amount,
                "action": "stock_adjusted"
            })
        
        return {
            "message": "庫存調整成功",
            "medicine_id": medicine_id,
            "medicine_name": medicine.name,
            "old_amount": old_amount,
            "new_amount": medicine.amount,
            "adjustment": quantity,
            "type": adjustment_type,
            "reason": reason,
            "operator": operator
        }
        
    except Exception as e:
        db.rollback()
        logger.error(f"調整庫存失敗: {str(e)}")
        raise HTTPException(status_code=500, detail=f"調整失敗: {str(e)}")

@app.get("/api/ros2/pending-orders")
async def get_pending_orders(db: Session = Depends(get_db)):
    """獲取所有待處理訂單（供ROS2詢問）"""
    logger.info("ROS2請求查看所有待處理訂單")
    
    pending_prescriptions = db.query(Prescription)\
        .filter(Prescription.status == 'pending')\
        .order_by(Prescription.created_at.asc())\
        .all()
    
    orders = []
    for prescription in pending_prescriptions:
        medicines = db.query(PrescriptionMedicine)\
            .filter(PrescriptionMedicine.prescription_id == prescription.id)\
            .all()
        
        medicine_details = []
        for pm in medicines:
            basic_medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
            medicine_details.append({
                "medicine_id": pm.medicine_id,
                "medicine_name": basic_medicine.name if basic_medicine else "未知藥物",
                "dosage": pm.dosage,
                "frequency": pm.frequency,
                "quantity": pm.quantity,
                "position": basic_medicine.position if basic_medicine else "未知位置"
            })
        
        orders.append({
            "order_id": f"ORDER_{prescription.id:04d}",
            "prescription_id": prescription.id,
            "patient_name": prescription.patient_name,
            "patient_id": prescription.patient_id,
            "doctor_name": prescription.doctor_name,
            "diagnosis": prescription.diagnosis,
            "created_at": prescription.created_at.isoformat(),
            "medicine_count": len(medicine_details),
            "medicines": medicine_details,
            "priority": "normal",
            "estimated_duration": len(medicine_details) * 60
        })
    
    logger.info(f"找到 {len(orders)} 個待處理訂單")
    return {
        "total_pending": len(orders),
        "orders": orders
    }

@app.post("/api/ros2/request-order-confirmation")
async def request_order_confirmation(request_data: dict, db: Session = Depends(get_db)):
    """ROS2請求執行特定訂單的確認"""
    logger.info("ROS2請求執行訂單確認")
    
    prescription_id = request_data.get("prescription_id")
    requester_id = request_data.get("requester_id", "unknown")
    
    if not prescription_id:
        raise HTTPException(status_code=400, detail="必須提供 prescription_id")
    
    prescription = db.query(Prescription).filter(
        Prescription.id == prescription_id,
        Prescription.status == 'pending'
    ).first()
    
    if not prescription:
        return {
            "success": False,
            "message": "處方籤不存在或已被處理",
            "order": None
        }
    
    # 檢查庫存可用性
    availability = check_stock_availability(prescription_id, db)
    
    if not availability["all_available"]:
        return {
            "success": False,
            "message": "庫存不足，無法執行訂單",
            "order": None,
            "stock_issues": availability["medicines"]
        }
    
    # 準備訂單詳情
    medicines = db.query(PrescriptionMedicine)\
        .filter(PrescriptionMedicine.prescription_id == prescription_id)\
        .all()
    
    medicine_details = []
    for pm in medicines:
        basic_medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
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
        "estimated_duration": len(medicine_details) * 60,
        "status": "awaiting_confirmation"
    }
    
    logger.info(f"準備執行訂單確認: {order_data['order_id']} (處方籤 {prescription.id})")
    
    return {
        "success": True,
        "message": f"訂單 {order_data['order_id']} 準備就緒，等待確認",
        "order": order_data,
        "confirmation_required": True
    }

@app.post("/api/ros2/confirm-and-execute-order")
async def confirm_and_execute_order(request_data: dict, db: Session = Depends(get_db)):
    """確認並執行訂單"""
    logger.info("ROS2確認並執行訂單")
    
    prescription_id = request_data.get("prescription_id")
    confirmed = request_data.get("confirmed", False)
    requester_id = request_data.get("requester_id", "unknown")
    
    if not prescription_id:
        raise HTTPException(status_code=400, detail="必須提供 prescription_id")
    
    if not confirmed:
        return {
            "success": False,
            "message": "訂單未被確認，執行中止"
        }
    
    prescription = db.query(Prescription).filter(
        Prescription.id == prescription_id,
        Prescription.status == 'pending'
    ).first()
    
    if not prescription:
        return {
            "success": False,
            "message": "處方籤不存在或已被處理"
        }
    
    try:
        # 將狀態更新為 processing（這會觸發庫存扣減）
        old_status = prescription.status
        prescription.status = 'processing'
        
        # 處理庫存變化
        stock_result = handle_prescription_stock_change(prescription_id, old_status, 'processing', db)
        
        db.commit()
        
        # 如果ROS2可用，加入處理佇列
        if ROS2_AVAILABLE and ros2_node:
            order_data = {
                "order_id": f"ORDER_{prescription.id:04d}",
                "prescription_id": prescription.id,
                "patient_name": prescription.patient_name,
                "status": "processing",
                "assigned_to": requester_id
            }
            ros2_node.add_order(order_data)
        
        logger.info(f"訂單 ORDER_{prescription.id:04d} 已確認並開始執行")
        
        return {
            "success": True,
            "message": f"訂單 ORDER_{prescription.id:04d} 已開始執行",
            "order_id": f"ORDER_{prescription.id:04d}",
            "prescription_id": prescription.id,
            "stock_changes": stock_result,
            "status": "processing"
        }
        
    except Exception as e:
        db.rollback()
        logger.error(f"執行訂單失敗: {str(e)}")
        raise HTTPException(status_code=500, detail=f"執行失敗: {str(e)}")

@app.post("/api/ros2/complete-order")
async def complete_order(request_data: dict, db: Session = Depends(get_db)):
    """標記訂單為完成"""
    logger.info("ROS2標記訂單完成")
    
    prescription_id = request_data.get("prescription_id")
    completion_notes = request_data.get("notes", "")
    
    if not prescription_id:
        raise HTTPException(status_code=400, detail="必須提供 prescription_id")
    
    prescription = db.query(Prescription).filter(
        Prescription.id == prescription_id,
        Prescription.status == 'processing'
    ).first()
    
    if not prescription:
        return {
            "success": False,
            "message": "處方籤不存在或狀態不正確"
        }
    
    prescription.status = 'completed'
    db.commit()
    
    logger.info(f"訂單 ORDER_{prescription.id:04d} 已完成")
    
    return {
        "success": True,
        "message": f"訂單 ORDER_{prescription.id:04d} 已完成",
        "order_id": f"ORDER_{prescription.id:04d}",
        "prescription_id": prescription.id,
        "completion_time": datetime.now().isoformat(),
        "notes": completion_notes
    }

@app.post("/api/ros2/request-next-order")
async def request_next_order(request_data: dict, db: Session = Depends(get_db)):
    """ROS2 主控制器請求下一個最舊的待處理訂單"""
    logger.info("ROS2 主控制器請求下一個訂單")
    
    requester_id = request_data.get("requester_id", "unknown")
    requested_priority = request_data.get("priority", "any")
    
    # 查找最舊的 pending 狀態處方籤
    query = db.query(Prescription).filter(Prescription.status == 'pending')
    
    # 按創建時間排序，優先處理最舊的
    prescription = query.order_by(Prescription.created_at.asc()).first()
    
    if not prescription:
        logger.info("沒有待處理的處方籤")
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
        basic_medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
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
        "estimated_duration": len(medicine_details) * 60
    }
    
    logger.info(f"分配訂單給 ROS2 主控制器: {order_data['order_id']} (處方籤 {prescription.id})")
    logger.info(f"包含 {len(medicine_details)} 種藥物")
    
    return {
        "success": True,
        "message": f"分配訂單 {order_data['order_id']}",
        "order": order_data
    }

@app.get("/api/medicine/search/{medicine_name}")
async def search_medicine_by_name(medicine_name: str, db: Session = Depends(get_db)):
    """根據藥物名稱搜尋完整資訊（基本+詳細）"""
    logger.info(f"搜尋藥物: {medicine_name}")
    
    # 模糊搜尋藥物名稱
    medicines = db.query(Medicine).filter(
        Medicine.name.contains(medicine_name),
        Medicine.is_active == True
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
        detailed = db.query(MedicineDetail).filter(
            MedicineDetail.medicine_id == medicine.id
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

@app.post("/api/ros2/query-medicine")
async def ros2_query_medicine(query_data: dict, db: Session = Depends(get_db)):
    """ROS2查詢藥物詳細資訊"""
    logger.info("ROS2查詢藥物資訊")
    
    medicine_name = query_data.get("medicine_name")
    medicine_id = query_data.get("medicine_id")
    include_stock = query_data.get("include_stock", True)
    include_detailed = query_data.get("include_detailed", True)
    
    if not medicine_name and not medicine_id:
        raise HTTPException(status_code=400, detail="必須提供 medicine_name 或 medicine_id")
    
    # 查找藥物
    if medicine_id:
        medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    else:
        medicine = db.query(Medicine).filter(Medicine.name == medicine_name).first()
    
    if not medicine:
        return {
            "found": False,
            "message": f"找不到藥物: {medicine_name or medicine_id}",
            "medicine": None
        }
    
    # 基本資訊
    medicine_info = {
        "basic_info": {
            "id": medicine.id,
            "name": medicine.name,
            "position": medicine.position,
            "manufacturer": medicine.manufacturer,
            "dosage": medicine.dosage,
            "is_active": medicine.is_active
        }
    }
    
    # 包含庫存資訊
    if include_stock:
        medicine_info["stock_info"] = {
            "current_amount": medicine.amount,
            "position": medicine.position,
            "last_updated": medicine.updated_at.isoformat()
        }
    
    # 包含詳細資訊
    if include_detailed:
        detailed = db.query(MedicineDetail).filter(
            MedicineDetail.medicine_id == medicine.id
        ).first()
        
        if detailed:
            medicine_info["detailed_info"] = {
                "description": detailed.description,
                "ingredient": detailed.ingredient,
                "category": detailed.category,
                "usage_method": detailed.usage_method,
                "unit_dose": detailed.unit_dose,
                "side_effects": detailed.side_effects,
                "storage_conditions": detailed.storage_conditions,
                "expiry_date": detailed.expiry_date.isoformat() if detailed.expiry_date else None,
                "appearance_type": detailed.appearance_type,
                "notes": detailed.notes
            }
        else:
            medicine_info["detailed_info"] = None
    
    logger.info(f"ROS2查詢藥物 {medicine.name} 完成")
    
    return {
        "found": True,
        "medicine": medicine_info
    }

@app.post("/api/ros2/batch-query-medicines")
async def ros2_batch_query_medicines(query_data: dict, db: Session = Depends(get_db)):
    """ROS2批量查詢多個藥物資訊"""
    logger.info("ROS2批量查詢藥物資訊")
    
    medicine_list = query_data.get("medicines", [])  # [{"name": "xxx"}, {"id": 123}, ...]
    include_stock = query_data.get("include_stock", True)
    include_detailed = query_data.get("include_detailed", False)
    
    if not medicine_list:
        raise HTTPException(status_code=400, detail="必須提供藥物列表")
    
    results = []
    for item in medicine_list:
        medicine_name = item.get("name")
        medicine_id = item.get("id")
        
        if medicine_id:
            medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
        elif medicine_name:
            medicine = db.query(Medicine).filter(Medicine.name == medicine_name).first()
        else:
            results.append({
                "query": item,
                "found": False,
                "message": "無效的查詢參數"
            })
            continue
        
        if not medicine:
            results.append({
                "query": item,
                "found": False,
                "message": f"找不到藥物: {medicine_name or medicine_id}"
            })
            continue
        
        # 構建回應
        medicine_info = {
            "id": medicine.id,
            "name": medicine.name,
            "position": medicine.position,
            "manufacturer": medicine.manufacturer,
            "dosage": medicine.dosage,
            "is_active": medicine.is_active
        }
        
        if include_stock:
            medicine_info["current_amount"] = medicine.amount
        
        if include_detailed:
            detailed = db.query(MedicineDetail).filter(
                MedicineDetail.medicine_id == medicine.id
            ).first()
            if detailed:
                medicine_info.update({
                    "description": detailed.description,
                    "ingredient": detailed.ingredient,
                    "category": detailed.category,
                    "usage_method": detailed.usage_method,
                    "side_effects": detailed.side_effects,
                    "storage_conditions": detailed.storage_conditions
                })
        
        results.append({
            "query": item,
            "found": True,
            "medicine": medicine_info
        })
    
    logger.info(f"批量查詢完成，處理 {len(medicine_list)} 個請求")
    
    return {
        "total_queries": len(medicine_list),
        "results": results
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
    print("醫院藥物管理系統 - 乾淨版本")
    print("ROS2狀態: 模擬環境")
    print("整合管理界面: http://localhost:8001/integrated_medicine_management.html")
    print("處方籤管理: http://localhost:8001/Prescription.html")
    print("醫生界面: http://localhost:8001/doctor.html")
    print("API文檔: http://localhost:8001/docs")
    
    uvicorn.run(app, host="0.0.0.0", port=8001, log_level="info")
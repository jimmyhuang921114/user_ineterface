"""
醫院藥物管理系統 - 正式版服務器
不含測試資料的生產環境版本
"""

from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import logging
import uvicorn
from datetime import datetime
import time

# 使用正式版數據庫
from database_production import get_db, Medicine, MedicineDetail, Prescription, PrescriptionMedicine, init_production_database

# 導入 ROS2 藥物查詢服務
from ros2_medicine_query_service import get_medicine_query_service

# 嘗試導入 ROS2 mock
try:
    from ros2_mock_clean import MockHospitalROS2Node as ROS2Mock
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    ROS2Mock = None

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("hospital_system_production")

# 創建 FastAPI 應用
app = FastAPI(
    title="醫院藥物管理系統 - 正式版",
    description="正式生產環境版本，不含測試資料",
    version="1.0.0"
)

# 靜態文件
app.mount("/static", StaticFiles(directory="static"), name="static")

# ROS2 模擬
ros2_node = None
if ROS2_AVAILABLE:
    try:
        ros2_node = ROS2Mock()
        logger.info("ROS2 模擬環境已啟動")
    except Exception as e:
        logger.warning(f"ROS2 模擬啟動失敗: {e}")
        ROS2_AVAILABLE = False

# Pydantic 模型
class MedicineCreate(BaseModel):
    name: str
    amount: int = 0
    position: str
    manufacturer: Optional[str] = ""
    dosage: Optional[str] = ""

class MedicineDetailCreate(BaseModel):
    description: Optional[str] = ""
    ingredient: Optional[str] = ""
    category: Optional[str] = ""
    usage_method: Optional[str] = ""
    unit_dose: Optional[float] = 0.0
    side_effects: Optional[str] = ""
    storage_conditions: Optional[str] = ""
    expiry_date: Optional[str] = ""
    barcode: Optional[str] = ""
    appearance_type: Optional[str] = ""
    notes: Optional[str] = ""

class MedicineUpdate(BaseModel):
    name: Optional[str] = None
    amount: Optional[int] = None
    position: Optional[str] = None
    manufacturer: Optional[str] = None
    dosage: Optional[str] = None

class StockAdjustment(BaseModel):
    medicine_name: Optional[str] = None
    medicine_id: Optional[int] = None
    adjustment: int
    reason: Optional[str] = ""

class PrescriptionCreate(BaseModel):
    patient_name: str
    patient_id: Optional[str] = ""
    doctor_name: Optional[str] = "系統醫生"
    diagnosis: Optional[str] = "一般診療"
    medicines: List[Dict[str, Any]]

class PrescriptionStatusUpdate(BaseModel):
    status: str

class ROS2OrderRequest(BaseModel):
    order_id: str
    prescription_id: int
    action: str = "process"

class ROS2MedicineQuery(BaseModel):
    medicine_name: Optional[str] = None
    medicine_id: Optional[int] = None
    include_detailed: bool = False

class ROS2BatchMedicineQuery(BaseModel):
    medicines: List[str]
    include_detailed: bool = False

# 根路由
@app.get("/")
async def root():
    return FileResponse("static/integrated_medicine_management.html")

# HTML 頁面路由
@app.get("/integrated_medicine_management.html")
async def integrated_medicine_management():
    return FileResponse("static/integrated_medicine_management.html")

@app.get("/doctor.html")
async def doctor_page():
    return FileResponse("static/doctor.html")

@app.get("/Prescription.html")
async def prescription_page():
    return FileResponse("static/Prescription.html")

@app.get("/ros2_client.html")
async def ros2_client_page():
    return FileResponse("static/ros2_client.html")

# 系統狀態 API
@app.get("/api/system/status")
def get_system_status():
    """獲取系統狀態"""
    return {
        "status": "running",
        "version": "1.0.0 - Production",
        "environment": "production",
        "ros2_available": ROS2_AVAILABLE,
        "ros2_mode": "mock" if ROS2_AVAILABLE else "disabled",
        "database": "production",
        "message": "醫院藥物管理系統正式版運行中"
    }

# 藥物 API
@app.get("/api/medicine/basic")
def get_basic_medicines(db: Session = Depends(get_db)):
    """獲取基本藥物列表"""
    logger.info("開始獲取基本藥物列表")
    
    medicines = db.query(Medicine).filter(Medicine.is_active == True).all()
    
    result = []
    for medicine in medicines:
        result.append({
            "id": medicine.id,
            "name": medicine.name,
            "amount": medicine.amount,
            "position": medicine.position,
            "manufacturer": medicine.manufacturer,
            "dosage": medicine.dosage
        })
    
    logger.info(f"基本藥物列表構建完成，共 {len(result)} 種藥物")
    
    # 發布到 ROS2
    if ROS2_AVAILABLE and ros2_node:
        ros2_node._send_to_ros2_master(result)
        logger.info("藥物資料已發布到ROS2")
    
    return result

@app.get("/api/medicine/detailed")
def get_detailed_medicines(db: Session = Depends(get_db)):
    """獲取詳細藥物列表"""
    logger.info("開始獲取詳細藥物列表")
    
    details = db.query(MedicineDetail).join(Medicine).filter(Medicine.is_active == True).all()
    
    result = []
    for detail in details:
        result.append({
            "id": detail.id,
            "medicine_id": detail.medicine_id,
            "medicine_name": detail.medicine.name,
            "description": detail.description,
            "ingredient": detail.ingredient,
            "category": detail.category,
            "usage_method": detail.usage_method,
            "unit_dose": detail.unit_dose,
            "side_effects": detail.side_effects,
            "storage_conditions": detail.storage_conditions,
            "expiry_date": detail.expiry_date,
            "barcode": detail.barcode,
            "appearance_type": detail.appearance_type,
            "notes": detail.notes
        })
    
    logger.info(f"詳細藥物列表構建完成，共 {len(result)} 種藥物")
    return result

@app.post("/api/medicine/unified")
async def create_unified_medicine(medicine_data: dict, db: Session = Depends(get_db)):
    """創建統一藥物（基本+詳細）"""
    try:
        # 檢查是否為嵌套格式 {basic: {...}, detailed: {...}}
        if "basic" in medicine_data and "detailed" in medicine_data:
            basic_info = medicine_data.get("basic", {})
            detailed_info = medicine_data.get("detailed", {})
            combined_data = {**basic_info, **detailed_info}
        else:
            combined_data = medicine_data
        
        # 分離基本和詳細資料
        basic_data = {
            "name": combined_data.get("name"),
            "amount": combined_data.get("amount", 0),
            "position": combined_data.get("position"),
            "manufacturer": combined_data.get("manufacturer", ""),
            "dosage": combined_data.get("dosage", ""),
            "is_active": combined_data.get("is_active", True)
        }
        
        # 驗證必需字段
        if not basic_data["name"]:
            raise HTTPException(status_code=400, detail="藥物名稱不能為空")
        
        # 創建基本藥物
        basic_medicine = Medicine(**basic_data)
        db.add(basic_medicine)
        db.flush()
        
        # 創建詳細藥物（如果有詳細資料）
        detailed_id = None
        if any(key in combined_data for key in ["description", "ingredient", "category"]):
            # 生成唯一的 barcode（如果為空）
            barcode = combined_data.get("barcode", "")
            if not barcode:
                barcode = f"BC{int(time.time() * 1000) % 1000000:06d}"
            
            detailed_data = {
                "medicine_id": basic_medicine.id,
                "description": combined_data.get("description", ""),
                "ingredient": combined_data.get("ingredient", ""),
                "category": combined_data.get("category", ""),
                "usage_method": combined_data.get("usage_method", ""),
                "unit_dose": float(combined_data.get("unit_dose", 0)),
                "side_effects": combined_data.get("side_effects", ""),
                "storage_conditions": combined_data.get("storage_conditions", ""),
                "expiry_date": combined_data.get("expiry_date", ""),
                "barcode": barcode,
                "appearance_type": combined_data.get("appearance_type", ""),
                "notes": combined_data.get("notes", "")
            }
            
            detailed_medicine = MedicineDetail(**detailed_data)
            db.add(detailed_medicine)
            db.flush()
            detailed_id = detailed_medicine.id
        
        db.commit()
        
        result = {
            "message": "統一藥物創建成功",
            "basic_id": basic_medicine.id,
            "detailed_id": detailed_id
        }
        
        # 發布到 ROS2
        if ROS2_AVAILABLE and ros2_node:
            ros2_node._send_to_ros2_master({
                "id": basic_medicine.id,
                "name": basic_medicine.name,
                "amount": basic_medicine.amount,
                "position": basic_medicine.position,
                "manufacturer": basic_medicine.manufacturer,
                "dosage": basic_medicine.dosage,
                "action": "created_unified"
            })
        
        return result
        
    except HTTPException:
        raise
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

# 處方籤 API
@app.get("/api/prescription/")
def get_prescriptions(db: Session = Depends(get_db)):
    """獲取處方籤列表"""
    logger.info("獲取處方籤列表")
    
    prescriptions = db.query(Prescription).order_by(Prescription.created_at.desc()).all()
    
    result = []
    for prescription in prescriptions:
        medicine_count = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.prescription_id == prescription.id
        ).count()
        
        result.append({
            "id": prescription.id,
            "patient_name": prescription.patient_name,
            "patient_id": prescription.patient_id,
            "doctor_name": prescription.doctor_name,
            "diagnosis": prescription.diagnosis,
            "status": prescription.status,
            "created_at": prescription.created_at.isoformat(),
            "prescription_date": prescription.prescription_date.isoformat(),
            "medicine_count": medicine_count
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
        
        # 確保必需的字段存在
        patient_name = prescription_data.get('patient_name', '')
        doctor_name = prescription_data.get('doctor_name', '系統醫生')
        
        if not patient_name:
            raise HTTPException(status_code=400, detail="患者姓名不能為空")
        
        if not doctor_name:
            doctor_name = '系統醫生'
        
        # 生成 patient_id（如果沒有提供）
        patient_id = prescription_data.get('patient_id', '')
        if not patient_id:
            patient_id = f"P{int(time.time() * 1000) % 1000000:06d}"
        
        # 創建處方籤，預設狀態為 pending
        prescription = Prescription(
            patient_name=patient_name,
            patient_id=patient_id,
            doctor_name=doctor_name,
            diagnosis=prescription_data.get('diagnosis', '一般診療'),
            status='pending'
        )
        db.add(prescription)
        db.commit()
        db.refresh(prescription)
        logger.info(f"處方籤基本資料已創建，ID: {prescription.id}, 患者ID: {patient_id}")
        
        # 處理藥物列表
        added_medicines = 0
        stock_changes = []
        
        for i, medicine_info in enumerate(medicines_list):
            logger.debug(f"處理第 {i+1} 個藥物: {medicine_info}")
            
            if isinstance(medicine_info, list) and len(medicine_info) >= 4:
                medicine_name = medicine_info[0]
                dosage = medicine_info[1]
                quantity_str = str(medicine_info[2])
                quantity = int(quantity_str) if quantity_str.isdigit() else 1
                frequency = medicine_info[3]
            elif isinstance(medicine_info, dict):
                medicine_name = medicine_info.get('name', '')
                dosage = medicine_info.get('dosage', '')
                quantity = int(medicine_info.get('quantity', 1))
                frequency = medicine_info.get('frequency', '')
                duration = medicine_info.get('duration', '7天')
                notes = medicine_info.get('notes', '')
            else:
                logger.warning(f"藥物資料格式錯誤: {medicine_info}")
                continue
            
            # 查找藥物ID
            medicine = db.query(Medicine).filter(Medicine.name == medicine_name).first()
            if medicine:
                # 檢查庫存
                if medicine.amount < quantity:
                    db.rollback()
                    raise HTTPException(status_code=400, detail=f"庫存不足: {medicine_name} 需要{quantity}，現有{medicine.amount}")
                
                # 立即扣減庫存
                old_amount = medicine.amount
                medicine.amount -= quantity
                stock_changes.append({
                    "medicine_name": medicine.name,
                    "deducted": quantity,
                    "old_amount": old_amount,
                    "new_amount": medicine.amount
                })
                
                prescription_medicine = PrescriptionMedicine(
                    prescription_id=prescription.id,
                    medicine_id=medicine.id,
                    dosage=dosage,
                    frequency=frequency,
                    duration=duration if isinstance(medicine_info, dict) else "7天",
                    quantity=quantity,
                    instructions=notes if isinstance(medicine_info, dict) else ""
                )
                db.add(prescription_medicine)
                added_medicines += 1
                logger.debug(f"藥物 {medicine_name} 已加入處方籤")
            else:
                logger.warning(f"找不到藥物: {medicine_name}")
        
        db.commit()
        logger.info(f"處方籤創建完成，共添加 {added_medicines} 種藥物")
        
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
            
            # 一次處理一個訂單 - 檢查是否有正在處理的訂單
            if not ros2_node.has_pending_orders():
                ros2_node.add_order_to_queue(order_data)
                ros2_node.publish_order_status({
                    "type": "order_created",
                    "order": order_data
                })
                logger.info(f"ROS2訂單已創建: {order_data['order_id']}")
            else:
                logger.info("ROS2正在處理其他訂單，當前訂單將等待")
        
        return result
        
    except Exception as e:
        db.rollback()
        logger.error(f"創建處方籤失敗: {e}")
        raise HTTPException(status_code=400, detail=f"創建失敗: {str(e)}")

# ROS2 API
@app.get("/api/ros2/status")
def get_ros2_status():
    """獲取ROS2狀態"""
    if not ROS2_AVAILABLE:
        return {"status": "unavailable", "message": "ROS2 不可用"}
    
    return {
        "status": "available",
        "mode": "mock",
        "message": "ROS2 模擬環境運行中",
        "node_active": ros2_node is not None
    }

@app.post("/api/ros2/service/basic-medicine")
def ros2_basic_medicine_service(query: ROS2MedicineQuery, db: Session = Depends(get_db)):
    """ROS2 服務 - 僅獲取基本藥物資訊"""
    logger.info(f"ROS2 基本藥物服務請求: {query}")
    
    # 查詢基本藥物
    if query.medicine_name:
        medicines = db.query(Medicine).filter(
            Medicine.name.contains(query.medicine_name),
            Medicine.is_active == True
        ).all()
    elif query.medicine_id:
        medicines = db.query(Medicine).filter(
            Medicine.id == query.medicine_id,
            Medicine.is_active == True
        ).all()
    else:
        medicines = db.query(Medicine).filter(Medicine.is_active == True).all()
    
    basic_medicines = []
    for medicine in medicines:
        basic_medicines.append({
            "id": medicine.id,
            "name": medicine.name,
            "amount": medicine.amount,
            "position": medicine.position,
            "manufacturer": medicine.manufacturer,
            "dosage": medicine.dosage
        })
    
    # 發布到 ROS2
    if ROS2_AVAILABLE and ros2_node:
        ros2_node._send_to_ros2_master(basic_medicines)
    
    logger.info(f"基本藥物服務完成，返回 {len(basic_medicines)} 個藥物")
    
    return {
        "success": True,
        "message": f"基本藥物資訊獲取成功，共 {len(basic_medicines)} 個",
        "medicines": basic_medicines
    }

@app.post("/api/ros2/service/detailed-medicine")
def ros2_detailed_medicine_service(query: ROS2MedicineQuery, db: Session = Depends(get_db)):
    """ROS2 服務 - 僅獲取詳細藥物資訊"""
    logger.info(f"ROS2 詳細藥物服務請求: {query}")
    
    # 查詢詳細藥物
    query_obj = db.query(MedicineDetail).join(Medicine).filter(Medicine.is_active == True)
    
    if query.medicine_name:
        query_obj = query_obj.filter(Medicine.name.contains(query.medicine_name))
    elif query.medicine_id:
        query_obj = query_obj.filter(Medicine.id == query.medicine_id)
    
    details = query_obj.all()
    
    detailed_medicines = []
    basic_medicines = []
    
    for detail in details:
        detailed_medicines.append({
            "id": detail.id,
            "medicine_id": detail.medicine_id,
            "medicine_name": detail.medicine.name,
            "description": detail.description,
            "ingredient": detail.ingredient,
            "category": detail.category,
            "usage_method": detail.usage_method,
            "unit_dose": detail.unit_dose,
            "side_effects": detail.side_effects,
            "storage_conditions": detail.storage_conditions,
            "expiry_date": detail.expiry_date,
            "barcode": detail.barcode,
            "appearance_type": detail.appearance_type,
            "notes": detail.notes
        })
        
        if query.include_detailed:
            basic_medicines.append({
                "id": detail.medicine.id,
                "name": detail.medicine.name,
                "amount": detail.medicine.amount,
                "position": detail.medicine.position,
                "manufacturer": detail.medicine.manufacturer,
                "dosage": detail.medicine.dosage
            })
    
    # 發布到 ROS2
    if ROS2_AVAILABLE and ros2_node:
        ros2_node._send_to_ros2_master(detailed_medicines)
    
    logger.info(f"詳細藥物服務完成，返回 {len(detailed_medicines)} 個詳細資訊")
    
    return {
        "success": True,
        "message": f"詳細藥物資訊獲取成功，共 {len(detailed_medicines)} 個",
        "detailed_medicines": detailed_medicines,
        "basic_medicines": basic_medicines if query.include_detailed else None
    }

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
        "message": "ROS2 服務已就緒" if ROS2_AVAILABLE and ros2_node else "ROS2 服務不可用",
        "order_processing": "一次處理一個訂單" if ROS2_AVAILABLE else "不可用"
    }

@app.post("/api/ros2/complete-order")
def complete_ros2_order(order_data: dict, db: Session = Depends(get_db)):
    """ROS2 訂單完成回調"""
    try:
        prescription_id = order_data.get("prescription_id")
        if prescription_id:
            prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
            if prescription:
                prescription.status = "completed"
                db.commit()
                logger.info(f"ROS2標記訂單完成: 處方籤 {prescription_id}")
                return {"success": True, "message": "訂單已完成"}
        
        return {"success": False, "message": "找不到處方籤"}
    except Exception as e:
        logger.error(f"完成訂單失敗: {e}")
        return {"success": False, "message": str(e)}

@app.post("/api/ros2/query-medicine-detail")
def query_medicine_detail(query: ROS2MedicineQuery):
    """查詢藥物詳細資訊（YAML 格式）"""
    try:
        medicine_service = get_medicine_query_service()
        result = medicine_service.query_medicine_detail(query.medicine_name)
        return {"success": True, "detail": result}
    except Exception as e:
        logger.error(f"查詢藥物詳細資訊失敗: {e}")
        return {"success": False, "message": str(e)}

@app.post("/api/ros2/process-order")
def process_ros2_order(order_data: dict):
    """處理 ROS2 訂單請求"""
    try:
        medicine_service = get_medicine_query_service()
        result = medicine_service.process_order(order_data)
        return {"success": True, "message": result}
    except Exception as e:
        logger.error(f"處理 ROS2 訂單失敗: {e}")
        return {"success": False, "message": str(e)}

@app.get("/api/ros2/service-status")
def get_ros2_service_status():
    """獲取 ROS2 服務狀態"""
    try:
        medicine_service = get_medicine_query_service()
        status = medicine_service.get_status()
        return {
            "service_running": status.get("status") == "running",
            "current_order": status.get("current_order"),
            "processing": status.get("processing", False),
            "base_url": status.get("base_url"),
            "service_name": status.get("service_name")
        }
    except Exception as e:
        logger.error(f"獲取 ROS2 服務狀態失敗: {e}")
        return {
            "service_running": False,
            "current_order": None,
            "processing": False,
            "error": str(e)
        }

if __name__ == "__main__":
    # 初始化資料庫
    init_production_database()
    
    logger.info("醫院藥物管理系統 - 正式版")
    logger.info("ROS2狀態: 模擬環境" if ROS2_AVAILABLE else "ROS2狀態: 不可用")
    logger.info("整合管理界面: http://localhost:8001/integrated_medicine_management.html")
    logger.info("處方籤管理: http://localhost:8001/Prescription.html")
    logger.info("醫生界面: http://localhost:8001/doctor.html")
    logger.info("API文檔: http://localhost:8001/docs")
    
    uvicorn.run(app, host="0.0.0.0", port=8001)
#!/usr/bin/env python3
"""
醫院藥物管理系統 - 真實 ROS2 版本
包含 ROS2 但不具備模擬功能，需要連接到真實的 ROS2 環境
"""

import asyncio
import logging
import threading
import time
from datetime import datetime
from typing import List, Optional, Dict, Any

import uvicorn
from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from sqlalchemy.orm import Session
from pydantic import BaseModel

# 導入資料庫相關
from database_clean import (
    engine, SessionLocal, Base,
    Medicine, MedicineDetail, Prescription, PrescriptionMedicine
)

# 嘗試導入真實 ROS2
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
    print("真實 ROS2 環境已載入")
except ImportError:
    print("錯誤: 無法載入 ROS2 環境")
    print("請確保已安裝 ROS2 並執行:")
    print("source /opt/ros/humble/setup.bash")
    print("或者使用其他版本的系統")
    exit(1)

# 配置日誌
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('hospital_system_ros2_real.log', encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('hospital_system_ros2_real')

# 創建資料庫表格
Base.metadata.create_all(bind=engine)

# 資料庫依賴
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# Pydantic 模型
class MedicineCreate(BaseModel):
    name: str
    amount: int
    position: str
    manufacturer: str
    dosage: str

class MedicineDetailCreate(BaseModel):
    medicine_id: int
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

class UnifiedMedicineCreate(BaseModel):
    # 基本資訊
    name: str
    amount: int
    position: str
    manufacturer: str
    dosage: str
    
    # 詳細資訊
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

# ROS2 節點類別
class HospitalROS2Node(Node):
    def __init__(self):
        super().__init__('hospital_medicine_system')
        
        # 發布器
        self.medicine_publisher = self.create_publisher(String, 'hospital/medicine_info', 10)
        self.order_publisher = self.create_publisher(String, 'hospital/order_status', 10)
        
        # 訂閱器
        self.order_subscriber = self.create_subscription(
            String,
            'hospital/order_request',
            self.order_callback,
            10
        )
        
        # 服務完成訂閱器
        self.completion_subscriber = self.create_subscription(
            String,
            'hospital/order_completed',
            self.completion_callback,
            10
        )
        
        self.get_logger().info('醫院藥物管理 ROS2 節點已啟動')
        
        # 存儲當前處理的訂單
        self.current_orders = {}

    def order_callback(self, msg):
        """處理來自機器人的訂單請求"""
        self.get_logger().info(f'收到訂單請求: {msg.data}')
        # 這裡可以添加處理邏輯

    def completion_callback(self, msg):
        """處理來自機器人的完成通知"""
        self.get_logger().info(f'收到完成通知: {msg.data}')
        # 解析完成的訂單 ID
        try:
            import json
            data = json.loads(msg.data)
            prescription_id = data.get('prescription_id')
            if prescription_id:
                # 這裡可以調用 API 更新訂單狀態
                pass
        except Exception as e:
            self.get_logger().error(f'解析完成通知失敗: {e}')

    def publish_medicine_info(self, medicine_data):
        """發布藥物資訊到 ROS2"""
        import json
        msg = String()
        msg.data = json.dumps(medicine_data, ensure_ascii=False)
        self.medicine_publisher.publish(msg)
        self.get_logger().info('藥物資訊已發布到 ROS2')

    def publish_order_status(self, order_data):
        """發布訂單狀態到 ROS2"""
        import json
        msg = String()
        msg.data = json.dumps(order_data, ensure_ascii=False)
        self.order_publisher.publish(msg)
        self.get_logger().info(f'訂單狀態已發布到 ROS2: {order_data}')

# 全域 ROS2 節點
ros2_node = None

def init_ros2():
    """初始化 ROS2"""
    global ros2_node
    if ROS2_AVAILABLE:
        try:
            rclpy.init()
            ros2_node = HospitalROS2Node()
            
            # 在單獨的線程中運行 ROS2
            def ros2_spin():
                rclpy.spin(ros2_node)
            
            ros2_thread = threading.Thread(target=ros2_spin, daemon=True)
            ros2_thread.start()
            
            logger.info("ROS2 節點已啟動")
            return True
        except Exception as e:
            logger.error(f"ROS2 初始化失敗: {e}")
            return False
    return False

# 創建 FastAPI 應用
app = FastAPI(
    title="醫院藥物管理系統 - 真實 ROS2 版本",
    description="整合真實 ROS2 環境的醫院藥物管理系統",
    version="2.0.0"
)

# 添加 CORS 中間件
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 掛載靜態檔案
app.mount("/static", StaticFiles(directory="static"), name="static")

# 啟動事件
@app.on_event("startup")
async def startup_event():
    """應用啟動時的初始化"""
    logger.info("正在啟動醫院藥物管理系統...")
    
    if not init_ros2():
        logger.error("ROS2 初始化失敗，系統無法啟動")
        raise RuntimeError("需要真實的 ROS2 環境")
    
    logger.info("系統啟動完成")

@app.on_event("shutdown")
async def shutdown_event():
    """應用關閉時的清理"""
    global ros2_node
    if ros2_node:
        ros2_node.destroy_node()
    if ROS2_AVAILABLE:
        rclpy.shutdown()
    logger.info("系統已關閉")

# 根路由
@app.get("/")
async def root():
    """根路由，重導向到主界面"""
    return FileResponse('static/integrated_medicine_management.html')

# 靜態檔案路由
@app.get("/{filename}")
async def serve_static_files(filename: str):
    """提供靜態檔案"""
    try:
        return FileResponse(f'static/{filename}')
    except:
        raise HTTPException(status_code=404, detail="檔案未找到")

# 系統狀態 API
@app.get("/api/system/status")
async def get_system_status():
    """獲取系統狀態"""
    return {
        "status": "running",
        "ros2_available": ROS2_AVAILABLE,
        "ros2_mode": "real" if ROS2_AVAILABLE else "unavailable",
        "timestamp": datetime.now().isoformat()
    }

# ROS2 狀態 API
@app.get("/api/ros2/status")
async def get_ros2_status():
    """獲取 ROS2 狀態"""
    if not ROS2_AVAILABLE or not ros2_node:
        raise HTTPException(status_code=503, detail="ROS2 不可用")
    
    return {
        "available": True,
        "mode": "real",
        "node_name": ros2_node.get_name(),
        "timestamp": datetime.now().isoformat()
    }

# 藥物管理 API
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
            "dosage": medicine.dosage,
            "is_active": medicine.is_active,
            "created_at": medicine.created_at.isoformat() if medicine.created_at else None,
            "updated_at": medicine.updated_at.isoformat() if medicine.updated_at else None
        })
    
    # 發布到 ROS2
    if ros2_node:
        ros2_node.publish_medicine_info({
            "type": "basic_medicines",
            "data": result,
            "count": len(result)
        })
    
    logger.info(f"基本藥物列表構建完成，共 {len(result)} 種藥物")
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
            "unit_dose": float(detail.unit_dose),
            "side_effects": detail.side_effects,
            "storage_conditions": detail.storage_conditions,
            "expiry_date": detail.expiry_date,
            "barcode": detail.barcode,
            "appearance_type": detail.appearance_type,
            "notes": detail.notes
        })
    
    logger.info(f"詳細藥物列表構建完成，共 {len(result)} 種藥物")
    return result

@app.post("/api/medicine/detailed")
def create_detailed_medicine(medicine_detail: MedicineDetailCreate, db: Session = Depends(get_db)):
    """創建詳細藥物資訊"""
    logger.info(f"正在為藥物 ID {medicine_detail.medicine_id} 創建詳細資訊")
    
    # 檢查藥物是否存在
    medicine = db.query(Medicine).filter(Medicine.id == medicine_detail.medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物不存在")
    
    # 檢查是否已有詳細資訊
    existing_detail = db.query(MedicineDetail).filter(
        MedicineDetail.medicine_id == medicine_detail.medicine_id
    ).first()
    if existing_detail:
        raise HTTPException(status_code=400, detail="該藥物已有詳細資訊")
    
    # 創建詳細資訊
    db_detail = MedicineDetail(**medicine_detail.dict())
    db.add(db_detail)
    db.commit()
    db.refresh(db_detail)
    
    logger.info(f"詳細藥物資訊創建成功: {db_detail.id}")
    return {"message": "詳細藥物資訊創建成功", "detail_id": db_detail.id}

@app.post("/api/medicine/unified")
def create_unified_medicine(medicine: UnifiedMedicineCreate, db: Session = Depends(get_db)):
    """創建統一藥物（包含基本和詳細資訊）"""
    logger.info(f"正在創建統一藥物: {medicine.name}")
    
    try:
        # 創建基本藥物資訊
        basic_medicine = Medicine(
            name=medicine.name,
            amount=medicine.amount,
            position=medicine.position,
            manufacturer=medicine.manufacturer,
            dosage=medicine.dosage
        )
        db.add(basic_medicine)
        db.flush()  # 獲取 ID
        
        # 創建詳細藥物資訊
        detailed_medicine = MedicineDetail(
            medicine_id=basic_medicine.id,
            description=medicine.description,
            ingredient=medicine.ingredient,
            category=medicine.category,
            usage_method=medicine.usage_method,
            unit_dose=medicine.unit_dose,
            side_effects=medicine.side_effects,
            storage_conditions=medicine.storage_conditions,
            expiry_date=medicine.expiry_date,
            barcode=medicine.barcode,
            appearance_type=medicine.appearance_type,
            notes=medicine.notes
        )
        db.add(detailed_medicine)
        db.flush()  # 確保詳細資訊也有 ID
        
        db.commit()
        
        logger.info(f"統一藥物創建成功: 基本 ID {basic_medicine.id}, 詳細 ID {detailed_medicine.id}")
        return {
            "message": "統一藥物創建成功",
            "medicine_id": basic_medicine.id,
            "detail_id": detailed_medicine.id
        }
        
    except Exception as e:
        db.rollback()
        logger.error(f"創建統一藥物失敗: {e}")
        raise HTTPException(status_code=500, detail=f"創建失敗: {str(e)}")

@app.put("/api/medicine/{medicine_id}")
def update_medicine(medicine_id: int, medicine_update: MedicineUpdate, db: Session = Depends(get_db)):
    """更新藥物資訊"""
    logger.info(f"正在更新藥物 ID: {medicine_id}")
    
    medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物不存在")
    
    # 更新非 None 的欄位
    update_data = medicine_update.dict(exclude_unset=True)
    for field, value in update_data.items():
        setattr(medicine, field, value)
    
    medicine.updated_at = datetime.now()
    db.commit()
    db.refresh(medicine)
    
    logger.info(f"藥物更新成功: {medicine_id}")
    return {"message": "藥物更新成功", "medicine_id": medicine_id}

@app.delete("/api/medicine/{medicine_identifier}")
def delete_medicine(medicine_identifier: str, db: Session = Depends(get_db)):
    """刪除藥物（支援 ID 或名稱）"""
    logger.info(f"正在刪除藥物: {medicine_identifier}")
    
    # 嘗試作為 ID 查詢
    medicine = None
    try:
        medicine_id = int(medicine_identifier)
        medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    except ValueError:
        # 如果不是數字，則作為名稱查詢
        medicine = db.query(Medicine).filter(Medicine.name == medicine_identifier).first()
    
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物不存在")
    
    # 檢查是否在處方籤中使用
    prescription_count = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.medicine_id == medicine.id
    ).count()
    
    if prescription_count > 0:
        # 如果在處方籤中使用，則設為不活躍而不是刪除
        medicine.is_active = False
        medicine.updated_at = datetime.now()
        db.commit()
        logger.info(f"藥物已設為不活躍（因為在處方籤中使用）: {medicine.id}")
        return {"message": "藥物已設為不活躍（因為在處方籤中使用）", "medicine_id": medicine.id}
    else:
        # 可以安全刪除
        medicine_id = medicine.id
        db.delete(medicine)
        db.commit()
        logger.info(f"藥物已刪除: {medicine_id}")
        return {"message": "藥物已刪除", "medicine_id": medicine_id}

@app.get("/api/medicine/search/{medicine_name}")
def search_medicine_by_name(medicine_name: str, db: Session = Depends(get_db)):
    """根據名稱搜尋藥物"""
    logger.info(f"搜尋藥物: {medicine_name}")
    
    medicines = db.query(Medicine).filter(
        Medicine.name.ilike(f"%{medicine_name}%"),
        Medicine.is_active == True
    ).all()
    
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
            "created_at": medicine.created_at.isoformat() if medicine.created_at else None,
            "updated_at": medicine.updated_at.isoformat() if medicine.updated_at else None
        })
    
    logger.info(f"搜尋到 {len(result)} 個結果")
    return result

def _adjust_medicine_stock_impl(db: Session, medicine_id: int = None, medicine_name: str = None, adjustment: int = 0, reason: str = ""):
    """調整藥物庫存的內部實現"""
    medicine = None
    
    if medicine_id:
        medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    elif medicine_name:
        medicine = db.query(Medicine).filter(Medicine.name == medicine_name).first()
    
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物不存在")
    
    old_amount = medicine.amount
    new_amount = old_amount + adjustment
    
    if new_amount < 0:
        raise HTTPException(status_code=400, detail="庫存不足")
    
    medicine.amount = new_amount
    medicine.updated_at = datetime.now()
    db.commit()
    
    logger.info(f"藥物 {medicine.name} 庫存調整: {old_amount} -> {new_amount} (調整: {adjustment}, 原因: {reason})")
    
    return {
        "message": "庫存調整成功",
        "medicine_id": medicine.id,
        "medicine_name": medicine.name,
        "old_amount": old_amount,
        "new_amount": new_amount,
        "adjustment": adjustment,
        "reason": reason
    }

@app.post("/api/medicine/adjust-stock")
def adjust_medicine_stock(stock_adjustment: StockAdjustment, db: Session = Depends(get_db)):
    """調整藥物庫存"""
    logger.info(f"正在調整庫存: {stock_adjustment}")
    
    return _adjust_medicine_stock_impl(
        db=db,
        medicine_id=stock_adjustment.medicine_id,
        medicine_name=stock_adjustment.medicine_name,
        adjustment=stock_adjustment.adjustment,
        reason=stock_adjustment.reason
    )

@app.post("/api/medicine/{medicine_id}/adjust-stock")
def adjust_medicine_stock_by_id(medicine_id: int, stock_adjustment: StockAdjustment, db: Session = Depends(get_db)):
    """根據 ID 調整藥物庫存"""
    logger.info(f"正在調整藥物 {medicine_id} 的庫存")
    
    return _adjust_medicine_stock_impl(
        db=db,
        medicine_id=medicine_id,
        adjustment=stock_adjustment.adjustment,
        reason=stock_adjustment.reason
    )

# 處方籤管理 API
@app.get("/api/prescription/")
def get_prescriptions(db: Session = Depends(get_db)):
    """獲取處方籤列表"""
    logger.info("獲取處方籤列表")
    
    prescriptions = db.query(Prescription).all()
    
    result = []
    for prescription in prescriptions:
        medicines = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.prescription_id == prescription.id
        ).all()
        
        medicine_count = len(medicines)
        
        result.append({
            "id": prescription.id,
            "patient_name": prescription.patient_name,
            "doctor_name": prescription.doctor_name,
            "status": prescription.status,
            "medicine_count": medicine_count,
            "created_at": prescription.created_at.isoformat() if prescription.created_at else None,
            "updated_at": prescription.updated_at.isoformat() if prescription.updated_at else None
        })
    
    logger.info(f"返回 {len(result)} 張處方籤")
    return result

@app.post("/api/prescription/")
def create_prescription(prescription: PrescriptionCreate, db: Session = Depends(get_db)):
    """創建處方籤並立即扣減庫存"""
    logger.info(f"正在創建處方籤，患者: {prescription.patient_name}, 醫生: {prescription.doctor_name}")
    
    try:
        # 首先檢查所有藥物的庫存是否足夠
        insufficient_medicines = []
        for medicine_info in prescription.medicines:
            medicine_name = medicine_info.get("name")
            quantity = int(medicine_info.get("quantity", 1))
            
            medicine = db.query(Medicine).filter(
                Medicine.name == medicine_name,
                Medicine.is_active == True
            ).first()
            
            if not medicine:
                raise HTTPException(status_code=404, detail=f"藥物不存在: {medicine_name}")
            
            if medicine.amount < quantity:
                insufficient_medicines.append({
                    "name": medicine_name,
                    "required": quantity,
                    "available": medicine.amount
                })
        
        if insufficient_medicines:
            raise HTTPException(
                status_code=400, 
                detail=f"庫存不足: {insufficient_medicines}"
            )
        
        # 創建處方籤
        db_prescription = Prescription(
            patient_name=prescription.patient_name,
            doctor_name=prescription.doctor_name,
            status="pending"  # 新處方籤預設為 pending 狀態
        )
        db.add(db_prescription)
        db.flush()  # 獲取 prescription ID
        
        # 添加藥物並立即扣減庫存
        total_medicines = 0
        for medicine_info in prescription.medicines:
            medicine_name = medicine_info.get("name")
            quantity = int(medicine_info.get("quantity", 1))
            dosage = medicine_info.get("dosage", "")
            frequency = medicine_info.get("frequency", "")
            duration = medicine_info.get("duration", "")
            notes = medicine_info.get("notes", "")
            
            medicine = db.query(Medicine).filter(
                Medicine.name == medicine_name,
                Medicine.is_active == True
            ).first()
            
            # 立即扣減庫存
            medicine.amount -= quantity
            medicine.updated_at = datetime.now()
            
            # 添加處方籤藥物記錄
            prescription_medicine = PrescriptionMedicine(
                prescription_id=db_prescription.id,
                medicine_id=medicine.id,
                quantity=quantity,
                dosage=dosage,
                frequency=frequency,
                duration=duration,
                notes=notes
            )
            db.add(prescription_medicine)
            total_medicines += 1
        
        db.commit()
        
        # 發布到 ROS2
        if ros2_node:
            ros2_node.publish_order_status({
                "type": "new_prescription",
                "prescription_id": db_prescription.id,
                "patient_name": prescription.patient_name,
                "medicine_count": total_medicines,
                "status": "pending"
            })
        
        logger.info(f"處方籤創建成功: {db_prescription.id}，已扣減庫存")
        return {
            "message": "處方籤創建成功，庫存已扣減",
            "prescription_id": db_prescription.id,
            "medicine_count": total_medicines
        }
        
    except Exception as e:
        db.rollback()
        logger.error(f"創建處方籤失敗: {e}")
        raise HTTPException(status_code=500, detail=f"創建失敗: {str(e)}")

@app.get("/api/prescription/{prescription_id}")
def get_prescription_detail(prescription_id: int, db: Session = Depends(get_db)):
    """獲取處方籤詳情"""
    logger.info(f"獲取處方籤詳情: {prescription_id}")
    
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="處方籤不存在")
    
    medicines = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.prescription_id == prescription_id
    ).all()
    
    medicine_list = []
    for pm in medicines:
        medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
        medicine_list.append({
            "id": pm.medicine_id,
            "medicine_name": medicine.name if medicine else "未知藥物",
            "quantity": pm.quantity,
            "dosage": pm.dosage,
            "frequency": pm.frequency,
            "duration": pm.duration,
            "notes": pm.notes
        })
    
    result = {
        "id": prescription.id,
        "patient_name": prescription.patient_name,
        "doctor_name": prescription.doctor_name,
        "status": prescription.status,
        "created_at": prescription.created_at.isoformat() if prescription.created_at else None,
        "updated_at": prescription.updated_at.isoformat() if prescription.updated_at else None,
        "medicines": medicine_list
    }
    
    logger.info(f"處方籤詳情獲取成功: {prescription_id}")
    return result

@app.put("/api/prescription/{prescription_id}/status")
def update_prescription_status(prescription_id: int, status_update: PrescriptionStatusUpdate, db: Session = Depends(get_db)):
    """更新處方籤狀態"""
    logger.info(f"更新處方籤 {prescription_id} 狀態為: {status_update.status}")
    
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="處方籤不存在")
    
    old_status = prescription.status
    prescription.status = status_update.status
    prescription.updated_at = datetime.now()
    
    # 處理庫存變更
    handle_prescription_stock_change(db, prescription_id, old_status, status_update.status)
    
    db.commit()
    
    # 發布到 ROS2
    if ros2_node:
        ros2_node.publish_order_status({
            "type": "status_update",
            "prescription_id": prescription_id,
            "old_status": old_status,
            "new_status": status_update.status
        })
    
    logger.info(f"處方籤 {prescription_id} 狀態已更新: {old_status} -> {status_update.status}")
    return {"message": "處方籤狀態更新成功", "old_status": old_status, "new_status": status_update.status}

def handle_prescription_stock_change(db: Session, prescription_id: int, old_status: str, new_status: str):
    """處理處方籤狀態變更時的庫存調整"""
    logger.info(f"處理處方籤 {prescription_id} 庫存變更: {old_status} -> {new_status}")
    
    # 只處理取消狀態的庫存恢復
    if new_status == "cancelled" and old_status != "cancelled":
        # 恢復庫存
        medicines = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.prescription_id == prescription_id
        ).all()
        
        for pm in medicines:
            medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
            if medicine:
                medicine.amount += pm.quantity
                medicine.updated_at = datetime.now()
                logger.info(f"恢復藥物 {medicine.name} 庫存: +{pm.quantity}")
    
    elif old_status == "cancelled" and new_status != "cancelled":
        # 重新扣減庫存（取消的處方籤重新啟用）
        medicines = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.prescription_id == prescription_id
        ).all()
        
        for pm in medicines:
            medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
            if medicine:
                if medicine.amount < pm.quantity:
                    raise HTTPException(
                        status_code=400, 
                        detail=f"藥物 {medicine.name} 庫存不足，無法重新啟用處方籤"
                    )
                medicine.amount -= pm.quantity
                medicine.updated_at = datetime.now()
                logger.info(f"重新扣減藥物 {medicine.name} 庫存: -{pm.quantity}")

@app.get("/api/prescription/pending/next")
def get_next_pending_prescription(db: Session = Depends(get_db)):
    """獲取下一個待處理的處方籤"""
    logger.info("獲取下一個待處理的處方籤")
    
    prescription = db.query(Prescription).filter(
        Prescription.status == "pending"
    ).order_by(Prescription.created_at.asc()).first()
    
    if not prescription:
        return {"message": "沒有待處理的處方籤", "prescription": None}
    
    medicines = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.prescription_id == prescription.id
    ).all()
    
    medicine_list = []
    for pm in medicines:
        medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
        medicine_list.append({
            "id": pm.medicine_id,
            "medicine_name": medicine.name if medicine else "未知藥物",
            "quantity": pm.quantity,
            "dosage": pm.dosage,
            "frequency": pm.frequency,
            "duration": pm.duration,
            "notes": pm.notes
        })
    
    result = {
        "id": prescription.id,
        "patient_name": prescription.patient_name,
        "doctor_name": prescription.doctor_name,
        "status": prescription.status,
        "created_at": prescription.created_at.isoformat() if prescription.created_at else None,
        "medicines": medicine_list
    }
    
    logger.info(f"找到待處理處方籤: {prescription.id}")
    return {"message": "找到待處理處方籤", "prescription": result}

# ROS2 整合 API
@app.get("/api/ros2/pending-orders")
def get_pending_orders(db: Session = Depends(get_db)):
    """獲取所有待處理訂單"""
    if not ROS2_AVAILABLE or not ros2_node:
        raise HTTPException(status_code=503, detail="ROS2 不可用")
    
    logger.info("獲取待處理訂單列表")
    
    prescriptions = db.query(Prescription).filter(
        Prescription.status == "pending"
    ).order_by(Prescription.created_at.asc()).all()
    
    orders = []
    for prescription in prescriptions:
        medicines = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.prescription_id == prescription.id
        ).all()
        
        medicine_list = []
        for pm in medicines:
            medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
            medicine_list.append({
                "medicine_id": pm.medicine_id,
                "medicine_name": medicine.name if medicine else "未知藥物",
                "quantity": pm.quantity,
                "position": medicine.position if medicine else "未知位置"
            })
        
        orders.append({
            "prescription_id": prescription.id,
            "patient_name": prescription.patient_name,
            "doctor_name": prescription.doctor_name,
            "created_at": prescription.created_at.isoformat() if prescription.created_at else None,
            "medicines": medicine_list,
            "total_medicines": len(medicine_list)
        })
    
    logger.info(f"找到 {len(orders)} 個待處理訂單")
    return {"orders": orders, "total": len(orders)}

@app.post("/api/ros2/request-order-confirmation")
def request_order_confirmation(order_request: ROS2OrderRequest, db: Session = Depends(get_db)):
    """請求訂單確認"""
    if not ROS2_AVAILABLE or not ros2_node:
        raise HTTPException(status_code=503, detail="ROS2 不可用")
    
    logger.info(f"請求訂單確認: {order_request.prescription_id}")
    
    prescription = db.query(Prescription).filter(
        Prescription.id == order_request.prescription_id,
        Prescription.status == "pending"
    ).first()
    
    if not prescription:
        raise HTTPException(status_code=404, detail="找不到待處理的處方籤")
    
    medicines = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.prescription_id == prescription.id
    ).all()
    
    medicine_list = []
    for pm in medicines:
        medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
        medicine_list.append({
            "medicine_id": pm.medicine_id,
            "medicine_name": medicine.name if medicine else "未知藥物",
            "quantity": pm.quantity,
            "position": medicine.position if medicine else "未知位置"
        })
    
    order_info = {
        "prescription_id": prescription.id,
        "patient_name": prescription.patient_name,
        "doctor_name": prescription.doctor_name,
        "medicines": medicine_list,
        "total_medicines": len(medicine_list)
    }
    
    # 發布到 ROS2 請求確認
    ros2_node.publish_order_status({
        "type": "order_confirmation_request",
        "order": order_info
    })
    
    logger.info(f"訂單確認請求已發送: {order_request.prescription_id}")
    return {
        "message": "訂單確認請求已發送到 ROS2",
        "order": order_info
    }

@app.post("/api/ros2/confirm-and-execute-order")
def confirm_and_execute_order(order_request: ROS2OrderRequest, db: Session = Depends(get_db)):
    """確認並執行訂單"""
    if not ROS2_AVAILABLE or not ros2_node:
        raise HTTPException(status_code=503, detail="ROS2 不可用")
    
    logger.info(f"確認並執行訂單: {order_request.prescription_id}")
    
    prescription = db.query(Prescription).filter(
        Prescription.id == order_request.prescription_id,
        Prescription.status == "pending"
    ).first()
    
    if not prescription:
        raise HTTPException(status_code=404, detail="找不到待處理的處方籤")
    
    # 更新狀態為處理中
    prescription.status = "processing"
    prescription.updated_at = datetime.now()
    db.commit()
    
    medicines = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.prescription_id == prescription.id
    ).all()
    
    medicine_list = []
    for pm in medicines:
        medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
        medicine_list.append({
            "medicine_id": pm.medicine_id,
            "medicine_name": medicine.name if medicine else "未知藥物",
            "quantity": pm.quantity,
            "position": medicine.position if medicine else "未知位置"
        })
    
    order_info = {
        "prescription_id": prescription.id,
        "patient_name": prescription.patient_name,
        "doctor_name": prescription.doctor_name,
        "medicines": medicine_list,
        "total_medicines": len(medicine_list),
        "status": "processing"
    }
    
    # 發布到 ROS2 開始執行
    ros2_node.publish_order_status({
        "type": "order_execution_start",
        "order": order_info
    })
    
    logger.info(f"訂單執行已開始: {order_request.prescription_id}")
    return {
        "message": "訂單確認成功，ROS2 開始執行",
        "order": order_info
    }

@app.post("/api/ros2/complete-order")
def complete_order(order_request: ROS2OrderRequest, db: Session = Depends(get_db)):
    """標記訂單完成"""
    logger.info(f"標記訂單完成: {order_request.prescription_id}")
    
    prescription = db.query(Prescription).filter(
        Prescription.id == order_request.prescription_id
    ).first()
    
    if not prescription:
        raise HTTPException(status_code=404, detail="處方籤不存在")
    
    if prescription.status != "processing":
        raise HTTPException(status_code=400, detail="只有處理中的訂單可以標記為完成")
    
    # 更新狀態為完成
    prescription.status = "completed"
    prescription.updated_at = datetime.now()
    db.commit()
    
    # 發布到 ROS2
    if ros2_node:
        ros2_node.publish_order_status({
            "type": "order_completed",
            "prescription_id": prescription.id,
            "patient_name": prescription.patient_name
        })
    
    logger.info(f"訂單已標記為完成: {order_request.prescription_id}")
    return {
        "message": "訂單已標記為完成",
        "prescription_id": prescription.id,
        "status": "completed"
    }

@app.post("/api/ros2/query-medicine")
def query_medicine_info(query: ROS2MedicineQuery, db: Session = Depends(get_db)):
    """查詢藥物資訊"""
    if not ROS2_AVAILABLE or not ros2_node:
        raise HTTPException(status_code=503, detail="ROS2 不可用")
    
    logger.info(f"ROS2 查詢藥物資訊: {query}")
    
    medicine = None
    if query.medicine_id:
        medicine = db.query(Medicine).filter(Medicine.id == query.medicine_id).first()
    elif query.medicine_name:
        medicine = db.query(Medicine).filter(Medicine.name == query.medicine_name).first()
    
    if not medicine:
        return {
            "success": False,
            "message": "藥物不存在",
            "medicine": None
        }
    
    basic_info = {
        "id": medicine.id,
        "name": medicine.name,
        "amount": medicine.amount,
        "position": medicine.position,
        "manufacturer": medicine.manufacturer,
        "dosage": medicine.dosage,
        "is_active": medicine.is_active
    }
    
    result = {
        "success": True,
        "message": "藥物資訊獲取成功",
        "medicine": basic_info
    }
    
    if query.include_detailed:
        detail = db.query(MedicineDetail).filter(
            MedicineDetail.medicine_id == medicine.id
        ).first()
        
        if detail:
            result["detailed_info"] = {
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
    
    logger.info(f"藥物資訊查詢完成: {medicine.name}")
    return result

@app.post("/api/ros2/batch-query-medicines")
def batch_query_medicines(query: ROS2BatchMedicineQuery, db: Session = Depends(get_db)):
    """批量查詢藥物資訊"""
    if not ROS2_AVAILABLE or not ros2_node:
        raise HTTPException(status_code=503, detail="ROS2 不可用")
    
    logger.info(f"ROS2 批量查詢藥物: {query}")
    
    medicines = []
    
    if query.medicine_ids:
        medicines.extend(
            db.query(Medicine).filter(Medicine.id.in_(query.medicine_ids)).all()
        )
    
    if query.medicine_names:
        medicines.extend(
            db.query(Medicine).filter(Medicine.name.in_(query.medicine_names)).all()
        )
    
    # 去重
    unique_medicines = {m.id: m for m in medicines}.values()
    
    result_medicines = []
    for medicine in unique_medicines:
        basic_info = {
            "id": medicine.id,
            "name": medicine.name,
            "amount": medicine.amount,
            "position": medicine.position,
            "manufacturer": medicine.manufacturer,
            "dosage": medicine.dosage,
            "is_active": medicine.is_active
        }
        
        medicine_data = {"basic_info": basic_info}
        
        if query.include_detailed:
            detail = db.query(MedicineDetail).filter(
                MedicineDetail.medicine_id == medicine.id
            ).first()
            
            if detail:
                medicine_data["detailed_info"] = {
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
        
        result_medicines.append(medicine_data)
    
    logger.info(f"批量查詢完成，找到 {len(result_medicines)} 個藥物")
    return {
        "success": True,
        "message": f"批量查詢完成，找到 {len(result_medicines)} 個藥物",
        "medicines": result_medicines,
        "total": len(result_medicines)
    }

# 錯誤處理
@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    logger.error(f"HTTP錯誤: {exc.status_code} - {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail}
    )

if __name__ == "__main__":
    print("醫院藥物管理系統 - 真實 ROS2 版本")
    print("==================================================")
    print("注意: 此版本需要真實的 ROS2 環境")
    print("請確保已正確設置 ROS2 環境變數")
    print("==================================================")
    print("整合管理界面: http://localhost:8001/integrated_medicine_management.html")
    print("處方籤管理: http://localhost:8001/Prescription.html")
    print("醫生界面: http://localhost:8001/doctor.html")
    print("API文檔: http://localhost:8001/docs")
    print("ROS2狀態: 真實環境")
    print("==================================================")
    
    uvicorn.run(app, host="0.0.0.0", port=8001, log_level="info")
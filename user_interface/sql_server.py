#!/usr/bin/env python3
"""
Hospital Medicine Management System - SQL Database Version
醫院藥物管理系統 - SQL資料庫版本
"""

from fastapi import FastAPI, HTTPException, Depends, WebSocket, WebSocketDisconnect, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from pydantic import BaseModel
from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, desc
from typing import List, Optional, Dict, Any
from datetime import datetime, date
import json
import asyncio
import uvicorn
import yaml

# 導入資料庫模型和配置
from database import (
    get_db, init_database,
    MedicineBasic as DBMedicineBasic,
    MedicineDetailed as DBMedicineDetailed,
    Prescription as DBPrescription,
    PrescriptionMedicine as DBPrescriptionMedicine,
    StockLog as DBStockLog,
    SystemLog as DBSystemLog
)
from yaml_storage import YAMLMedicineStorage

# Pydantic Models for API
class MedicineBasicCreate(BaseModel):
    name: str
    amount: int
    usage_days: Optional[int] = None
    position: str
    manufacturer: Optional[str] = ""
    dosage: Optional[str] = ""
    prompt: Optional[str] = ""

class MedicineBasicResponse(BaseModel):
    id: int
    name: str
    amount: int
    usage_days: Optional[int]
    position: str
    manufacturer: Optional[str]
    dosage: Optional[str]
    prompt: Optional[str]
    created_time: datetime
    updated_time: datetime
    is_active: bool

    class Config:
        from_attributes = True

class MedicineDetailedCreate(BaseModel):
    medicine_id: int
    description: Optional[str] = ""
    ingredient: Optional[str] = ""
    category: Optional[str] = ""
    usage_method: Optional[str] = ""
    unit_dose: Optional[str] = ""
    side_effects: Optional[str] = ""
    storage_conditions: Optional[str] = ""
    expiry_date: Optional[date] = None
    barcode: Optional[str] = ""
    appearance_type: Optional[str] = ""
    notes: Optional[str] = ""

class MedicineDetailedResponse(BaseModel):
    id: int
    medicine_id: int
    description: Optional[str]
    ingredient: Optional[str]
    category: Optional[str]
    usage_method: Optional[str]
    unit_dose: Optional[str]
    side_effects: Optional[str]
    storage_conditions: Optional[str]
    expiry_date: Optional[date]
    barcode: Optional[str]
    appearance_type: Optional[str]
    notes: Optional[str]
    created_time: datetime
    updated_time: datetime

    class Config:
        from_attributes = True

class UnifiedMedicineCreate(BaseModel):
    basic_data: MedicineBasicCreate
    detailed_data: Optional[MedicineDetailedCreate] = None

class PrescriptionCreate(BaseModel):
    patient_name: str
    patient_id: str
    doctor_name: str
    diagnosis: Optional[str] = ""
    medicines: List[Dict[str, Any]]  # [{"medicine_id": 1, "dosage": "500mg", "frequency": "每日三次", "duration": "7天", "instructions": ""}]

class PrescriptionResponse(BaseModel):
    id: int
    patient_name: str
    patient_id: str
    doctor_name: str
    diagnosis: Optional[str]
    status: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class StockAdjustment(BaseModel):
    medicine_name: str
    action: str  # "add" or "subtract"
    quantity: int
    reason: Optional[str] = ""

# FastAPI App
app = FastAPI(title="醫院藥物管理系統 - SQL版本", version="2.1.0")

# CORS設定
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 靜態文件
app.mount("/css", StaticFiles(directory="static/css"), name="css")
app.mount("/js", StaticFiles(directory="static/js"), name="js")

# 初始化YAML儲存
yaml_storage = YAMLMedicineStorage()

# WebSocket管理器
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except:
                await self.disconnect(connection)

manager = ConnectionManager()

# 系統記錄函數
def log_system_action(db: Session, action: str, table_name: str = None, record_id: int = None, details: str = None, request: Request = None):
    """記錄系統操作"""
    log = DBSystemLog(
        action=action,
        table_name=table_name,
        record_id=record_id,
        details=details,
        ip_address=request.client.host if request else None,
        user_agent=request.headers.get("user-agent") if request else None
    )
    db.add(log)
    db.commit()

# HTML頁面路由
@app.get("/")
async def read_root():
    return FileResponse("static/html/Medicine.html")

@app.get("/Medicine.html")
async def medicine_page():
    return FileResponse("static/html/Medicine.html")

@app.get("/doctor.html")
async def doctor_page():
    return FileResponse("static/html/doctor.html")

@app.get("/Prescription.html")
async def prescription_page():
    return FileResponse("static/html/Prescription.html")

@app.get("/unified_medicine.html")
async def unified_medicine_page():
    return FileResponse("static/html/unified_medicine.html")

@app.get("/integrated_medicine_management.html")
async def integrated_management_page():
    return FileResponse("static/html/integrated_medicine_management.html")



# WebSocket端點
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            # 處理接收到的訊息
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# ========== 基本藥物 API ==========

@app.get("/api/medicine/basic", response_model=Dict[str, Any])
async def get_basic_medicines(db: Session = Depends(get_db)):
    """獲取所有基本藥物"""
    medicines = db.query(DBMedicineBasic).filter(DBMedicineBasic.is_active == True).all()
    return {
        "medicines": [
            {
                "id": med.id,
                "name": med.name,
                "amount": med.amount,
                "usage_days": med.usage_days,
                "position": med.position,
                "manufacturer": med.manufacturer,
                "dosage": med.dosage,
                "prompt": med.prompt,
                "created_time": med.created_time.isoformat(),
                "updated_time": med.updated_time.isoformat()
            }
            for med in medicines
        ]
    }

@app.post("/api/medicine/basic", response_model=MedicineBasicResponse)
async def create_basic_medicine(medicine: MedicineBasicCreate, request: Request, db: Session = Depends(get_db)):
    """創建基本藥物"""
    # 檢查藥物名稱是否已存在
    existing = db.query(DBMedicineBasic).filter(DBMedicineBasic.name == medicine.name).first()
    if existing:
        raise HTTPException(status_code=400, detail="藥物名稱已存在")
    
    db_medicine = DBMedicineBasic(**medicine.dict())
    db.add(db_medicine)
    db.commit()
    db.refresh(db_medicine)
    
    # 記錄系統操作
    log_system_action(db, "create_basic_medicine", "medicine_basic", db_medicine.id, f"創建藥物: {medicine.name}", request)
    
    # 自動同步YAML
    await auto_sync_yaml()
    
    # WebSocket通知
    notification = {
        "type": "basic_medicine_created",
        "medicine": {
            "id": db_medicine.id,
            "name": db_medicine.name,
            "amount": db_medicine.amount,
            "position": db_medicine.position
        },
        "timestamp": datetime.now().isoformat()
    }
    await manager.broadcast(json.dumps(notification, ensure_ascii=False))
    
    return db_medicine

@app.get("/api/medicine/basic/{medicine_id}", response_model=MedicineBasicResponse)
async def get_basic_medicine(medicine_id: int, db: Session = Depends(get_db)):
    """獲取特定基本藥物"""
    medicine = db.query(DBMedicineBasic).filter(DBMedicineBasic.id == medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物未找到")
    return medicine

@app.put("/api/medicine/basic/{medicine_id}", response_model=MedicineBasicResponse)
async def update_basic_medicine(medicine_id: int, medicine: MedicineBasicCreate, request: Request, db: Session = Depends(get_db)):
    """更新基本藥物"""
    db_medicine = db.query(DBMedicineBasic).filter(DBMedicineBasic.id == medicine_id).first()
    if not db_medicine:
        raise HTTPException(status_code=404, detail="藥物未找到")
    
    # 更新欄位
    for field, value in medicine.dict().items():
        setattr(db_medicine, field, value)
    
    db.commit()
    db.refresh(db_medicine)
    
    # 記錄系統操作
    log_system_action(db, "update_basic_medicine", "medicine_basic", medicine_id, f"更新藥物: {medicine.name}", request)
    
    # 自動同步YAML
    await auto_sync_yaml()
    
    return db_medicine

@app.delete("/api/medicine/{medicine_name}")
async def delete_medicine(medicine_name: str, request: Request, db: Session = Depends(get_db)):
    """刪除藥物（軟刪除）"""
    medicine = db.query(DBMedicineBasic).filter(DBMedicineBasic.name == medicine_name).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物未找到")
    
    # 軟刪除
    medicine.is_active = False
    db.commit()
    
    # 記錄系統操作
    log_system_action(db, "delete_medicine", "medicine_basic", medicine.id, f"刪除藥物: {medicine_name}", request)
    
    return {"message": f"藥物 {medicine_name} 已刪除"}

# ========== 詳細藥物 API ==========

@app.get("/api/medicine/detailed", response_model=Dict[str, Any])
async def get_detailed_medicines(db: Session = Depends(get_db)):
    """獲取所有詳細藥物"""
    medicines = db.query(DBMedicineDetailed).join(DBMedicineBasic).filter(DBMedicineBasic.is_active == True).all()
    return {
        "medicines": [
            {
                "id": med.id,
                "medicine_id": med.medicine_id,
                "medicine_name": med.basic_medicine.name,
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
                "notes": med.notes,
                "created_time": med.created_time.isoformat(),
                "updated_time": med.updated_time.isoformat()
            }
            for med in medicines
        ]
    }

@app.post("/api/medicine/detailed", response_model=MedicineDetailedResponse)
async def create_detailed_medicine(medicine: MedicineDetailedCreate, request: Request, db: Session = Depends(get_db)):
    """創建詳細藥物資料"""
    # 檢查基本藥物是否存在
    basic_medicine = db.query(DBMedicineBasic).filter(DBMedicineBasic.id == medicine.medicine_id).first()
    if not basic_medicine:
        raise HTTPException(status_code=404, detail="基本藥物未找到")
    
    # 檢查是否已有詳細資料
    existing = db.query(DBMedicineDetailed).filter(DBMedicineDetailed.medicine_id == medicine.medicine_id).first()
    if existing:
        raise HTTPException(status_code=400, detail="該藥物已有詳細資料")
    
    db_medicine = DBMedicineDetailed(**medicine.dict())
    db.add(db_medicine)
    db.commit()
    db.refresh(db_medicine)
    
    # 記錄系統操作
    log_system_action(db, "create_detailed_medicine", "medicine_detailed", db_medicine.id, f"創建詳細資料: {basic_medicine.name}", request)
    
    # 自動同步YAML
    await auto_sync_yaml()
    
    return db_medicine

# ========== 統一藥物 API ==========

@app.post("/api/medicine/unified")
async def create_unified_medicine(medicine: UnifiedMedicineCreate, request: Request, db: Session = Depends(get_db)):
    """統一創建基本+詳細藥物資料"""
    try:
        # 創建基本藥物
        db_basic = DBMedicineBasic(**medicine.basic_data.dict())
        db.add(db_basic)
        db.commit()
        db.refresh(db_basic)
        
        # 創建詳細藥物（如果有提供）
        db_detailed = None
        if medicine.detailed_data:
            detailed_dict = medicine.detailed_data.dict()
            detailed_dict["medicine_id"] = db_basic.id
            db_detailed = DBMedicineDetailed(**detailed_dict)
            db.add(db_detailed)
            db.commit()
            db.refresh(db_detailed)
        
        # 記錄系統操作
        log_system_action(db, "create_unified_medicine", "medicine_basic", db_basic.id, f"統一創建藥物: {medicine.basic_data.name}", request)
        
        # 自動同步YAML
        await auto_sync_yaml()
        
        # WebSocket通知
        notification = {
            "type": "unified_medicine_created",
            "basic_medicine": {
                "id": db_basic.id,
                "name": db_basic.name,
                "amount": db_basic.amount,
                "position": db_basic.position,
                "prompt": db_basic.prompt
            },
            "detailed_medicine": db_detailed.id if db_detailed else None,
            "timestamp": datetime.now().isoformat()
        }
        await manager.broadcast(json.dumps(notification, ensure_ascii=False))
        
        return {
            "message": "✅ 統一藥物資料已保存",
            "basic_medicine": {
                "id": db_basic.id,
                "name": db_basic.name,
                "amount": db_basic.amount,
                "position": db_basic.position,
                "prompt": db_basic.prompt
            },
            "detailed_medicine": db_detailed.id if db_detailed else None,
            "yaml_exported": True
        }
        
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"保存失敗: {str(e)}")

# ========== 庫存管理 API ==========

@app.post("/api/medicine/adjust-stock")
async def adjust_stock(adjustment: StockAdjustment, request: Request, db: Session = Depends(get_db)):
    """調整藥物庫存"""
    # 尋找藥物
    medicine = db.query(DBMedicineBasic).filter(DBMedicineBasic.name == adjustment.medicine_name).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="藥物未找到")
    
    # 記錄調整前數量
    quantity_before = medicine.amount
    
    # 計算新數量
    if adjustment.action == "add":
        quantity_change = adjustment.quantity
        new_amount = medicine.amount + adjustment.quantity
    elif adjustment.action == "subtract":
        quantity_change = -adjustment.quantity
        new_amount = medicine.amount - adjustment.quantity
        if new_amount < 0:
            raise HTTPException(status_code=400, detail="庫存不足，無法減少此數量")
    else:
        raise HTTPException(status_code=400, detail="無效的操作類型")
    
    # 更新庫存
    medicine.amount = new_amount
    
    # 創建庫存記錄
    stock_log = DBStockLog(
        medicine_id=medicine.id,
        action=adjustment.action,
        quantity_before=quantity_before,
        quantity_change=quantity_change,
        quantity_after=new_amount,
        reason=adjustment.reason,
        operator="系統操作"
    )
    db.add(stock_log)
    
    db.commit()
    
    # 記錄系統操作
    log_system_action(db, f"stock_{adjustment.action}", "medicine_basic", medicine.id, 
                     f"{adjustment.action} {adjustment.quantity} units", request)
    
    return {
        "message": f"庫存已{'增加' if adjustment.action == 'add' else '減少'} {adjustment.quantity}",
        "medicine_name": adjustment.medicine_name,
        "quantity_before": quantity_before,
        "quantity_after": new_amount,
        "action": adjustment.action
    }

# ========== 處方籤 API ==========

@app.get("/api/prescription/", response_model=Dict[str, Any])
async def get_prescriptions(db: Session = Depends(get_db)):
    """獲取所有處方籤"""
    prescriptions = db.query(DBPrescription).order_by(desc(DBPrescription.created_at)).all()
    return {
        "prescriptions": [
            {
                "id": p.id,
                "patient_name": p.patient_name,
                "patient_id": p.patient_id,
                "doctor_name": p.doctor_name,
                "diagnosis": p.diagnosis,
                "status": p.status,
                "created_at": p.created_at.isoformat(),
                "medicines": [
                    [
                        pm.medicine.name,
                        pm.dosage,
                        pm.frequency,
                        pm.duration,
                        pm.instructions or ""
                    ]
                    for pm in p.medicines
                ]
            }
            for p in prescriptions
        ]
    }

@app.post("/api/prescription/", response_model=Dict[str, Any])
async def create_prescription(prescription: PrescriptionCreate, request: Request, db: Session = Depends(get_db)):
    """創建處方籤"""
    try:
        # 創建處方籤主記錄
        db_prescription = DBPrescription(
            patient_name=prescription.patient_name,
            patient_id=prescription.patient_id,
            doctor_name=prescription.doctor_name,
            diagnosis=prescription.diagnosis
        )
        db.add(db_prescription)
        db.commit()
        db.refresh(db_prescription)
        
        # 添加藥物明細
        for med_data in prescription.medicines:
            # 根據藥物名稱查找ID
            medicine = db.query(DBMedicineBasic).filter(DBMedicineBasic.name == med_data[0]).first()
            if medicine:
                db_med = DBPrescriptionMedicine(
                    prescription_id=db_prescription.id,
                    medicine_id=medicine.id,
                    dosage=med_data[1],
                    frequency=med_data[2],
                    duration=med_data[3],
                    instructions=med_data[4] if len(med_data) > 4 else ""
                )
                db.add(db_med)
        
        db.commit()
        
        # 記錄系統操作
        log_system_action(db, "create_prescription", "prescriptions", db_prescription.id, 
                         f"開立處方籤: {prescription.patient_name}", request)
        
        return {
            "message": "處方籤已成功開立",
            "prescription_id": db_prescription.id,
            "patient_name": prescription.patient_name,
            "medicines_count": len(prescription.medicines)
        }
        
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"創建處方籤失敗: {str(e)}")

@app.delete("/api/prescription/{prescription_id}")
async def delete_prescription(prescription_id: int, request: Request, db: Session = Depends(get_db)):
    """刪除處方籤"""
    prescription = db.query(DBPrescription).filter(DBPrescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="處方籤未找到")
    
    # 更新狀態為已取消
    prescription.status = "cancelled"
    db.commit()
    
    # 記錄系統操作
    log_system_action(db, "cancel_prescription", "prescriptions", prescription_id, f"取消處方籤: {prescription.patient_name}", request)
    
    return {"message": "處方籤已取消"}

# ========== YAML匯出 API ==========

async def auto_sync_yaml():
    """自動同步YAML，靜默執行"""
    try:
        yaml_storage.sync_json_to_yaml()
        yaml_storage.export_yaml_for_ros2()
    except Exception as e:
        print(f"⚠️ 自動YAML同步失敗: {e}")

@app.post("/api/export/yaml/sync")
async def sync_yaml_export():
    """手動觸發YAML同步"""
    try:
        yaml_storage.sync_json_to_yaml()
        basic_path, detailed_path = yaml_storage.export_yaml_for_ros2()
        
        return {
            "message": "✅ YAML同步和匯出完成",
            "basic_yaml": str(basic_path),
            "detailed_yaml": str(detailed_path),
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"YAML匯出失敗: {str(e)}")

# ========== ROS2 API端點 ==========

@app.get("/api/ros2/prescription")
async def ros2_get_prescriptions(db: Session = Depends(get_db)):
    """ROS2格式的處方籤資料"""
    prescriptions = db.query(DBPrescription).filter(DBPrescription.status == "active").all()
    
    return {
        "status": "success",
        "type": "prescription_orders",
        "timestamp": datetime.now().isoformat(),
        "count": len(prescriptions),
        "data": [
            {
                "order_id": p.id,
                "patient_name": p.patient_name,
                "patient_id": p.patient_id,
                "doctor_name": p.doctor_name,
                "diagnosis": p.diagnosis,
                "created_at": p.created_at.isoformat(),
                "medicines": [
                    {
                        "name": pm.medicine.name,
                        "dosage": pm.dosage,
                        "frequency": pm.frequency,
                        "duration": pm.duration,
                        "instructions": pm.instructions,
                        "medicine_prompt": pm.medicine.prompt
                    }
                    for pm in p.medicines
                ]
            }
            for p in prescriptions
        ],
        "ros2_compatible": True
    }

# 啟動事件
@app.on_event("startup")
async def startup_event():
    """應用啟動時初始化資料庫"""
    print("🏥 醫院藥物管理系統 - SQL版本啟動中...")
    init_database()
    print("✅ 資料庫初始化完成")

if __name__ == "__main__":
    print("🏥 醫院藥物管理系統 - SQL資料庫版本")
    print("==================================================")
    print("🌐 網頁界面: http://localhost:8000/Medicine.html")
    print("📋 處方籤管理: http://localhost:8000/Prescription.html")
    print("👨‍⚕️ 醫生界面: http://localhost:8000/doctor.html")
    print("🔄 統一表單: http://localhost:8000/unified_medicine.html")
    print("🎛️ 整合管理: http://localhost:8000/integrated_medicine_management.html")
    print("📖 API文檔: http://localhost:8000/docs")
    print("💾 資料庫: SQLite (./data/hospital_management.db)")
    print("==================================================")
    
    uvicorn.run(
        "sql_server:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
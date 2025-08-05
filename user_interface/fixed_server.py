#!/usr/bin/env python3
"""
Hospital Medicine Management System - Real-time Enhanced Version
"""

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from pydantic import BaseModel
from pathlib import Path
from datetime import datetime
from typing import List, Optional, Dict, Any
import json
import asyncio
import uvicorn
import yaml
from yaml_storage import YAMLMedicineStorage

# Pydantic Models
class MedicineBasic(BaseModel):
    name: str
    amount: int
    usage_days: Optional[int] = None
    position: str
    manufacturer: Optional[str] = ""
    dosage: Optional[str] = ""
    created_time: Optional[str] = None
    updated_time: Optional[str] = None

class MedicineDetailed(BaseModel):
    medicine_name: str
    description: Optional[str] = ""
    side_effects: Optional[str] = ""
    appearance: Optional[Dict[str, str]] = {}
    storage_conditions: Optional[str] = ""
    expiry_date: Optional[str] = ""
    notes: Optional[str] = ""
    ingredient: Optional[str] = ""
    category: Optional[str] = ""
    usage_method: Optional[str] = ""
    unit_dose: Optional[str] = ""
    barcode: Optional[str] = ""
    appearance_type: Optional[str] = ""
    created_time: Optional[str] = None
    updated_time: Optional[str] = None

class MedicineOrder(BaseModel):
    id: str
    order_data: Dict[str, Dict[str, Any]]
    timestamp: Optional[str] = None
    status: Optional[str] = "pending"

class OrderStatus(BaseModel):
    order_id: str
    status: str  # pending, processing, completed, failed
    message: Optional[str] = ""
    timestamp: Optional[str] = None

class Prescription(BaseModel):
    patient_name: str
    patient_id: str
    doctor_name: str
    medicines: List[List[str]]
    created_at: str

# FastAPI App
app = FastAPI(title="醫院藥物管理系統", version="2.0.0")

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

# WebSocket連接管理
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except:
                pass

manager = ConnectionManager()

# YAML儲存實例
yaml_storage = YAMLMedicineStorage()

# JSON文件操作函數
def load_basic_medicines():
    try:
        with open("medicine_basic_data.json", "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return []

def save_basic_medicines(data):
    with open("medicine_basic_data.json", "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_detailed_medicines():
    try:
        with open("medicine_detailed_data.json", "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return []

def save_detailed_medicines(data):
    with open("medicine_detailed_data.json", "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_orders():
    try:
        with open("orders_data.json", "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return []

def save_orders(data):
    with open("orders_data.json", "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_prescriptions():
    try:
        with open("prescription_data.json", "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return []

def save_prescriptions(data):
    with open("prescription_data.json", "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

# 靜態頁面路由
@app.get("/")
async def root():
    return FileResponse("static/html/medicine_integrated.html")

@app.get("/doctor.html")
async def doctor_page():
    return FileResponse("static/html/doctor.html")

@app.get("/medicine_integrated.html")
async def integrated_page():
    return FileResponse("static/html/medicine_integrated.html")

@app.get("/simple_test.html")
async def test_page():
    return FileResponse("static/html/simple_test.html")

@app.get("/Prescription.html")
async def prescription_page():
    return FileResponse("static/html/Prescription.html")

# WebSocket端點 - 實時通知
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            # 處理來自客戶端的消息
            print(f"📨 收到WebSocket消息: {data}")
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# 基本藥物資料API
@app.post("/api/medicine/basic")
async def create_basic_medicine(medicine: MedicineBasic):
    medicines = load_basic_medicines()
    
    # 添加時間戳
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    medicine_dict = medicine.dict()
    
    # 檢查是否已存在
    existing_index = None
    for i, existing in enumerate(medicines):
        if existing["name"] == medicine.name:
            existing_index = i
            break
    
    if existing_index is not None:
        # 更新現有藥物
        medicine_dict["created_time"] = medicines[existing_index].get("created_time", current_time)
        medicine_dict["updated_time"] = current_time
        medicines[existing_index] = medicine_dict
    else:
        # 新增藥物
        medicine_dict["created_time"] = current_time
        medicine_dict["updated_time"] = current_time
        medicines.append(medicine_dict)
    
    save_basic_medicines(medicines)
    
    # 實時通知
    notification = {
        "type": "medicine_basic_updated",
        "medicine": medicine_dict,
        "timestamp": current_time
    }
    await manager.broadcast(json.dumps(notification, ensure_ascii=False))
    
    return {"message": "基本藥物資料已保存", "medicine": medicine_dict}

@app.get("/api/medicine/")
async def get_all_medicines():
    basic_medicines = load_basic_medicines()
    detailed_medicines = load_detailed_medicines()
    return {
        "basic_medicines": basic_medicines,
        "detailed_medicines": detailed_medicines,
        "total_basic": len(basic_medicines),
        "total_detailed": len(detailed_medicines)
    }

@app.get("/api/medicine/basic")
async def get_basic_medicines():
    medicines = load_basic_medicines()
    return {"medicines": medicines}

@app.get("/api/medicine/basic/{medicine_name}")
async def get_basic_medicine(medicine_name: str):
    medicines = load_basic_medicines()
    for medicine in medicines:
        if medicine["name"] == medicine_name:
            return {"medicine": medicine}
    raise HTTPException(status_code=404, detail="藥物未找到")

# 詳細藥物資料API
@app.post("/api/medicine/detailed")
async def create_detailed_medicine(medicine: MedicineDetailed):
    medicines = load_detailed_medicines()
    
    # 添加時間戳
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    medicine_dict = medicine.dict()
    
    # 檢查是否已存在
    existing_index = None
    for i, existing in enumerate(medicines):
        if existing["medicine_name"] == medicine.medicine_name:
            existing_index = i
            break
    
    if existing_index is not None:
        # 更新現有藥物
        medicine_dict["created_time"] = medicines[existing_index].get("created_time", current_time)
        medicine_dict["updated_time"] = current_time
        medicines[existing_index] = medicine_dict
    else:
        # 新增藥物
        medicine_dict["created_time"] = current_time
        medicine_dict["updated_time"] = current_time
        medicines.append(medicine_dict)
    
    save_detailed_medicines(medicines)
    
    # 實時通知
    notification = {
        "type": "medicine_detailed_updated",
        "medicine": medicine_dict,
        "timestamp": current_time
    }
    await manager.broadcast(json.dumps(notification, ensure_ascii=False))
    
    return {"message": "詳細藥物資料已保存", "medicine": medicine_dict}

@app.get("/api/medicine/detailed")
async def get_detailed_medicines():
    medicines = load_detailed_medicines()
    return {"medicines": medicines}

@app.get("/api/medicine/detailed/{medicine_name}")
async def get_detailed_medicine(medicine_name: str):
    medicines = load_detailed_medicines()
    for medicine in medicines:
        if medicine["medicine_name"] == medicine_name:
            return {"medicine": medicine}
    raise HTTPException(status_code=404, detail="詳細藥物資料未找到")

# 整合藥物資料API
@app.get("/api/medicine/integrated/{medicine_name}")
async def get_integrated_medicine(medicine_name: str):
    basic_medicines = load_basic_medicines()
    detailed_medicines = load_detailed_medicines()
    
    basic_data = None
    detailed_data = None
    
    for medicine in basic_medicines:
        if medicine["name"] == medicine_name:
            basic_data = medicine
            break
    
    for medicine in detailed_medicines:
        if medicine["medicine_name"] == medicine_name:
            detailed_data = medicine
            break
    
    if not basic_data and not detailed_data:
        raise HTTPException(status_code=404, detail="藥物資料未找到")
    
    return {
        "medicine_name": medicine_name,
        "basic_data": basic_data,
        "detailed_data": detailed_data
    }

# 訂單管理API
@app.post("/api/orders")
async def create_order(order: MedicineOrder):
    orders = load_orders()
    
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    order_dict = order.dict()
    order_dict["timestamp"] = current_time
    order_dict["status"] = "pending"
    
    orders.append(order_dict)
    save_orders(orders)
    
    # 實時通知新訂單
    notification = {
        "type": "new_order",
        "order": order_dict,
        "timestamp": current_time
    }
    await manager.broadcast(json.dumps(notification, ensure_ascii=False))
    
    return {"message": "訂單已接收", "order": order_dict}

@app.get("/api/orders")
async def get_orders():
    orders = load_orders()
    return {"orders": orders}

@app.get("/api/orders/{order_id}")
async def get_order(order_id: str):
    orders = load_orders()
    for order in orders:
        if order["id"] == order_id:
            return {"order": order}
    raise HTTPException(status_code=404, detail="訂單未找到")

@app.post("/api/orders/{order_id}/status")
async def update_order_status(order_id: str, status: OrderStatus):
    orders = load_orders()
    
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    for i, order in enumerate(orders):
        if order["id"] == order_id:
            orders[i]["status"] = status.status
            orders[i]["status_message"] = status.message
            orders[i]["status_updated_at"] = current_time
            
            save_orders(orders)
            
            # 實時通知狀態更新
            notification = {
                "type": "order_status_updated",
                "order_id": order_id,
                "status": status.status,
                "message": status.message,
                "timestamp": current_time
            }
            await manager.broadcast(json.dumps(notification, ensure_ascii=False))
            
            return {"message": "訂單狀態已更新", "order": orders[i]}
    
    raise HTTPException(status_code=404, detail="訂單未找到")

# ROS2 API端點
@app.get("/api/ros2/medicine/basic")
async def ros2_get_basic_medicines():
    medicines = load_basic_medicines()
    return {
        "status": "success",
        "type": "basic_medicines",
        "timestamp": datetime.now().isoformat(),
        "count": len(medicines),
        "data": medicines,
        "ros2_compatible": True
    }

@app.get("/api/ros2/medicine/detailed")
async def ros2_get_detailed_medicines():
    medicines = load_detailed_medicines()
    return {
        "status": "success",
        "type": "detailed_medicines",
        "timestamp": datetime.now().isoformat(),
        "count": len(medicines),
        "data": medicines,
        "ros2_compatible": True
    }

@app.get("/api/ros2/medicine/integrated/{medicine_name}")
async def ros2_get_integrated_medicine(medicine_name: str):
    basic_medicines = load_basic_medicines()
    detailed_medicines = load_detailed_medicines()
    
    basic_data = next((m for m in basic_medicines if m["name"] == medicine_name), None)
    detailed_data = next((m for m in detailed_medicines if m["medicine_name"] == medicine_name), None)
    
    if not basic_data and not detailed_data:
        return {
            "status": "error",
            "type": "integrated_medicine",
            "timestamp": datetime.now().isoformat(),
            "error": "Medicine not found",
            "ros2_compatible": True
        }
    
    return {
        "status": "success",
        "type": "integrated_medicine",
        "timestamp": datetime.now().isoformat(),
        "medicine_name": medicine_name,
        "basic_data": basic_data,
        "detailed_data": detailed_data,
        "ros2_compatible": True
    }

@app.get("/api/ros2/orders")
async def ros2_get_orders():
    orders = load_orders()
    return {
        "status": "success",
        "type": "orders",
        "timestamp": datetime.now().isoformat(),
        "count": len(orders),
        "data": orders,
        "ros2_compatible": True
    }

@app.post("/api/ros2/orders")
async def ros2_create_order(order: MedicineOrder):
    # 接收來自ROS2的訂單請求
    orders = load_orders()
    
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    order_dict = order.dict()
    order_dict["timestamp"] = current_time
    order_dict["status"] = "received_from_ros2"
    
    orders.append(order_dict)
    save_orders(orders)
    
    # 實時通知 - ROS2訂單
    notification = {
        "type": "ros2_order_request",
        "order": order_dict,
        "timestamp": current_time,
        "message": f"收到來自ROS2的新訂單: {order.id}"
    }
    await manager.broadcast(json.dumps(notification, ensure_ascii=False))
    
    return {
        "status": "success",
        "type": "order_response",
        "timestamp": datetime.now().isoformat(),
        "order_id": order.id,
        "message": "訂單已接收並處理",
        "ros2_compatible": True
    }

# 處方籤API
@app.get("/api/prescription/")
async def get_prescriptions():
    prescriptions = load_prescriptions()
    return {"prescriptions": prescriptions}

@app.post("/api/prescription/")
async def create_prescription(prescription: Prescription):
    prescriptions = load_prescriptions()
    
    prescription_dict = prescription.dict()
    prescription_dict["created_at"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    prescriptions.append(prescription_dict)
    save_prescriptions(prescriptions)
    
    return {"message": "處方籤已保存", "prescription": prescription_dict}

# 統一藥物添加API (基本+詳細)
@app.post("/api/medicine/unified")
async def add_unified_medicine(basic_data: MedicineBasic, detailed_data: Optional[MedicineDetailed] = None):
    try:
        # 添加到JSON檔案
        basic_medicines = load_basic_medicines()
        basic_dict = basic_data.dict()
        basic_dict["created_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        basic_dict["updated_time"] = basic_dict["created_time"]
        
        basic_medicines.append(basic_dict)
        save_basic_medicines(basic_medicines)
        
        detailed_dict = None
        if detailed_data:
            detailed_medicines = load_detailed_medicines()
            detailed_dict = detailed_data.dict()
            detailed_dict["medicine_name"] = basic_data.name
            detailed_dict["created_time"] = basic_dict["created_time"]
            detailed_dict["updated_time"] = basic_dict["created_time"]
            
            detailed_medicines.append(detailed_dict)
            save_detailed_medicines(detailed_medicines)
        
        # 同步到YAML
        yaml_storage.add_medicine_to_yaml(basic_dict, detailed_dict)
        
        # 導出ROS2格式
        yaml_storage.export_yaml_for_ros2()
        
        # 實時通知
        notification = {
            "type": "unified_medicine_added",
            "basic_medicine": basic_dict,
            "detailed_medicine": detailed_dict,
            "timestamp": basic_dict["created_time"]
        }
        await manager.broadcast(json.dumps(notification, ensure_ascii=False))
        
        return {
            "message": "✅ 統一藥物資料已保存",
            "basic_medicine": basic_dict,
            "detailed_medicine": detailed_dict,
            "yaml_exported": True
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"保存失敗: {str(e)}")

# YAML匯出API
@app.get("/api/export/yaml/sync")
async def sync_yaml_export():
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

# 獲取YAML格式的藥物資料
@app.get("/api/medicine/yaml/basic")
async def get_basic_medicines_yaml():
    try:
        medicines = yaml_storage.get_basic_medicines_yaml()
        return {
            "medicines": medicines,
            "total": len(medicines),
            "format": "yaml",
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"獲取YAML資料失敗: {str(e)}")

@app.get("/api/medicine/yaml/detailed")
async def get_detailed_medicines_yaml():
    try:
        medicines = yaml_storage.get_detailed_medicines_yaml()
        return {
            "medicines": medicines,
            "total": len(medicines),
            "format": "yaml",
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"獲取YAML資料失敗: {str(e)}")

@app.get("/api/ros2/prescription")
async def ros2_get_prescriptions():
    prescriptions = load_prescriptions()
    return {
        "status": "success",
        "type": "prescriptions",
        "timestamp": datetime.now().isoformat(),
        "count": len(prescriptions),
        "data": prescriptions,
        "ros2_compatible": True
    }

if __name__ == "__main__":
    print("🌐 醫院藥物管理系統已啟動 (實時版本)")
    print("=" * 60)
    print("📋 主要頁面:")
    print("   🏠 主頁: http://localhost:8000/")
    print("   💊 整合藥物管理: http://localhost:8000/medicine_integrated.html")
    print("   👨‍⚕️ 醫生界面: http://localhost:8000/doctor.html")
    print("   🧪 API測試: http://localhost:8000/simple_test.html")
    print("   📋 處方籤: http://localhost:8000/Prescription.html")
    print()
    print("🔗 API端點:")
    print("   📊 基本藥物: http://localhost:8000/api/medicine/basic")
    print("   📋 詳細藥物: http://localhost:8000/api/medicine/detailed")
    print("   📦 訂單管理: http://localhost:8000/api/orders")
    print("   🤖 ROS2整合: http://localhost:8000/api/ros2/")
    print("   🔄 實時WebSocket: ws://localhost:8000/ws")
    print()
    print("📖 API文檔: http://localhost:8000/docs")
    print("📁 數據存儲:")
    print("   📊 基本資料: medicine_basic_data.json")
    print("   📋 詳細資料: medicine_detailed_data.json")
    print("   📦 訂單資料: orders_data.json")
    print("   💊 處方籤: prescription_data.json")
    print("=" * 60)
    
    uvicorn.run(app, host="0.0.0.0", port=8000)
#!/usr/bin/env python3
"""
Stable Hospital Medicine Management System
Based on previous working versions with complete UI and functionality
"""

from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse
from sqlalchemy import create_engine, Column, Integer, String, Float, DateTime, ForeignKey, Text
from sqlalchemy.orm import declarative_base, sessionmaker, relationship, Session
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
import uvicorn
import logging
import json
import os
import requests
import yaml
from pathlib import Path

# Database setup
DATABASE_URL = "sqlite:///./hospital_medicine_stable.db"
engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# Database Models
class Medicine(Base):
    __tablename__ = "medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(255), nullable=False, index=True)
    amount = Column(Integer, default=0)
    position = Column(String(100), default="A1")
    description = Column(Text, default="")
    category = Column(String(100), default="General")

class Prescription(Base):
    __tablename__ = "prescription"
    
    id = Column(Integer, primary_key=True, index=True)
    patient_name = Column(String(255), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    status = Column(String(50), default="pending")

class PrescriptionMedicine(Base):
    __tablename__ = "prescription_medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    prescription_id = Column(Integer, ForeignKey("prescription.id"), nullable=False)
    medicine_id = Column(Integer, ForeignKey("medicine.id"), nullable=False)
    amount = Column(Integer, nullable=False)

# Pydantic Models
class MedicineCreate(BaseModel):
    name: str
    amount: int = 100
    position: str = "A1"
    description: str = ""
    category: str = "General"

class PrescriptionCreate(BaseModel):
    patient_name: str
    medicines: List[Dict[str, Any]]

# FastAPI App
app = FastAPI(title="Hospital Medicine System", version="1.0.0")

# Logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("hospital")

# Database functions
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def init_database():
    Base.metadata.create_all(bind=engine)
    db = SessionLocal()
    if db.query(Medicine).count() == 0:
        medicines = [
            Medicine(name="Aspirin", amount=100, position="A1", description="Pain reliever", category="Pain Relief"),
            Medicine(name="Vitamin C", amount=50, position="A2", description="Vitamin supplement", category="Vitamins"),
            Medicine(name="Ibuprofen", amount=75, position="B1", description="Anti-inflammatory", category="Pain Relief"),
            Medicine(name="Paracetamol", amount=120, position="B2", description="Fever reducer", category="Pain Relief"),
            Medicine(name="Calcium", amount=60, position="C1", description="Bone health", category="Supplements")
        ]
        for med in medicines:
            db.add(med)
        db.commit()
    db.close()

# API Routes
@app.get("/")
async def root():
    return {"message": "Hospital Medicine System", "status": "running"}

@app.get("/api/system/status")
async def get_system_status():
    return {"status": "running", "ros_mode": "ready", "database": "connected"}

# Medicine endpoints
@app.get("/api/medicine/")
async def get_medicines(db: Session = Depends(get_db)):
    medicines = db.query(Medicine).all()
    return [{"id": m.id, "name": m.name, "amount": m.amount, "position": m.position, 
             "description": m.description, "category": m.category} for m in medicines]

@app.get("/api/medicine/basic")
async def get_basic_medicines(db: Session = Depends(get_db)):
    medicines = db.query(Medicine).all()
    return [{"id": m.id, "name": m.name, "amount": m.amount, "position": m.position} for m in medicines]

@app.get("/api/medicine/detailed")
async def get_detailed_medicines(db: Session = Depends(get_db)):
    medicines = db.query(Medicine).all()
    return [{"id": m.id, "name": m.name, "amount": m.amount, "position": m.position, 
             "description": m.description, "category": m.category} for m in medicines]

@app.post("/api/medicine/")
async def create_medicine(medicine: MedicineCreate, db: Session = Depends(get_db)):
    db_medicine = Medicine(**medicine.dict())
    db.add(db_medicine)
    db.commit()
    db.refresh(db_medicine)
    return {"id": db_medicine.id, "name": db_medicine.name, "message": "Medicine created"}

@app.post("/api/medicine/unified")
async def create_medicine_unified(request: Request, db: Session = Depends(get_db)):
    data = await request.json()
    medicine = MedicineCreate(**data)
    db_medicine = Medicine(**medicine.dict())
    db.add(db_medicine)
    db.commit()
    db.refresh(db_medicine)
    return {"id": db_medicine.id, "name": db_medicine.name, "message": "Medicine created"}

# Prescription endpoints
@app.get("/api/prescription/")
async def get_prescriptions(db: Session = Depends(get_db)):
    prescriptions = db.query(Prescription).all()
    result = []
    for p in prescriptions:
        medicines = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.prescription_id == p.id
        ).all()
        medicine_list = []
        for pm in medicines:
            med = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
            if med:
                medicine_list.append({"id": med.id, "name": med.name, "amount": pm.amount})
        
        result.append({
            "id": p.id,
            "patient_name": p.patient_name,
            "created_at": p.created_at.isoformat() if p.created_at else None,
            "status": p.status,
            "medicines": medicine_list
        })
    return result

@app.post("/api/prescription/")
async def create_prescription(prescription: PrescriptionCreate, db: Session = Depends(get_db)):
    db_prescription = Prescription(patient_name=prescription.patient_name)
    db.add(db_prescription)
    db.commit()
    db.refresh(db_prescription)
    
    for med_data in prescription.medicines:
        pm = PrescriptionMedicine(
            prescription_id=db_prescription.id,
            medicine_id=med_data["medicine_id"],
            amount=med_data["amount"]
        )
        db.add(pm)
        
        # Deduct stock
        medicine = db.query(Medicine).filter(Medicine.id == med_data["medicine_id"]).first()
        if medicine:
            medicine.amount = max(0, medicine.amount - med_data["amount"])
    
    db.commit()
    return {"id": db_prescription.id, "message": "Prescription created"}

@app.put("/api/prescription/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_data: dict, db: Session = Depends(get_db)):
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="Prescription not found")
    
    prescription.status = status_data.get("status", prescription.status)
    db.commit()
    return {"message": "Status updated"}

# ROS2 Adapter endpoints
current_orders = {}

@app.get("/api/order/next")
async def get_next_order(db: Session = Depends(get_db)):
    """ROS2 order pulling endpoint"""
    try:
        # Get pending prescriptions
        prescriptions = await get_prescriptions(db)
        pending = [p for p in prescriptions if p.get('status') == 'pending']
        
        if not pending:
            return JSONResponse(status_code=204, content={})
        
        # Get oldest one
        oldest = min(pending, key=lambda x: x.get('id', 0))
        
        # Convert to order format
        medicine_list = []
        for i, med in enumerate(oldest.get('medicines', [])):
            medicine_list.append({
                "name": med.get('name', 'Unknown'),
                "amount": med.get('amount', 1),
                "locate": [i//5 + 1, i%5 + 1],
                "prompt": "tablet"
            })
        
        order = {
            "order_id": f"{oldest['id']:06d}",
            "prescription_id": oldest['id'],
            "patient_name": oldest.get('patient_name', 'Unknown'),
            "medicine": medicine_list
        }
        
        # Mark as processing
        prescription = db.query(Prescription).filter(Prescription.id == oldest['id']).first()
        if prescription:
            prescription.status = "processing"
            db.commit()
        
        # Track order
        current_orders[order['order_id']] = {'prescription_id': oldest['id'], 'status': 'processing'}
        
        return {"order": order, "yaml": yaml.safe_dump(order, allow_unicode=True)}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/order/progress")
async def report_progress(payload: Dict[str, Any]):
    """ROS2 progress reporting"""
    order_id = payload.get('order_id')
    logger.info(f"Order {order_id} progress: {payload.get('message', '')}")
    return {"status": "received"}

@app.post("/api/order/complete")
async def report_complete(payload: Dict[str, Any], db: Session = Depends(get_db)):
    """ROS2 completion reporting"""
    order_id = payload.get('order_id')
    status = payload.get('status', 'success')
    
    if order_id in current_orders:
        prescription_id = current_orders[order_id]['prescription_id']
        prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
        if prescription:
            prescription.status = 'completed' if status == 'success' else 'failed'
            db.commit()
        del current_orders[order_id]
    
    return {"status": "completed"}

@app.get("/api/order/status")
async def get_order_status():
    """Get current order status"""
    return {"current_orders": current_orders, "total_processing": len(current_orders)}

# Static file serving for original files
if os.path.exists("static"):
    app.mount("/static", StaticFiles(directory="static"), name="static")

# Simple web interface routes
@app.get("/integrated_medicine_management.html", response_class=HTMLResponse)
async def integrated_medicine_page():
    return """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>整合藥物管理系統</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Microsoft JhengHei', 'PingFang TC', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            background-attachment: fixed;
            color: #2c3e50;
            line-height: 1.6;
            min-height: 100vh;
        }

        .container {
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
        }

        .header {
            background: white;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 5px 25px rgba(0, 0, 0, 0.1);
            margin-bottom: 30px;
            text-align: center;
        }

        .header h1 {
            color: #2c3e50;
            font-size: 2.5em;
            margin-bottom: 10px;
        }

        .tabs {
            display: flex;
            background: white;
            border-radius: 12px 12px 0 0;
            box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
            margin-bottom: 0;
        }
        
        .tab {
            flex: 1;
            padding: 18px 24px;
            text-align: center;
            cursor: pointer;
            background: #f8f9fa;
            border: none;
            font-size: 16px;
            font-weight: 600;
            color: #495057;
            transition: all 0.3s ease;
        }
        
        .tab:first-child { border-radius: 12px 0 0 0; }
        .tab:last-child { border-radius: 0 12px 0 0; }
        
        .tab.active {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
            transform: translateY(-2px);
            box-shadow: 0 4px 15px rgba(102, 126, 234, 0.4);
        }
        
        .tab-content {
            background: white;
            padding: 30px;
            border-radius: 0 0 12px 12px;
            box-shadow: 0 5px 25px rgba(0, 0, 0, 0.1);
            min-height: 500px;
        }
        
        .form-group {
            margin-bottom: 20px;
        }
        
        .form-group label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: #2c3e50;
            font-size: 14px;
        }
        
        .form-group input, .form-group select, .form-group textarea {
            width: 100%;
            padding: 12px 15px;
            border: 2px solid #e9ecef;
            border-radius: 8px;
            font-size: 14px;
            transition: all 0.3s ease;
            background: #f8f9fa;
        }
        
        .form-group input:focus, .form-group select:focus, .form-group textarea:focus {
            outline: none;
            border-color: #667eea;
            background: white;
            box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.1);
        }
        
        .btn {
            padding: 12px 24px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-size: 14px;
            font-weight: 600;
            transition: all 0.3s ease;
            display: inline-flex;
            align-items: center;
            gap: 8px;
            text-decoration: none;
            margin: 5px;
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
        }
        
        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(102, 126, 234, 0.4);
        }
        
        .btn-success {
            background: linear-gradient(135deg, #28a745, #20c997);
            color: white;
        }
        
        .btn-success:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(40, 167, 69, 0.4);
        }
        
        .table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 20px;
            background: white;
            border-radius: 8px;
            overflow: hidden;
            box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
        }
        
        .table th {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
            padding: 15px;
            text-align: left;
            font-weight: 600;
            font-size: 14px;
        }
        
        .table td {
            padding: 15px;
            border-bottom: 1px solid #e9ecef;
            color: #495057;
        }
        
        .table tr:hover {
            background: #f8f9fa;
        }

        .grid-2 {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }

        .hidden { display: none !important; }
        .text-center { text-align: center; }
        .mb-3 { margin-bottom: 1rem; }
        .mt-3 { margin-top: 1rem; }

        .alert {
            padding: 15px 20px;
            border-radius: 8px;
            margin: 15px 0;
            font-weight: 500;
        }

        .alert-success {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }

        .alert-error {
            background: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }

        .status-badge {
            padding: 6px 12px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: 600;
            text-transform: uppercase;
        }
        
        .status-high { background: #d4edda; color: #155724; }
        .status-medium { background: #fff3cd; color: #856404; }
        .status-low { background: #f8d7da; color: #721c24; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>整合藥物管理系統</h1>
            <p>醫院藥物庫存與處方管理</p>
        </div>

        <div class="tabs">
            <button class="tab active" onclick="showTab('medicine')" id="medicine-tab">
                藥物管理
            </button>
            <button class="tab" onclick="showTab('stock')" id="stock-tab">
                庫存查詢
            </button>
            <button class="tab" onclick="showTab('system')" id="system-tab">
                系統狀態
            </button>
        </div>

        <!-- 藥物管理 Tab -->
        <div id="medicine-content" class="tab-content">
            <h3>新增藥物</h3>
            <div class="grid-2">
                <div class="form-group">
                    <label for="medicineName">藥物名稱 *</label>
                    <input type="text" id="medicineName" placeholder="請輸入藥物名稱">
                </div>
                <div class="form-group">
                    <label for="medicineAmount">庫存數量 *</label>
                    <input type="number" id="medicineAmount" value="100" min="0">
                </div>
            </div>
            <div class="grid-2">
                <div class="form-group">
                    <label for="medicinePosition">存放位置</label>
                    <input type="text" id="medicinePosition" value="A1" placeholder="例: A1">
                </div>
                <div class="form-group">
                    <label for="medicineCategory">藥物分類</label>
                    <select id="medicineCategory">
                        <option value="General">一般藥物</option>
                        <option value="Pain Relief">止痛藥</option>
                        <option value="Vitamins">維他命</option>
                        <option value="Supplements">營養補充品</option>
                        <option value="Antibiotics">抗生素</option>
                    </select>
                </div>
            </div>
            <div class="form-group">
                <label for="medicineDescription">藥物描述</label>
                <textarea id="medicineDescription" rows="3" placeholder="請輸入藥物用途或說明"></textarea>
            </div>
            <button class="btn btn-primary" onclick="addMedicine()">
                新增藥物
            </button>

            <h3 class="mt-3">現有藥物列表</h3>
            <button class="btn btn-success mb-3" onclick="loadMedicines()">
                重新載入
            </button>
            <table class="table" id="medicineTable">
                <thead>
                    <tr>
                        <th>ID</th>
                        <th>藥物名稱</th>
                        <th>庫存數量</th>
                        <th>存放位置</th>
                        <th>分類</th>
                        <th>描述</th>
                    </tr>
                </thead>
                <tbody id="medicineList">
                    <tr>
                        <td colspan="6" class="text-center">載入中...</td>
                    </tr>
                </tbody>
            </table>
        </div>

        <!-- 庫存查詢 Tab -->
        <div id="stock-content" class="tab-content hidden">
            <h3>庫存查詢</h3>
            <div class="form-group">
                <label for="searchQuery">搜尋藥物</label>
                <input type="text" id="searchQuery" placeholder="輸入藥物名稱或分類進行搜尋">
            </div>
            <button class="btn btn-primary" onclick="searchMedicines()">
                搜尋
            </button>
            <div id="searchResults" class="mt-3">
                <!-- 搜尋結果將顯示在這裡 -->
            </div>
        </div>

        <!-- 系統狀態 Tab -->
        <div id="system-content" class="tab-content hidden">
            <h3>系統狀態</h3>
            <div class="grid-2">
                <div style="background: #f8f9fa; padding: 20px; border-radius: 8px;">
                    <h4>資料庫狀態</h4>
                    <p id="dbStatus">檢查中...</p>
                </div>
                <div style="background: #f8f9fa; padding: 20px; border-radius: 8px;">
                    <h4>ROS2 狀態</h4>
                    <p id="rosStatus">檢查中...</p>
                </div>
            </div>
            <button class="btn btn-success" onclick="checkSystemStatus()">
                重新檢查
            </button>
        </div>
    </div>

    <script>
        let medicines = [];
        
        function showTab(tabName) {
            // Hide all tab contents
            document.querySelectorAll('.tab-content').forEach(content => {
                content.classList.add('hidden');
            });
            
            // Remove active class from all tabs
            document.querySelectorAll('.tab').forEach(tab => {
                tab.classList.remove('active');
            });
            
            // Show selected tab content
            document.getElementById(tabName + '-content').classList.remove('hidden');
            
            // Add active class to selected tab
            document.getElementById(tabName + '-tab').classList.add('active');
            
            // Load content based on tab
            if (tabName === 'medicine') {
                loadMedicines();
            } else if (tabName === 'system') {
                checkSystemStatus();
            }
        }

        async function addMedicine() {
            const name = document.getElementById('medicineName').value;
            const amount = parseInt(document.getElementById('medicineAmount').value) || 0;
            const position = document.getElementById('medicinePosition').value || 'A1';
            const category = document.getElementById('medicineCategory').value;
            const description = document.getElementById('medicineDescription').value;

            if (!name.trim()) {
                showAlert('請輸入藥物名稱', 'error');
                return;
            }

            const data = {
                name: name.trim(),
                amount: amount,
                position: position,
                category: category,
                description: description
            };

            try {
                const response = await fetch('/api/medicine/', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });
                
                if (response.ok) {
                    const result = await response.json();
                    showAlert('藥物新增成功: ' + result.name, 'success');
                    clearMedicineForm();
                    loadMedicines();
                } else {
                    throw new Error('新增失敗');
                }
            } catch (error) {
                showAlert('錯誤: ' + error.message, 'error');
            }
        }

        async function loadMedicines() {
            try {
                const response = await fetch('/api/medicine/');
                medicines = await response.json();
                displayMedicines();
            } catch (error) {
                showAlert('載入藥物資料失敗: ' + error.message, 'error');
            }
        }

        function displayMedicines() {
            const tbody = document.getElementById('medicineList');
            if (medicines.length === 0) {
                tbody.innerHTML = '<tr><td colspan="6" class="text-center">尚無藥物資料</td></tr>';
                return;
            }

            tbody.innerHTML = medicines.map(med => `
                <tr>
                    <td>${med.id}</td>
                    <td><strong>${med.name}</strong></td>
                    <td><span class="status-badge ${med.amount > 50 ? 'status-high' : med.amount > 10 ? 'status-medium' : 'status-low'}">${med.amount}</span></td>
                    <td>${med.position}</td>
                    <td>${med.category}</td>
                    <td>${med.description}</td>
                </tr>
            `).join('');
        }

        async function searchMedicines() {
            const query = document.getElementById('searchQuery').value.trim();
            const resultsDiv = document.getElementById('searchResults');
            
            if (!query) {
                resultsDiv.innerHTML = '<div class="alert alert-error">請輸入搜尋關鍵字</div>';
                return;
            }

            try {
                const response = await fetch('/api/medicine/');
                const allMedicines = await response.json();
                
                const filtered = allMedicines.filter(med => 
                    med.name.toLowerCase().includes(query.toLowerCase()) ||
                    med.category.toLowerCase().includes(query.toLowerCase()) ||
                    med.description.toLowerCase().includes(query.toLowerCase())
                );

                if (filtered.length === 0) {
                    resultsDiv.innerHTML = '<div class="alert alert-error">找不到符合條件的藥物</div>';
                    return;
                }

                resultsDiv.innerHTML = `
                    <h4>搜尋結果 (${filtered.length} 項)</h4>
                    <table class="table">
                        <thead>
                            <tr><th>名稱</th><th>庫存</th><th>位置</th><th>分類</th><th>描述</th></tr>
                        </thead>
                        <tbody>
                            ${filtered.map(med => `
                                <tr>
                                    <td><strong>${med.name}</strong></td>
                                    <td><span class="status-badge ${med.amount > 50 ? 'status-high' : med.amount > 10 ? 'status-medium' : 'status-low'}">${med.amount}</span></td>
                                    <td>${med.position}</td>
                                    <td>${med.category}</td>
                                    <td>${med.description}</td>
                                </tr>
                            `).join('')}
                        </tbody>
                    </table>
                `;
            } catch (error) {
                resultsDiv.innerHTML = '<div class="alert alert-error">搜尋失敗: ' + error.message + '</div>';
            }
        }

        async function checkSystemStatus() {
            try {
                const response = await fetch('/api/system/status');
                const status = await response.json();
                
                document.getElementById('dbStatus').innerHTML = `
                    <span class="status-badge status-high">✓ 正常連線</span>
                    <br><small>資料庫: ${status.database}</small>
                `;
                
                document.getElementById('rosStatus').innerHTML = `
                    <span class="status-badge status-high">✓ ${status.ros_mode}</span>
                    <br><small>狀態: ${status.status}</small>
                `;
            } catch (error) {
                document.getElementById('dbStatus').innerHTML = `<span class="status-badge status-low">✗ 連線失敗</span>`;
                document.getElementById('rosStatus').innerHTML = `<span class="status-badge status-low">✗ 檢查失敗</span>`;
            }
        }

        function clearMedicineForm() {
            document.getElementById('medicineName').value = '';
            document.getElementById('medicineAmount').value = '100';
            document.getElementById('medicinePosition').value = 'A1';
            document.getElementById('medicineCategory').value = 'General';
            document.getElementById('medicineDescription').value = '';
        }

        function showAlert(message, type) {
            // Remove existing alerts
            document.querySelectorAll('.alert').forEach(alert => {
                if (!alert.classList.contains('permanent')) {
                    alert.remove();
                }
            });

            const alertDiv = document.createElement('div');
            alertDiv.className = `alert alert-${type === 'error' ? 'error' : 'success'}`;
            alertDiv.innerHTML = message;
            
            const container = document.querySelector('.tab-content:not(.hidden)');
            container.insertBefore(alertDiv, container.firstChild);
            
            setTimeout(() => alertDiv.remove(), 5000);
        }

        // Initialize page
        document.addEventListener('DOMContentLoaded', function() {
            loadMedicines();
            checkSystemStatus();
        });
    </script>
</body>
</html>
    """

@app.get("/doctor.html", response_class=HTMLResponse)
async def doctor_page():
    return """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>醫生工作台 - 醫院管理系統</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Microsoft JhengHei', 'PingFang TC', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            background-attachment: fixed;
            color: #2c3e50;
            line-height: 1.6;
            min-height: 100vh;
            padding: 20px;
        }

        .container {
            max-width: 1000px;
            margin: 0 auto;
            padding: 20px;
        }

        .header {
            background: white;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 5px 25px rgba(0, 0, 0, 0.1);
            margin-bottom: 30px;
            text-align: center;
        }

        .header h1 {
            color: #2c3e50;
            font-size: 2.5em;
            margin-bottom: 10px;
        }

        .content {
            background: white;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 5px 25px rgba(0, 0, 0, 0.1);
            min-height: 600px;
        }

        .form-group {
            margin-bottom: 20px;
        }
        
        .form-group label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: #2c3e50;
            font-size: 14px;
        }
        
        .form-group input, .form-group select {
            width: 100%;
            padding: 12px 15px;
            border: 2px solid #e9ecef;
            border-radius: 8px;
            font-size: 14px;
            transition: all 0.3s ease;
            background: #f8f9fa;
        }
        
        .form-group input:focus, .form-group select:focus {
            outline: none;
            border-color: #667eea;
            background: white;
            box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.1);
        }

        .btn {
            padding: 12px 24px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-size: 14px;
            font-weight: 600;
            transition: all 0.3s ease;
            display: inline-flex;
            align-items: center;
            gap: 8px;
            text-decoration: none;
            margin: 5px;
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
        }
        
        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(102, 126, 234, 0.4);
        }
        
        .btn-success {
            background: linear-gradient(135deg, #28a745, #20c997);
            color: white;
        }
        
        .btn-success:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(40, 167, 69, 0.4);
        }

        .medicine-card {
            background: #f8f9fa;
            border-radius: 8px;
            padding: 15px;
            margin: 10px 0;
            border: 2px solid #e9ecef;
            transition: all 0.3s ease;
            cursor: pointer;
        }
        
        .medicine-card:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
        }
        
        .medicine-card.selected {
            border-color: #667eea;
            background: linear-gradient(135deg, rgba(102, 126, 234, 0.1), rgba(118, 75, 162, 0.1));
        }

        .medicine-selection {
            max-height: 400px;
            overflow-y: auto;
            border: 1px solid #e9ecef;
            border-radius: 8px;
            padding: 15px;
            background: #f8f9fa;
        }

        .selected-medicines {
            background: #e8f5e8;
            border-radius: 8px;
            padding: 15px;
            margin: 15px 0;
        }

        .alert {
            padding: 15px 20px;
            border-radius: 8px;
            margin: 15px 0;
            font-weight: 500;
        }

        .alert-success {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }

        .alert-error {
            background: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }

        .grid-2 {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }

        .hidden { display: none !important; }
        .text-center { text-align: center; }
        .mb-3 { margin-bottom: 1rem; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>醫生工作台</h1>
            <p>處方籤開立與管理系統</p>
        </div>

        <div class="content">
            <h3>開立新處方籤</h3>
            <div class="form-group">
                <label for="patientName">病患姓名 *</label>
                <input type="text" id="patientName" placeholder="請輸入病患姓名">
            </div>

            <div class="form-group">
                <label>藥物選擇</label>
                <button class="btn btn-success mb-3" onclick="loadAvailableMedicines()">
                    載入可用藥物
                </button>
                <div id="medicineSelection" class="medicine-selection">
                    <p class="text-center">請點擊上方按鈕載入藥物資料</p>
                </div>
            </div>

            <div id="selectedMedicinesDiv" class="selected-medicines hidden">
                <h4>已選擇的藥物</h4>
                <div id="selectedMedicinesList"></div>
            </div>

            <button class="btn btn-primary" onclick="createPrescription()">
                開立處方籤
            </button>
        </div>
    </div>

    <script>
        let availableMedicines = [];
        let selectedMedicines = [];

        async function loadAvailableMedicines() {
            const container = document.getElementById('medicineSelection');
            container.innerHTML = '<p class="text-center">載入中...</p>';
            
            try {
                const response = await fetch('/api/medicine/');
                availableMedicines = await response.json();
                displayMedicineSelection();
            } catch (error) {
                container.innerHTML = '<div class="alert alert-error">載入失敗: ' + error.message + '</div>';
            }
        }

        function displayMedicineSelection() {
            const container = document.getElementById('medicineSelection');
            
            if (availableMedicines.length === 0) {
                container.innerHTML = '<p class="text-center">尚無可用藥物</p>';
                return;
            }

            container.innerHTML = availableMedicines.map(med => `
                <div class="medicine-card" onclick="toggleMedicine(${med.id})" id="med-card-${med.id}">
                    <div style="display: flex; justify-content: space-between; align-items: center;">
                        <div>
                            <h4>${med.name}</h4>
                            <p><strong>庫存:</strong> ${med.amount} | <strong>位置:</strong> ${med.position}</p>
                            <p><strong>分類:</strong> ${med.category}</p>
                            <p><strong>說明:</strong> ${med.description}</p>
                        </div>
                        <div>
                            <input type="number" id="amount-${med.id}" min="1" max="${med.amount}" value="1" 
                                   style="width: 80px; margin-left: 10px;" onclick="event.stopPropagation();">
                            <span style="font-size: 12px; color: #666;">數量</span>
                        </div>
                    </div>
                </div>
            `).join('');
        }

        function toggleMedicine(medId) {
            const card = document.getElementById(`med-card-${medId}`);
            const medicine = availableMedicines.find(m => m.id === medId);
            const amount = parseInt(document.getElementById(`amount-${medId}`).value) || 1;
            
            const existingIndex = selectedMedicines.findIndex(m => m.medicine_id === medId);
            
            if (existingIndex >= 0) {
                // Remove from selection
                selectedMedicines.splice(existingIndex, 1);
                card.classList.remove('selected');
            } else {
                // Add to selection
                selectedMedicines.push({
                    medicine_id: medId,
                    name: medicine.name,
                    amount: amount,
                    max_amount: medicine.amount
                });
                card.classList.add('selected');
            }
            
            updateSelectedMedicinesDisplay();
        }

        function updateSelectedMedicinesDisplay() {
            const container = document.getElementById('selectedMedicinesDiv');
            const list = document.getElementById('selectedMedicinesList');
            
            if (selectedMedicines.length === 0) {
                container.classList.add('hidden');
                return;
            }
            
            container.classList.remove('hidden');
            list.innerHTML = selectedMedicines.map((med, index) => `
                <div style="display: flex; justify-content: space-between; align-items: center; padding: 10px; border: 1px solid #ddd; border-radius: 8px; margin: 5px 0;">
                    <span><strong>${med.name}</strong> - ${med.amount} 顆</span>
                    <button class="btn" style="background: #dc3545; color: white; padding: 5px 10px;" onclick="removeSelectedMedicine(${index})">移除</button>
                </div>
            `).join('');
        }

        function removeSelectedMedicine(index) {
            const med = selectedMedicines[index];
            selectedMedicines.splice(index, 1);
            
            // Update visual selection
            const card = document.getElementById(`med-card-${med.medicine_id}`);
            if (card) {
                card.classList.remove('selected');
            }
            
            updateSelectedMedicinesDisplay();
        }

        async function createPrescription() {
            const patientName = document.getElementById('patientName').value.trim();
            
            if (!patientName) {
                showAlert('請輸入病患姓名', 'error');
                return;
            }
            
            if (selectedMedicines.length === 0) {
                showAlert('請選擇至少一種藥物', 'error');
                return;
            }
            
            // Update amounts from inputs
            selectedMedicines.forEach(med => {
                const input = document.getElementById(`amount-${med.medicine_id}`);
                if (input) {
                    med.amount = parseInt(input.value) || 1;
                }
            });
            
            const data = {
                patient_name: patientName,
                medicines: selectedMedicines.map(med => ({
                    medicine_id: med.medicine_id,
                    amount: med.amount
                }))
            };
            
            try {
                const response = await fetch('/api/prescription/', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });
                
                if (response.ok) {
                    const result = await response.json();
                    showAlert(`處方籤開立成功！處方籤編號: ${result.id}`, 'success');
                    clearPrescriptionForm();
                } else {
                    throw new Error('處方籤開立失敗');
                }
            } catch (error) {
                showAlert('錯誤: ' + error.message, 'error');
            }
        }

        function clearPrescriptionForm() {
            document.getElementById('patientName').value = '';
            selectedMedicines = [];
            document.querySelectorAll('.medicine-card.selected').forEach(card => {
                card.classList.remove('selected');
            });
            updateSelectedMedicinesDisplay();
        }

        function showAlert(message, type) {
            // Remove existing alerts
            document.querySelectorAll('.alert').forEach(alert => {
                if (!alert.querySelector('h4')) {
                    alert.remove();
                }
            });

            const alertDiv = document.createElement('div');
            alertDiv.className = `alert alert-${type === 'error' ? 'error' : 'success'}`;
            alertDiv.innerHTML = message;
            
            const content = document.querySelector('.content');
            content.insertBefore(alertDiv, content.firstChild);
            
            setTimeout(() => alertDiv.remove(), 5000);
        }

        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            loadAvailableMedicines();
        });
    </script>
</body>
</html>
    """

@app.get("/Prescription.html", response_class=HTMLResponse)
async def prescription_management_page():
    return """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>處方籤管理 - 醫院管理系統</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Microsoft JhengHei', 'PingFang TC', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            background-attachment: fixed;
            color: #2c3e50;
            line-height: 1.6;
            min-height: 100vh;
            padding: 20px;
        }

        .container {
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
        }

        .header {
            background: white;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 5px 25px rgba(0, 0, 0, 0.1);
            margin-bottom: 30px;
            text-align: center;
        }

        .header h1 {
            color: #2c3e50;
            font-size: 2.5em;
            margin-bottom: 10px;
        }

        .content {
            background: white;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 5px 25px rgba(0, 0, 0, 0.1);
            min-height: 600px;
        }

        .btn {
            padding: 12px 24px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-size: 14px;
            font-weight: 600;
            transition: all 0.3s ease;
            display: inline-flex;
            align-items: center;
            gap: 8px;
            text-decoration: none;
            margin: 5px;
        }
        
        .btn-success {
            background: linear-gradient(135deg, #28a745, #20c997);
            color: white;
        }
        
        .btn-success:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(40, 167, 69, 0.4);
        }

        .btn-primary {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
        }
        
        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(102, 126, 234, 0.4);
        }

        .btn-warning {
            background: linear-gradient(135deg, #ffc107, #ff8f00);
            color: white;
        }

        .table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 20px;
            background: white;
            border-radius: 8px;
            overflow: hidden;
            box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
        }
        
        .table th {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
            padding: 15px;
            text-align: left;
            font-weight: 600;
            font-size: 14px;
        }
        
        .table td {
            padding: 15px;
            border-bottom: 1px solid #e9ecef;
            color: #495057;
            vertical-align: top;
        }
        
        .table tr:hover {
            background: #f8f9fa;
        }

        .status-badge {
            padding: 6px 12px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: 600;
            text-transform: uppercase;
            display: inline-block;
        }
        
        .status-pending {
            background: #fff3cd;
            color: #856404;
        }
        
        .status-processing {
            background: #d1ecf1;
            color: #0c5460;
        }
        
        .status-completed {
            background: #d4edda;
            color: #155724;
        }
        
        .status-failed {
            background: #f8d7da;
            color: #721c24;
        }

        .medicine-list {
            font-size: 13px;
            color: #666;
        }

        .medicine-item {
            display: inline-block;
            background: #f8f9fa;
            padding: 4px 8px;
            margin: 2px;
            border-radius: 4px;
            border: 1px solid #e9ecef;
        }

        .text-center { text-align: center; }
        .mb-3 { margin-bottom: 1rem; }

        .alert {
            padding: 15px 20px;
            border-radius: 8px;
            margin: 15px 0;
            font-weight: 500;
        }

        .alert-success {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }

        .alert-error {
            background: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }

        .stats-container {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }

        .stat-card {
            background: #f8f9fa;
            padding: 20px;
            border-radius: 8px;
            text-align: center;
            box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
        }

        .stat-number {
            font-size: 2em;
            font-weight: bold;
            color: #667eea;
        }

        .stat-label {
            color: #666;
            font-size: 0.9em;
            text-transform: uppercase;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>處方籤管理系統</h1>
            <p>監控和管理所有處方籤狀態</p>
        </div>

        <div class="content">
            <!-- 統計卡片 -->
            <div class="stats-container">
                <div class="stat-card">
                    <div class="stat-number" id="pendingCount">-</div>
                    <div class="stat-label">待處理</div>
                </div>
                <div class="stat-card">
                    <div class="stat-number" id="processingCount">-</div>
                    <div class="stat-label">處理中</div>
                </div>
                <div class="stat-card">
                    <div class="stat-number" id="completedCount">-</div>
                    <div class="stat-label">已完成</div>
                </div>
                <div class="stat-card">
                    <div class="stat-number" id="totalCount">-</div>
                    <div class="stat-label">總計</div>
                </div>
            </div>

            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px;">
                <h3>處方籤列表</h3>
                <div>
                    <button class="btn btn-success" onclick="loadPrescriptions()">
                        重新載入
                    </button>
                    <button class="btn btn-primary" onclick="toggleAutoRefresh()">
                        <span id="autoRefreshIcon">⏸️</span> <span id="autoRefreshText">停止自動更新</span>
                    </button>
                </div>
            </div>

            <table class="table">
                <thead>
                    <tr>
                        <th>處方籤編號</th>
                        <th>病患姓名</th>
                        <th>建立時間</th>
                        <th>狀態</th>
                        <th>藥物清單</th>
                        <th>操作</th>
                    </tr>
                </thead>
                <tbody id="prescriptionList">
                    <tr>
                        <td colspan="6" class="text-center">載入中...</td>
                    </tr>
                </tbody>
            </table>
        </div>
    </div>

    <script>
        let autoRefreshTimer = null;
        let isAutoRefreshing = true;

        async function loadPrescriptions() {
            const tbody = document.getElementById('prescriptionList');
            
            try {
                const response = await fetch('/api/prescription/');
                const prescriptions = await response.json();
                
                updateStats(prescriptions);
                
                if (prescriptions.length === 0) {
                    tbody.innerHTML = '<tr><td colspan="6" class="text-center">尚無處方籤資料</td></tr>';
                    return;
                }

                tbody.innerHTML = prescriptions.map(p => {
                    const createdDate = new Date(p.created_at);
                    const medicineList = p.medicines.map(m => 
                        `<span class="medicine-item">${m.name} (${m.amount})</span>`
                    ).join(' ');
                    
                    let actions = '';
                    if (p.status === 'pending') {
                        actions = `<button class="btn btn-primary" onclick="updateStatus(${p.id}, 'processing')">開始處理</button>`;
                    } else if (p.status === 'processing') {
                        actions = `<button class="btn btn-success" onclick="updateStatus(${p.id}, 'completed')">標記完成</button>`;
                    }
                    
                    return `
                        <tr>
                            <td><strong>#${p.id.toString().padStart(6, '0')}</strong></td>
                            <td>${p.patient_name}</td>
                            <td>${createdDate.toLocaleString('zh-TW')}</td>
                            <td><span class="status-badge status-${p.status}">${getStatusText(p.status)}</span></td>
                            <td class="medicine-list">${medicineList}</td>
                            <td>${actions}</td>
                        </tr>
                    `;
                }).join('');
                
            } catch (error) {
                tbody.innerHTML = `<tr><td colspan="6" class="text-center"><div class="alert alert-error">載入失敗: ${error.message}</div></td></tr>`;
            }
        }

        function updateStats(prescriptions) {
            const stats = {
                pending: 0,
                processing: 0,
                completed: 0,
                failed: 0,
                total: prescriptions.length
            };
            
            prescriptions.forEach(p => {
                if (stats.hasOwnProperty(p.status)) {
                    stats[p.status]++;
                }
            });
            
            document.getElementById('pendingCount').textContent = stats.pending;
            document.getElementById('processingCount').textContent = stats.processing;
            document.getElementById('completedCount').textContent = stats.completed;
            document.getElementById('totalCount').textContent = stats.total;
        }

        function getStatusText(status) {
            const statusMap = {
                'pending': '待處理',
                'processing': '處理中',
                'completed': '已完成',
                'failed': '失敗'
            };
            return statusMap[status] || status;
        }

        async function updateStatus(id, status) {
            try {
                const response = await fetch(`/api/prescription/${id}/status`, {
                    method: 'PUT',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({status: status})
                });
                
                if (response.ok) {
                    showAlert(`處方籤 #${id.toString().padStart(6, '0')} 狀態已更新為: ${getStatusText(status)}`, 'success');
                    loadPrescriptions();
                } else {
                    throw new Error('狀態更新失敗');
                }
            } catch (error) {
                showAlert('錯誤: ' + error.message, 'error');
            }
        }

        function toggleAutoRefresh() {
            if (isAutoRefreshing) {
                clearInterval(autoRefreshTimer);
                document.getElementById('autoRefreshIcon').textContent = '▶️';
                document.getElementById('autoRefreshText').textContent = '開始自動更新';
                isAutoRefreshing = false;
            } else {
                startAutoRefresh();
                document.getElementById('autoRefreshIcon').textContent = '⏸️';
                document.getElementById('autoRefreshText').textContent = '停止自動更新';
                isAutoRefreshing = true;
            }
        }

        function startAutoRefresh() {
            autoRefreshTimer = setInterval(loadPrescriptions, 5000);
        }

        function showAlert(message, type) {
            // Remove existing alerts
            document.querySelectorAll('.alert').forEach(alert => {
                if (!alert.classList.contains('permanent')) {
                    alert.remove();
                }
            });

            const alertDiv = document.createElement('div');
            alertDiv.className = `alert alert-${type === 'error' ? 'error' : 'success'}`;
            alertDiv.innerHTML = message;
            
            const content = document.querySelector('.content');
            content.insertBefore(alertDiv, content.firstChild);
            
            setTimeout(() => alertDiv.remove(), 5000);
        }

        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            loadPrescriptions();
            startAutoRefresh();
        });
    </script>
</body>
</html>
    """

if __name__ == "__main__":
    print("Starting Stable Hospital Medicine Management System...")
    init_database()
    
    print("\nSystem ready!")
    print("Web Interfaces:")
    print("- Medicine Management: http://localhost:8001/integrated_medicine_management.html")
    print("- Doctor Interface: http://localhost:8001/doctor.html")
    print("- Prescription Management: http://localhost:8001/Prescription.html")
    print("\nROS2 Endpoints:")
    print("- Get Order: GET http://localhost:8001/api/order/next")
    print("- Report Progress: POST http://localhost:8001/api/order/progress")
    print("- Report Complete: POST http://localhost:8001/api/order/complete")
    
    uvicorn.run(app, host="0.0.0.0", port=8001)
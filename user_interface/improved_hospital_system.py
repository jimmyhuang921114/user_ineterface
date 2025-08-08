#!/usr/bin/env python3
"""
Improved Hospital Medicine Management System
Enhanced layout with left sidebar navigation connecting three HTML pages
"""

from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, JSONResponse
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
DATABASE_URL = "sqlite:///./hospital_medicine_improved.db"
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

# Common CSS for all pages
COMMON_CSS = """
* {
    margin: 0;
    padding: 0;
    box-sizing: border-box;
}

body {
    font-family: 'Microsoft JhengHei', 'PingFang TC', 'Segoe UI', Arial, sans-serif;
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    background-attachment: fixed;
    color: #2c3e50;
    line-height: 1.6;
    min-height: 100vh;
}

/* Left Sidebar Navigation */
.sidebar {
    position: fixed;
    left: 0;
    top: 0;
    width: 280px;
    height: 100vh;
    background: rgba(44, 62, 80, 0.95);
    backdrop-filter: blur(10px);
    z-index: 1000;
    padding: 20px;
    box-shadow: 4px 0 20px rgba(0, 0, 0, 0.3);
    border-right: 2px solid rgba(255, 255, 255, 0.1);
}

.sidebar .logo {
    text-align: center;
    margin-bottom: 40px;
    padding-bottom: 20px;
    border-bottom: 2px solid rgba(255, 255, 255, 0.1);
}

.sidebar .logo h2 {
    color: #ecf0f1;
    font-size: 1.5em;
    font-weight: 700;
    letter-spacing: 1px;
    margin-bottom: 5px;
}

.sidebar .logo p {
    color: #bdc3c7;
    font-size: 0.9em;
}

.nav-button {
    display: block;
    width: 100%;
    padding: 15px 20px;
    margin: 10px 0;
    background: linear-gradient(45deg, #3498db, #2980b9);
    color: white;
    text-decoration: none;
    border: none;
    border-radius: 12px;
    cursor: pointer;
    font-size: 15px;
    font-weight: 600;
    transition: all 0.3s ease;
    text-align: left;
    display: flex;
    align-items: center;
    gap: 12px;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

.nav-button:hover {
    background: linear-gradient(45deg, #2980b9, #1f4e79);
    transform: translateX(8px);
    box-shadow: 0 8px 25px rgba(52, 152, 219, 0.4);
}

.nav-button.active {
    background: linear-gradient(45deg, #e74c3c, #c0392b);
    transform: translateX(8px);
    box-shadow: 0 8px 25px rgba(231, 76, 60, 0.4);
}

.nav-button .icon {
    font-size: 18px;
    width: 24px;
    text-align: center;
}

/* Main Content Area */
.main-content {
    margin-left: 280px;
    padding: 30px;
    min-height: 100vh;
}

.page-header {
    background: white;
    padding: 40px;
    border-radius: 16px;
    box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
    margin-bottom: 30px;
    text-align: center;
    border: 1px solid rgba(255, 255, 255, 0.2);
}

.page-header h1 {
    color: #2c3e50;
    font-size: 2.8em;
    margin-bottom: 10px;
    font-weight: 700;
}

.page-header p {
    color: #7f8c8d;
    font-size: 1.2em;
    font-weight: 400;
}

.content-card {
    background: white;
    padding: 40px;
    border-radius: 16px;
    box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
    border: 1px solid rgba(255, 255, 255, 0.2);
    margin-bottom: 30px;
}

/* Form Styling */
.form-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
    gap: 25px;
    margin-bottom: 30px;
}

.form-group {
    margin-bottom: 25px;
}

.form-group label {
    display: block;
    margin-bottom: 8px;
    font-weight: 600;
    color: #2c3e50;
    font-size: 15px;
}

.form-group input, 
.form-group select, 
.form-group textarea {
    width: 100%;
    padding: 15px 20px;
    border: 2px solid #e9ecef;
    border-radius: 12px;
    font-size: 15px;
    transition: all 0.3s ease;
    background: #f8f9fa;
    font-family: inherit;
}

.form-group input:focus, 
.form-group select:focus, 
.form-group textarea:focus {
    outline: none;
    border-color: #667eea;
    background: white;
    box-shadow: 0 0 0 4px rgba(102, 126, 234, 0.1);
    transform: translateY(-1px);
}

/* Button Styling */
.btn {
    padding: 15px 30px;
    border: none;
    border-radius: 12px;
    cursor: pointer;
    font-size: 15px;
    font-weight: 600;
    transition: all 0.3s ease;
    display: inline-flex;
    align-items: center;
    gap: 10px;
    text-decoration: none;
    margin: 8px;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

.btn-primary {
    background: linear-gradient(135deg, #667eea, #764ba2);
    color: white;
}

.btn-primary:hover {
    transform: translateY(-3px);
    box-shadow: 0 12px 35px rgba(102, 126, 234, 0.4);
}

.btn-success {
    background: linear-gradient(135deg, #28a745, #20c997);
    color: white;
}

.btn-success:hover {
    transform: translateY(-3px);
    box-shadow: 0 12px 35px rgba(40, 167, 69, 0.4);
}

.btn-warning {
    background: linear-gradient(135deg, #ffc107, #ff8f00);
    color: white;
}

.btn-warning:hover {
    transform: translateY(-3px);
    box-shadow: 0 12px 35px rgba(255, 193, 7, 0.4);
}

/* Table Styling */
.data-table {
    width: 100%;
    border-collapse: collapse;
    margin-top: 25px;
    background: white;
    border-radius: 12px;
    overflow: hidden;
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
}

.data-table th {
    background: linear-gradient(135deg, #667eea, #764ba2);
    color: white;
    padding: 20px;
    text-align: left;
    font-weight: 600;
    font-size: 14px;
    text-transform: uppercase;
    letter-spacing: 1px;
}

.data-table td {
    padding: 20px;
    border-bottom: 1px solid #e9ecef;
    color: #495057;
    font-size: 14px;
}

.data-table tr:hover {
    background: #f8f9fa;
    transform: scale(1.01);
    transition: all 0.2s ease;
}

/* Status Badges */
.status-badge {
    padding: 8px 16px;
    border-radius: 20px;
    font-size: 12px;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    display: inline-block;
}

.status-high { background: #d4edda; color: #155724; }
.status-medium { background: #fff3cd; color: #856404; }
.status-low { background: #f8d7da; color: #721c24; }

.status-pending { background: #fff3cd; color: #856404; }
.status-processing { background: #d1ecf1; color: #0c5460; }
.status-completed { background: #d4edda; color: #155724; }
.status-failed { background: #f8d7da; color: #721c24; }

/* Card Components */
.medicine-card {
    background: white;
    border-radius: 12px;
    padding: 25px;
    margin: 15px 0;
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
    border: 2px solid #e9ecef;
    transition: all 0.3s ease;
    cursor: pointer;
}

.medicine-card:hover {
    transform: translateY(-5px);
    box-shadow: 0 12px 35px rgba(0, 0, 0, 0.15);
    border-color: #667eea;
}

.medicine-card.selected {
    border-color: #667eea;
    background: linear-gradient(135deg, rgba(102, 126, 234, 0.1), rgba(118, 75, 162, 0.1));
}

.stats-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    gap: 25px;
    margin-bottom: 30px;
}

.stat-card {
    background: white;
    padding: 30px;
    border-radius: 12px;
    text-align: center;
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
    border: 1px solid rgba(255, 255, 255, 0.2);
    transition: transform 0.3s ease;
}

.stat-card:hover {
    transform: translateY(-5px);
}

.stat-number {
    font-size: 2.5em;
    font-weight: bold;
    color: #667eea;
    margin-bottom: 10px;
}

.stat-label {
    color: #666;
    font-size: 1em;
    text-transform: uppercase;
    letter-spacing: 1px;
    font-weight: 600;
}

/* Utility Classes */
.text-center { text-align: center; }
.mb-3 { margin-bottom: 1.5rem; }
.mt-3 { margin-top: 1.5rem; }
.hidden { display: none !important; }

/* Alert Messages */
.alert {
    padding: 20px 25px;
    border-radius: 12px;
    margin: 20px 0;
    font-weight: 500;
    border-left: 4px solid;
}

.alert-success {
    background: #d4edda;
    color: #155724;
    border-color: #28a745;
}

.alert-error {
    background: #f8d7da;
    color: #721c24;
    border-color: #dc3545;
}

/* Responsive Design */
@media (max-width: 768px) {
    .sidebar {
        width: 250px;
    }
    
    .main-content {
        margin-left: 250px;
        padding: 20px;
    }
    
    .form-grid {
        grid-template-columns: 1fr;
    }
    
    .stats-grid {
        grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
    }
}

/* Loading Animation */
.loading {
    display: inline-block;
    width: 25px;
    height: 25px;
    border: 3px solid #f3f3f3;
    border-top: 3px solid #667eea;
    border-radius: 50%;
    animation: spin 1s linear infinite;
}

@keyframes spin {
    0% { transform: rotate(0deg); }
    100% { transform: rotate(360deg); }
}
"""

# Common JavaScript for navigation
COMMON_JS = """
// Navigation functionality
function navigateToPage(page) {
    window.location.href = page;
}

// Utility functions
function showAlert(message, type = 'success') {
    // Remove existing alerts
    document.querySelectorAll('.alert').forEach(alert => {
        if (!alert.classList.contains('permanent')) {
            alert.remove();
        }
    });

    const alertDiv = document.createElement('div');
    alertDiv.className = `alert alert-${type === 'error' ? 'error' : 'success'}`;
    alertDiv.innerHTML = message;
    
    const container = document.querySelector('.content-card');
    if (container) {
        container.insertBefore(alertDiv, container.firstChild);
        setTimeout(() => alertDiv.remove(), 5000);
    }
}

// Set active navigation button
function setActiveNav(currentPage) {
    document.querySelectorAll('.nav-button').forEach(btn => {
        btn.classList.remove('active');
    });
    
    const activeBtn = document.querySelector(`[href="${currentPage}"]`);
    if (activeBtn) {
        activeBtn.classList.add('active');
    }
}
"""

# API Routes (same as stable version)
@app.get("/")
async def root():
    return {"message": "Hospital Medicine System", "status": "running"}

@app.get("/api/system/status")
async def get_system_status():
    return {"status": "running", "ros_mode": "ready", "database": "connected"}

@app.get("/api/medicine/")
async def get_medicines(db: Session = Depends(get_db)):
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

# ROS2 endpoints
current_orders = {}

@app.get("/api/order/next")
async def get_next_order(db: Session = Depends(get_db)):
    try:
        prescriptions = await get_prescriptions(db)
        pending = [p for p in prescriptions if p.get('status') == 'pending']
        
        if not pending:
            return JSONResponse(status_code=204, content={})
        
        oldest = min(pending, key=lambda x: x.get('id', 0))
        
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
        
        prescription = db.query(Prescription).filter(Prescription.id == oldest['id']).first()
        if prescription:
            prescription.status = "processing"
            db.commit()
        
        current_orders[order['order_id']] = {'prescription_id': oldest['id'], 'status': 'processing'}
        
        return {"order": order, "yaml": yaml.safe_dump(order, allow_unicode=True)}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/order/progress")
async def report_progress(payload: Dict[str, Any]):
    order_id = payload.get('order_id')
    logger.info(f"Order {order_id} progress: {payload.get('message', '')}")
    return {"status": "received"}

@app.post("/api/order/complete")
async def report_complete(payload: Dict[str, Any], db: Session = Depends(get_db)):
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

# HTML Pages with improved layout
@app.get("/medicine", response_class=HTMLResponse)
@app.get("/medicine.html", response_class=HTMLResponse)
async def medicine_management_page():
    return f"""
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>藥物管理 - 醫院管理系統</title>
    <style>{COMMON_CSS}</style>
</head>
<body>
    <!-- Left Sidebar Navigation -->
    <div class="sidebar">
        <div class="logo">
            <h2>醫院管理系統</h2>
            <p>Hospital Management System</p>
        </div>
        
        <a href="/medicine.html" class="nav-button active">
            <span class="icon">💊</span>
            <span>藥物管理</span>
        </a>
        
        <a href="/doctor.html" class="nav-button">
            <span class="icon">👨‍⚕️</span>
            <span>醫生工作台</span>
        </a>
        
        <a href="/prescription.html" class="nav-button">
            <span class="icon">📋</span>
            <span>處方籤管理</span>
        </a>
    </div>

    <!-- Main Content -->
    <div class="main-content">
        <div class="page-header">
            <h1>藥物管理系統</h1>
            <p>管理醫院藥物庫存與資訊</p>
        </div>

        <div class="content-card">
            <h2>新增藥物</h2>
            <div class="form-grid">
                <div class="form-group">
                    <label for="medicineName">藥物名稱 *</label>
                    <input type="text" id="medicineName" placeholder="請輸入藥物名稱">
                </div>
                <div class="form-group">
                    <label for="medicineAmount">庫存數量 *</label>
                    <input type="number" id="medicineAmount" value="100" min="0">
                </div>
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
                <span>➕</span> 新增藥物
            </button>
        </div>

        <div class="content-card">
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px;">
                <h2>藥物列表</h2>
                <button class="btn btn-success" onclick="loadMedicines()">
                    <span>🔄</span> 重新載入
                </button>
            </div>
            
            <table class="data-table">
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
                        <td colspan="6" class="text-center">
                            <div class="loading"></div> 載入中...
                        </td>
                    </tr>
                </tbody>
            </table>
        </div>
    </div>

    <script>
        {COMMON_JS}
        
        let medicines = [];
        
        async function addMedicine() {{
            const name = document.getElementById('medicineName').value;
            const amount = parseInt(document.getElementById('medicineAmount').value) || 0;
            const position = document.getElementById('medicinePosition').value || 'A1';
            const category = document.getElementById('medicineCategory').value;
            const description = document.getElementById('medicineDescription').value;

            if (!name.trim()) {{
                showAlert('請輸入藥物名稱', 'error');
                return;
            }}

            const data = {{
                name: name.trim(),
                amount: amount,
                position: position,
                category: category,
                description: description
            }};

            try {{
                const response = await fetch('/api/medicine/', {{
                    method: 'POST',
                    headers: {{'Content-Type': 'application/json'}},
                    body: JSON.stringify(data)
                }});
                
                if (response.ok) {{
                    const result = await response.json();
                    showAlert('藥物新增成功: ' + result.name, 'success');
                    clearMedicineForm();
                    loadMedicines();
                }} else {{
                    throw new Error('新增失敗');
                }}
            }} catch (error) {{
                showAlert('錯誤: ' + error.message, 'error');
            }}
        }}

        async function loadMedicines() {{
            try {{
                const response = await fetch('/api/medicine/');
                medicines = await response.json();
                displayMedicines();
            }} catch (error) {{
                showAlert('載入藥物資料失敗: ' + error.message, 'error');
            }}
        }}

        function displayMedicines() {{
            const tbody = document.getElementById('medicineList');
            if (medicines.length === 0) {{
                tbody.innerHTML = '<tr><td colspan="6" class="text-center">尚無藥物資料</td></tr>';
                return;
            }}

            tbody.innerHTML = medicines.map(med => `
                <tr>
                    <td>${{med.id}}</td>
                    <td><strong>${{med.name}}</strong></td>
                    <td><span class="status-badge ${{med.amount > 50 ? 'status-high' : med.amount > 10 ? 'status-medium' : 'status-low'}}">${{med.amount}}</span></td>
                    <td>${{med.position}}</td>
                    <td>${{med.category}}</td>
                    <td>${{med.description}}</td>
                </tr>
            `).join('');
        }}

        function clearMedicineForm() {{
            document.getElementById('medicineName').value = '';
            document.getElementById('medicineAmount').value = '100';
            document.getElementById('medicinePosition').value = 'A1';
            document.getElementById('medicineCategory').value = 'General';
            document.getElementById('medicineDescription').value = '';
        }}

        // Initialize page
        document.addEventListener('DOMContentLoaded', function() {{
            setActiveNav('/medicine.html');
            loadMedicines();
        }});
    </script>
</body>
</html>
    """

@app.get("/doctor", response_class=HTMLResponse)
@app.get("/doctor.html", response_class=HTMLResponse)
async def doctor_page():
    return f"""
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>醫生工作台 - 醫院管理系統</title>
    <style>{COMMON_CSS}</style>
</head>
<body>
    <!-- Left Sidebar Navigation -->
    <div class="sidebar">
        <div class="logo">
            <h2>醫院管理系統</h2>
            <p>Hospital Management System</p>
        </div>
        
        <a href="/medicine.html" class="nav-button">
            <span class="icon">💊</span>
            <span>藥物管理</span>
        </a>
        
        <a href="/doctor.html" class="nav-button active">
            <span class="icon">👨‍⚕️</span>
            <span>醫生工作台</span>
        </a>
        
        <a href="/prescription.html" class="nav-button">
            <span class="icon">📋</span>
            <span>處方籤管理</span>
        </a>
    </div>

    <!-- Main Content -->
    <div class="main-content">
        <div class="page-header">
            <h1>醫生工作台</h1>
            <p>處方籤開立與藥物選擇</p>
        </div>

        <div class="content-card">
            <h2>開立新處方籤</h2>
            <div class="form-group">
                <label for="patientName">病患姓名 *</label>
                <input type="text" id="patientName" placeholder="請輸入病患姓名">
            </div>

            <div class="form-group">
                <label>藥物選擇</label>
                <button class="btn btn-success mb-3" onclick="loadAvailableMedicines()">
                    <span>🔄</span> 載入可用藥物
                </button>
                <div id="medicineSelection" style="max-height: 400px; overflow-y: auto; border: 1px solid #e9ecef; border-radius: 12px; padding: 20px; background: #f8f9fa;">
                    <p class="text-center">請點擊上方按鈕載入藥物資料</p>
                </div>
            </div>

            <div id="selectedMedicinesDiv" class="hidden" style="background: #e8f5e8; border-radius: 12px; padding: 20px; margin: 20px 0;">
                <h3>已選擇的藥物</h3>
                <div id="selectedMedicinesList"></div>
            </div>

            <button class="btn btn-primary" onclick="createPrescription()">
                <span>📝</span> 開立處方籤
            </button>
        </div>
    </div>

    <script>
        {COMMON_JS}
        
        let availableMedicines = [];
        let selectedMedicines = [];

        async function loadAvailableMedicines() {{
            const container = document.getElementById('medicineSelection');
            container.innerHTML = '<p class="text-center"><span class="loading"></span> 載入中...</p>';
            
            try {{
                const response = await fetch('/api/medicine/');
                availableMedicines = await response.json();
                displayMedicineSelection();
            }} catch (error) {{
                container.innerHTML = '<div class="alert alert-error">載入失敗: ' + error.message + '</div>';
            }}
        }}

        function displayMedicineSelection() {{
            const container = document.getElementById('medicineSelection');
            
            if (availableMedicines.length === 0) {{
                container.innerHTML = '<p class="text-center">尚無可用藥物</p>';
                return;
            }}

            container.innerHTML = availableMedicines.map(med => `
                <div class="medicine-card" onclick="toggleMedicine(${{med.id}})" id="med-card-${{med.id}}">
                    <div style="display: flex; justify-content: space-between; align-items: center;">
                        <div>
                            <h4>${{med.name}}</h4>
                            <p><strong>庫存:</strong> ${{med.amount}} | <strong>位置:</strong> ${{med.position}}</p>
                            <p><strong>分類:</strong> ${{med.category}}</p>
                            <p><strong>說明:</strong> ${{med.description}}</p>
                        </div>
                        <div>
                            <input type="number" id="amount-${{med.id}}" min="1" max="${{med.amount}}" value="1" 
                                   style="width: 80px; padding: 8px; border: 2px solid #e9ecef; border-radius: 8px;" onclick="event.stopPropagation();">
                            <span style="font-size: 12px; color: #666; display: block; text-align: center; margin-top: 5px;">數量</span>
                        </div>
                    </div>
                </div>
            `).join('');
        }}

        function toggleMedicine(medId) {{
            const card = document.getElementById(`med-card-${{medId}}`);
            const medicine = availableMedicines.find(m => m.id === medId);
            const amount = parseInt(document.getElementById(`amount-${{medId}}`).value) || 1;
            
            const existingIndex = selectedMedicines.findIndex(m => m.medicine_id === medId);
            
            if (existingIndex >= 0) {{
                selectedMedicines.splice(existingIndex, 1);
                card.classList.remove('selected');
            }} else {{
                selectedMedicines.push({{
                    medicine_id: medId,
                    name: medicine.name,
                    amount: amount,
                    max_amount: medicine.amount
                }});
                card.classList.add('selected');
            }}
            
            updateSelectedMedicinesDisplay();
        }}

        function updateSelectedMedicinesDisplay() {{
            const container = document.getElementById('selectedMedicinesDiv');
            const list = document.getElementById('selectedMedicinesList');
            
            if (selectedMedicines.length === 0) {{
                container.classList.add('hidden');
                return;
            }}
            
            container.classList.remove('hidden');
            list.innerHTML = selectedMedicines.map((med, index) => `
                <div style="display: flex; justify-content: space-between; align-items: center; padding: 15px; border: 1px solid #ddd; border-radius: 12px; margin: 10px 0; background: white;">
                    <span><strong>${{med.name}}</strong> - ${{med.amount}} 顆</span>
                    <button class="btn" style="background: #dc3545; color: white; padding: 8px 16px;" onclick="removeSelectedMedicine(${{index}})">移除</button>
                </div>
            `).join('');
        }}

        function removeSelectedMedicine(index) {{
            const med = selectedMedicines[index];
            selectedMedicines.splice(index, 1);
            
            const card = document.getElementById(`med-card-${{med.medicine_id}}`);
            if (card) {{
                card.classList.remove('selected');
            }}
            
            updateSelectedMedicinesDisplay();
        }}

        async function createPrescription() {{
            const patientName = document.getElementById('patientName').value.trim();
            
            if (!patientName) {{
                showAlert('請輸入病患姓名', 'error');
                return;
            }}
            
            if (selectedMedicines.length === 0) {{
                showAlert('請選擇至少一種藥物', 'error');
                return;
            }}
            
            selectedMedicines.forEach(med => {{
                const input = document.getElementById(`amount-${{med.medicine_id}}`);
                if (input) {{
                    med.amount = parseInt(input.value) || 1;
                }}
            }});
            
            const data = {{
                patient_name: patientName,
                medicines: selectedMedicines.map(med => ({{
                    medicine_id: med.medicine_id,
                    amount: med.amount
                }}))
            }};
            
            try {{
                const response = await fetch('/api/prescription/', {{
                    method: 'POST',
                    headers: {{'Content-Type': 'application/json'}},
                    body: JSON.stringify(data)
                }});
                
                if (response.ok) {{
                    const result = await response.json();
                    showAlert(`處方籤開立成功！處方籤編號: ${{result.id}}`, 'success');
                    clearPrescriptionForm();
                }} else {{
                    throw new Error('處方籤開立失敗');
                }}
            }} catch (error) {{
                showAlert('錯誤: ' + error.message, 'error');
            }}
        }}

        function clearPrescriptionForm() {{
            document.getElementById('patientName').value = '';
            selectedMedicines = [];
            document.querySelectorAll('.medicine-card.selected').forEach(card => {{
                card.classList.remove('selected');
            }});
            updateSelectedMedicinesDisplay();
        }}

        document.addEventListener('DOMContentLoaded', function() {{
            setActiveNav('/doctor.html');
            loadAvailableMedicines();
        }});
    </script>
</body>
</html>
    """

@app.get("/prescription", response_class=HTMLResponse)
@app.get("/prescription.html", response_class=HTMLResponse)
async def prescription_page():
    return f"""
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>處方籤管理 - 醫院管理系統</title>
    <style>{COMMON_CSS}</style>
</head>
<body>
    <!-- Left Sidebar Navigation -->
    <div class="sidebar">
        <div class="logo">
            <h2>醫院管理系統</h2>
            <p>Hospital Management System</p>
        </div>
        
        <a href="/medicine.html" class="nav-button">
            <span class="icon">💊</span>
            <span>藥物管理</span>
        </a>
        
        <a href="/doctor.html" class="nav-button">
            <span class="icon">👨‍⚕️</span>
            <span>醫生工作台</span>
        </a>
        
        <a href="/prescription.html" class="nav-button active">
            <span class="icon">📋</span>
            <span>處方籤管理</span>
        </a>
    </div>

    <!-- Main Content -->
    <div class="main-content">
        <div class="page-header">
            <h1>處方籤管理系統</h1>
            <p>監控和管理所有處方籤狀態</p>
        </div>

        <!-- Statistics Cards -->
        <div class="stats-grid">
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

        <div class="content-card">
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 25px;">
                <h2>處方籤列表</h2>
                <div>
                    <button class="btn btn-success" onclick="loadPrescriptions()">
                        <span>🔄</span> 重新載入
                    </button>
                    <button class="btn btn-warning" onclick="toggleAutoRefresh()">
                        <span id="autoRefreshIcon">⏸️</span> <span id="autoRefreshText">停止自動更新</span>
                    </button>
                </div>
            </div>

            <table class="data-table">
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
                        <td colspan="6" class="text-center">
                            <div class="loading"></div> 載入中...
                        </td>
                    </tr>
                </tbody>
            </table>
        </div>
    </div>

    <script>
        {COMMON_JS}
        
        let autoRefreshTimer = null;
        let isAutoRefreshing = true;

        async function loadPrescriptions() {{
            const tbody = document.getElementById('prescriptionList');
            
            try {{
                const response = await fetch('/api/prescription/');
                const prescriptions = await response.json();
                
                updateStats(prescriptions);
                
                if (prescriptions.length === 0) {{
                    tbody.innerHTML = '<tr><td colspan="6" class="text-center">尚無處方籤資料</td></tr>';
                    return;
                }}

                tbody.innerHTML = prescriptions.map(p => {{
                    const createdDate = new Date(p.created_at);
                    const medicineList = p.medicines.map(m => 
                        `<span style="display: inline-block; background: #f8f9fa; padding: 6px 12px; margin: 2px; border-radius: 6px; border: 1px solid #e9ecef; font-size: 12px;">${{m.name}} (${{m.amount}})</span>`
                    ).join(' ');
                    
                    let actions = '';
                    if (p.status === 'pending') {{
                        actions = `<button class="btn btn-primary" onclick="updateStatus(${{p.id}}, 'processing')" style="padding: 8px 16px; font-size: 12px;">開始處理</button>`;
                    }} else if (p.status === 'processing') {{
                        actions = `<button class="btn btn-success" onclick="updateStatus(${{p.id}}, 'completed')" style="padding: 8px 16px; font-size: 12px;">標記完成</button>`;
                    }}
                    
                    return `
                        <tr>
                            <td><strong>#${{p.id.toString().padStart(6, '0')}}</strong></td>
                            <td>${{p.patient_name}}</td>
                            <td>${{createdDate.toLocaleString('zh-TW')}}</td>
                            <td><span class="status-badge status-${{p.status}}">${{getStatusText(p.status)}}</span></td>
                            <td style="font-size: 13px;">${{medicineList}}</td>
                            <td>${{actions}}</td>
                        </tr>
                    `;
                }}).join('');
                
            }} catch (error) {{
                tbody.innerHTML = `<tr><td colspan="6" class="text-center"><div class="alert alert-error">載入失敗: ${{error.message}}</div></td></tr>`;
            }}
        }}

        function updateStats(prescriptions) {{
            const stats = {{
                pending: 0,
                processing: 0,
                completed: 0,
                failed: 0,
                total: prescriptions.length
            }};
            
            prescriptions.forEach(p => {{
                if (stats.hasOwnProperty(p.status)) {{
                    stats[p.status]++;
                }}
            }});
            
            document.getElementById('pendingCount').textContent = stats.pending;
            document.getElementById('processingCount').textContent = stats.processing;
            document.getElementById('completedCount').textContent = stats.completed;
            document.getElementById('totalCount').textContent = stats.total;
        }}

        function getStatusText(status) {{
            const statusMap = {{
                'pending': '待處理',
                'processing': '處理中',
                'completed': '已完成',
                'failed': '失敗'
            }};
            return statusMap[status] || status;
        }}

        async function updateStatus(id, status) {{
            try {{
                const response = await fetch(`/api/prescription/${{id}}/status`, {{
                    method: 'PUT',
                    headers: {{'Content-Type': 'application/json'}},
                    body: JSON.stringify({{status: status}})
                }});
                
                if (response.ok) {{
                    showAlert(`處方籤 #${{id.toString().padStart(6, '0')}} 狀態已更新為: ${{getStatusText(status)}}`, 'success');
                    loadPrescriptions();
                }} else {{
                    throw new Error('狀態更新失敗');
                }}
            }} catch (error) {{
                showAlert('錯誤: ' + error.message, 'error');
            }}
        }}

        function toggleAutoRefresh() {{
            if (isAutoRefreshing) {{
                clearInterval(autoRefreshTimer);
                document.getElementById('autoRefreshIcon').textContent = '▶️';
                document.getElementById('autoRefreshText').textContent = '開始自動更新';
                isAutoRefreshing = false;
            }} else {{
                startAutoRefresh();
                document.getElementById('autoRefreshIcon').textContent = '⏸️';
                document.getElementById('autoRefreshText').textContent = '停止自動更新';
                isAutoRefreshing = true;
            }}
        }}

        function startAutoRefresh() {{
            autoRefreshTimer = setInterval(loadPrescriptions, 5000);
        }}

        document.addEventListener('DOMContentLoaded', function() {{
            setActiveNav('/prescription.html');
            loadPrescriptions();
            startAutoRefresh();
        }});
    </script>
</body>
</html>
    """

if __name__ == "__main__":
    print("Starting Improved Hospital Medicine Management System...")
    init_database()
    
    print("\nSystem ready with improved layout!")
    print("Web Interfaces:")
    print("- Medicine Management: http://localhost:8001/medicine.html")
    print("- Doctor Interface: http://localhost:8001/doctor.html")
    print("- Prescription Management: http://localhost:8001/prescription.html")
    print("\nROS2 Endpoints:")
    print("- Get Order: GET http://localhost:8001/api/order/next")
    print("- Report Progress: POST http://localhost:8001/api/order/progress")
    print("- Report Complete: POST http://localhost:8001/api/order/complete")
    
    uvicorn.run(app, host="0.0.0.0", port=8001)
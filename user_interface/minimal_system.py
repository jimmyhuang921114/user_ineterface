#!/usr/bin/env python3
"""
Minimal Hospital Medicine Management System
Clean, working system with ROS2 integration
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
import subprocess
import sys
import signal
import time
import requests
import yaml
from pathlib import Path

# Database setup
DATABASE_URL = "sqlite:///./hospital_medicine.db"
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
            Medicine(name="Aspirin", amount=100, position="A1", description="Pain reliever"),
            Medicine(name="Vitamin C", amount=50, position="A2", description="Vitamin supplement"),
            Medicine(name="Ibuprofen", amount=75, position="B1", description="Anti-inflammatory")
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

# ROS2 Adapter in same process
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

# Simple HTML responses for web interface
@app.get("/medicine", response_class=HTMLResponse)
async def medicine_page():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Medicine Management</title>
        <meta charset="UTF-8">
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; }
            .container { max-width: 800px; margin: 0 auto; }
            table { width: 100%; border-collapse: collapse; margin: 20px 0; }
            th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
            th { background-color: #f2f2f2; }
            input, button { padding: 10px; margin: 5px; }
            button { background-color: #4CAF50; color: white; border: none; cursor: pointer; }
            button:hover { background-color: #45a049; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Medicine Management</h1>
            
            <div>
                <h2>Add New Medicine</h2>
                <input type="text" id="name" placeholder="Medicine Name">
                <input type="number" id="amount" placeholder="Amount" value="100">
                <input type="text" id="position" placeholder="Position" value="A1">
                <input type="text" id="description" placeholder="Description">
                <button onclick="addMedicine()">Add Medicine</button>
            </div>
            
            <div>
                <h2>Current Medicines</h2>
                <button onclick="loadMedicines()">Refresh</button>
                <table id="medicineTable">
                    <thead>
                        <tr><th>ID</th><th>Name</th><th>Amount</th><th>Position</th><th>Description</th></tr>
                    </thead>
                    <tbody id="medicineList"></tbody>
                </table>
            </div>
        </div>
        
        <script>
            async function addMedicine() {
                const data = {
                    name: document.getElementById('name').value,
                    amount: parseInt(document.getElementById('amount').value) || 100,
                    position: document.getElementById('position').value || 'A1',
                    description: document.getElementById('description').value || ''
                };
                
                try {
                    const response = await fetch('/api/medicine/', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify(data)
                    });
                    const result = await response.json();
                    alert('Medicine added: ' + result.name);
                    loadMedicines();
                    document.getElementById('name').value = '';
                    document.getElementById('description').value = '';
                } catch (error) {
                    alert('Error: ' + error.message);
                }
            }
            
            async function loadMedicines() {
                try {
                    const response = await fetch('/api/medicine/');
                    const medicines = await response.json();
                    const tbody = document.getElementById('medicineList');
                    tbody.innerHTML = '';
                    
                    medicines.forEach(med => {
                        const row = tbody.insertRow();
                        row.innerHTML = `
                            <td>${med.id}</td>
                            <td>${med.name}</td>
                            <td>${med.amount}</td>
                            <td>${med.position}</td>
                            <td>${med.description}</td>
                        `;
                    });
                } catch (error) {
                    alert('Error loading medicines: ' + error.message);
                }
            }
            
            // Load medicines on page load
            window.onload = loadMedicines;
        </script>
    </body>
    </html>
    """

@app.get("/doctor", response_class=HTMLResponse)
async def doctor_page():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Doctor Interface</title>
        <meta charset="UTF-8">
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; }
            .container { max-width: 800px; margin: 0 auto; }
            input, button, select { padding: 10px; margin: 5px; }
            button { background-color: #2196F3; color: white; border: none; cursor: pointer; }
            button:hover { background-color: #1976D2; }
            .medicine-item { border: 1px solid #ddd; padding: 10px; margin: 5px; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Doctor Interface</h1>
            
            <div>
                <h2>Create Prescription</h2>
                <input type="text" id="patientName" placeholder="Patient Name">
                <button onclick="loadAvailableMedicines()">Load Medicines</button>
                <div id="medicineSelection"></div>
                <button onclick="createPrescription()">Create Prescription</button>
            </div>
        </div>
        
        <script>
            let availableMedicines = [];
            let selectedMedicines = [];
            
            async function loadAvailableMedicines() {
                try {
                    const response = await fetch('/api/medicine/');
                    availableMedicines = await response.json();
                    displayMedicineSelection();
                } catch (error) {
                    alert('Error loading medicines: ' + error.message);
                }
            }
            
            function displayMedicineSelection() {
                const div = document.getElementById('medicineSelection');
                div.innerHTML = '<h3>Select Medicines:</h3>';
                
                availableMedicines.forEach(med => {
                    div.innerHTML += `
                        <div class="medicine-item">
                            <input type="checkbox" id="med_${med.id}" onchange="toggleMedicine(${med.id})">
                            <label for="med_${med.id}">${med.name} (Stock: ${med.amount})</label>
                            <input type="number" id="amount_${med.id}" min="1" max="${med.amount}" value="1" style="width:60px;">
                        </div>
                    `;
                });
            }
            
            function toggleMedicine(medId) {
                const checkbox = document.getElementById(`med_${medId}`);
                const amount = parseInt(document.getElementById(`amount_${medId}`).value) || 1;
                
                if (checkbox.checked) {
                    selectedMedicines.push({medicine_id: medId, amount: amount});
                } else {
                    selectedMedicines = selectedMedicines.filter(m => m.medicine_id !== medId);
                }
            }
            
            async function createPrescription() {
                const patientName = document.getElementById('patientName').value;
                if (!patientName || selectedMedicines.length === 0) {
                    alert('Please enter patient name and select medicines');
                    return;
                }
                
                // Update amounts from inputs
                selectedMedicines.forEach(med => {
                    med.amount = parseInt(document.getElementById(`amount_${med.medicine_id}`).value) || 1;
                });
                
                const data = {
                    patient_name: patientName,
                    medicines: selectedMedicines
                };
                
                try {
                    const response = await fetch('/api/prescription/', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify(data)
                    });
                    const result = await response.json();
                    alert('Prescription created: ' + result.id);
                    document.getElementById('patientName').value = '';
                    selectedMedicines = [];
                    loadAvailableMedicines();
                } catch (error) {
                    alert('Error: ' + error.message);
                }
            }
        </script>
    </body>
    </html>
    """

@app.get("/prescriptions", response_class=HTMLResponse)
async def prescriptions_page():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Prescription Management</title>
        <meta charset="UTF-8">
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; }
            .container { max-width: 1000px; margin: 0 auto; }
            table { width: 100%; border-collapse: collapse; margin: 20px 0; }
            th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
            th { background-color: #f2f2f2; }
            button { padding: 5px 10px; margin: 2px; }
            .pending { background-color: #fff3cd; }
            .processing { background-color: #d1ecf1; }
            .completed { background-color: #d4edda; }
            .failed { background-color: #f8d7da; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Prescription Management</h1>
            <button onclick="loadPrescriptions()">Refresh</button>
            
            <table id="prescriptionTable">
                <thead>
                    <tr><th>ID</th><th>Patient</th><th>Created</th><th>Status</th><th>Medicines</th><th>Actions</th></tr>
                </thead>
                <tbody id="prescriptionList"></tbody>
            </table>
        </div>
        
        <script>
            async function loadPrescriptions() {
                try {
                    const response = await fetch('/api/prescription/');
                    const prescriptions = await response.json();
                    const tbody = document.getElementById('prescriptionList');
                    tbody.innerHTML = '';
                    
                    prescriptions.forEach(p => {
                        const row = tbody.insertRow();
                        row.className = p.status;
                        row.innerHTML = `
                            <td>${p.id}</td>
                            <td>${p.patient_name}</td>
                            <td>${new Date(p.created_at).toLocaleString()}</td>
                            <td>${p.status}</td>
                            <td>${p.medicines.map(m => `${m.name} (${m.amount})`).join(', ')}</td>
                            <td>
                                ${p.status === 'pending' ? `<button onclick="updateStatus(${p.id}, 'processing')">Start</button>` : ''}
                                ${p.status === 'processing' ? `<button onclick="updateStatus(${p.id}, 'completed')">Complete</button>` : ''}
                            </td>
                        `;
                    });
                } catch (error) {
                    alert('Error loading prescriptions: ' + error.message);
                }
            }
            
            async function updateStatus(id, status) {
                try {
                    const response = await fetch(`/api/prescription/${id}/status`, {
                        method: 'PUT',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify({status: status})
                    });
                    await response.json();
                    loadPrescriptions();
                } catch (error) {
                    alert('Error updating status: ' + error.message);
                }
            }
            
            // Auto-refresh every 5 seconds
            setInterval(loadPrescriptions, 5000);
            window.onload = loadPrescriptions;
        </script>
    </body>
    </html>
    """

if __name__ == "__main__":
    print("Starting Hospital Medicine Management System...")
    init_database()
    
    print("\nSystem ready!")
    print("Web Interfaces:")
    print("- Medicine Management: http://localhost:8001/medicine")
    print("- Doctor Interface: http://localhost:8001/doctor")
    print("- Prescription Management: http://localhost:8001/prescriptions")
    print("\nROS2 Endpoints:")
    print("- Get Order: GET http://localhost:8001/api/order/next")
    print("- Report Progress: POST http://localhost:8001/api/order/progress")
    print("- Report Complete: POST http://localhost:8001/api/order/complete")
    
    uvicorn.run(app, host="0.0.0.0", port=8001)
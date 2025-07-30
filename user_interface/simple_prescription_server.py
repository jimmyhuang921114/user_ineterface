#!/usr/bin/env python3
"""

Simple Prescription Management System
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, HTMLResponse
from pydantic import BaseModel
from datetime import datetime
from typing import List, Optional
import uvicorn
import json

# Pydantic
class PrescriptionMedicine(BaseModel):
    medicine_name: str
    dosage: str
    frequency: str
    duration: str
    instructions: Optional[str] = ""

class PrescriptionCreate(BaseModel):
    patient_name: str
    doctor_name: str
    medicines: List[PrescriptionMedicine]
    diagnosis: Optional[str] = ""
    instructions: Optional[str] = ""
    priority: Optional[str] = "normal"

#  FastAPI
app = FastAPI(
    title="",
    description="Prescription Management System",
    version="1.0.0"
)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

#  ()
prescriptions_db = []
next_prescription_id = 1

#
@app.get("/")
async def root():
    return {
        "message": "",
        "status": "",
        "version": "1.0.0",
        "total_prescriptions": len(prescriptions_db),
        "features": [
            "",
            "",
            "ROS2"
        ]
    }

# ===  API ===
@app.post("/api/prescription/")
async def create_prescription(prescription: PrescriptionCreate):
    """"""
    global next_prescription_id

    new_prescription = {
        "id": next_prescription_id,
        "patient_name": prescription.patient_name,
        "doctor_name": prescription.doctor_name,
        "medicines": [med.dict() for med in prescription.medicines],
        "diagnosis": prescription.diagnosis,
        "instructions": prescription.instructions,
        "priority": prescription.priority,
        "status": "pending",
        "created_time": datetime.now().isoformat(),
        "updated_time": datetime.now().isoformat()
    }

    prescriptions_db.append(new_prescription)
    next_prescription_id += 1

    return new_prescription

@app.get("/api/prescription/")
async def get_all_prescriptions():
    """"""
    return prescriptions_db

@app.get("/api/prescription/{prescription_id}")
async def get_prescription(prescription_id: int):
    """"""
    prescription = next((p for p in prescriptions_db if p["id"] == prescription_id), None)
    if not prescription:
        raise HTTPException(status_code=404, detail="")
    return prescription

@app.put("/api/prescription/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_data: dict):
    """"""
    prescription = next((p for p in prescriptions_db if p["id"] == prescription_id), None)
    if not prescription:
        raise HTTPException(status_code=404, detail="")

    new_status = status_data.get("status", "pending")
    prescription["status"] = new_status
    prescription["updated_time"] = datetime.now().isoformat()

    return {"success": True, "message": "", "new_status": new_status}

@app.get("/api/prescription/status/{status}")
async def get_prescriptions_by_status(status: str):
    """"""
    filtered_prescriptions = [p for p in prescriptions_db if p["status"] == status]
    return filtered_prescriptions

# === ROS2  API ===
@app.post("/api/ros2/medicine/request")
async def request_medicine_info_from_ros2(request_data: dict):
    """ROS2"""
    medicine_name = request_data.get("medicine_name", "")

    # ROS2
    medicine_info = {
        "medicine_name": medicine_name,
        "available_stock": 100,
        "position": "A1-01",
        "detailed_info": f" for {medicine_name}",
        "service_response_time": datetime.now().isoformat()
    }

    return {
        "success": True,
        "medicine_info": medicine_info,
        "ros2_service_status": "active"
    }

@app.get("/api/ros2/services/status")
async def get_ros2_services_status():
    """ROS2"""
    return {
        "services": [
            {"service_name": "medicine_info_service", "status": "active"},
            {"service_name": "prescription_notification", "status": "active"}
        ],
        "total_services": 2,
        "active_services": 2,
        "last_update": datetime.now().isoformat()
    }

# ===  ===
@app.get("/Prescription.html")
async def serve_prescription_page():
    """"""
    html_content = """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title></title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f5f5f5; }
        .container { max-width: 1000px; margin: 0 auto; background: white; padding: 20px; border-radius: 8px; }
        .header { background: #2c3e50; color: white; padding: 20px; border-radius: 8px; margin-bottom: 20px; }
        .section { margin: 20px 0; padding: 20px; border: 1px solid #ddd; border-radius: 8px; }
        .form-group { margin: 15px 0; }
        .form-group label { display: block; margin-bottom: 5px; font-weight: bold; }
        .form-group input, .form-group select, .form-group textarea {
            width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px;
        }
        .btn { padding: 10px 20px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; }
        .btn-primary { background: #3498db; color: white; }
        .btn-success { background: #27ae60; color: white; }
        .btn-warning { background: #f39c12; color: white; }
        .prescription-item {
            padding: 15px; margin: 10px 0; border: 1px solid #ddd; border-radius: 8px;
            background: white;
        }
        .status-pending { color: #f39c12; font-weight: bold; }
        .status-processing { color: #3498db; font-weight: bold; }
        .status-completed { color: #27ae60; font-weight: bold; }
        .medicine-item { background: #f8f9fa; padding: 10px; margin: 5px 0; border-radius: 4px; }
        .message { padding: 10px; margin: 10px 0; border-radius: 4px; }
        .message-success { background: #d4edda; color: #155724; }
        .message-error { background: #f8d7da; color: #721c24; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1> - ROS2</h1>
            <p></p>
        </div>

        <!--  -->
        <div class="section">
            <h2></h2>
            <form id="prescriptionForm">
                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 20px;">
                    <div>
                        <div class="form-group">
                            <label></label>
                            <input type="text" id="patientName" required>
                        </div>
                        <div class="form-group">
                            <label></label>
                            <input type="text" id="doctorName" required>
                        </div>
                    </div>
                    <div>
                        <div class="form-group">
                            <label></label>
                            <textarea id="diagnosis" rows="2"></textarea>
                        </div>
                        <div class="form-group">
                            <label></label>
                            <textarea id="instructions" rows="2"></textarea>
                        </div>
                    </div>
                </div>

                <h3></h3>
                <div id="medicinesContainer">
                    <div class="medicine-item">
                        <div style="display: grid; grid-template-columns: 2fr 1fr 1fr 1fr auto; gap: 10px;">
                            <input type="text" placeholder="" class="medicine-name" required>
                            <input type="text" placeholder="" class="medicine-dosage" required>
                            <input type="text" placeholder="" class="medicine-frequency" required>
                            <input type="text" placeholder="" class="medicine-duration" required>
                            <button type="button" class="btn btn-warning" onclick="removeMedicine(this)"></button>
                        </div>
                        <textarea placeholder="" class="medicine-instructions" style="margin-top: 10px; width: 100%;"></textarea>
                    </div>
                </div>
                <button type="button" class="btn btn-primary" onclick="addMedicine()"></button>

                <div style="margin-top: 20px;">
                    <button type="submit" class="btn btn-success"></button>
                    <button type="button" class="btn btn-warning" onclick="clearForm()"></button>
                </div>
            </form>
        </div>

        <!--  -->
        <div class="section">
            <h2></h2>
            <div style="margin-bottom: 15px;">
                <button class="btn btn-primary" onclick="loadPrescriptions()"></button>
                <button class="btn btn-warning" onclick="loadPrescriptions('pending')"></button>
                <button class="btn btn-success" onclick="loadPrescriptions('completed')"></button>
            </div>
            <div id="prescriptionsList"></div>
        </div>

        <!--  -->
        <div id="messageArea"></div>
    </div>

    <script>
        const API_BASE = 'http://localhost:8001';

        function showMessage(message, type = 'info') {
            const messageArea = document.getElementById('messageArea');
            const messageDiv = document.createElement('div');
            messageDiv.className = `message message-${type}`;
            messageDiv.textContent = message;
            messageArea.appendChild(messageDiv);

            setTimeout(() => messageDiv.remove(), 5000);
        }

        function addMedicine() {
            const container = document.getElementById('medicinesContainer');
            const medicineItem = document.createElement('div');
            medicineItem.className = 'medicine-item';
            medicineItem.innerHTML = `
                <div style="display: grid; grid-template-columns: 2fr 1fr 1fr 1fr auto; gap: 10px;">
                    <input type="text" placeholder="" class="medicine-name" required>
                    <input type="text" placeholder="" class="medicine-dosage" required>
                    <input type="text" placeholder="" class="medicine-frequency" required>
                    <input type="text" placeholder="" class="medicine-duration" required>
                    <button type="button" class="btn btn-warning" onclick="removeMedicine(this)"></button>
                </div>
                <textarea placeholder="" class="medicine-instructions" style="margin-top: 10px; width: 100%;"></textarea>
            `;
            container.appendChild(medicineItem);
        }

        function removeMedicine(button) {
            button.closest('.medicine-item').remove();
        }

        function clearForm() {
            document.getElementById('prescriptionForm').reset();
            const container = document.getElementById('medicinesContainer');
            container.innerHTML = `
                <div class="medicine-item">
                    <div style="display: grid; grid-template-columns: 2fr 1fr 1fr 1fr auto; gap: 10px;">
                        <input type="text" placeholder="" class="medicine-name" required>
                        <input type="text" placeholder="" class="medicine-dosage" required>
                        <input type="text" placeholder="" class="medicine-frequency" required>
                        <input type="text" placeholder="" class="medicine-duration" required>
                        <button type="button" class="btn btn-warning" onclick="removeMedicine(this)"></button>
                    </div>
                    <textarea placeholder="" class="medicine-instructions" style="margin-top: 10px; width: 100%;"></textarea>
                </div>
            `;
        }

        document.getElementById('prescriptionForm').addEventListener('submit', async (e) => {
            e.preventDefault();

            const medicines = [];
            const medicineItems = document.querySelectorAll('.medicine-item');

            medicineItems.forEach(item => {
                const name = item.querySelector('.medicine-name').value;
                const dosage = item.querySelector('.medicine-dosage').value;
                const frequency = item.querySelector('.medicine-frequency').value;
                const duration = item.querySelector('.medicine-duration').value;
                const instructions = item.querySelector('.medicine-instructions').value;

                if (name && dosage && frequency && duration) {
                    medicines.push({
                        medicine_name: name,
                        dosage: dosage,
                        frequency: frequency,
                        duration: duration,
                        instructions: instructions
                    });
                }
            });

            if (medicines.length === 0) {
                showMessage('', 'error');
                return;
            }

            const prescriptionData = {
                patient_name: document.getElementById('patientName').value,
                doctor_name: document.getElementById('doctorName').value,
                diagnosis: document.getElementById('diagnosis').value,
                instructions: document.getElementById('instructions').value,
                medicines: medicines
            };

            try {
                const response = await fetch(`${API_BASE}/api/prescription/`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(prescriptionData)
                });

                if (response.ok) {
                    const result = await response.json();
                    showMessage(`ID: ${result.id}`, 'success');
                    clearForm();
                    loadPrescriptions();
                } else {
                    throw new Error('');
                }
            } catch (error) {
                showMessage(': ' + error.message, 'error');
            }
        });

        async function loadPrescriptions(status = null) {
            try {
                let url = `${API_BASE}/api/prescription/`;
                if (status) url = `${API_BASE}/api/prescription/status/${status}`;

                const response = await fetch(url);
                const prescriptions = await response.json();

                const container = document.getElementById('prescriptionsList');
                container.innerHTML = '';

                prescriptions.forEach(prescription => {
                    const item = document.createElement('div');
                    item.className = 'prescription-item';
                    item.innerHTML = `
                        <div>
                            <h4> #${prescription.id} - ${prescription.patient_name}</h4>
                            <p><strong>:</strong> ${prescription.doctor_name}</p>
                            <p><strong>:</strong> ${prescription.diagnosis || ''}</p>
                            <p><strong>:</strong> ${prescription.medicines.length}</p>
                            <p><strong>:</strong> <span class="status-${prescription.status}">${getStatusText(prescription.status)}</span></p>
                            <p><strong>:</strong> ${new Date(prescription.created_time).toLocaleString()}</p>
                            <div>
                                <button class="btn btn-primary" onclick="viewPrescription(${prescription.id})"></button>
                                <button class="btn btn-warning" onclick="updateStatus(${prescription.id}, 'processing')"></button>
                                <button class="btn btn-success" onclick="updateStatus(${prescription.id}, 'completed')"></button>
                            </div>
                        </div>
                    `;
                    container.appendChild(item);
                });

                if (prescriptions.length === 0) {
                    container.innerHTML = '<p></p>';
                }
            } catch (error) {
                showMessage(': ' + error.message, 'error');
            }
        }

        function getStatusText(status) {
            const statusMap = {
                'pending': '',
                'processing': '',
                'completed': '',
                'cancelled': ''
            };
            return statusMap[status] || status;
        }

        async function viewPrescription(id) {
            try {
                const response = await fetch(`${API_BASE}/api/prescription/${id}`);
                const prescription = await response.json();

                const medicineList = prescription.medicines.map(med =>
                    `${med.medicine_name} ${med.dosage} ${med.frequency} ${med.duration}`
                ).join('\\n');

                alert(` #${id}\\n\\n: ${prescription.patient_name}\\n: ${prescription.doctor_name}\\n: ${prescription.diagnosis}\\n\\n:\\n${medicineList}`);

            } catch (error) {
                showMessage(': ' + error.message, 'error');
            }
        }

        async function updateStatus(prescriptionId, newStatus) {
            try {
                const response = await fetch(`${API_BASE}/api/prescription/${prescriptionId}/status`, {
                    method: 'PUT',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ status: newStatus })
                });

                if (response.ok) {
                    showMessage('', 'success');
                    loadPrescriptions();
                } else {
                    throw new Error('');
                }
            } catch (error) {
                showMessage(': ' + error.message, 'error');
            }
        }

        //
        document.addEventListener('DOMContentLoaded', () => {
            loadPrescriptions();
        });
    </script>
</body>
</html>
    """
    return HTMLResponse(content=html_content, status_code=200)

#
def init_test_data():
    """"""
    pass

if __name__ == "__main__":
    print("")
    print("=" * 50)
    print(": http://localhost:8001")
    print("API: http://localhost:8001/docs")
    print(": http://localhost:8001/Prescription.html")
    print("=" * 50)

    try:
        uvicorn.run(app, host="0.0.0.0", port=8001, log_level="info")
    except KeyboardInterrupt:
        print("\n")
    except Exception as e:
        print(f": {e}")
        exit(1)
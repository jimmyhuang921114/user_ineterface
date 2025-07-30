#!/usr/bin/env python3
"""
 - ROS2
Prescription Management System - ROS2 Integration
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse, HTMLResponse
from pydantic import BaseModel
from pathlib import Path
from datetime import datetime
from typing import List, Optional, Dict, Any
import uvicorn
import json
import asyncio

# Pydantic
class PrescriptionMedicine(BaseModel):
    medicine_name: str
    dosage: str
    frequency: str
    duration: str
    instructions: Optional[str] = ""

class PrescriptionCreate(BaseModel):
    patient_name: str
    patient_id: Optional[str] = ""
    doctor_name: str
    doctor_id: Optional[str] = ""
    medicines: List[PrescriptionMedicine]
    diagnosis: Optional[str] = ""
    instructions: Optional[str] = ""
    priority: Optional[str] = "normal"  # normal, urgent, emergency

class PrescriptionStatus(BaseModel):
    prescription_id: int
    status: str  # pending, processing, completed, cancelled
    updated_by: str
    notes: Optional[str] = ""

class ROS2ServiceStatus(BaseModel):
    service_name: str
    status: str  # active, inactive, error
    last_call: Optional[str] = ""

#  FastAPI
app = FastAPI(
    title=" - ROS2",
    description="Prescription Management System - ROS2 Integration",
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

#
base_dir = Path(__file__).resolve().parent
static_dir = base_dir / "static"

#
try:
    if (static_dir / "css").exists():
        app.mount("/css", StaticFiles(directory=static_dir / "css"), name="css")
    if (static_dir / "js").exists():
        app.mount("/js", StaticFiles(directory=static_dir / "js"), name="js")
    if (static_dir / "html").exists():
        app.mount("/html", StaticFiles(directory=static_dir / "html"), name="html")
except Exception as e:
    print(f": : {e}")

#  ()
prescriptions_db = []
prescription_status_db = []
ros2_services_db = []
next_prescription_id = 1

#
@app.get("/")
async def root():
    return {
        "message": " - ROS2",
        "status": "",
        "version": "1.0.0",
        "statistics": {
            "total_prescriptions": len(prescriptions_db),
            "pending_prescriptions": len([p for p in prescriptions_db if p.get("status") == "pending"]),
            "completed_prescriptions": len([p for p in prescriptions_db if p.get("status") == "completed"]),
            "active_ros2_services": len([s for s in ros2_services_db if s.get("status") == "active"])
        },
        "features": [
            "",
            "",
            "ROS2",
            ""
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
        "patient_id": prescription.patient_id,
        "doctor_name": prescription.doctor_name,
        "doctor_id": prescription.doctor_id,
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

    #
    status_record = {
        "prescription_id": new_prescription["id"],
        "status": "pending",
        "updated_by": prescription.doctor_name,
        "updated_time": datetime.now().isoformat(),
        "notes": ""
    }
    prescription_status_db.append(status_record)

    # ROS2
    await notify_ros2_service("prescription_created", new_prescription)

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

    #
    status_history = [s for s in prescription_status_db if s["prescription_id"] == prescription_id]

    return {
        "prescription": prescription,
        "status_history": status_history
    }

@app.put("/api/prescription/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_update: PrescriptionStatus):
    """ (ROS2)"""
    prescription = next((p for p in prescriptions_db if p["id"] == prescription_id), None)
    if not prescription:
        raise HTTPException(status_code=404, detail="")

    #
    prescription["status"] = status_update.status
    prescription["updated_time"] = datetime.now().isoformat()

    #
    status_record = {
        "prescription_id": prescription_id,
        "status": status_update.status,
        "updated_by": status_update.updated_by,
        "updated_time": datetime.now().isoformat(),
        "notes": status_update.notes or f": {status_update.status}"
    }
    prescription_status_db.append(status_record)

    # ROS2
    await notify_ros2_service("prescription_status_updated", {
        "prescription_id": prescription_id,
        "new_status": status_update.status,
        "updated_by": status_update.updated_by
    })

    return {"success": True, "message": "", "new_status": status_update.status}

@app.get("/api/prescription/doctor/{doctor_name}")
async def get_prescriptions_by_doctor(doctor_name: str):
    """"""
    doctor_prescriptions = [p for p in prescriptions_db if p["doctor_name"] == doctor_name]
    return doctor_prescriptions

@app.get("/api/prescription/status/{status}")
async def get_prescriptions_by_status(status: str):
    """"""
    status_prescriptions = [p for p in prescriptions_db if p["status"] == status]
    return status_prescriptions

# === ROS2  API ===
@app.post("/api/ros2/medicine/request")
async def request_medicine_info_from_ros2(medicine_name: str):
    """ROS2"""
    try:
        # ROS2
        service_call = {
            "service_name": "get_medicine_info",
            "request_time": datetime.now().isoformat(),
            "medicine_name": medicine_name,
            "status": "requested"
        }

        # ROS2
        #
        medicine_info = {
            "medicine_name": medicine_name,
            "available_stock": 100,
            "position": "A1-01",
            "detailed_info": f"for {medicine_name}",
            "service_response_time": datetime.now().isoformat()
        }

        return {
            "success": True,
            "service_call": service_call,
            "medicine_info": medicine_info
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"ROS2: {str(e)}")

@app.get("/api/ros2/services/status")
async def get_ros2_services_status():
    """ROS2"""
    return {
        "services": ros2_services_db,
        "total_services": len(ros2_services_db),
        "active_services": len([s for s in ros2_services_db if s.get("status") == "active"]),
        "last_update": datetime.now().isoformat()
    }

@app.post("/api/ros2/services/register")
async def register_ros2_service(service_info: ROS2ServiceStatus):
    """ROS2"""
    service = {
        "service_name": service_info.service_name,
        "status": service_info.status,
        "last_call": service_info.last_call,
        "registered_time": datetime.now().isoformat()
    }

    #
    existing_service = next((s for s in ros2_services_db if s["service_name"] == service_info.service_name), None)
    if existing_service:
        existing_service.update(service)
    else:
        ros2_services_db.append(service)

    return {"success": True, "message": "ROS2", "service": service}

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
        .container { max-width: 1200px; margin: 0 auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
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
        .btn-danger { background: #e74c3c; color: white; }
        .status-pending { color: #f39c12; }
        .status-processing { color: #3498db; }
        .status-completed { color: #27ae60; }
        .status-cancelled { color: #e74c3c; }
        .medicine-item { background: #f8f9fa; padding: 10px; margin: 5px 0; border-radius: 4px; border-left: 4px solid #3498db; }
        .prescription-list { max-height: 400px; overflow-y: auto; }
        .prescription-item {
            padding: 15px; margin: 10px 0; border: 1px solid #ddd; border-radius: 8px;
            background: white; cursor: pointer; transition: background 0.3s;
        }
        .prescription-item:hover { background: #f8f9fa; }
        .ros2-status { display: inline-block; padding: 4px 8px; border-radius: 4px; font-size: 12px; }
        .ros2-active { background: #d4edda; color: #155724; }
        .ros2-inactive { background: #f8d7da; color: #721c24; }
        .message { padding: 10px; margin: 10px 0; border-radius: 4px; }
        .message-success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
        .message-error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
        .message-info { background: #d1ecf1; color: #0c5460; border: 1px solid #bee5eb; }
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
                            <label>ID ()</label>
                            <input type="text" id="patientId">
                        </div>
                        <div class="form-group">
                            <label></label>
                            <input type="text" id="doctorName" required>
                        </div>
                        <div class="form-group">
                            <label>ID ()</label>
                            <input type="text" id="doctorId">
                        </div>
                    </div>
                    <div>
                        <div class="form-group">
                            <label></label>
                            <textarea id="diagnosis" rows="3"></textarea>
                        </div>
                        <div class="form-group">
                            <label></label>
                            <textarea id="instructions" rows="3"></textarea>
                        </div>
                        <div class="form-group">
                            <label></label>
                            <select id="priority">
                                <option value="normal"></option>
                                <option value="urgent"></option>
                                <option value="emergency"></option>
                            </select>
                        </div>
                    </div>
                </div>

                <h3></h3>
                <div id="medicinesContainer">
                    <div class="medicine-item">
                        <div style="display: grid; grid-template-columns: 2fr 1fr 1fr 1fr auto; gap: 10px; align-items: center;">
                            <input type="text" placeholder="" class="medicine-name" required>
                            <input type="text" placeholder="" class="medicine-dosage" required>
                            <input type="text" placeholder="" class="medicine-frequency" required>
                            <input type="text" placeholder="" class="medicine-duration" required>
                            <button type="button" class="btn btn-danger" onclick="removeMedicine(this)"></button>
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
                <button class="btn btn-danger" onclick="loadPrescriptions('cancelled')"></button>
            </div>
            <div id="prescriptionsList" class="prescription-list"></div>
        </div>

        <!-- ROS2 -->
        <div class="section">
            <h2>ROS2</h2>
            <div id="ros2Status"></div>
            <button class="btn btn-primary" onclick="updateROS2Status()"></button>
        </div>

        <!--  -->
        <div id="messageArea"></div>
    </div>

    <script>
        // APIURL
        const API_BASE = 'http://localhost:8000';

        //
        function showMessage(message, type = 'info') {
            const messageArea = document.getElementById('messageArea');
            const messageDiv = document.createElement('div');
            messageDiv.className = `message message-${type}`;
            messageDiv.textContent = message;
            messageArea.appendChild(messageDiv);

            setTimeout(() => {
                messageDiv.remove();
            }, 5000);
        }

        //
        function addMedicine() {
            const container = document.getElementById('medicinesContainer');
            const medicineItem = document.createElement('div');
            medicineItem.className = 'medicine-item';
            medicineItem.innerHTML = `
                <div style="display: grid; grid-template-columns: 2fr 1fr 1fr 1fr auto; gap: 10px; align-items: center;">
                    <input type="text" placeholder="" class="medicine-name" required>
                    <input type="text" placeholder="" class="medicine-dosage" required>
                    <input type="text" placeholder="" class="medicine-frequency" required>
                    <input type="text" placeholder="" class="medicine-duration" required>
                    <button type="button" class="btn btn-danger" onclick="removeMedicine(this)"></button>
                </div>
                <textarea placeholder="" class="medicine-instructions" style="margin-top: 10px; width: 100%;"></textarea>
            `;
            container.appendChild(medicineItem);
        }

        //
        function removeMedicine(button) {
            button.closest('.medicine-item').remove();
        }

        //
        function clearForm() {
            document.getElementById('prescriptionForm').reset();
            const container = document.getElementById('medicinesContainer');
            container.innerHTML = `
                <div class="medicine-item">
                    <div style="display: grid; grid-template-columns: 2fr 1fr 1fr 1fr auto; gap: 10px; align-items: center;">
                        <input type="text" placeholder="" class="medicine-name" required>
                        <input type="text" placeholder="" class="medicine-dosage" required>
                        <input type="text" placeholder="" class="medicine-frequency" required>
                        <input type="text" placeholder="" class="medicine-duration" required>
                        <button type="button" class="btn btn-danger" onclick="removeMedicine(this)"></button>
                    </div>
                    <textarea placeholder="" class="medicine-instructions" style="margin-top: 10px; width: 100%;"></textarea>
                </div>
            `;
        }

        //
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
                patient_id: document.getElementById('patientId').value,
                doctor_name: document.getElementById('doctorName').value,
                doctor_id: document.getElementById('doctorId').value,
                diagnosis: document.getElementById('diagnosis').value,
                instructions: document.getElementById('instructions').value,
                priority: document.getElementById('priority').value,
                medicines: medicines
            };

            try {
                const response = await fetch(`${API_BASE}/api/prescription/`, {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
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

        //
        async function loadPrescriptions(status = null) {
            try {
                let url = `${API_BASE}/api/prescription/`;
                if (status) {
                    url = `${API_BASE}/api/prescription/status/${status}`;
                }

                const response = await fetch(url);
                const prescriptions = await response.json();

                const container = document.getElementById('prescriptionsList');
                container.innerHTML = '';

                prescriptions.forEach(prescription => {
                    const item = document.createElement('div');
                    item.className = 'prescription-item';
                    item.innerHTML = `
                        <div style="display: flex; justify-content: between; align-items: center;">
                            <div style="flex: 1;">
                                <h4> #${prescription.id} - ${prescription.patient_name}</h4>
                                <p><strong>:</strong> ${prescription.doctor_name}</p>
                                <p><strong>:</strong> ${prescription.diagnosis || ''}</p>
                                <p><strong>:</strong> ${prescription.medicines.length}</p>
                                <p><strong>:</strong> ${new Date(prescription.created_time).toLocaleString()}</p>
                            </div>
                            <div>
                                <span class="status-${prescription.status}">${getStatusText(prescription.status)}</span>
                                <div style="margin-top: 10px;">
                                    <button class="btn btn-primary" onclick="viewPrescription(${prescription.id})"></button>
                                    <button class="btn btn-warning" onclick="updateStatus(${prescription.id}, 'processing')"></button>
                                    <button class="btn btn-success" onclick="updateStatus(${prescription.id}, 'completed')"></button>
                                </div>
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

        //
        function getStatusText(status) {
            const statusMap = {
                'pending': '',
                'processing': '',
                'completed': '',
                'cancelled': ''
            };
            return statusMap[status] || status;
        }

        //
        async function viewPrescription(id) {
            try {
                const response = await fetch(`${API_BASE}/api/prescription/${id}`);
                const data = await response.json();

                let medicinesHtml = '';
                data.prescription.medicines.forEach(med => {
                    medicinesHtml += `
                        <li><strong>${med.medicine_name}</strong> - ${med.dosage} ${med.frequency} ${med.duration}
                        ${med.instructions ? '<br><em>' + med.instructions + '</em>' : ''}</li>
                    `;
                });

                alert(` #${id}\\n\\n: ${data.prescription.patient_name}\\n: ${data.prescription.doctor_name}\\n: ${data.prescription.diagnosis}\\n\\n:\\n${data.prescription.medicines.map(m => `${m.medicine_name} ${m.dosage} ${m.frequency} ${m.duration}`).join('\\n')}`);

            } catch (error) {
                showMessage(': ' + error.message, 'error');
            }
        }

        //
        async function updateStatus(prescriptionId, newStatus) {
            try {
                const response = await fetch(`${API_BASE}/api/prescription/${prescriptionId}/status`, {
                    method: 'PUT',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        prescription_id: prescriptionId,
                        status: newStatus,
                        updated_by: 'System User',
                        notes: ` ${getStatusText(newStatus)}`
                    })
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

        // ROS2
        async function updateROS2Status() {
            try {
                const response = await fetch(`${API_BASE}/api/ros2/services/status`);
                const data = await response.json();

                const container = document.getElementById('ros2Status');
                container.innerHTML = `
                    <p><strong>:</strong> ${data.total_services}</p>
                    <p><strong>:</strong> ${data.active_services}</p>
                    <p><strong>:</strong> ${new Date(data.last_update).toLocaleString()}</p>
                `;

                if (data.services.length > 0) {
                    data.services.forEach(service => {
                        const serviceDiv = document.createElement('div');
                        serviceDiv.innerHTML = `
                            <span class="ros2-status ros2-${service.status}">${service.service_name}: ${service.status}</span>
                        `;
                        container.appendChild(serviceDiv);
                    });
                }

                showMessage('ROS2', 'success');
            } catch (error) {
                showMessage('ROS2: ' + error.message, 'error');
            }
        }

        //
        document.addEventListener('DOMContentLoaded', () => {
            loadPrescriptions();
            updateROS2Status();
        });
    </script>
</body>
</html>
    """
    return HTMLResponse(content=html_content, status_code=200)

# ===  ===
async def notify_ros2_service(event_type: str, data: Dict[str, Any]):
    """ROS2"""
    notification = {
        "event_type": event_type,
        "timestamp": datetime.now().isoformat(),
        "data": data
    }

    # ROS2
    #
    print(f"ROS2: {notification}")

    #
    service_status = {
        "service_name": f"notification_{event_type}",
        "status": "active",
        "last_call": datetime.now().isoformat()
    }

    existing_service = next((s for s in ros2_services_db if s["service_name"] == service_status["service_name"]), None)
    if existing_service:
        existing_service.update(service_status)
    else:
        ros2_services_db.append(service_status)

#
def init_test_data():
    """"""
    global prescriptions_db, prescription_status_db, ros2_services_db

    # ROS2
    test_services = [
        {
            "service_name": "medicine_info_service",
            "status": "active",
            "last_call": datetime.now().isoformat(),
            "registered_time": datetime.now().isoformat()
        },
        {
            "service_name": "prescription_notification",
            "status": "active",
            "last_call": datetime.now().isoformat(),
            "registered_time": datetime.now().isoformat()
        }
    ]
    ros2_services_db.extend(test_services)

if __name__ == "__main__":
    print(" - ROS2")
    print("=" * 70)
    print("...")
    print(":  + ROS2 + ")
    print(": http://localhost:8001")
    print("API: http://localhost:8001/docs")
    print(": http://localhost:8001/Prescription.html")
    print("=" * 70)

    #
    init_test_data()
    print(f"ROS2: {len(ros2_services_db)} ")

    try:
        uvicorn.run(app, host="0.0.0.0", port=8001, log_level="info")
    except KeyboardInterrupt:
        print("\n")
    except Exception as e:
        print(f": : {e}")
        exit(1)
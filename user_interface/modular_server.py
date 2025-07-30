#!/usr/bin/env python3
"""
 -
Hospital Management System - Modular Main Server
API
"""

import uvicorn
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from pathlib import Path
import os
import asyncio
from datetime import datetime

# 導入API模組
from api import medicine_api, prescription_api, medical_record_api
#
from data_persistence import data_persistence

# FastAPI
app = FastAPI(
    title=" API",
    description=" - ",
    version="2.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
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
static_path = Path(__file__).parent / "static"
if static_path.exists():
    app.mount("/css", StaticFiles(directory=static_path / "css"), name="css")
    app.mount("/js", StaticFiles(directory=static_path / "js"), name="js")

# API
app.include_router(medicine_api.router)
app.include_router(prescription_api.router)
app.include_router(medical_record_api.router)

#
def init_test_data():
    """"""
    print("...")

    #
    test_medicines = [
        {"name": "", "amount": 100, "usage_days": 7, "position": "A1-01"},
        {"name": "", "amount": 50, "usage_days": 3, "position": "A1-02"},
        {"name": "(Propranolol)", "amount": 30, "usage_days": 14, "position": "B2-03"},
        {"name": "", "amount": 25, "usage_days": 10, "position": "C3-01"}
    ]

    for i, med_data in enumerate(test_medicines, 1):
        medicine_api.medicines_db.append({
            "id": i,
            "name": med_data["name"],
            "amount": med_data["amount"],
            "usage_days": med_data["usage_days"],
            "position": med_data["position"],
            "create_time": "2025-01-01T00:00:00"
        })
    medicine_api.next_medicine_id = len(test_medicines) + 1

    #
    detailed_medicines = {
        "(Propranolol)": {
            "": {
                "": "(Propranolol)",
                "": "",
                "": "10"
            },
            "": {
                "": "",
                "": ""
            },
            "": {
                "1": "202801",
                "2": "TP071014",
                "3": "009102"
            },
            "": {
                "": "",
                "": "Propranolol HCl"
            },
            "": "",
            "": "-",
            "": "",
            "": "",
            "": "C D ",
            "": ""
        },
        "": {
            "": {
                "": " (Antipsychotics)",
                "": "5 ",
                "": " (Oral use)",
                "": "1 special pill"
            },
            "": {
                "": " ",
                "": ""
            },
            "": {
                "": "2027/08/02"
            },
            "": "",
            "": "",
            "": ""
        }
    }

    medicine_api.detailed_medicines_db.update(detailed_medicines)

    #
    test_prescription = {
        "id": 1,
        "patient_name": "",
        "doctor_name": "",
        "medicines": [
            {
                "medicine_name": "",
                "dosage": "100mg",
                "frequency": "",
                "duration": "7",
                "instructions": ""
            }
        ],
        "diagnosis": "",
        "status": "pending",
        "prescription_date": "2025-01-01",
        "created_time": "2025-01-01T00:00:00",
        "updated_time": "2025-01-01T00:00:00"
    }

    prescription_api.prescriptions_db.append(test_prescription)
    prescription_api.next_prescription_id = 2

    print(f" :")
    print(f"   - : {len(medicine_api.medicines_db)} ")
    print(f"   - : {len(medicine_api.detailed_medicines_db)} ")
    print(f"   - : {len(prescription_api.prescriptions_db)} ")

# ===  ===
@app.get("/", response_class=HTMLResponse)
async def root():
    """"""
    return """
    <!DOCTYPE html>
    <html lang="zh-TW">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title></title>
        <link rel="stylesheet" href="/css/unified_style.css">
    </head>
    <body>
        <div class="sidebar">
            <h2></h2>
            <button onclick="location.href='/doctor.html'"></button>
            <button onclick="location.href='/Medicine.html'"></button>
            <button onclick="location.href='/Prescription.html'"></button>
            <button onclick="location.href='/docs'">API </button>
        </div>
        <div class="main-content">
            <div class="header fade-in">
                <h1></h1>
                <p> - </p>
            </div>
            <div class="card fade-in">
                <div style="padding: 30px; text-align: center;">
                    <h2></h2>
                    <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 20px; margin-top: 30px;">
                        <div style="padding: 20px; background: rgba(52, 152, 219, 0.1); border-radius: 10px;">
                            <h3> </h3>
                            <p></p>
                            <button class="btn btn-primary" onclick="location.href='/doctor.html'"></button>
                        </div>
                        <div style="padding: 20px; background: rgba(39, 174, 96, 0.1); border-radius: 10px;">
                            <h3> </h3>
                            <p></p>
                            <button class="btn btn-success" onclick="location.href='/Medicine.html'"></button>
                        </div>
                        <div style="padding: 20px; background: rgba(243, 156, 18, 0.1); border-radius: 10px;">
                            <h3> </h3>
                            <p></p>
                            <button class="btn btn-warning" onclick="location.href='/Prescription.html'"></button>
                        </div>
                    </div>
                    <div style="margin-top: 30px;">
                        <h3> </h3>
                        <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-top: 20px;">
                            <div style="padding: 15px; background: rgba(155, 89, 182, 0.1); border-radius: 8px;">
                                <strong>API</strong><br>
                                <small>API</small>
                            </div>
                            <div style="padding: 15px; background: rgba(231, 76, 60, 0.1); border-radius: 8px;">
                                <strong></strong><br>
                                <small></small>
                            </div>
                            <div style="padding: 15px; background: rgba(23, 162, 184, 0.1); border-radius: 8px;">
                                <strong></strong><br>
                                <small></small>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </body>
    </html>
    """

@app.get("/doctor.html")
async def serve_doctor_page():
    """"""
    html_file = Path(__file__).parent / "static" / "html" / "doctor.html"
    if html_file.exists():
        return FileResponse(html_file)
    return HTMLResponse("<h1></h1>", status_code=404)

@app.get("/Medicine.html")
async def serve_medicine_page():
    """"""
    html_file = Path(__file__).parent / "static" / "html" / "Medicine.html"
    if html_file.exists():
        return FileResponse(html_file)
    return HTMLResponse("<h1></h1>", status_code=404)

@app.get("/Prescription.html")
async def serve_prescription_page():
    """"""
    html_file = Path(__file__).parent / "static" / "html" / "Prescription.html"
    if html_file.exists():
        return FileResponse(html_file)
    return HTMLResponse("<h1></h1>", status_code=404)

# === API ===
@app.get("/api/system/status")
async def system_status():
    """"""
    data_info = data_persistence.get_data_info()

    return {
        "system": "",
        "version": "2.0.0",
        "status": "",
        "architecture": "API",
        "persistence": "JSON",
        "components": {
            "medicine_api": "API",
            "prescription_api": "API",
            "data_persistence": ""
        },
        "statistics": {
            "total_medicines": len(medicine_api.medicines_db),
            "detailed_medicines": len(medicine_api.detailed_medicines_db),
            "total_prescriptions": len(prescription_api.prescriptions_db)
        },
        "data_files": data_info
    }

@app.post("/api/system/save")
async def manual_save():
    """"""
    success = save_persistent_data()
    return {
        "success": success,
        "message": "" if success else "",
        "timestamp": datetime.now().isoformat()
    }

@app.post("/api/system/backup")
async def create_backup():
    """"""
    backup_path = data_persistence.create_backup()
    return {
        "success": backup_path is not None,
        "backup_path": backup_path,
        "message": "" if backup_path else "",
        "timestamp": datetime.now().isoformat()
    }

@app.get("/api/system/backups")
async def list_backups():
    """"""
    backups = data_persistence.list_backups()
    return {
        "backups": backups,
        "count": len(backups)
    }

@app.post("/api/system/restore/{backup_name}")
async def restore_backup(backup_name: str):
    """"""
    success = data_persistence.restore_backup(backup_name)

    if success:
        #
        load_persistent_data()

    return {
        "success": success,
        "message": "" if success else "",
        "backup_name": backup_name,
        "timestamp": datetime.now().isoformat()
    }

#
def load_persistent_data():
    """"""
    print(" ...")

    try:
        #
        data = data_persistence.load_all_data()

        # 更新API模組的資料
        medicine_api.medicines_db[:] = data['medicines_db']
        medicine_api.next_medicine_id = data['next_medicine_id']
        medicine_api.detailed_medicines_db.clear()
        medicine_api.detailed_medicines_db.update(data['detailed_medicines_db'])
        
        prescription_api.prescriptions_db[:] = data['prescriptions_db']
        prescription_api.prescription_status_db[:] = data['prescription_status_db']
        prescription_api.next_prescription_id = data['next_prescription_id']
        
        # 載入病例資料
        if 'medical_records_db' in data:
            medical_record_api.medical_records_db[:] = data['medical_records_db']
            medical_record_api.next_record_id = data.get('next_record_id', 1)

        total_medicines = len(medicine_api.medicines_db)
        total_detailed = len(medicine_api.detailed_medicines_db)
        total_prescriptions = len(prescription_api.prescriptions_db)

        print(f" :")
        print(f"   - : {total_medicines} ")
        print(f"   - : {total_detailed} ")
        print(f"   - : {total_prescriptions} ")

        return True

    except Exception as e:
        print(f" : {e}")
        print(" ")
        return False

#
def save_persistent_data():
    """"""
    try:
        return data_persistence.save_all_data(
            medicine_api.medicines_db,
            medicine_api.next_medicine_id,
            medicine_api.detailed_medicines_db,
            prescription_api.prescriptions_db,
            prescription_api.prescription_status_db,
            prescription_api.next_prescription_id
        )
    except Exception as e:
        print(f" : {e}")
        return False

#
async def auto_save_task():
    """5"""
    while True:
        try:
            await asyncio.sleep(300)  # 5
            success = save_persistent_data()
            if success:
                print(f"  - {datetime.now().strftime('%H:%M:%S')}")
            else:
                print(f"  - {datetime.now().strftime('%H:%M:%S')}")
        except Exception as e:
            print(f" : {e}")

#
@app.on_event("startup")
async def startup_event():
    print("=" * 60)
    print("  - ...")
    print("=" * 60)

    #
    if not load_persistent_data():
        #
        init_test_data()

    #
    asyncio.create_task(auto_save_task())
    print("⏰ 5")

    print("\n ")
    print(" :")
    print("   : http://localhost:8000")
    print("   : http://localhost:8000/doctor.html")
    print("   : http://localhost:8000/Medicine.html")
    print("   : http://localhost:8000/Prescription.html")
    print("   API: http://localhost:8000/docs")
    print(" : JSON + ")
    print("=" * 60)

#
@app.on_event("shutdown")
async def shutdown_event():
    print("\n ...")
    success = save_persistent_data()
    if success:
        print(" ")
    else:
        print(" ")
    print(" ")

if __name__ == "__main__":
    uvicorn.run(
        "modular_server:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
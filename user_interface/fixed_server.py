#!/usr/bin/env python3
"""
 -
Hospital Medicine Management System - Enhanced Version

"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from pydantic import BaseModel
from pathlib import Path
from datetime import datetime
from typing import List, Optional, Dict, Any
import uvicorn
import json

# Pydantic
class MedicineBasic(BaseModel):
    name: str
    amount: int
    usage_days: int
    position: str

class MedicineDetailed(BaseModel):
    : Dict[str, str]
    : Dict[str, str]
    : Optional[Dict[str, str]] = {}
    : Dict[str, str]
    : str
    : str
    : Optional[str] = ""
    : Optional[str] = ""
    : Optional[str] = ""
    : Optional[str] = ""
    : Optional[str] = ""

class Patient(BaseModel):
    name: str
    age: int
    gender: str
    phone: str
    address: str
    medical_history: Optional[str] = ""
    current_medications: Optional[List[str]] = []
    allergies: Optional[str] = ""

class PatientRecord(BaseModel):
    patient_id: int
    visit_date: str
    diagnosis: str
    prescribed_medicines: List[str]
    dosage_instructions: str
    doctor_notes: str

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
    title=" - ",
    description="Hospital Medicine Management System - Enhanced",
    version="2.0.0"
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
medicines_db = []
detailed_medicines_db = {}
patients_db = []
patient_records_db = []
prescriptions_db = []
next_medicine_id = 1
next_patient_id = 1
next_record_id = 1
next_prescription_id = 1

#
@app.get("/")
async def root():
    return {
        "message": " - ",
        "status": "",
        "version": "2.0.0",
        "statistics": {
            "medicines": len(medicines_db),
            "detailed_medicines": len(detailed_medicines_db),
            "patients": len(patients_db),
            "records": len(patient_records_db)
        },
        "new_features": [
            "",
            "",
            "",
            "JSON"
        ]
    }

#
@app.get("/api/test")
async def test_api():
    return {
        "status": "success",
        "message": "API",
        "time": datetime.now().isoformat(),
        "system_stats": {
            "medicines": len(medicines_db),
            "detailed_medicines": len(detailed_medicines_db),
            "patients": len(patients_db),
            "records": len(patient_records_db),
            "prescriptions": len(prescriptions_db)
        }
    }

# ===  API ===
@app.get("/api/medicine/")
async def get_all_medicines():
    """"""
    return medicines_db

@app.post("/api/medicine/")
async def create_medicine(medicine: MedicineBasic):
    """"""
    global next_medicine_id
    new_medicine = {
        "id": next_medicine_id,
        "name": medicine.name,
        "amount": medicine.amount,
        "usage_days": medicine.usage_days,
        "position": medicine.position,
        "create_time": datetime.now().isoformat()
    }
    medicines_db.append(new_medicine)
    next_medicine_id += 1
    return new_medicine

@app.put("/api/medicine/{medicine_id}")
async def update_medicine(medicine_id: int, medicine: MedicineBasic):
    """"""
    for i, existing_medicine in enumerate(medicines_db):
        if existing_medicine["id"] == medicine_id:
            medicines_db[i].update({
                "name": medicine.name,
                "amount": medicine.amount,
                "usage_days": medicine.usage_days,
                "position": medicine.position,
                "updated_time": datetime.now().isoformat()
            })
            return medicines_db[i]
    raise HTTPException(status_code=404, detail="")

@app.delete("/api/medicine/{medicine_id}")
async def delete_medicine(medicine_id: int):
    """"""
    global medicines_db
    original_length = len(medicines_db)
    medicines_db = [m for m in medicines_db if m["id"] != medicine_id]
    if len(medicines_db) == original_length:
        raise HTTPException(status_code=404, detail="")
    return {"success": True, "message": "", "id": medicine_id}

# ===  API ===
@app.post("/api/medicine/detailed/")
async def create_detailed_medicine(data: dict):
    """"""
    medicine_name = data.get("medicine_name")
    medicine_data = data.get("medicine_data")

    if not medicine_name or not medicine_data:
        raise HTTPException(status_code=400, detail="")

    detailed_medicines_db[medicine_name] = {
        **medicine_data,
        "created_time": datetime.now().isoformat()
    }
    return {"success": True, "message": f": {medicine_name}"}

@app.get("/api/medicine/detailed/{medicine_name}")
async def get_detailed_medicine(medicine_name: str):
    """"""
    if medicine_name not in detailed_medicines_db:
        raise HTTPException(status_code=404, detail="")
    return detailed_medicines_db[medicine_name]

@app.get("/api/medicine/detailed/")
async def get_all_detailed_medicines():
    """"""
    return detailed_medicines_db

@app.get("/api/medicine/search/code/{code}")
async def search_medicine_by_code(code: str):
    """"""
    results = {}
    code_upper = code.upper()

    for name, data in detailed_medicines_db.items():
        packaging_codes = data.get("", {})
        #
        for code_key, code_value in packaging_codes.items():
            if code_upper in str(code_value).upper():
                results[name] = {
                    **data,
                    "matched_code": {
                        "type": code_key,
                        "value": code_value,
                        "search_term": code
                    }
                }
                break

    if not results:
        raise HTTPException(status_code=404, detail=f" '{code}' ")
    return results

# ===  API ===
@app.get("/api/medicine/integrated/{medicine_name}")
async def get_integrated_medicine_info(medicine_name: str):
    """ ( + )"""
    result = {
        "medicine_name": medicine_name,
        "basic_info": None,
        "detailed_info": None,
        "status": "not_found"
    }

    #
    for medicine in medicines_db:
        if medicine["name"].lower() == medicine_name.lower():
            result["basic_info"] = medicine
            break

    #
    if medicine_name in detailed_medicines_db:
        result["detailed_info"] = detailed_medicines_db[medicine_name]

    #
    if result["basic_info"] and result["detailed_info"]:
        result["status"] = "complete"  #
    elif result["basic_info"]:
        result["status"] = "basic_only"  #
    elif result["detailed_info"]:
        result["status"] = "detailed_only"  #
    else:
        raise HTTPException(status_code=404, detail="")

    return result

# === JSON  ===
@app.get("/api/export/complete")
async def export_complete_system():
    """"""
    export_data = {
        "export_type": "",
        "export_date": datetime.now().isoformat(),
        "system_version": "2.0.0",
        "statistics": {
            "basic_medicines": len(medicines_db),
            "detailed_medicines": len(detailed_medicines_db),
            "patients": len(patients_db),
            "records": len(patient_records_db)
        },
        "data": {
            "basic_medicines": medicines_db,
            "detailed_medicines": detailed_medicines_db,
            "patients": patients_db,
            "patient_records": patient_records_db
        }
    }
    return JSONResponse(
        content=export_data,
        headers={"Content-Disposition": "attachment; filename=complete_system_export.json"}
    )

@app.get("/api/export/medicines/integrated")
async def export_integrated_medicines():
    """"""
    integrated_data = {}

    #
    for medicine in medicines_db:
        name = medicine["name"]
        medicine_export = {
            "": name,
            "": {
                "ID": medicine["id"],
                "": medicine["amount"],
                "": medicine["position"],
                "": medicine["usage_days"],
                "": medicine["create_time"]
            }
        }

        #
        if name in detailed_medicines_db:
            medicine_export[""] = detailed_medicines_db[name]
            medicine_export[""] = "complete"
        else:
            medicine_export[""] = "basic_only"

        integrated_data[name] = medicine_export

    #
    for name in detailed_medicines_db:
        if name not in integrated_data:
            integrated_data[name] = {
                "": name,
                "": detailed_medicines_db[name],
                "": "detailed_only"
            }

    export_data = {
        "export_type": "",
        "total_count": len(integrated_data),
        "export_date": datetime.now().isoformat(),
        "description": "",
        "data": integrated_data
    }

    return JSONResponse(
        content=export_data,
        headers={"Content-Disposition": "attachment; filename=integrated_medicines.json"}
    )

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

@app.get("/api/prescription/doctor/{doctor_name}")
async def get_prescriptions_by_doctor(doctor_name: str):
    """"""
    doctor_prescriptions = [p for p in prescriptions_db if p["doctor_name"] == doctor_name]
    return doctor_prescriptions

# ===  ===
@app.get("/Medicine.html")
async def serve_medicine_page():
    html_file = static_dir / "html" / "Medicine.html"
    if html_file.exists():
        return FileResponse(html_file, media_type='text/html')
    else:
        return JSONResponse(content={"error": "Medicine.html not found"}, status_code=404)

@app.get("/Prescription.html")
async def serve_prescription_page():
    html_file = static_dir / "html" / "Prescription.html"
    if html_file.exists():
        return FileResponse(html_file, media_type='text/html')
    else:
        return JSONResponse(content={"error": "Prescription.html not found"}, status_code=404)

@app.get("/doctor.html")
async def serve_doctor_page():
    html_file = static_dir / "html" / "doctor.html"
    if html_file.exists():
        return FileResponse(html_file, media_type='text/html')
    else:
        return JSONResponse(content={"error": "doctor.html not found"}, status_code=404)

#
def init_test_data():
    """"""
    global medicines_db, detailed_medicines_db, patients_db, patient_records_db
    global next_medicine_id, next_patient_id, next_record_id

    #
    test_medicines = [
        {
            "id": 1,
            "name": "",
            "amount": 100,
            "usage_days": 30,
            "position": "A1-01",
            "create_time": datetime.now().isoformat()
        },
        {
            "id": 2,
            "name": "",
            "amount": 50,
            "usage_days": 60,
            "position": "A1-02",
            "create_time": datetime.now().isoformat()
        },
        {
            "id": 3,
            "name": "",
            "amount": 25,
            "usage_days": 90,
            "position": "B2-03",
            "create_time": datetime.now().isoformat()
        }
    ]
    medicines_db.extend(test_medicines)
    next_medicine_id = 4

    #
    detailed_medicines_db[""] = {
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
        "": "--AV",
        "": "",
        "": "1. 2. 3.  COPD 4. 5.  Clonidine",
        "": "C D ",
        "": "",
        "created_time": datetime.now().isoformat()
    }

    #
    detailed_medicines_db[""] = {
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
        "": "",
        "created_time": datetime.now().isoformat()
    }

if __name__ == "__main__":
    print(" - ")
    print("=" * 70)
    print("...")
    print(":  +  + ")
    print(": http://localhost:8000")
    print("API: http://localhost:8000/docs")
    print(": http://localhost:8000/Medicine.html")
    print("API: http://localhost:8000/api/test")
    print(": http://localhost:8000/api/export/complete")
    print("=" * 70)

    #
    init_test_data()
    print(f":")
    print(f"  : {len(medicines_db)} ")
    print(f"  : {len(detailed_medicines_db)} ")

    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    except KeyboardInterrupt:
        print("\n")
    except Exception as e:
        print(f": : {e}")
        exit(1)
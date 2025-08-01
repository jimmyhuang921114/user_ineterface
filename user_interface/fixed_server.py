#!/usr/bin/env python3
"""
醫院藥物管理系統 - 增強版
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

# Pydantic 資料模型
class MedicineBasic(BaseModel):
    name: str
    amount: int
    usage_days: int
    position: str

class MedicineDetailed(BaseModel):
    basic_info: Dict[str, str]
    appearance: Dict[str, str]
    packaging: Optional[Dict[str, str]] = {}
    ingredients: Dict[str, str]
    manufacturer: str
    indications: str
    contraindications: Optional[str] = ""
    side_effects: Optional[str] = ""
    dosage: Optional[str] = ""
    storage: Optional[str] = ""
    warnings: Optional[str] = ""

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

# 創建 FastAPI 應用
app = FastAPI(
    title="醫院藥物管理系統 - 增強版",
    description="Hospital Medicine Management System - Enhanced",
    version="2.0.0"
)

# CORS 設定
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 靜態文件設定
base_dir = Path(__file__).resolve().parent
static_dir = base_dir / "static"

# 掛載靜態文件
try:
    if (static_dir / "css").exists():
        app.mount("/css", StaticFiles(directory=static_dir / "css"), name="css")
    if (static_dir / "js").exists():
        app.mount("/js", StaticFiles(directory=static_dir / "js"), name="js")
    if (static_dir / "html").exists():
        app.mount("/html", StaticFiles(directory=static_dir / "html"), name="html")
except Exception as e:
    print(f"靜態文件掛載錯誤: {e}")

# 初始化資料庫
medicines_db = []
detailed_medicines_db = {}
patients_db = []
patient_records_db = []
next_medicine_id = 1
next_patient_id = 1
next_record_id = 1

# 根路徑
@app.get("/")
async def root():
    return {
        "message": "醫院藥物管理系統 - 增強版",
        "status": "運行中",
        "version": "2.0.0",
        "statistics": {
            "medicines": len(medicines_db),
            "detailed_medicines": len(detailed_medicines_db),
            "patients": len(patients_db),
            "records": len(patient_records_db)
        },
        "new_features": [
            "增強的藥物管理",
            "詳細藥物資訊",
            "患者管理系統",
            "JSON 導出功能"
        ]
    }

# API 測試端點
@app.get("/api/test")
async def test_api():
    return {
        "status": "success",
        "message": "API 運行正常",
        "time": datetime.now().isoformat(),
        "system_stats": {
            "medicines": len(medicines_db),
            "patients": len(patients_db),
            "records": len(patient_records_db)
        }
    }

# === 基本藥物 API ===
@app.get("/api/medicine/")
async def get_all_medicines():
    """獲取所有基本藥物資料"""
    return medicines_db

@app.post("/api/medicine/")
async def create_medicine(medicine: MedicineBasic):
    """創建新的基本藥物"""
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
    """更新基本藥物資料"""
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
    raise HTTPException(status_code=404, detail="藥物未找到")

@app.delete("/api/medicine/{medicine_id}")
async def delete_medicine(medicine_id: int):
    """刪除基本藥物"""
    global medicines_db
    original_length = len(medicines_db)
    medicines_db = [m for m in medicines_db if m["id"] != medicine_id]
    if len(medicines_db) == original_length:
        raise HTTPException(status_code=404, detail="藥物未找到")
    return {"success": True, "message": "藥物刪除成功", "id": medicine_id}

# === 詳細藥物 API ===
@app.post("/api/medicine/detailed/")
async def create_detailed_medicine(medicine_name: str, medicine_data: MedicineDetailed):
    """創建詳細藥物資料"""
    detailed_medicines_db[medicine_name] = {
        **medicine_data.dict(),
        "created_time": datetime.now().isoformat()
    }
    return {"success": True, "message": f"詳細藥物資料創建成功: {medicine_name}"}

@app.get("/api/medicine/detailed/{medicine_name}")
async def get_detailed_medicine(medicine_name: str):
    """獲取特定詳細藥物資料"""
    if medicine_name not in detailed_medicines_db:
        raise HTTPException(status_code=404, detail="詳細藥物資料未找到")
    return detailed_medicines_db[medicine_name]

@app.get("/api/medicine/detailed/")
async def get_all_detailed_medicines():
    """獲取所有詳細藥物資料"""
    return detailed_medicines_db

@app.get("/api/medicine/search/detailed/{query}")
async def search_detailed_medicines(query: str):
    """搜尋詳細藥物資料"""
    results = {}
    query_lower = query.lower()
    for name, data in detailed_medicines_db.items():
        if (query_lower in name.lower() or 
            query_lower in data.get("basic_info", {}).get("name", "").lower() or 
            query_lower in data.get("appearance", {}).get("description", "").lower()):
            results[name] = data
    if not results:
        raise HTTPException(status_code=404, detail="未找到匹配的藥物")
    return results

@app.get("/api/medicine/search/code/{code}")
async def search_medicine_by_code(code: str):
    """根據代碼搜尋藥物"""
    results = {}
    code_upper = code.upper()
    for name, data in detailed_medicines_db.items():
        packaging_codes = data.get("packaging", {})  # 包裝代碼
        for code_key, code_value in packaging_codes.items():
            if code_upper in code_value.upper():
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
        raise HTTPException(status_code=404, detail=f"未找到代碼為 '{code}' 的藥物")
    return results

@app.get("/api/medicine/search/exact-code/{code}")
async def search_medicine_by_exact_code(code: str):
    """根據精確代碼搜尋藥物"""
    results = {}
    for name, data in detailed_medicines_db.items():
        packaging_codes = data.get("packaging", {})
        for code_key, code_value in packaging_codes.items():
            if code == code_value:
                results[name] = {
                    **data,
                    "matched_code": {
                        "type": code_key,
                        "value": code_value,
                        "match_type": "exact"
                    }
                }
                break
    if not results:
        raise HTTPException(status_code=404, detail=f"未找到代碼為 '{code}' 的藥物")
    return results

@app.get("/api/medicine/codes/")
async def get_all_medicine_codes():
    """獲取所有藥物代碼"""
    all_codes = {}
    for name, data in detailed_medicines_db.items():
        packaging_codes = data.get("packaging", {})
        if packaging_codes:
            all_codes[name] = {
                "medicine_name": data.get("basic_info", {}).get("name", name),
                "manufacturer": data.get("basic_info", {}).get("manufacturer", ""),
                "codes": packaging_codes
            }
    return {
        "total_medicines": len(all_codes),
        "medicines_with_codes": all_codes
    }

# === 患者 API ===
@app.get("/api/patients/")
async def get_all_patients():
    """獲取所有患者資料"""
    return patients_db

@app.post("/api/patients/")
async def create_patient(patient: Patient):
    """創建新患者"""
    global next_patient_id
    new_patient = {
        "id": next_patient_id,
        **patient.dict(),
        "created_time": datetime.now().isoformat()
    }
    patients_db.append(new_patient)
    next_patient_id += 1
    return new_patient

@app.get("/api/patients/{patient_id}")
async def get_patient(patient_id: int):
    """獲取特定患者資料"""
    for patient in patients_db:
        if patient["id"] == patient_id:
            return patient
    raise HTTPException(status_code=404, detail="患者未找到")

@app.put("/api/patients/{patient_id}")
async def update_patient(patient_id: int, patient_update: Patient):
    """更新患者資料"""
    for i, patient in enumerate(patients_db):
        if patient["id"] == patient_id:
            patients_db[i].update({
                **patient_update.dict(),
                "updated_time": datetime.now().isoformat()
            })
            return patients_db[i]
    raise HTTPException(status_code=404, detail="患者未找到")

@app.delete("/api/patients/{patient_id}")
async def delete_patient(patient_id: int):
    """刪除患者"""
    global patients_db, patient_records_db
    # 刪除患者
    original_length = len(patients_db)
    patients_db = [p for p in patients_db if p["id"] != patient_id]
    if len(patients_db) == original_length:
        raise HTTPException(status_code=404, detail="患者未找到")
    # 同時刪除相關記錄
    patient_records_db = [r for r in patient_records_db if r["patient_id"] != patient_id]
    return {"success": True, "message": "患者刪除成功", "patient_id": patient_id}

# === 患者記錄 API ===
@app.get("/api/records/")
async def get_all_records():
    """獲取所有患者記錄"""
    return patient_records_db

@app.post("/api/records/")
async def create_record(record: PatientRecord):
    """創建新患者記錄"""
    global next_record_id
    new_record = {
        "id": next_record_id,
        **record.dict(),
        "created_time": datetime.now().isoformat()
    }
    patient_records_db.append(new_record)
    next_record_id += 1
    return new_record

@app.get("/api/records/patient/{patient_id}")
async def get_patient_records(patient_id: int):
    """獲取特定患者的所有記錄"""
    records = [r for r in patient_records_db if r["patient_id"] == patient_id]
    return records

@app.delete("/api/records/{record_id}")
async def delete_record(record_id: int):
    """刪除患者記錄"""
    global patient_records_db
    original_length = len(patient_records_db)
    patient_records_db = [r for r in patient_records_db if r["id"] != record_id]
    if len(patient_records_db) == original_length:
        raise HTTPException(status_code=404, detail="記錄未找到")
    return {"success": True, "message": "記錄刪除成功", "record_id": record_id}

# === 整合藥物資訊 API ===
@app.get("/api/medicine/integrated/{medicine_name}")
async def get_integrated_medicine_info(medicine_name: str):
    """獲取整合的藥物資訊 (基本 + 詳細)"""
    result = {
        "medicine_name": medicine_name,
        "basic_info": None,
        "detailed_info": None,
        "status": "not_found"
    }
    
    # 查找基本資訊
    for medicine in medicines_db:
        if medicine["name"].lower() == medicine_name.lower():
            result["basic_info"] = medicine
            break
    
    # 查找詳細資訊
    if medicine_name in detailed_medicines_db:
        result["detailed_info"] = detailed_medicines_db[medicine_name]
    
    # 設定狀態
    if result["basic_info"] and result["detailed_info"]:
        result["status"] = "complete"
    elif result["basic_info"]:
        result["status"] = "basic_only"
    elif result["detailed_info"]:
        result["status"] = "detailed_only"
    else:
        raise HTTPException(status_code=404, detail="藥物未找到")
    
    return result

@app.get("/api/medicine/integrated/")
async def get_all_integrated_medicines():
    """獲取所有整合的藥物資訊"""
    integrated_medicines = {}
    
    # 處理基本藥物
    for medicine in medicines_db:
        name = medicine["name"]
        integrated_medicines[name] = {
            "basic_info": medicine,
            "detailed_info": detailed_medicines_db.get(name, None),
            "status": "complete" if name in detailed_medicines_db else "basic_only"
        }
    
    # 處理只有詳細資訊的藥物
    for name in detailed_medicines_db:
        if name not in integrated_medicines:
            integrated_medicines[name] = {
                "basic_info": None,
                "detailed_info": detailed_medicines_db[name],
                "status": "detailed_only"
            }
    
    return {
        "total_medicines": len(integrated_medicines),
        "integrated_medicines": integrated_medicines
    }

# === JSON 導出 API ===
@app.get("/api/export/medicines/basic")
async def export_basic_medicines():
    """導出基本藥物資料為 JSON"""
    export_data = {
        "export_type": "基本藥物資料",
        "total_count": len(medicines_db),
        "export_date": datetime.now().isoformat(),
        "data": medicines_db
    }
    return JSONResponse(
        content=export_data,
        headers={"Content-Disposition": "attachment; filename=basic_medicines.json"}
    )

@app.get("/api/export/medicines/detailed")
async def export_detailed_medicines():
    """導出詳細藥物資料為 JSON"""
    export_data = {
        "export_type": "詳細藥物資料",
        "total_count": len(detailed_medicines_db),
        "export_date": datetime.now().isoformat(),
        "data": detailed_medicines_db
    }
    return JSONResponse(
        content=export_data,
        headers={"Content-Disposition": "attachment; filename=detailed_medicines.json"}
    )

@app.get("/api/export/patients")
async def export_patients():
    """導出患者資料為 JSON"""
    export_data = {
        "export_type": "患者資料",
        "total_count": len(patients_db),
        "export_date": datetime.now().isoformat(),
        "data": patients_db
    }
    return JSONResponse(
        content=export_data,
        headers={"Content-Disposition": "attachment; filename=patients.json"}
    )

@app.get("/api/export/records")
async def export_records():
    """導出患者記錄為 JSON"""
    export_data = {
        "export_type": "患者記錄",
        "total_count": len(patient_records_db),
        "export_date": datetime.now().isoformat(),
        "data": patient_records_db
    }
    return JSONResponse(
        content=export_data,
        headers={"Content-Disposition": "attachment; filename=patient_records.json"}
    )

@app.get("/api/export/medicines/integrated")
async def export_integrated_medicines():
    """導出整合藥物資料 (基本 + 詳細)"""
    integrated_data = {}
    
    # 獲取所有整合資料
    all_integrated = await get_all_integrated_medicines()
    for name, data in all_integrated["integrated_medicines"].items():
        medicine_export = {
            "藥物名稱": name,
            "基本資訊": "有" if data["basic_info"] else "無",
            "詳細資訊": "有" if data["detailed_info"] else "無",
            "狀態": data["status"]
        }
        
        # 添加基本資訊
        if data["basic_info"]:
            medicine_export["基本資料"] = {
                "ID": data["basic_info"]["id"],
                "數量": data["basic_info"]["amount"],
                "位置": data["basic_info"]["position"],
                "使用天數": data["basic_info"]["usage_days"],
                "建立時間": data["basic_info"]["create_time"]
            }
        
        # 添加詳細資訊
        if data["detailed_info"]:
            medicine_export["詳細資料"] = data["detailed_info"]
        
        integrated_data[name] = medicine_export
    
    export_data = {
        "export_type": "整合藥物資料",
        "total_count": len(integrated_data),
        "export_date": datetime.now().isoformat(),
        "description": "包含基本和詳細藥物資訊的完整資料",
        "data": integrated_data
    }
    return JSONResponse(
        content=export_data,
        headers={"Content-Disposition": "attachment; filename=integrated_medicines.json"}
    )

@app.get("/api/export/complete")
async def export_complete_system():
    """導出完整系統資料"""
    export_data = {
        "export_type": "完整系統資料",
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

# === 網頁路由 ===
@app.get("/Medicine.html")
async def serve_medicine_page():
    html_file = static_dir / "html" / "Medicine.html"
    if html_file.exists():
        return FileResponse(html_file, media_type='text/html')
    else:
        return JSONResponse(content={"error": "Medicine.html not found"}, status_code=404)

@app.get("/Patients.html")
async def serve_patients_page():
    html_file = static_dir / "html" / "Patients.html"
    if html_file.exists():
        return FileResponse(html_file, media_type='text/html')
    else:
        return JSONResponse(content={"error": "Patients.html not found"}, status_code=404)

@app.get("/Records.html")
async def serve_records_page():
    html_file = static_dir / "html" / "Records.html"
    if html_file.exists():
        return FileResponse(html_file, media_type='text/html')
    else:
        return JSONResponse(content={"error": "Records.html not found"}, status_code=404)

# 初始化測試資料
def init_test_data():
    """初始化測試資料"""
    global medicines_db, detailed_medicines_db, patients_db, patient_records_db
    global next_medicine_id, next_patient_id, next_record_id
    
    # 測試基本藥物
    test_medicines = [
        {
            "id": 1,
            "name": "阿斯匹靈",
            "amount": 100,
            "usage_days": 30,
            "position": "A1-01",
            "create_time": datetime.now().isoformat()
        },
        {
            "id": 2,
            "name": "普拿疼",
            "amount": 50,
            "usage_days": 60,
            "position": "A1-02",
            "create_time": datetime.now().isoformat()
        }
    ]
    medicines_db.extend(test_medicines)
    next_medicine_id = 3
    
    # 測試詳細藥物
    detailed_medicines_db["普拿疼"] = {
        "basic_info": {
            "name": "普拿疼 (Propranolol)",
            "manufacturer": "台灣拜耳",
            "strength": "10"
        },
        "appearance": {
            "color": "白色",
            "shape": "圓形"
        },
        "packaging": {
            "1": "202801",
            "2": "TP071014",
            "3": "009102"
        },
        "ingredients": {
            "active": "普拿疼",
            "inactive": "Propranolol HCl"
        },
        "manufacturer": "台灣拜耳",
        "indications": "--AV",
        "contraindications": "",
        "side_effects": "",
        "dosage": "1. 每日三次 2. 飯後服用 3. 避免 COPD 4. 注意血壓 5. 避免 Clonidine",
        "storage": "C D 級",
        "warnings": "",
        "created_time": datetime.now().isoformat()
    }
    
    detailed_medicines_db["利他能"] = {
        "basic_info": {
            "name": "利他能 (Antipsychotics)",
            "manufacturer": "5 毫克",
            "form": "口服 (Oral use)",
            "unit": "1 special pill"
        },
        "appearance": {
            "color": "藍色",
            "shape": "橢圓形"
        },
        "packaging": {
            "expiry": "2027/08/02"
        },
        "ingredients": "",
        "manufacturer": "",
        "indications": "",
        "contraindications": "",
        "side_effects": "",
        "dosage": "",
        "storage": "",
        "warnings": "",
        "created_time": datetime.now().isoformat()
    }
    
    # 測試患者
    test_patients = [
        {
            "id": 1,
            "name": "張三",
            "age": 45,
            "gender": "男",
            "phone": "0912345678",
            "address": "台北市信義區",
            "medical_history": "高血壓、糖尿病",
            "current_medications": ["普拿疼"],
            "allergies": "無",
            "created_time": datetime.now().isoformat()
        }
    ]
    patients_db.extend(test_patients)
    next_patient_id = 2
    
    # 測試記錄
    test_records = [
        {
            "id": 1,
            "patient_id": 1,
            "visit_date": datetime.now().isoformat(),
            "diagnosis": "高血壓",
            "prescribed_medicines": ["普拿疼 10mg"],
            "dosage_instructions": "每日三次，飯後服用",
            "doctor_notes": "定期監測血壓",
            "created_time": datetime.now().isoformat()
        }
    ]
    patient_records_db.extend(test_records)
    next_record_id = 2

if __name__ == "__main__":
    print("醫院藥物管理系統 - 增強版")
    print("=" * 70)
    print("系統啟動中...")
    print("功能: 基本藥物 + 詳細藥物 + 患者管理")
    print("網址: http://localhost:8000")
    print("API文檔: http://localhost:8000/docs")
    print("藥物管理: http://localhost:8000/Medicine.html")
    print("患者管理: http://localhost:8000/Patients.html")
    print("記錄管理: http://localhost:8000/Records.html")
    print("API測試: http://localhost:8000/api/test")
    print("完整導出: http://localhost:8000/api/export/complete")
    print("=" * 70)
    
    # 初始化測試資料
    init_test_data()
    print(f"初始化完成:")
    print(f" 基本藥物: {len(medicines_db)} 筆")
    print(f" 詳細藥物: {len(detailed_medicines_db)} 筆")
    print(f" 患者資料: {len(patients_db)} 筆")
    print(f" 患者記錄: {len(patient_records_db)} 筆")
    
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    except KeyboardInterrupt:
        print("\n系統已停止")
    except Exception as e:
        print(f"啟動錯誤: {e}")
        exit(1)
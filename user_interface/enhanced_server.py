#!/usr/bin/env python3
"""
醫院藥物管理系統 - 增強版本
Hospital Medicine Management System - Enhanced Version
支援病人管理和詳細藥物資訊
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
    基本資訊: Dict[str, str]
    外觀: Dict[str, str]
    包裝編號: Dict[str, str]
    其他資訊: Dict[str, str]
    適應症: str
    可能的副作用: str
    使用說明: str
    注意事項: str
    懷孕分級: str
    儲存條件: str

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

# 建立 FastAPI 應用
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
    print(f"警告: 靜態文件掛載失敗: {e}")

# 資料庫 (記憶體儲存)
medicines_db = []
detailed_medicines_db = {}
patients_db = []
patient_records_db = []
next_medicine_id = 1
next_patient_id = 1
next_record_id = 1

# 根路由
@app.get("/")
async def root():
    return {
        "message": "🏥 醫院藥物管理系統 - 增強版",
        "status": "✅ 運行中",
        "version": "2.0.0",
        "statistics": {
            "medicines": len(medicines_db),
            "detailed_medicines": len(detailed_medicines_db),
            "patients": len(patients_db),
            "records": len(patient_records_db)
        },
        "new_features": [
            "病人資料管理",
            "詳細藥物資訊",
            "病例記錄系統",
            "增強型JSON導出"
        ]
    }

# 測試端點
@app.get("/api/test")
async def test_api():
    return {
        "status": "success",
        "message": "✅ API連接正常",
        "time": datetime.now().isoformat(),
        "system_stats": {
            "medicines": len(medicines_db),
            "patients": len(patients_db),
            "records": len(patient_records_db)
        }
    }

# === 基本藥物管理 API ===
@app.get("/api/medicine/")
async def get_all_medicines():
    """獲取所有基本藥物資訊"""
    return medicines_db

@app.post("/api/medicine/")
async def create_medicine(medicine: MedicineBasic):
    """新增基本藥物"""
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

@app.delete("/api/medicine/{medicine_id}")
async def delete_medicine(medicine_id: int):
    """刪除藥物"""
    global medicines_db
    original_length = len(medicines_db)
    medicines_db = [m for m in medicines_db if m["id"] != medicine_id]
    
    if len(medicines_db) == original_length:
        raise HTTPException(status_code=404, detail="藥物未找到")
    
    return {"success": True, "message": "藥物刪除成功", "id": medicine_id}

# === 詳細藥物資訊 API ===
@app.post("/api/medicine/detailed/")
async def create_detailed_medicine(medicine_name: str, medicine_data: MedicineDetailed):
    """新增詳細藥物資訊"""
    detailed_medicines_db[medicine_name] = {
        **medicine_data.dict(),
        "created_time": datetime.now().isoformat()
    }
    return {"success": True, "message": f"詳細藥物資訊已新增: {medicine_name}"}

@app.get("/api/medicine/detailed/{medicine_name}")
async def get_detailed_medicine(medicine_name: str):
    """獲取特定藥物的詳細資訊"""
    if medicine_name not in detailed_medicines_db:
        raise HTTPException(status_code=404, detail="找不到該藥物的詳細資訊")
    
    return detailed_medicines_db[medicine_name]

@app.get("/api/medicine/detailed/")
async def get_all_detailed_medicines():
    """獲取所有詳細藥物資訊"""
    return detailed_medicines_db

@app.get("/api/medicine/search/detailed/{query}")
async def search_detailed_medicines(query: str):
    """搜尋詳細藥物資訊"""
    results = {}
    query_lower = query.lower()
    
    for name, data in detailed_medicines_db.items():
        if (query_lower in name.lower() or 
            query_lower in data.get("基本資訊", {}).get("名稱", "").lower() or
            query_lower in data.get("其他資訊", {}).get("藥物全名", "").lower()):
            results[name] = data
    
    if not results:
        raise HTTPException(status_code=404, detail="找不到匹配的藥物")
    
    return results

@app.get("/api/medicine/search/code/{code}")
async def search_medicine_by_code(code: str):
    """根據包裝編號搜尋藥物"""
    results = {}
    code_upper = code.upper()
    
    for name, data in detailed_medicines_db.items():
        packaging_codes = data.get("包裝編號", {})
        
        # 檢查所有包裝編號
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
        raise HTTPException(status_code=404, detail=f"找不到包裝編號包含 '{code}' 的藥物")
    
    return results

@app.get("/api/medicine/search/exact-code/{code}")
async def search_medicine_by_exact_code(code: str):
    """根據完整包裝編號精確搜尋藥物"""
    results = {}
    
    for name, data in detailed_medicines_db.items():
        packaging_codes = data.get("包裝編號", {})
        
        # 檢查完全匹配的編號
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
        raise HTTPException(status_code=404, detail=f"找不到包裝編號為 '{code}' 的藥物")
    
    return results

@app.get("/api/medicine/codes/")
async def get_all_medicine_codes():
    """獲取所有藥物的包裝編號"""
    all_codes = {}
    
    for name, data in detailed_medicines_db.items():
        packaging_codes = data.get("包裝編號", {})
        if packaging_codes:
            all_codes[name] = {
                "medicine_name": data.get("基本資訊", {}).get("名稱", name),
                "manufacturer": data.get("基本資訊", {}).get("廠商", ""),
                "codes": packaging_codes
            }
    
    return {
        "total_medicines": len(all_codes),
        "medicines_with_codes": all_codes
    }

# === 病人管理 API ===
@app.get("/api/patients/")
async def get_all_patients():
    """獲取所有病人"""
    return patients_db

@app.post("/api/patients/")
async def create_patient(patient: Patient):
    """新增病人"""
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
    """獲取特定病人資訊"""
    for patient in patients_db:
        if patient["id"] == patient_id:
            return patient
    raise HTTPException(status_code=404, detail="病人未找到")

@app.put("/api/patients/{patient_id}")
async def update_patient(patient_id: int, patient_update: Patient):
    """更新病人資訊"""
    for i, patient in enumerate(patients_db):
        if patient["id"] == patient_id:
            patients_db[i].update({
                **patient_update.dict(),
                "updated_time": datetime.now().isoformat()
            })
            return patients_db[i]
    raise HTTPException(status_code=404, detail="病人未找到")

@app.delete("/api/patients/{patient_id}")
async def delete_patient(patient_id: int):
    """刪除病人"""
    global patients_db, patient_records_db
    
    # 刪除病人資料
    original_length = len(patients_db)
    patients_db = [p for p in patients_db if p["id"] != patient_id]
    
    if len(patients_db) == original_length:
        raise HTTPException(status_code=404, detail="病人未找到")
    
    # 同時刪除相關病例記錄
    patient_records_db = [r for r in patient_records_db if r["patient_id"] != patient_id]
    
    return {"success": True, "message": "病人及相關記錄已刪除", "patient_id": patient_id}

# === 病例記錄 API ===
@app.get("/api/records/")
async def get_all_records():
    """獲取所有病例記錄"""
    return patient_records_db

@app.post("/api/records/")
async def create_record(record: PatientRecord):
    """新增病例記錄"""
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
    """獲取特定病人的所有記錄"""
    records = [r for r in patient_records_db if r["patient_id"] == patient_id]
    return records

@app.delete("/api/records/{record_id}")
async def delete_record(record_id: int):
    """刪除病例記錄"""
    global patient_records_db
    original_length = len(patient_records_db)
    patient_records_db = [r for r in patient_records_db if r["id"] != record_id]
    
    if len(patient_records_db) == original_length:
        raise HTTPException(status_code=404, detail="病例記錄未找到")
    
    return {"success": True, "message": "病例記錄已刪除", "record_id": record_id}

# === JSON 導出功能 ===
@app.get("/api/export/medicines/basic")
async def export_basic_medicines():
    """導出基本藥物資訊"""
    export_data = {
        "export_type": "基本藥物資訊",
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
    """導出詳細藥物資訊"""
    export_data = {
        "export_type": "詳細藥物資訊",
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
    """導出病人資料"""
    export_data = {
        "export_type": "病人資料",
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
    """導出病例記錄"""
    export_data = {
        "export_type": "病例記錄",
        "total_count": len(patient_records_db),
        "export_date": datetime.now().isoformat(),
        "data": patient_records_db
    }
    
    return JSONResponse(
        content=export_data,
        headers={"Content-Disposition": "attachment; filename=patient_records.json"}
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

# === 前端頁面路由 ===
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
    
    # 基本藥物資料
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
            "name": "心律錠",
            "amount": 50,
            "usage_days": 60,
            "position": "A1-02",
            "create_time": datetime.now().isoformat()
        }
    ]
    medicines_db.extend(test_medicines)
    next_medicine_id = 3
    
    # 詳細藥物資料
    detailed_medicines_db["心律錠"] = {
        "基本資訊": {
            "名稱": "心律錠(Propranolol)",
            "廠商": "生達",
            "劑量": "10毫克"
        },
        "外觀": {
            "顏色": "明紫紅",
            "形狀": "圓扁形"
        },
        "包裝編號": {
            "編號1": "202801",
            "編號2": "TP071014",
            "編號3": "衛署藥製字第009102號"
        },
        "其他資訊": {
            "公司全名": "生達化學製藥股份有限公司",
            "藥物全名": "Propranolol HCl"
        },
        "適應症": "狹心症、不整律（上心室性不整律、心室性心搏過速）、原發性及腎性高血壓、偏頭痛控制、原發性震顫控制、焦慮性心搏過速、甲狀腺毒症輔助劑、親鉻細胞瘤",
        "可能的副作用": "常見-心智混亂、疲憊、睏倦、心跳徐緩、雙手皮膚感覺異常；偶見-發燒、體重減輕、關節痛、陽萎、性慾降低、暈眩、失眠、心悸、AV阻斷、口乾、噁心、胃灼熱、支氣管痙攣",
        "使用說明": "用法用量請遵照醫囑；除特別要求外，一般建議於飯後服用。",
        "注意事項": "1. 本藥會掩飾低血糖症狀並延長低血糖時間；2. 突然停藥可能引發戒斷症狀；3. 氣喘或 COPD 禁用；4. 服藥期間勿駕車或操作危險機械；5. 禁忌合併 Clonidine，以免血壓急劇下降。",
        "懷孕分級": "C級；若於妊娠第二或第三期則為 D 級。",
        "儲存條件": "請連同藥袋存放於緊密容器內，室溫乾燥避光；避免孩童取得。",
        "created_time": datetime.now().isoformat()
    }
    
    # 測試病人資料
    test_patients = [
        {
            "id": 1,
            "name": "張小明",
            "age": 45,
            "gender": "男",
            "phone": "0912345678",
            "address": "台北市信義區",
            "medical_history": "高血壓病史",
            "current_medications": ["心律錠"],
            "allergies": "無",
            "created_time": datetime.now().isoformat()
        }
    ]
    patients_db.extend(test_patients)
    next_patient_id = 2
    
    # 測試病例記錄
    test_records = [
        {
            "id": 1,
            "patient_id": 1,
            "visit_date": datetime.now().isoformat(),
            "diagnosis": "高血壓",
            "prescribed_medicines": ["心律錠 10mg"],
            "dosage_instructions": "每日一次，飯後服用",
            "doctor_notes": "血壓控制良好，繼續服藥",
            "created_time": datetime.now().isoformat()
        }
    ]
    patient_records_db.extend(test_records)
    next_record_id = 2

if __name__ == "__main__":
    print("🏥 醫院藥物管理系統 - 增強版")
    print("=" * 70)
    print("🚀 正在啟動伺服器...")
    print("🆕 新功能: 病人管理 + 詳細藥物資訊 + 病例記錄")
    print("🌐 伺服器地址: http://localhost:8000")
    print("📱 API文檔: http://localhost:8000/docs")
    print("💊 藥物管理: http://localhost:8000/Medicine.html")
    print("👥 病人管理: http://localhost:8000/Patients.html")
    print("📋 病例記錄: http://localhost:8000/Records.html")
    print("🧪 API測試: http://localhost:8000/api/test")
    print("📦 完整導出: http://localhost:8000/api/export/complete")
    print("=" * 70)
    
    # 初始化測試資料
    init_test_data()
    print(f"✅ 已載入測試資料:")
    print(f"   💊 基本藥物: {len(medicines_db)} 個")
    print(f"   🔬 詳細藥物: {len(detailed_medicines_db)} 個")
    print(f"   👥 病人: {len(patients_db)} 個")
    print(f"   📋 病例: {len(patient_records_db)} 個")
    
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    except KeyboardInterrupt:
        print("\n👋 伺服器已停止")
    except Exception as e:
        print(f"❌ 伺服器啟動失敗: {e}")
        exit(1)
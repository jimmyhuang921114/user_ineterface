#!/usr/bin/env python3
"""
藥物管理API模組
Medicine Management API Module
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime

# 創建API路由器
router = APIRouter(prefix="/api/medicine", tags=["藥物管理"])

# Pydantic模型
class MedicineBasic(BaseModel):
    name: str
    amount: int
    usage_days: int
    position: str

class MedicineDetailed(BaseModel):
    基本資訊: Dict[str, str]
    外觀: Dict[str, str]
    包裝編號: Optional[Dict[str, str]] = {}
    其他資訊: Dict[str, str]
    適應症: str
    可能的副作用: str
    使用說明: Optional[str] = ""
    注意事項: Optional[str] = ""
    懷孕分級: Optional[str] = ""
    儲存條件: Optional[str] = ""
    條碼: Optional[str] = ""

# 資料儲存 (這些變數將被主伺服器導入)
medicines_db = []
detailed_medicines_db = {}
next_medicine_id = 1

# === 基本藥物管理API ===
@router.get("/")
async def get_all_medicines():
    """獲取所有基本藥物資訊"""
    return medicines_db

@router.post("/")
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

@router.put("/{medicine_id}")
async def update_medicine(medicine_id: int, medicine: MedicineBasic):
    """更新基本藥物"""
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

@router.delete("/{medicine_id}")
async def delete_medicine(medicine_id: int):
    """刪除藥物"""
    global medicines_db
    original_length = len(medicines_db)
    medicines_db = [m for m in medicines_db if m["id"] != medicine_id]
    if len(medicines_db) == original_length:
        raise HTTPException(status_code=404, detail="藥物未找到")
    return {"success": True, "message": "藥物刪除成功", "id": medicine_id}

# === 詳細藥物資訊API ===
@router.post("/detailed/")
async def create_detailed_medicine(data: dict):
    """新增詳細藥物資訊"""
    medicine_name = data.get("medicine_name")
    medicine_data = data.get("medicine_data")
    
    if not medicine_name or not medicine_data:
        raise HTTPException(status_code=400, detail="缺少必要資料")
    
    detailed_medicines_db[medicine_name] = {
        **medicine_data,
        "created_time": datetime.now().isoformat()
    }
    return {"success": True, "message": f"詳細藥物資訊已新增: {medicine_name}"}

@router.get("/detailed/{medicine_name}")
async def get_detailed_medicine(medicine_name: str):
    """獲取特定藥物的詳細資訊"""
    if medicine_name not in detailed_medicines_db:
        raise HTTPException(status_code=404, detail="找不到該藥物的詳細資訊")
    return detailed_medicines_db[medicine_name]

@router.get("/detailed/")
async def get_all_detailed_medicines():
    """獲取所有詳細藥物資訊"""
    return detailed_medicines_db

@router.get("/search/code/{code}")
async def search_medicine_by_code(code: str):
    """根據包裝編號搜尋藥物"""
    results = {}
    code_upper = code.upper()
    
    for name, data in detailed_medicines_db.items():
        packaging_codes = data.get("包裝編號", {})
        # 檢查所有包裝編號
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
        raise HTTPException(status_code=404, detail=f"找不到包裝編號包含 '{code}' 的藥物")
    return results

# === 整合藥物管理API ===
@router.get("/integrated/{medicine_name}")
async def get_integrated_medicine_info(medicine_name: str):
    """獲取整合的藥物資訊 (庫存 + 詳細資訊)"""
    result = {
        "medicine_name": medicine_name,
        "basic_info": None,
        "detailed_info": None,
        "status": "not_found"
    }
    
    # 搜尋基本庫存資訊
    for medicine in medicines_db:
        if medicine["name"].lower() == medicine_name.lower():
            result["basic_info"] = medicine
            break
    
    # 搜尋詳細藥物資訊
    if medicine_name in detailed_medicines_db:
        result["detailed_info"] = detailed_medicines_db[medicine_name]
    
    # 決定狀態
    if result["basic_info"] and result["detailed_info"]:
        result["status"] = "complete"  # 有庫存也有詳細資訊
    elif result["basic_info"]:
        result["status"] = "basic_only"  # 只有庫存資訊
    elif result["detailed_info"]:
        result["status"] = "detailed_only"  # 只有詳細資訊
    else:
        raise HTTPException(status_code=404, detail="藥物未找到")
    
    return result
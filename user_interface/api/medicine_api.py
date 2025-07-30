#!/usr/bin/env python3
"""
藥物管理API模組
Medicine Management API Module
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime

# 藥物管理API路由器
router = APIRouter(prefix="/api/medicine", tags=["藥物管理"])

# Pydantic數據模型
class MedicineBasic(BaseModel):
    name: str
    amount: int
    usage_days: int
    position: str

class MedicineDetailed(BaseModel):
    medicine_name: str
    medicine_data: Dict[str, Any]

# 記憶體內資料庫 (將被持久化儲存取代)
medicines_db = []
detailed_medicines_db = {}
next_medicine_id = 1

# === 基本藥物API端點 ===
@router.get("/")
async def get_all_medicines():
    """獲取所有基本藥物"""
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
    
    return {
        "message": "藥物新增成功",
        "medicine": new_medicine
    }

@router.put("/{medicine_id}")
async def update_medicine(medicine_id: int, medicine: MedicineBasic):
    """更新基本藥物資訊"""
    for med in medicines_db:
        if med["id"] == medicine_id:
            med.update({
                "name": medicine.name,
                "amount": medicine.amount,
                "usage_days": medicine.usage_days,
                "position": medicine.position,
                "updated_time": datetime.now().isoformat()
            })
            return {
                "message": "藥物更新成功",
                "medicine": med
            }
    
    raise HTTPException(status_code=404, detail="藥物未找到")

@router.delete("/{medicine_id}")
async def delete_medicine(medicine_id: int):
    """刪除基本藥物"""
    for i, med in enumerate(medicines_db):
        if med["id"] == medicine_id:
            deleted_medicine = medicines_db.pop(i)
            return {
                "message": "藥物刪除成功",
                "medicine": deleted_medicine
            }
    
    raise HTTPException(status_code=404, detail="藥物未找到")

# === 詳細藥物API端點 ===
@router.post("/detailed/")
async def create_detailed_medicine(detailed: MedicineDetailed):
    """新增詳細藥物資訊"""
    medicine_name = detailed.medicine_name
    
    if medicine_name in detailed_medicines_db:
        raise HTTPException(status_code=400, detail="該藥物的詳細資訊已存在")
    
    detailed_medicines_db[medicine_name] = {
        **detailed.medicine_data,
        "created_time": datetime.now().isoformat()
    }
    
    return {
        "message": "詳細藥物資訊新增成功",
        "medicine_name": medicine_name,
        "data": detailed_medicines_db[medicine_name]
    }

@router.get("/detailed/{medicine_name}")
async def get_detailed_medicine(medicine_name: str):
    """獲取特定藥物的詳細資訊"""
    if medicine_name not in detailed_medicines_db:
        raise HTTPException(status_code=404, detail="藥物詳細資訊未找到")
    
    return {
        "medicine_name": medicine_name,
        "data": detailed_medicines_db[medicine_name]
    }

@router.get("/detailed/")
async def get_all_detailed_medicines():
    """獲取所有詳細藥物資訊"""
    return detailed_medicines_db

# === 藥物搜索API端點 ===
@router.get("/search/code/{code}")
async def search_medicine_by_code(code: str):
    """按編號搜索藥物"""
    results = []
    
    # 在詳細藥物資訊中搜索
    for medicine_name, medicine_data in detailed_medicines_db.items():
        # 檢查包裝編號
        if "包裝編號" in medicine_data:
            packaging_codes = medicine_data["包裝編號"]
            if isinstance(packaging_codes, dict):
                for key, value in packaging_codes.items():
                    if code in str(value):
                        results.append({
                            "medicine_name": medicine_name,
                            "found_in": f"包裝編號.{key}",
                            "code_value": value,
                            "data": medicine_data
                        })
        
        # 檢查條碼
        if "條碼" in medicine_data and code in str(medicine_data["條碼"]):
            results.append({
                "medicine_name": medicine_name,
                "found_in": "條碼",
                "code_value": medicine_data["條碼"],
                "data": medicine_data
            })
    
    if not results:
        raise HTTPException(status_code=404, detail="未找到包含該編號的藥物")
    
    return {
        "search_code": code,
        "results": results
    }

# === 整合查詢API端點 ===
@router.get("/integrated/{medicine_name}")
async def get_integrated_medicine_info(medicine_name: str):
    """獲取整合的藥物資訊 (基本 + 詳細)"""
    # 查找基本資訊
    basic_info = None
    for med in medicines_db:
        if med["name"] == medicine_name:
            basic_info = med
            break
    
    # 查找詳細資訊
    detailed_info = detailed_medicines_db.get(medicine_name)
    
    if not basic_info and not detailed_info:
        raise HTTPException(status_code=404, detail="藥物資訊未找到")
    
    return {
        "medicine_name": medicine_name,
        "basic_info": basic_info,
        "detailed_info": detailed_info,
        "has_complete_info": basic_info is not None and detailed_info is not None
    }
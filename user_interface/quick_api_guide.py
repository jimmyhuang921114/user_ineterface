#!/usr/bin/env python3
"""
醫院系統API - 快速使用指南
Quick API Usage Guide
"""

import requests
import json

# API基礎地址
API_BASE = "http://localhost:8000"

# === 1. 基本藥物查詢 ===
def get_medicine_info(medicine_name):
    """獲取藥物詳細資訊"""
    response = requests.get(f"{API_BASE}/api/medicine/detailed/{medicine_name}")
    return response.json() if response.status_code == 200 else None

# === 2. 根據編號查藥物 ===
def find_medicine_by_code(code):
    """根據包裝編號查找藥物"""
    response = requests.get(f"{API_BASE}/api/medicine/search/code/{code}")
    return response.json() if response.status_code == 200 else None

# === 3. 獲取病人資訊 ===
def get_patient_info(patient_id):
    """獲取病人資訊"""
    response = requests.get(f"{API_BASE}/api/patients/{patient_id}")
    return response.json() if response.status_code == 200 else None

# === 4. 獲取病人病例 ===
def get_patient_records(patient_id):
    """獲取病人的所有病例記錄"""
    response = requests.get(f"{API_BASE}/api/records/patient/{patient_id}")
    return response.json() if response.status_code == 200 else []

# === 5. 整合查詢（庫存+詳細資訊） ===
def get_complete_medicine_info(medicine_name):
    """獲取完整藥物資訊（庫存+詳細）"""
    response = requests.get(f"{API_BASE}/api/medicine/integrated/{medicine_name}")
    return response.json() if response.status_code == 200 else None

# === 6. 導出JSON ===
def export_data(data_type="complete"):
    """導出資料為JSON
    data_type: 'medicines', 'patients', 'records', 'complete'
    """
    endpoints = {
        'medicines': '/api/export/medicines/integrated',
        'patients': '/api/export/patients',
        'records': '/api/export/records',
        'complete': '/api/export/complete'
    }
    
    url = f"{API_BASE}{endpoints.get(data_type, endpoints['complete'])}"
    response = requests.get(url)
    return response.json() if response.status_code == 200 else None

# === 使用範例 ===
if __name__ == "__main__":
    print("🏥 醫院系統API - 快速使用範例")
    print("=" * 40)
    
    # 1. 查詢心律錠資訊
    print("1. 查詢心律錠詳細資訊:")
    heart_med = get_medicine_info("心律錠")
    if heart_med:
        print(f"   名稱: {heart_med.get('基本資訊', {}).get('名稱')}")
        print(f"   廠商: {heart_med.get('基本資訊', {}).get('廠商')}")
    
    # 2. 根據編號查藥物
    print("\n2. 根據編號202801查藥物:")
    code_result = find_medicine_by_code("202801")
    if code_result:
        for name in code_result.keys():
            print(f"   找到: {name}")
    
    # 3. 獲取完整藥物資訊
    print("\n3. 獲取心律錠完整資訊:")
    complete_info = get_complete_medicine_info("心律錠")
    if complete_info:
        print(f"   狀態: {complete_info.get('status')}")
        if complete_info.get('basic_info'):
            print(f"   庫存: {complete_info['basic_info']['amount']}")
    
    # 4. 導出所有藥物資料
    print("\n4. 導出藥物資料:")
    medicines_data = export_data('medicines')
    if medicines_data:
        print(f"   成功導出 {medicines_data.get('total_count')} 個藥物")
        
        # 保存到文件
        with open('my_medicines.json', 'w', encoding='utf-8') as f:
            json.dump(medicines_data, f, ensure_ascii=False, indent=2)
        print("   已保存到 my_medicines.json")
    
    print("\n✅ 範例執行完成！")

# === 常用功能函數 ===
def quick_medicine_lookup(name_or_code):
    """快速藥物查詢（名稱或編號）"""
    # 先嘗試按名稱查詢
    result = get_medicine_info(name_or_code)
    if result:
        return {"type": "name", "data": result}
    
    # 再嘗試按編號查詢
    result = find_medicine_by_code(name_or_code)
    if result:
        return {"type": "code", "data": result}
    
    return None

def get_patient_complete_info(patient_id):
    """獲取病人完整資訊（基本資料+病例）"""
    patient = get_patient_info(patient_id)
    if not patient:
        return None
    
    records = get_patient_records(patient_id)
    return {
        "patient": patient,
        "records": records,
        "total_visits": len(records)
    }
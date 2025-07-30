#!/usr/bin/env python3
"""
快速測試醫院管理系統
Quick Test Hospital Management System
"""

import requests
import json
from datetime import datetime

def test_system():
    """測試系統功能"""
    API_BASE = "http://localhost:8000"
    
    print("🏥 醫院管理系統 - 快速測試")
    print("=" * 50)
    
    # 測試1: API連接
    print("1. 測試API連接...")
    try:
        response = requests.get(f"{API_BASE}/api/test", timeout=5)
        if response.status_code == 200:
            print("✅ API連接正常")
            print(f"   回應: {response.json()}")
        else:
            print(f"❌ API連接失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ API連接錯誤: {e}")
        print("請先啟動伺服器: python3 fixed_server.py")
        return
    
    # 測試2: 新增基本藥物
    print("\n2. 測試新增基本藥物...")
    medicine_data = {
        "name": "測試藥物_Doctor",
        "amount": 50,
        "usage_days": 14,
        "position": "DOC-01"
    }
    
    try:
        response = requests.post(f"{API_BASE}/api/medicine/", 
                               json=medicine_data, timeout=5)
        if response.status_code == 200:
            result = response.json()
            print(f"✅ 基本藥物新增成功: ID={result['id']}")
            medicine_id = result['id']
        else:
            print(f"❌ 基本藥物新增失敗: {response.status_code}")
            medicine_id = None
    except Exception as e:
        print(f"❌ 基本藥物新增錯誤: {e}")
        medicine_id = None
    
    # 測試3: 新增詳細藥物資訊
    print("\n3. 測試新增詳細藥物資訊...")
    detailed_data = {
        "medicine_name": "測試藥物_Doctor",
        "medicine_data": {
            "基本資訊": {
                "名稱": "測試藥物_Doctor",
                "廠商": "測試製藥公司",
                "劑量": "10毫克",
                "服用方式": "口服"
            },
            "外觀": {
                "顏色": "白色",
                "形狀": "圓形"
            },
            "包裝編號": {
                "編號1": "DOC001",
                "編號2": "TEST002"
            },
            "適應症": "測試用藥物，用於驗證系統功能",
            "可能的副作用": "無已知副作用",
            "使用說明": "按醫生指示服用",
            "儲存條件": "室溫保存"
        }
    }
    
    try:
        response = requests.post(f"{API_BASE}/api/medicine/detailed/", 
                               json=detailed_data, timeout=5)
        if response.status_code == 200:
            print("✅ 詳細藥物資訊新增成功")
        else:
            print(f"❌ 詳細藥物資訊新增失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 詳細藥物資訊新增錯誤: {e}")
    
    # 測試4: 開立處方
    print("\n4. 測試開立處方...")
    prescription_data = {
        "patient_name": "測試病人",
        "doctor_name": "測試醫生",
        "diagnosis": "測試診斷",
        "medicines": [{
            "medicine_name": "測試藥物_Doctor",
            "dosage": "10mg",
            "frequency": "每日三次",
            "duration": "7天",
            "instructions": "飯後服用"
        }]
    }
    
    try:
        response = requests.post(f"{API_BASE}/api/prescription/", 
                               json=prescription_data, timeout=5)
        if response.status_code == 200:
            result = response.json()
            print(f"✅ 處方開立成功: ID={result['id']}")
        else:
            print(f"❌ 處方開立失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 處方開立錯誤: {e}")
    
    # 測試5: 獲取所有資料
    print("\n5. 測試獲取系統資料...")
    try:
        # 獲取基本藥物
        response = requests.get(f"{API_BASE}/api/medicine/", timeout=5)
        if response.status_code == 200:
            medicines = response.json()
            print(f"✅ 基本藥物總數: {len(medicines)}")
        
        # 獲取詳細藥物
        response = requests.get(f"{API_BASE}/api/medicine/detailed/", timeout=5)
        if response.status_code == 200:
            detailed = response.json()
            print(f"✅ 詳細藥物總數: {len(detailed)}")
        
        # 獲取處方
        response = requests.get(f"{API_BASE}/api/prescription/", timeout=5)
        if response.status_code == 200:
            prescriptions = response.json()
            print(f"✅ 處方總數: {len(prescriptions)}")
            
    except Exception as e:
        print(f"❌ 獲取資料錯誤: {e}")
    
    print("\n" + "=" * 50)
    print("🎯 測試完成！")
    print("\n📖 使用指南:")
    print("1. 醫生工作台: http://localhost:8000/doctor.html")
    print("2. 藥物管理: http://localhost:8000/Medicine.html")  
    print("3. 處方管理: http://localhost:8000/Prescription.html")
    print("\n🔧 Doctor頁面功能:")
    print("- 基本藥物資訊標籤: 填寫藥物基本資料")
    print("- 詳細藥物資訊標籤: 填寫完整藥物資訊")
    print("- 開立處方標籤: 開立病人處方")
    print("\n💾 資料儲存:")
    print("- 基本資料儲存到 Medicine 系統")
    print("- 詳細資料儲存到 Medicine 詳細資訊")
    print("- 處方資料儲存到 Prescription 系統")

if __name__ == "__main__":
    test_system()
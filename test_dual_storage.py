#!/usr/bin/env python3
"""
雙JSON存儲功能測試腳本
"""

import requests
import json
import time
from datetime import datetime

API_BASE = "http://localhost:8000"

def test_basic_medicine_api():
    """測試基本藥物API"""
    print("🧪 測試基本藥物API...")
    
    # 測試GET請求
    try:
        response = requests.get(f"{API_BASE}/api/medicine/basic")
        print(f"GET基本藥物: {response.status_code} - {response.text[:100]}...")
    except Exception as e:
        print(f"GET基本藥物錯誤: {e}")
    
    # 測試POST請求
    test_data = {
        "name": f"測試藥物_{int(time.time())}",
        "amount": 100,
        "usage_days": 7,
        "position": "TEST-A1",
        "manufacturer": "測試製藥公司",
        "dosage": "100mg"
    }
    
    try:
        response = requests.post(
            f"{API_BASE}/api/medicine/basic",
            json=test_data,
            headers={'Content-Type': 'application/json'}
        )
        print(f"POST基本藥物: {response.status_code} - {response.text[:100]}...")
        return test_data["name"]
    except Exception as e:
        print(f"POST基本藥物錯誤: {e}")
        return None

def test_detailed_medicine_api(medicine_name):
    """測試詳細藥物API"""
    print("🧪 測試詳細藥物API...")
    
    # 測試GET請求
    try:
        response = requests.get(f"{API_BASE}/api/medicine/detailed")
        print(f"GET詳細藥物: {response.status_code} - {response.text[:100]}...")
    except Exception as e:
        print(f"GET詳細藥物錯誤: {e}")
    
    # 測試POST請求
    test_data = {
        "medicine_name": medicine_name or f"測試藥物_{int(time.time())}",
        "description": "這是一個測試用的藥物描述",
        "side_effects": "可能的副作用包括頭暈",
        "appearance": {
            "color": "白色",
            "shape": "圓形"
        },
        "storage_conditions": "室溫保存",
        "expiry_date": "2025-12-31",
        "notes": "測試用藥物，請勿實際使用"
    }
    
    try:
        response = requests.post(
            f"{API_BASE}/api/medicine/detailed",
            json=test_data,
            headers={'Content-Type': 'application/json'}
        )
        print(f"POST詳細藥物: {response.status_code} - {response.text[:100]}...")
    except Exception as e:
        print(f"POST詳細藥物錯誤: {e}")

def test_ros2_apis():
    """測試ROS2 API"""
    print("🤖 測試ROS2 API...")
    
    endpoints = [
        "/api/ros2/medicine/basic",
        "/api/ros2/medicine/detailed", 
        "/api/ros2/prescription"
    ]
    
    for endpoint in endpoints:
        try:
            response = requests.get(f"{API_BASE}{endpoint}")
            print(f"ROS2 {endpoint}: {response.status_code} - {response.text[:100]}...")
        except Exception as e:
            print(f"ROS2 {endpoint} 錯誤: {e}")

def test_integrated_api():
    """測試整合API"""
    print("🔗 測試整合API...")
    
    test_name = "測試藥物"
    try:
        response = requests.get(f"{API_BASE}/api/ros2/medicine/integrated/{test_name}")
        print(f"整合API: {response.status_code} - {response.text[:200]}...")
    except Exception as e:
        print(f"整合API錯誤: {e}")

def check_json_files():
    """檢查JSON文件是否存在"""
    print("📁 檢查JSON文件...")
    
    import os
    files = [
        "user_interface/medicine_basic_data.json",
        "user_interface/medicine_detailed_data.json", 
        "user_interface/prescription_data.json"
    ]
    
    for file_path in files:
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                print(f"✅ {file_path}: {len(data)} 筆資料")
            except Exception as e:
                print(f"❌ {file_path}: 讀取錯誤 - {e}")
        else:
            print(f"⚠️ {file_path}: 文件不存在")

def test_web_pages():
    """測試網頁是否可以訪問"""
    print("🌐 測試網頁訪問...")
    
    pages = [
        "/",
        "/doctor.html",
        "/medicine_integrated.html",
        "/Prescription.html",
        "/simple_test.html",
        "/docs"
    ]
    
    for page in pages:
        try:
            response = requests.get(f"{API_BASE}{page}")
            print(f"{page}: {response.status_code}")
        except Exception as e:
            print(f"{page}: 錯誤 - {e}")

def main():
    print("🚀 開始雙JSON存儲功能測試")
    print("=" * 50)
    
    # 測試網頁
    test_web_pages()
    print()
    
    # 測試API
    medicine_name = test_basic_medicine_api()
    print()
    
    test_detailed_medicine_api(medicine_name)
    print()
    
    test_ros2_apis()
    print()
    
    test_integrated_api()
    print()
    
    # 檢查文件
    check_json_files()
    print()
    
    print("✅ 測試完成！")
    print("=" * 50)

if __name__ == "__main__":
    main()
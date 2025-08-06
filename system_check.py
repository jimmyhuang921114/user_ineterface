#!/usr/bin/env python3
"""
醫院藥物管理系統 - 全面健康檢查
Hospital Medicine Management System - Comprehensive Health Check
"""

import requests
import json
import time
import os
from pathlib import Path
import subprocess

def check_server_status():
    """檢查服務器狀態"""
    print("🌐 檢查服務器狀態...")
    try:
        response = requests.get("http://localhost:8000/api/health", timeout=5)
        if response.status_code == 200:
            print("✅ 服務器正常運行")
            return True
        else:
            print(f"❌ 服務器回應異常: {response.status_code}")
            return False
    except requests.exceptions.RequestException as e:
        print(f"❌ 無法連接服務器: {e}")
        return False

def check_file_structure():
    """檢查檔案結構"""
    print("\n📁 檢查檔案結構...")
    
    required_files = [
        "user_interface/fixed_server.py",
        "user_interface/static/html/doctor.html",
        "user_interface/static/html/Medicine.html", 
        "user_interface/static/html/Prescription.html",
        "user_interface/static/html/integrated_medicine_management.html",
        "user_interface/test_all_functions.html",
        "user_interface/medicine_basic_data.json",
        "user_interface/prescription_data.json"
    ]
    
    missing_files = []
    for file_path in required_files:
        if os.path.exists(file_path):
            print(f"✅ {file_path}")
        else:
            print(f"❌ {file_path} - 檔案不存在")
            missing_files.append(file_path)
    
    return len(missing_files) == 0

def test_basic_apis():
    """測試基本API"""
    print("\n🔗 測試基本API...")
    
    api_tests = [
        ("GET", "/api/health", None, "健康檢查"),
        ("GET", "/api/medicine/basic", None, "基本藥物列表"),
        ("GET", "/api/prescription/", None, "處方籤列表")
    ]
    
    passed = 0
    total = len(api_tests)
    
    for method, endpoint, data, description in api_tests:
        try:
            url = f"http://localhost:8000{endpoint}"
            if method == "GET":
                response = requests.get(url, timeout=5)
            elif method == "POST":
                response = requests.post(url, json=data, timeout=5)
            
            if response.status_code in [200, 201]:
                print(f"✅ {description}: {response.status_code}")
                passed += 1
            else:
                print(f"❌ {description}: {response.status_code}")
        except Exception as e:
            print(f"❌ {description}: {e}")
    
    print(f"📊 API測試結果: {passed}/{total} 通過")
    return passed == total

def test_prescription_creation():
    """測試處方籤創建"""
    print("\n📋 測試處方籤創建...")
    
    test_prescription = {
        "patient_name": "系統測試病患",
        "patient_id": "P999999",
        "doctor_name": "系統測試醫師",
        "medicines": [
            ["測試藥物A", "1", "3", "測試用"],
            ["測試藥物B", "2", "5", "系統驗證"]
        ]
    }
    
    try:
        response = requests.post(
            "http://localhost:8000/api/prescription/",
            json=test_prescription,
            timeout=5
        )
        
        if response.status_code in [200, 201]:
            print("✅ 處方籤創建成功")
            return True
        else:
            print(f"❌ 處方籤創建失敗: {response.status_code}")
            print(f"錯誤詳情: {response.text}")
            return False
    except Exception as e:
        print(f"❌ 處方籤創建異常: {e}")
        return False

def test_web_pages():
    """測試網頁可訪問性"""
    print("\n🌐 測試網頁可訪問性...")
    
    pages = [
        ("/doctor.html", "醫生工作站"),
        ("/Medicine.html", "藥物管理"),
        ("/Prescription.html", "處方籤管理"),
        ("/integrated_medicine_management.html", "整合管理"),
        ("/test_all_functions.html", "功能測試頁面")
    ]
    
    passed = 0
    total = len(pages)
    
    for page, name in pages:
        try:
            response = requests.get(f"http://localhost:8000{page}", timeout=5)
            if response.status_code == 200:
                print(f"✅ {name}: 可正常訪問")
                passed += 1
            else:
                print(f"❌ {name}: HTTP {response.status_code}")
        except Exception as e:
            print(f"❌ {name}: {e}")
    
    print(f"📊 網頁測試結果: {passed}/{total} 通過")
    return passed == total

def check_data_files():
    """檢查數據檔案"""
    print("\n💾 檢查數據檔案...")
    
    data_files = [
        ("user_interface/medicine_basic_data.json", "基本藥物資料"),
        ("user_interface/medicine_detailed_data.json", "詳細藥物資料"),
        ("user_interface/prescription_data.json", "處方籤資料")
    ]
    
    all_valid = True
    
    for file_path, description in data_files:
        try:
            if os.path.exists(file_path):
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    print(f"✅ {description}: 檔案有效，包含 {len(data)} 筆記錄")
            else:
                print(f"❌ {description}: 檔案不存在")
                all_valid = False
        except json.JSONDecodeError:
            print(f"❌ {description}: JSON格式錯誤")
            all_valid = False
        except Exception as e:
            print(f"❌ {description}: {e}")
            all_valid = False
    
    return all_valid

def main():
    """主要檢查函數"""
    print("🏥 醫院藥物管理系統 - 全面健康檢查")
    print("=" * 50)
    
    checks = [
        ("服務器狀態", check_server_status),
        ("檔案結構", check_file_structure),
        ("數據檔案", check_data_files),
        ("基本API", test_basic_apis),
        ("處方籤創建", test_prescription_creation),
        ("網頁可訪問性", test_web_pages)
    ]
    
    passed_checks = 0
    total_checks = len(checks)
    
    for check_name, check_function in checks:
        print(f"\n🔍 執行 {check_name} 檢查...")
        if check_function():
            passed_checks += 1
            print(f"✅ {check_name} 檢查通過")
        else:
            print(f"❌ {check_name} 檢查失敗")
    
    print("\n" + "=" * 50)
    print("📊 系統健康檢查結果:")
    print(f"✅ 通過: {passed_checks}/{total_checks}")
    print(f"📈 健康度: {(passed_checks/total_checks)*100:.1f}%")
    
    if passed_checks == total_checks:
        print("🎉 系統完全健康！所有功能正常運作")
        return True
    else:
        print("⚠️  系統存在問題，請檢查失敗的項目")
        return False

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
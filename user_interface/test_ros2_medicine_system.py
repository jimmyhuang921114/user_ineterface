#!/usr/bin/env python3
"""
ROS2 藥物查詢與訂單系統測試腳本
演示系統的完整功能
"""

import requests
import json
import time

def test_medicine_query():
    """測試藥物詳細查詢功能"""
    print("🧪 測試藥物詳細查詢功能")
    print("-" * 40)
    
    # 首先創建測試藥物
    test_medicine = {
        "basic": {
            "name": "Antipsychotics",
            "amount": 100,
            "position": "A-001",
            "manufacturer": "測試製藥公司",
            "dosage": "1 special pill"
        },
        "detailed": {
            "description": "適用於12歲以上、體重至少40公斤，於5天內確診輕度至中度COVID-19，且具嚴重疾病風險因子的成人與兒童",
            "ingredient": "Nirmatrelvir",
            "category": "3CL proteinase inhibitors",
            "usage_method": "口服 (Oral use)",
            "side_effects": "味覺異常、腹瀉、噁心、嘔吐、頭痛",
            "storage_conditions": "室溫保存",
            "expiry_date": "2027/11/09",
            "barcode": "TEST-367842394",
            "appearance_type": "藍色條紋 白色外觀 圓扁形",
            "notes": "測試藥物資料"
        }
    }
    
    # 創建藥物
    response = requests.post(
        'http://localhost:8001/api/medicine/unified',
        json=test_medicine,
        timeout=10
    )
    
    if response.status_code == 200:
        print("✅ 測試藥物創建成功")
    else:
        print(f"❌ 測試藥物創建失敗: {response.status_code}")
        return False
    
    # 查詢藥物詳細資訊
    query_response = requests.post(
        'http://localhost:8001/api/ros2/query-medicine-detail',
        json={"medicine_name": "Antipsychotics"},
        timeout=10
    )
    
    if query_response.status_code == 200:
        result = query_response.json()
        print("✅ 藥物查詢成功")
        print("\n📋 查詢結果 (YAML 格式):")
        print(result.get('detail', '無詳細資訊'))
        return True
    else:
        print(f"❌ 藥物查詢失敗: {query_response.status_code}")
        return False

def test_order_processing():
    """測試訂單處理功能"""
    print("\n🚀 測試訂單處理功能")
    print("-" * 40)
    
    # 創建測試訂單
    order_data = {
        "order_id": "000001",
        "medicines": [
            {
                "name": "Antipsychotics",
                "quantity": 87
            },
            {
                "name": "測試藥物B",
                "quantity": 212
            }
        ]
    }
    
    # 發送訂單
    response = requests.post(
        'http://localhost:8001/api/ros2/process-order',
        json=order_data,
        timeout=15
    )
    
    if response.status_code == 200:
        result = response.json()
        print("✅ 訂單發送成功")
        print("\n📦 訂單處理結果:")
        print(result.get('message', '無詳細資訊'))
        
        # 監控訂單狀態
        print("\n⏳ 監控訂單處理狀態...")
        for i in range(20):  # 監控20秒
            status_response = requests.get('http://localhost:8001/api/ros2/service-status')
            if status_response.status_code == 200:
                status = status_response.json()
                if status.get('processing'):
                    print(f"   處理中: {status.get('current_order', '未知訂單')}")
                else:
                    print("   ✅ 訂單處理完成！")
                    break
            time.sleep(1)
        
        return True
    else:
        print(f"❌ 訂單發送失敗: {response.status_code}")
        return False

def test_system_status():
    """測試系統狀態"""
    print("\n📊 測試系統狀態功能")
    print("-" * 40)
    
    response = requests.get('http://localhost:8001/api/ros2/service-status')
    
    if response.status_code == 200:
        status = response.json()
        print("✅ 系統狀態獲取成功")
        print(f"   服務運行中: {status.get('service_running')}")
        print(f"   當前訂單: {status.get('current_order', '無')}")
        print(f"   正在處理: {status.get('processing')}")
        print(f"   服務名稱: {status.get('service_name', '未知')}")
        return True
    else:
        print(f"❌ 系統狀態獲取失敗: {response.status_code}")
        return False

def test_api_endpoints():
    """測試所有 API 端點"""
    print("\n🔌 測試 API 端點")
    print("-" * 40)
    
    endpoints = [
        ("GET", "/api/system/status", None),
        ("GET", "/api/ros2/status", None),
        ("GET", "/api/ros2/service/status", None),
        ("GET", "/ros2_client.html", None)
    ]
    
    for method, endpoint, data in endpoints:
        try:
            if method == "GET":
                response = requests.get(f'http://localhost:8001{endpoint}', timeout=5)
            else:
                response = requests.post(f'http://localhost:8001{endpoint}', json=data, timeout=5)
            
            if response.status_code == 200:
                print(f"✅ {method} {endpoint}")
            else:
                print(f"⚠️  {method} {endpoint} - {response.status_code}")
        except Exception as e:
            print(f"❌ {method} {endpoint} - {str(e)}")

def main():
    """主測試函數"""
    print("🎯 ROS2 藥物查詢與訂單系統完整測試")
    print("=" * 60)
    print("請確保系統已啟動在 http://localhost:8001")
    
    input("按 Enter 開始測試...")
    
    # 檢查服務器連接
    try:
        response = requests.get('http://localhost:8001/api/system/status', timeout=5)
        if response.status_code == 200:
            print("✅ 服務器連接正常")
        else:
            print("❌ 服務器連接異常")
            return
    except Exception as e:
        print(f"❌ 無法連接到服務器: {e}")
        return
    
    # 運行所有測試
    tests = [
        test_system_status,
        test_api_endpoints,
        test_medicine_query,
        test_order_processing
    ]
    
    passed = 0
    total = len(tests)
    
    for test_func in tests:
        try:
            if test_func():
                passed += 1
        except Exception as e:
            print(f"❌ 測試 {test_func.__name__} 發生錯誤: {e}")
    
    # 測試總結
    print("\n📊 測試總結")
    print("=" * 60)
    print(f"總測試數: {total}")
    print(f"通過測試: {passed}")
    print(f"失敗測試: {total - passed}")
    print(f"通過率: {(passed/total)*100:.1f}%")
    
    if passed == total:
        print("\n🎉 所有測試通過！系統運行正常！")
        print("\n🌐 可以開始使用以下功能:")
        print("   • 網頁界面: http://localhost:8001/ros2_client.html")
        print("   • 藥物查詢: 輸入藥物名稱獲取詳細 YAML 格式資訊")
        print("   • 訂單處理: 一次處理一個訂單，包含位置和提示符")
        print("   • 狀態監控: 即時追蹤系統和訂單狀態")
    else:
        print("\n⚠️ 部分測試失敗，請檢查系統配置")

if __name__ == "__main__":
    main()
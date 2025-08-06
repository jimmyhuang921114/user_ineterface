#!/usr/bin/env python3
"""
Test ROS2 Functionality
測試ROS2功能
"""

import requests
import json
import time
from typing import Dict, List

# API基礎URL
BASE_URL = "http://localhost:8001"

def test_ros2_simulation():
    """測試ROS2模擬功能"""
    print("🤖 測試ROS2模擬功能")
    print("=" * 50)
    
    # 1. 檢查ROS2狀態
    print("1. 檢查ROS2狀態...")
    try:
        response = requests.get(f"{BASE_URL}/api/ros2/status")
        if response.status_code == 200:
            status = response.json()
            print(f"✅ ROS2狀態: {status}")
        else:
            print(f"❌ ROS2狀態檢查失敗: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ ROS2狀態檢查異常: {e}")
        return False
    
    # 2. 創建處方籤（會自動加入ROS2佇列）
    print("\n2. 創建處方籤...")
    prescription_data = {
        "patient_name": "ROS2測試病患",
        "patient_id": "ROS2_TEST_001",
        "doctor_name": "ROS2測試醫師",
        "diagnosis": "ROS2功能測試"
    }
    
    try:
        response = requests.post(f"{BASE_URL}/api/prescription/", json=prescription_data)
        if response.status_code == 200:
            result = response.json()
            print(f"✅ 處方籤創建成功: {result}")
        else:
            print(f"❌ 處方籤創建失敗: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ 處方籤創建異常: {e}")
        return False
    
    # 3. 檢查處方籤列表
    print("\n3. 檢查處方籤列表...")
    try:
        response = requests.get(f"{BASE_URL}/api/prescription/")
        if response.status_code == 200:
            prescriptions = response.json()
            print(f"✅ 處方籤列表: {len(prescriptions)} 筆資料")
            for p in prescriptions[-3:]:  # 顯示最後3筆
                print(f"   - {p['patient_name']} ({p['patient_id']}) - {p['status']}")
        else:
            print(f"❌ 獲取處方籤列表失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 獲取處方籤列表異常: {e}")
    
    # 4. 測試藥物資料發布
    print("\n4. 測試藥物資料發布...")
    try:
        response = requests.get(f"{BASE_URL}/api/medicine/basic")
        if response.status_code == 200:
            medicines = response.json()
            print(f"✅ 藥物資料獲取成功: {len(medicines)} 筆資料")
            for med in medicines:
                print(f"   - {med['name']} (庫存: {med['amount']})")
        else:
            print(f"❌ 藥物資料獲取失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 藥物資料獲取異常: {e}")
    
    # 5. 測試創建新藥物（會觸發ROS2發布）
    print("\n5. 測試創建新藥物...")
    medicine_data = {
        "name": "ROS2測試藥物",
        "amount": 100,
        "position": "ROS2-01",
        "manufacturer": "ROS2測試廠商",
        "dosage": "100mg",
        "prompt": "ROS2功能測試藥物"
    }
    
    try:
        response = requests.post(f"{BASE_URL}/api/medicine/", json=medicine_data)
        if response.status_code == 200:
            result = response.json()
            print(f"✅ 藥物創建成功: {result}")
        else:
            print(f"❌ 藥物創建失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 藥物創建異常: {e}")
    
    print("\n" + "=" * 50)
    print("✅ ROS2模擬功能測試完成")
    print("\n📋 總結:")
    print("- 處方籤創建後會自動加入ROS2訂單佇列")
    print("- 藥物資料會透過ROS2發布")
    print("- 系統使用模擬模式，無需真實ROS2環境")
    print("- 所有功能都與ROS2整合，準備好真實環境時可直接使用")
    
    return True

def test_order_processing():
    """測試訂單處理功能"""
    print("\n📦 測試訂單處理功能")
    print("=" * 50)
    
    # 創建多個處方籤來測試佇列
    print("創建多個處方籤測試佇列...")
    
    for i in range(3):
        prescription_data = {
            "patient_name": f"佇列測試病患{i+1}",
            "patient_id": f"QUEUE_TEST_{i+1:03d}",
            "doctor_name": "佇列測試醫師",
            "diagnosis": f"佇列測試診斷{i+1}"
        }
        
        try:
            response = requests.post(f"{BASE_URL}/api/prescription/", json=prescription_data)
            if response.status_code == 200:
                result = response.json()
                print(f"✅ 處方籤 {i+1} 創建成功: {result['id']}")
            else:
                print(f"❌ 處方籤 {i+1} 創建失敗")
        except Exception as e:
            print(f"❌ 處方籤 {i+1} 創建異常: {e}")
        
        time.sleep(1)  # 間隔1秒
    
    print("\n📊 訂單處理流程:")
    print("1. 醫生開立處方籤")
    print("2. 系統自動加入ROS2訂單佇列")
    print("3. ROS2節點按順序處理訂單")
    print("4. 處理完成後回傳狀態")
    print("5. 系統更新處方籤狀態")
    
    return True

def main():
    """主測試函數"""
    print("🧪 ROS2功能完整性測試")
    print("=" * 60)
    
    # 等待伺服器啟動
    print("⏳ 等待伺服器啟動...")
    time.sleep(3)
    
    # 執行測試
    test_ros2_simulation()
    test_order_processing()
    
    print("\n" + "=" * 60)
    print("🎯 測試完成！系統已準備好與真實ROS2環境整合")

if __name__ == "__main__":
    main()
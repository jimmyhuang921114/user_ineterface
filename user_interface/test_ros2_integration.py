#!/usr/bin/env python3
"""
Test ROS2 Integration
測試ROS2整合功能
"""

import requests
import json
import time
from typing import Dict, List

# API基礎URL
BASE_URL = "http://localhost:8000"

def test_health_check():
    """測試健康檢查"""
    print("🔍 測試健康檢查...")
    try:
        response = requests.get(f"{BASE_URL}/api/health")
        if response.status_code == 200:
            data = response.json()
            print(f"✅ 健康檢查通過: {data}")
            return True
        else:
            print(f"❌ 健康檢查失敗: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ 健康檢查異常: {e}")
        return False

def test_medicine_operations():
    """測試藥物操作"""
    print("\n💊 測試藥物操作...")
    
    # 獲取藥物列表
    try:
        response = requests.get(f"{BASE_URL}/api/medicine/basic")
        if response.status_code == 200:
            medicines = response.json()
            print(f"✅ 獲取藥物列表成功: {len(medicines)} 筆資料")
            
            # 測試獲取詳細藥物資訊
            if medicines:
                medicine_id = medicines[0]["id"]
                detail_response = requests.get(f"{BASE_URL}/api/medicine/{medicine_id}")
                if detail_response.status_code == 200:
                    detail = detail_response.json()
                    print(f"✅ 獲取藥物詳細資訊成功: {detail['name']}")
                else:
                    print(f"❌ 獲取藥物詳細資訊失敗: {detail_response.status_code}")
        else:
            print(f"❌ 獲取藥物列表失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 藥物操作異常: {e}")

def test_prescription_operations():
    """測試處方籤操作"""
    print("\n📋 測試處方籤操作...")
    
    # 創建測試處方籤
    test_prescription = {
        "patient_name": "測試病患",
        "patient_id": "TEST001",
        "doctor_name": "測試醫師",
        "diagnosis": "測試診斷"
    }
    
    try:
        response = requests.post(f"{BASE_URL}/api/prescription/", json=test_prescription)
        if response.status_code == 200:
            result = response.json()
            print(f"✅ 創建處方籤成功: {result}")
            
            # 獲取處方籤列表
            list_response = requests.get(f"{BASE_URL}/api/prescription/")
            if list_response.status_code == 200:
                prescriptions = list_response.json()
                print(f"✅ 獲取處方籤列表成功: {len(prescriptions)} 筆資料")
            else:
                print(f"❌ 獲取處方籤列表失敗: {list_response.status_code}")
        else:
            print(f"❌ 創建處方籤失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 處方籤操作異常: {e}")

def test_ros2_status():
    """測試ROS2狀態"""
    print("\n🤖 測試ROS2狀態...")
    
    try:
        response = requests.get(f"{BASE_URL}/api/ros2/status")
        if response.status_code == 200:
            status = response.json()
            print(f"✅ ROS2狀態: {status}")
            
            # 測試佇列狀態
            queue_response = requests.get(f"{BASE_URL}/api/ros2/queue")
            if queue_response.status_code == 200:
                queue_status = queue_response.json()
                print(f"✅ 佇列狀態: {queue_status}")
            else:
                print(f"❌ 獲取佇列狀態失敗: {queue_response.status_code}")
        else:
            print(f"❌ 獲取ROS2狀態失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ ROS2狀態檢查異常: {e}")

def test_ros2_order():
    """測試ROS2訂單"""
    print("\n📦 測試ROS2訂單...")
    
    # 創建測試訂單
    test_order = {
        "patient_name": "ROS2測試病患",
        "patient_id": "ROS2_TEST001",
        "medicines": [
            {
                "name": "測試藥物A",
                "quantity": 2,
                "dosage": "500mg"
            },
            {
                "name": "測試藥物B", 
                "quantity": 1,
                "dosage": "100mg"
            }
        ]
    }
    
    try:
        response = requests.post(f"{BASE_URL}/api/ros2/order", json=test_order)
        if response.status_code == 200:
            result = response.json()
            print(f"✅ 添加ROS2訂單成功: {result}")
            
            # 等待一下讓訂單處理
            time.sleep(2)
            
            # 再次檢查佇列狀態
            queue_response = requests.get(f"{BASE_URL}/api/ros2/queue")
            if queue_response.status_code == 200:
                queue_status = queue_response.json()
                print(f"✅ 更新後佇列狀態: {queue_status}")
        else:
            print(f"❌ 添加ROS2訂單失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ ROS2訂單測試異常: {e}")

def main():
    """主測試函數"""
    print("🧪 開始ROS2整合功能測試")
    print("=" * 50)
    
    # 等待伺服器啟動
    print("⏳ 等待伺服器啟動...")
    time.sleep(3)
    
    # 執行測試
    test_health_check()
    test_medicine_operations()
    test_prescription_operations()
    test_ros2_status()
    test_ros2_order()
    
    print("\n" + "=" * 50)
    print("✅ 測試完成")

if __name__ == "__main__":
    main()
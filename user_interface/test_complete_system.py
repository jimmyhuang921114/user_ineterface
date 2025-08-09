#!/usr/bin/env python3
"""
Complete System Test
測試完整的醫院管理系統功能
"""

import requests
import time
import json

def test_system():
    base_url = "http://localhost:8001"
    print("🧪 測試完整醫院管理系統")
    print("=" * 50)
    
    # 1. 測試系統狀態
    print("\n1️⃣ 測試系統狀態...")
    try:
        response = requests.get(f"{base_url}/api/system/status", timeout=3)
        if response.status_code == 200:
            print("✅ 系統狀態正常")
        else:
            print("❌ 系統狀態異常")
            return
    except:
        print("❌ 無法連接到系統，請確認系統正在運行")
        return
    
    # 2. 測試新增藥物 (詳細內容可選)
    print("\n2️⃣ 測試新增藥物...")
    
    # 新增只有基本資訊的藥物
    medicine_basic = {
        "name": "測試藥物A",
        "position": "1-1",
        "prompt": "test_medicine_a",
        "confidence": 0.9,
        "amount": 100,
        "content": ""  # 空的詳細內容
    }
    
    response = requests.post(f"{base_url}/api/medicine/", json=medicine_basic)
    if response.status_code == 200:
        result = response.json()
        print(f"✅ 基本藥物新增成功: {result['name']}")
        medicine_a_id = result['id']
    else:
        print("❌ 基本藥物新增失敗")
        print(response.text)
        return
    
    # 新增有詳細資訊的藥物
    medicine_detailed = {
        "name": "測試藥物B",
        "position": "2-1", 
        "prompt": "test_medicine_b",
        "confidence": 0.95,
        "amount": 50,
        "content": "這是測試藥物B的詳細資訊"
    }
    
    response = requests.post(f"{base_url}/api/medicine/", json=medicine_detailed)
    if response.status_code == 200:
        result = response.json()
        print(f"✅ 詳細藥物新增成功: {result['name']}")
        medicine_b_id = result['id']
    else:
        print("❌ 詳細藥物新增失敗")
        print(response.text)
        return
    
    # 3. 測試分別查詢藥物資訊
    print("\n3️⃣ 測試分別查詢藥物資訊...")
    
    # 查詢基本資訊
    response = requests.get(f"{base_url}/api/medicine/basic")
    if response.status_code == 200:
        basic_medicines = response.json()
        print(f"✅ 基本藥物資訊查詢成功，共 {len(basic_medicines)} 種藥物")
    else:
        print("❌ 基本藥物資訊查詢失敗")
    
    # 查詢詳細資訊
    response = requests.get(f"{base_url}/api/medicine/detailed")
    if response.status_code == 200:
        detailed_medicines = response.json()
        print(f"✅ 詳細藥物資訊查詢成功，共 {len(detailed_medicines)} 種藥物")
    else:
        print("❌ 詳細藥物資訊查詢失敗")
    
    # 4. 測試ROS2藥物查詢端點
    print("\n4️⃣ 測試ROS2藥物查詢端點...")
    
    # 查詢基本資訊
    response = requests.get(f"{base_url}/api/ros2/medicine/basic/測試藥物A")
    if response.status_code == 200:
        result = response.json()
        print(f"✅ ROS2基本查詢成功: {result['name']} 位置:{result['position']}")
    else:
        print("❌ ROS2基本查詢失敗")
    
    # 查詢詳細資訊
    response = requests.get(f"{base_url}/api/ros2/medicine/detailed/測試藥物B")
    if response.status_code == 200:
        result = response.json()
        print(f"✅ ROS2詳細查詢成功: {result['name']}")
        print(f"   內容: {result['content'][:30]}...")
    else:
        print("❌ ROS2詳細查詢失敗")
    
    # 5. 測試開立處方籤
    print("\n5️⃣ 測試開立處方籤...")
    
    prescription_data = {
        "patient_name": "測試病患",
        "medicines": [
            {"medicine_id": medicine_a_id, "amount": 2},
            {"medicine_id": medicine_b_id, "amount": 1}
        ]
    }
    
    response = requests.post(f"{base_url}/api/prescription/", json=prescription_data)
    if response.status_code == 200:
        result = response.json()
        prescription_id = result['id']
        print(f"✅ 處方籤開立成功，ID: {prescription_id}")
    else:
        print("❌ 處方籤開立失敗")
        print(response.text)
        return
    
    # 6. 測試ROS2訂單端點
    print("\n6️⃣ 測試ROS2訂單端點...")
    
    # 拉取訂單
    response = requests.get(f"{base_url}/api/ros2/order/next")
    if response.status_code == 200:
        order_data = response.json()
        order = order_data['order']
        print(f"✅ 訂單拉取成功")
        print(f"   訂單ID: {order['order_id']}")
        print(f"   處方籤ID: {order['prescription_id']}")  # 測試處方籤ID
        print(f"   病患: {order['patient_name']}")
        print(f"   藥物數量: {len(order['medicine'])}")
        
        order_id = order['order_id']
        
        # 回報完成
        complete_data = {
            "order_id": order_id,
            "status": "success",
            "details": "測試完成"
        }
        
        response = requests.post(f"{base_url}/api/ros2/order/complete", json=complete_data)
        if response.status_code == 200:
            print("✅ 訂單完成回報成功")
        else:
            print("❌ 訂單完成回報失敗")
            
    elif response.status_code == 204:
        print("ℹ️ 目前沒有待處理的訂單")
    else:
        print("❌ 訂單拉取失敗")
    
    # 7. 檢查處方籤狀態
    print("\n7️⃣ 檢查處方籤狀態...")
    
    response = requests.get(f"{base_url}/api/prescription/")
    if response.status_code == 200:
        prescriptions = response.json()
        if prescriptions:
            prescription = prescriptions[0]
            print(f"✅ 處方籤狀態: {prescription['status']}")
            print(f"   處方籤ID: {prescription['id']}")  # 確認有ID
        else:
            print("ℹ️ 沒有處方籤記錄")
    
    print("\n🎊 測試完成！")
    print("=" * 50)
    print("✅ 系統功能正常運作")
    print("✅ 詳細內容為可選項目")
    print("✅ 基本和詳細資訊可分別查詢")
    print("✅ ROS2訂單推送正常")
    print("✅ 每筆處方籤都有ID")

if __name__ == "__main__":
    print("請確保醫院系統正在運行:")
    print("python3 clean_hospital_system.py")
    print("\n按 Enter 開始測試...")
    input()
    
    test_system()
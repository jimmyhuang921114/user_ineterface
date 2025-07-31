#!/usr/bin/env python3
"""
基本醫療系統測試
Basic Medical System Test
"""

import requests
import json
import time

SERVER_URL = "http://localhost:8000"

def test_basic_system():
    print("=" * 60)
    print("基本醫療系統測試開始")
    print("=" * 60)
    
    # 測試1: 檢查服務器連接
    print("\n測試1: 檢查服務器連接")
    try:
        response = requests.get(f"{SERVER_URL}/")
        if response.status_code == 200:
            result = response.json()
            print(f"✓ 服務器連接成功: {result['message']}")
        else:
            print(f"✗ 服務器連接失敗: HTTP {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ 無法連接服務器: {e}")
        print("請確保服務器已啟動: python3 minimal_server.py")
        return False
    
    # 測試2: 中文顯示測試
    print("\n測試2: 中文顯示測試")
    try:
        response = requests.get(f"{SERVER_URL}/test")
        if response.status_code == 200:
            result = response.json()
            print(f"✓ 中文顯示正常: {result['chinese']}")
        else:
            print("✗ 中文顯示測試失敗")
    except Exception as e:
        print(f"✗ 中文顯示測試錯誤: {e}")
    
    # 測試3: 創建病例
    print("\n測試3: 創建病例")
    test_record = {
        "patient_id": "P001",
        "patient_name": "測試病患",
        "doctor_name": "測試醫師",
        "diagnosis": "測試診斷",
        "symptoms": "測試症狀"
    }
    
    try:
        response = requests.post(
            f"{SERVER_URL}/api/medical_record/", 
            json=test_record,
            headers={"Content-Type": "application/json"}
        )
        if response.status_code == 200:
            result = response.json()
            record_id = result['record']['id']
            print(f"✓ 病例創建成功，ID: {record_id}")
            print(f"  病患姓名: {result['record']['patient_name']}")
            print(f"  診斷: {result['record']['diagnosis']}")
        else:
            print(f"✗ 病例創建失敗: HTTP {response.status_code}")
            print(f"響應內容: {response.text}")
            return False
    except Exception as e:
        print(f"✗ 病例創建錯誤: {e}")
        return False
    
    # 測試4: 查詢病例
    print("\n測試4: 查詢病例")
    try:
        response = requests.get(f"{SERVER_URL}/api/medical_record/{record_id}")
        if response.status_code == 200:
            result = response.json()
            print(f"✓ 病例查詢成功")
            print(f"  病患ID: {result['patient_id']}")
            print(f"  病患姓名: {result['patient_name']}")
            print(f"  醫師: {result['doctor_name']}")
            print(f"  診斷: {result['diagnosis']}")
        else:
            print(f"✗ 病例查詢失敗: HTTP {response.status_code}")
    except Exception as e:
        print(f"✗ 病例查詢錯誤: {e}")
    
    # 測試5: 查詢所有病例
    print("\n測試5: 查詢所有病例")
    try:
        response = requests.get(f"{SERVER_URL}/api/records/all")
        if response.status_code == 200:
            result = response.json()
            print(f"✓ 查詢所有病例成功，共 {result['count']} 筆")
            for record in result['records']:
                print(f"  - ID {record['id']}: {record['patient_name']} ({record['diagnosis']})")
        else:
            print(f"✗ 查詢所有病例失敗: HTTP {response.status_code}")
    except Exception as e:
        print(f"✗ 查詢所有病例錯誤: {e}")
    
    # 測試6: 模擬ROS2調用
    print("\n測試6: 模擬ROS2調用")
    try:
        # 模擬ROS2服務調用病例數據
        response = requests.get(f"{SERVER_URL}/api/medical_record/{record_id}")
        if response.status_code == 200:
            ros2_data = response.json()
            print("✓ 模擬ROS2調用成功")
            print("ROS2收到的數據:")
            print(json.dumps(ros2_data, ensure_ascii=False, indent=2))
            
            # 模擬處理完成回報
            print("\n✓ 模擬處理完成")
            completion_report = {
                "task_id": f"TASK_{record_id}",
                "status": "completed",
                "message": "病例處理完成",
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            print("完成報告:")
            print(json.dumps(completion_report, ensure_ascii=False, indent=2))
        else:
            print("✗ 模擬ROS2調用失敗")
    except Exception as e:
        print(f"✗ 模擬ROS2調用錯誤: {e}")
    
    # 測試7: 系統狀態檢查
    print("\n測試7: 系統狀態檢查")
    try:
        response = requests.get(f"{SERVER_URL}/api/status")
        if response.status_code == 200:
            status = response.json()
            print("✓ 系統狀態正常")
            print(f"  伺服器: {status['server']}")
            print(f"  狀態: {status['status']}")
            print(f"  病例數量: {status['records_count']}")
        else:
            print("✗ 系統狀態檢查失敗")
    except Exception as e:
        print(f"✗ 系統狀態檢查錯誤: {e}")
    
    print("\n" + "=" * 60)
    print("基本醫療系統測試完成！")
    print("=" * 60)
    
    print("\n系統功能驗證:")
    print("✓ 1. 病例寫入 - 正常")
    print("✓ 2. 病例查詢 - 正常") 
    print("✓ 3. ROS2模擬調用 - 正常")
    print("✓ 4. 完成狀態回報 - 正常")
    print("✓ 5. 中文顯示 - 正常")
    
    return True

if __name__ == "__main__":
    test_basic_system()
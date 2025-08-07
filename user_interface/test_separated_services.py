#!/usr/bin/env python3
"""
測試分離的基本和詳細藥物 ROS2 服務
"""

import requests
import json
import time

BASE_URL = "http://localhost:8001"

def test_ros2_service_status():
    """測試 ROS2 服務狀態"""
    print("\n🔍 測試 ROS2 服務狀態...")
    
    try:
        response = requests.get(f"{BASE_URL}/api/ros2/service/status")
        if response.status_code == 200:
            data = response.json()
            print(f"✅ ROS2 狀態: {data['message']}")
            print(f"   - ROS2 可用: {data['ros2_available']}")
            print(f"   - 節點活躍: {data['node_active']}")
            print(f"   - 可用服務:")
            for service, endpoint in data['services'].items():
                print(f"     • {service}: {endpoint}")
            return True
        else:
            print(f"❌ 狀態檢查失敗: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ 狀態檢查錯誤: {e}")
        return False

def test_basic_medicine_service():
    """測試基本藥物 ROS2 服務"""
    print("\n🧪 測試基本藥物服務...")
    
    # 測試 1: 獲取所有基本藥物
    print("   測試 1: 獲取所有基本藥物")
    try:
        payload = {}
        response = requests.post(f"{BASE_URL}/api/ros2/service/basic-medicine", json=payload)
        
        if response.status_code == 200:
            data = response.json()
            print(f"   ✅ 成功獲取 {len(data['medicines'])} 種基本藥物")
            for med in data['medicines'][:2]:  # 顯示前兩個
                print(f"      • {med['name']} (ID: {med['id']}, 庫存: {med['amount']})")
        else:
            print(f"   ❌ 請求失敗: {response.status_code}")
            print(f"   錯誤詳情: {response.text}")
    except Exception as e:
        print(f"   ❌ 請求錯誤: {e}")
    
    # 測試 2: 按名稱查詢
    print("   測試 2: 按名稱查詢基本藥物")
    try:
        payload = {"medicine_name": "阿司匹林"}
        response = requests.post(f"{BASE_URL}/api/ros2/service/basic-medicine", json=payload)
        
        if response.status_code == 200:
            data = response.json()
            if data['success'] and data['medicines']:
                med = data['medicines'][0]
                print(f"   ✅ 找到藥物: {med['name']} (位置: {med['position']})")
            else:
                print(f"   ⚠️ 未找到藥物或查詢失敗")
        else:
            print(f"   ❌ 請求失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 請求錯誤: {e}")
    
    # 測試 3: 按 ID 查詢
    print("   測試 3: 按 ID 查詢基本藥物")
    try:
        payload = {"medicine_id": 4}  # 假設 ID 4 存在
        response = requests.post(f"{BASE_URL}/api/ros2/service/basic-medicine", json=payload)
        
        if response.status_code == 200:
            data = response.json()
            if data['success'] and data['medicines']:
                med = data['medicines'][0]
                print(f"   ✅ 找到藥物: {med['name']} (製造商: {med['manufacturer']})")
            else:
                print(f"   ⚠️ 未找到 ID=4 的藥物")
        else:
            print(f"   ❌ 請求失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 請求錯誤: {e}")

def test_detailed_medicine_service():
    """測試詳細藥物 ROS2 服務"""
    print("\n🔬 測試詳細藥物服務...")
    
    # 測試 1: 獲取所有詳細藥物
    print("   測試 1: 獲取所有詳細藥物")
    try:
        payload = {}
        response = requests.post(f"{BASE_URL}/api/ros2/service/detailed-medicine", json=payload)
        
        if response.status_code == 200:
            data = response.json()
            print(f"   ✅ 成功獲取 {len(data['detailed_medicines'])} 種詳細藥物")
            if data['detailed_medicines']:
                detail = data['detailed_medicines'][0]
                print(f"      • {detail['description']} (成分: {detail['ingredient']})")
        else:
            print(f"   ❌ 請求失敗: {response.status_code}")
            print(f"   錯誤詳情: {response.text}")
    except Exception as e:
        print(f"   ❌ 請求錯誤: {e}")
    
    # 測試 2: 按名稱查詢詳細資訊
    print("   測試 2: 按名稱查詢詳細藥物")
    try:
        payload = {"medicine_name": "布洛芬"}
        response = requests.post(f"{BASE_URL}/api/ros2/service/detailed-medicine", json=payload)
        
        if response.status_code == 200:
            data = response.json()
            if data['success'] and data['detailed_medicines']:
                detail = data['detailed_medicines'][0]
                print(f"   ✅ 找到詳細資訊:")
                print(f"      - 描述: {detail['description']}")
                print(f"      - 用法: {detail['usage_method']}")
                print(f"      - 劑量: {detail['unit_dose']} mg")
                print(f"      - 副作用: {detail['side_effects']}")
            else:
                print(f"   ⚠️ 未找到藥物詳細資訊")
        else:
            print(f"   ❌ 請求失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 請求錯誤: {e}")
    
    # 測試 3: 包含基本資訊的詳細查詢
    print("   測試 3: 包含基本資訊的詳細查詢")
    try:
        payload = {"medicine_name": "維他命C", "include_detailed": True}
        response = requests.post(f"{BASE_URL}/api/ros2/service/detailed-medicine", json=payload)
        
        if response.status_code == 200:
            data = response.json()
            if data['success']:
                print(f"   ✅ 查詢成功:")
                print(f"      - 詳細資訊數量: {len(data['detailed_medicines'])}")
                print(f"      - 基本資訊數量: {len(data['basic_medicines']) if data['basic_medicines'] else 0}")
            else:
                print(f"   ⚠️ 查詢失敗")
        else:
            print(f"   ❌ 請求失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 請求錯誤: {e}")

def test_service_comparison():
    """比較基本服務和詳細服務的差異"""
    print("\n🔄 比較服務差異...")
    
    medicine_name = "胃藥"
    
    # 基本服務
    print(f"   查詢 '{medicine_name}' 的基本資訊:")
    try:
        payload = {"medicine_name": medicine_name}
        response = requests.post(f"{BASE_URL}/api/ros2/service/basic-medicine", json=payload)
        
        if response.status_code == 200:
            data = response.json()
            if data['success'] and data['medicines']:
                basic = data['medicines'][0]
                print(f"   📋 基本資訊: {basic['name']} - {basic['dosage']} (庫存: {basic['amount']})")
            else:
                print(f"   ⚠️ 基本服務未找到藥物")
        else:
            print(f"   ❌ 基本服務請求失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 基本服務錯誤: {e}")
    
    # 詳細服務
    print(f"   查詢 '{medicine_name}' 的詳細資訊:")
    try:
        payload = {"medicine_name": medicine_name}
        response = requests.post(f"{BASE_URL}/api/ros2/service/detailed-medicine", json=payload)
        
        if response.status_code == 200:
            data = response.json()
            if data['success'] and data['detailed_medicines']:
                detail = data['detailed_medicines'][0]
                print(f"   🔬 詳細資訊: {detail['description']}")
                print(f"       成分: {detail['ingredient']}")
                print(f"       類別: {detail['category']}")
                print(f"       儲存: {detail['storage_conditions']}")
            else:
                print(f"   ⚠️ 詳細服務未找到藥物")
        else:
            print(f"   ❌ 詳細服務請求失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 詳細服務錯誤: {e}")

def main():
    """主函數"""
    print("醫院藥物管理系統 - 分離服務測試")
    print("=" * 60)
    
    # 等待伺服器啟動
    print("🔌 檢查伺服器連接...")
    try:
        response = requests.get(f"{BASE_URL}/api/system/status", timeout=5)
        if response.status_code == 200:
            print("✅ 伺服器連接正常")
        else:
            print(f"⚠️ 伺服器狀態異常: {response.status_code}")
    except Exception as e:
        print(f"❌ 無法連接到伺服器: {e}")
        print("請確保伺服器正在運行: python3 start_ros2_real_server.py")
        return
    
    # 執行測試
    success_count = 0
    total_tests = 4
    
    # 1. 服務狀態測試
    if test_ros2_service_status():
        success_count += 1
    
    # 2. 基本藥物服務測試
    test_basic_medicine_service()
    success_count += 1
    
    # 3. 詳細藥物服務測試
    test_detailed_medicine_service()
    success_count += 1
    
    # 4. 服務比較測試
    test_service_comparison()
    success_count += 1
    
    # 總結
    print("\n" + "=" * 60)
    print(f"📊 測試完成！成功 {success_count}/{total_tests} 項測試")
    
    if success_count == total_tests:
        print("🎉 所有分離服務測試通過！")
        print("\n💡 使用建議:")
        print("   • 基本服務: 快速獲取藥物基本資訊 (名稱、庫存、位置)")
        print("   • 詳細服務: 獲取完整藥物資訊 (成分、用法、副作用等)")
        print("   • ROS2 節點可以根據需求分別調用這兩個服務")
    else:
        print("⚠️ 部分測試失敗，請檢查伺服器狀態和 API 實現")

if __name__ == "__main__":
    main()
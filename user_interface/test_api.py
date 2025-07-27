#!/usr/bin/env python3
"""
醫院藥物管理系統 API 測試腳本
"""

import requests
import json
from datetime import datetime

# API基礎URL
BASE_URL = "http://localhost:8000/api"

def test_medicine_api():
    """測試藥物管理API"""
    print("=== 醫院藥物管理系統 API 測試 ===\n")
    
    # 1. 測試獲取所有藥物（初始可能為空）
    print("1. 測試獲取所有藥物...")
    try:
        response = requests.get(f"{BASE_URL}/medicine/")
        print(f"   狀態碼: {response.status_code}")
        medicines = response.json()
        print(f"   當前藥物數量: {len(medicines)}")
        print(f"   藥物列表: {medicines}\n")
    except Exception as e:
        print(f"   錯誤: {e}\n")
    
    # 2. 測試新增藥物
    print("2. 測試新增藥物...")
    test_medicines = [
        {
            "name": "阿斯匹靈",
            "amount": 100,
            "usage_days": 30,
            "position": "A1-01"
        },
        {
            "name": "維他命C",
            "amount": 50,
            "usage_days": 60,
            "position": "A1-02"
        },
        {
            "name": "感冒糖漿",
            "amount": 25,
            "usage_days": 14,
            "position": "B2-03"
        }
    ]
    
    created_medicines = []
    for medicine in test_medicines:
        try:
            response = requests.post(f"{BASE_URL}/medicine/", json=medicine)
            if response.status_code == 200:
                created_medicine = response.json()
                created_medicines.append(created_medicine)
                print(f"   ✓ 成功新增: {medicine['name']} (ID: {created_medicine['id']})")
            else:
                print(f"   ✗ 新增失敗: {medicine['name']} - {response.text}")
        except Exception as e:
            print(f"   ✗ 新增失敗: {medicine['name']} - {e}")
    
    print(f"   成功新增 {len(created_medicines)} 個藥物\n")
    
    # 3. 測試獲取所有藥物（應該有數據了）
    print("3. 測試獲取更新後的藥物列表...")
    try:
        response = requests.get(f"{BASE_URL}/medicine/")
        medicines = response.json()
        print(f"   當前藥物數量: {len(medicines)}")
        for medicine in medicines:
            print(f"   - {medicine['name']}: {medicine['amount']}個, 位置: {medicine['position']}")
        print()
    except Exception as e:
        print(f"   錯誤: {e}\n")
    
    # 4. 測試搜尋功能
    print("4. 測試搜尋功能...")
    try:
        response = requests.get(f"{BASE_URL}/medicine/阿斯匹靈")
        if response.status_code == 200:
            medicine = response.json()
            print(f"   ✓ 找到藥物: {medicine['name']} - {medicine['amount']}個")
        else:
            print(f"   ✗ 搜尋失敗: {response.text}")
    except Exception as e:
        print(f"   錯誤: {e}")
    print()
    
    # 5. 測試更新功能
    if created_medicines:
        print("5. 測試更新功能...")
        try:
            medicine_id = created_medicines[0]['id']
            update_data = {"amount": 150}
            response = requests.put(f"{BASE_URL}/medicine/{medicine_id}", json=update_data)
            if response.status_code == 200:
                updated_medicine = response.json()
                print(f"   ✓ 更新成功: {updated_medicine['name']} 數量更新為 {updated_medicine['amount']}")
            else:
                print(f"   ✗ 更新失敗: {response.text}")
        except Exception as e:
            print(f"   錯誤: {e}")
        print()
    
    # 6. 測試JSON導出功能
    print("6. 測試JSON導出功能...")
    try:
        response = requests.get(f"{BASE_URL}/medicine/export/json")
        if response.status_code == 200:
            export_data = response.json()
            print(f"   ✓ 導出成功: 共 {export_data['total_medicines']} 個藥物")
            print(f"   ✓ 導出時間: {export_data['export_date']}")
            
            # 保存到文件
            filename = f"medicines_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(export_data, f, ensure_ascii=False, indent=2)
            print(f"   ✓ 已保存到文件: {filename}")
        else:
            print(f"   ✗ 導出失敗: {response.text}")
    except Exception as e:
        print(f"   錯誤: {e}")
    print()
    
    print("=== 測試完成 ===")
    print("如果所有測試都通過，說明你的前後端連接已經成功設置！")
    print("現在你可以訪問 http://localhost:8000/Medicine.html 來使用網頁界面")

if __name__ == "__main__":
    test_medicine_api()
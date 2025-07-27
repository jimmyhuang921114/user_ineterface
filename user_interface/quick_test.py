#!/usr/bin/env python3
import requests
import json
import time

def test_system():
    base_url = "http://localhost:8000"
    
    print("🧪 醫院藥物管理系統測試")
    print("=" * 40)
    
    # 等待伺服器啟動
    print("⏳ 等待伺服器啟動...")
    time.sleep(3)
    
    try:
        # 測試基本連接
        print("1. 測試基本連接...")
        response = requests.get(f"{base_url}/api/test", timeout=5)
        if response.status_code == 200:
            print("   ✅ 基本連接成功")
            print(f"   📊 {response.json()}")
        else:
            print(f"   ❌ 連接失敗: {response.status_code}")
            return
            
        # 測試獲取藥物列表
        print("2. 測試獲取藥物列表...")
        response = requests.get(f"{base_url}/api/medicine/", timeout=5)
        if response.status_code == 200:
            medicines = response.json()
            print(f"   ✅ 成功獲取 {len(medicines)} 個藥物")
            for med in medicines:
                print(f"   💊 {med['name']}: {med['amount']}個")
        else:
            print(f"   ❌ 獲取失敗: {response.status_code}")
            
        # 測試新增藥物
        print("3. 測試新增藥物...")
        new_medicine = {
            "name": "測試藥物",
            "amount": 20,
            "usage_days": 7,
            "position": "TEST-01"
        }
        response = requests.post(f"{base_url}/api/medicine/", json=new_medicine, timeout=5)
        if response.status_code == 200:
            created = response.json()
            print(f"   ✅ 成功新增藥物 ID: {created['id']}")
        else:
            print(f"   ❌ 新增失敗: {response.status_code}")
            
        # 測試JSON導出
        print("4. 測試JSON導出...")
        response = requests.get(f"{base_url}/api/medicine/export/json", timeout=5)
        if response.status_code == 200:
            export_data = response.json()
            print(f"   ✅ 導出成功: {export_data['total_medicines']} 個藥物")
        else:
            print(f"   ❌ 導出失敗: {response.status_code}")
            
        print("\n🎉 所有測試完成！")
        print("📱 你可以訪問以下頁面：")
        print(f"   🌐 主頁: {base_url}/")
        print(f"   📋 API文檔: {base_url}/docs")
        print(f"   💊 藥物管理: {base_url}/Medicine.html")
        
    except requests.exceptions.ConnectionError:
        print("❌ 無法連接到伺服器")
        print("💡 請確保伺服器正在運行")
    except Exception as e:
        print(f"❌ 測試失敗: {e}")

if __name__ == "__main__":
    test_system()
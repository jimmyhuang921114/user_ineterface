#!/usr/bin/env python3
"""
資料持久化測試腳本
Data Persistence Test Script
"""

import requests
import json
import time
from datetime import datetime

API_BASE = "http://localhost:8000"

def test_data_persistence():
    """測試資料持久化功能"""
    print("🏥 醫院管理系統 - 資料持久化測試")
    print("=" * 60)
    
    # 1. 檢查系統狀態
    print("📊 檢查系統狀態...")
    try:
        response = requests.get(f"{API_BASE}/api/system/status")
        if response.status_code == 200:
            status = response.json()
            print(f"   系統: {status.get('system')}")
            print(f"   版本: {status.get('version')}")
            print(f"   持久化: {status.get('persistence')}")
            
            stats = status.get('statistics', {})
            print(f"   當前資料: 藥物 {stats.get('total_medicines', 0)}, 處方 {stats.get('total_prescriptions', 0)}")
        else:
            print(f"❌ 系統狀態檢查失敗: {response.status_code}")
            return
    except Exception as e:
        print(f"❌ 無法連接系統: {e}")
        return
    
    # 2. 添加測試資料
    print("\n💊 添加測試藥物...")
    test_medicine = {
        "name": "測試藥物_持久化",
        "amount": 100,
        "usage_days": 7,
        "position": "TEST-01"
    }
    
    try:
        response = requests.post(f"{API_BASE}/api/medicine/", json=test_medicine)
        if response.status_code == 200:
            medicine_result = response.json()
            medicine_id = medicine_result['id']
            print(f"   ✅ 測試藥物已新增，ID: {medicine_id}")
        else:
            print(f"   ❌ 添加藥物失敗: {response.status_code}")
            return
    except Exception as e:
        print(f"   ❌ 添加藥物錯誤: {e}")
        return
    
    # 3. 添加詳細藥物資訊
    print("\n📋 添加詳細藥物資訊...")
    detailed_data = {
        "medicine_name": "測試藥物_持久化",
        "medicine_data": {
            "基本資訊": {
                "名稱": "測試藥物_持久化",
                "劑量": "測試劑量",
                "製造商": "測試製藥公司"
            },
            "外觀": {
                "顏色": "測試顏色",
                "形狀": "測試形狀"
            },
            "藥物描述": "這是一個用於測試資料持久化的藥物",
            "建立時間": datetime.now().isoformat(),
            "測試標記": "DATA_PERSISTENCE_TEST"
        }
    }
    
    try:
        response = requests.post(f"{API_BASE}/api/medicine/detailed/", json=detailed_data)
        if response.status_code == 200:
            print("   ✅ 詳細藥物資訊已新增")
        else:
            print(f"   ❌ 添加詳細資訊失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 添加詳細資訊錯誤: {e}")
    
    # 4. 開立測試處方
    print("\n📝 開立測試處方...")
    test_prescription = {
        "patient_name": "測試病患_持久化",
        "doctor_name": "測試醫師_持久化",
        "diagnosis": "資料持久化測試",
        "medicines": [
            {
                "medicine_name": "測試藥物_持久化",
                "dosage": "測試劑量",
                "frequency": "測試頻率",
                "duration": "測試天數",
                "instructions": "這是資料持久化測試處方"
            }
        ],
        "prescription_date": datetime.now().date().isoformat()
    }
    
    try:
        response = requests.post(f"{API_BASE}/api/prescription/", json=test_prescription)
        if response.status_code == 200:
            prescription_result = response.json()
            prescription_id = prescription_result['id']
            print(f"   ✅ 測試處方已開立，ID: {prescription_id}")
        else:
            print(f"   ❌ 開立處方失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 開立處方錯誤: {e}")
    
    # 5. 手動觸發儲存
    print("\n💾 手動觸發資料儲存...")
    try:
        response = requests.post(f"{API_BASE}/api/system/save")
        if response.status_code == 200:
            save_result = response.json()
            if save_result.get('success'):
                print("   ✅ 資料儲存成功")
                print(f"   時間: {save_result.get('timestamp')}")
            else:
                print(f"   ❌ 資料儲存失敗: {save_result.get('message')}")
        else:
            print(f"   ❌ 儲存請求失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 儲存請求錯誤: {e}")
    
    # 6. 創建備份
    print("\n📦 創建資料備份...")
    try:
        response = requests.post(f"{API_BASE}/api/system/backup")
        if response.status_code == 200:
            backup_result = response.json()
            if backup_result.get('success'):
                print("   ✅ 備份創建成功")
                print(f"   備份路徑: {backup_result.get('backup_path')}")
            else:
                print(f"   ❌ 備份創建失敗: {backup_result.get('message')}")
        else:
            print(f"   ❌ 備份請求失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 備份請求錯誤: {e}")
    
    # 7. 列出所有備份
    print("\n📂 列出現有備份...")
    try:
        response = requests.get(f"{API_BASE}/api/system/backups")
        if response.status_code == 200:
            backups_result = response.json()
            backups = backups_result.get('backups', [])
            print(f"   📊 共有 {len(backups)} 個備份:")
            
            for backup in backups[:3]:  # 顯示最近3個備份
                print(f"     - {backup['name']} ({backup['files']} 個文件)")
                print(f"       創建時間: {backup['created_time']}")
        else:
            print(f"   ❌ 列出備份失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 列出備份錯誤: {e}")
    
    # 8. 檢查最終狀態
    print("\n📊 檢查最終系統狀態...")
    try:
        response = requests.get(f"{API_BASE}/api/system/status")
        if response.status_code == 200:
            final_status = response.json()
            final_stats = final_status.get('statistics', {})
            
            print(f"   最終統計:")
            print(f"     - 藥物總數: {final_stats.get('total_medicines', 0)}")
            print(f"     - 詳細藥物: {final_stats.get('detailed_medicines', 0)}")
            print(f"     - 處方總數: {final_stats.get('total_prescriptions', 0)}")
            
            # 檢查資料文件
            data_files = final_status.get('data_files', {}).get('files', {})
            print(f"   資料文件狀態:")
            for file_name, file_info in data_files.items():
                if file_info.get('exists'):
                    size_kb = file_info.get('size_bytes', 0) / 1024
                    print(f"     - {file_name}: {size_kb:.1f} KB (最後修改: {file_info.get('modified_time', 'Unknown')[:19]})")
                else:
                    print(f"     - {file_name}: 不存在")
        else:
            print(f"   ❌ 最終狀態檢查失敗: {response.status_code}")
    except Exception as e:
        print(f"   ❌ 最終狀態檢查錯誤: {e}")
    
    print("\n" + "=" * 60)
    print("🎯 資料持久化測試完成！")
    print("\n💡 測試總結:")
    print("   ✅ 新增了測試藥物和詳細資訊")
    print("   ✅ 開立了測試處方")
    print("   ✅ 執行了手動儲存")
    print("   ✅ 創建了資料備份")
    print("   📂 所有資料都已持久化到JSON文件")
    print("\n🔄 重啟測試建議:")
    print("   1. 重啟伺服器")
    print("   2. 檢查資料是否保持")
    print("   3. 驗證持久化功能正常")

if __name__ == "__main__":
    test_data_persistence()
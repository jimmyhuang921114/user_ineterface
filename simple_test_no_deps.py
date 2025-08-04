#!/usr/bin/env python3
"""
簡化的系統狀態檢查腳本（無外部依賴）
"""

import os
import json
from pathlib import Path

def check_files():
    """檢查重要文件是否存在"""
    print("🔍 檢查文件狀態...")
    print("=" * 40)
    
    # 檢查主要文件
    files_to_check = [
        ("user_interface/main.py", "主服務器"),
        ("user_interface/fixed_server.py", "修改後的服務器"),
        ("user_interface/static/html/doctor.html", "醫生界面"),
        ("user_interface/static/html/medicine_integrated.html", "整合藥物管理"),
        ("user_interface/static/html/simple_test.html", "簡化測試頁面"),
        ("user_interface/static/html/Prescription.html", "處方籤管理"),
        ("user_interface/static/js/doctor.js", "醫生界面JS"),
        ("user_interface/static/js/Prescription.js", "處方籤JS"),
        ("user_interface/static/css/unified_style.css", "統一樣式"),
    ]
    
    for file_path, description in files_to_check:
        if os.path.exists(file_path):
            size = os.path.getsize(file_path)
            print(f"✅ {description}: {file_path} ({size} bytes)")
        else:
            print(f"❌ {description}: {file_path} (缺失)")
    
    print()

def check_json_data():
    """檢查JSON數據文件"""
    print("📊 檢查數據文件...")
    print("=" * 40)
    
    json_files = [
        ("user_interface/medicine_basic_data.json", "基本藥物資料"),
        ("user_interface/medicine_detailed_data.json", "詳細藥物資料"),
        ("user_interface/prescription_data.json", "處方籤資料")
    ]
    
    for file_path, description in json_files:
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                if isinstance(data, list):
                    print(f"✅ {description}: {len(data)} 筆資料")
                else:
                    print(f"✅ {description}: 存在（非陣列格式）")
            except Exception as e:
                print(f"⚠️ {description}: 讀取錯誤 - {e}")
        else:
            print(f"🆕 {description}: 尚未創建（首次使用時會自動創建）")
    
    print()

def check_api_endpoints():
    """檢查API端點定義（靜態檢查）"""
    print("🔌 檢查API端點定義...")
    print("=" * 40)
    
    server_file = "user_interface/fixed_server.py"
    if os.path.exists(server_file):
        try:
            with open(server_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 檢查重要的API端點
            endpoints = [
                ('@app.post("/api/medicine/basic")', '基本藥物POST'),
                ('@app.get("/api/medicine/basic")', '基本藥物GET'),
                ('@app.post("/api/medicine/detailed")', '詳細藥物POST'),
                ('@app.get("/api/medicine/detailed")', '詳細藥物GET'),
                ('@app.get("/api/ros2/medicine/basic")', 'ROS2基本藥物'),
                ('@app.get("/api/ros2/medicine/detailed")', 'ROS2詳細藥物'),
                ('@app.get("/api/ros2/prescription")', 'ROS2處方籤'),
                ('@app.get("/api/ros2/medicine/integrated/{medicine_name}")', 'ROS2整合API')
            ]
            
            for endpoint, description in endpoints:
                if endpoint in content:
                    print(f"✅ {description}: 已定義")
                else:
                    print(f"❌ {description}: 未找到")
        except Exception as e:
            print(f"❌ 讀取服務器文件錯誤: {e}")
    else:
        print(f"❌ 服務器文件不存在: {server_file}")
    
    print()

def check_integration_page():
    """檢查整合頁面內容"""
    print("🎨 檢查整合頁面...")
    print("=" * 40)
    
    page_file = "user_interface/static/html/medicine_integrated.html"
    if os.path.exists(page_file):
        try:
            with open(page_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 檢查關鍵內容
            checks = [
                ('整合藥物資料填寫', '標題'),
                ('基本藥物資料', '基本資料區塊'),
                ('詳細藥物資訊', '詳細資料區塊'),
                ('getElementById(\'medicineName\')', '藥物名稱欄位'),
                ('getElementById(\'amount\')', '數量欄位'),
                ('getElementById(\'position\')', '位置欄位'),
                ('api/medicine/basic', '基本藥物API調用'),
                ('api/medicine/detailed', '詳細藥物API調用')
            ]
            
            for check_text, description in checks:
                if check_text in content:
                    print(f"✅ {description}: 已實現")
                else:
                    print(f"❌ {description}: 未找到")
        except Exception as e:
            print(f"❌ 讀取頁面文件錯誤: {e}")
    else:
        print(f"❌ 整合頁面文件不存在: {page_file}")
    
    print()

def generate_usage_report():
    """生成使用報告"""
    print("📋 使用指南...")
    print("=" * 40)
    
    print("🚀 啟動系統:")
    print("   cd user_interface")
    print("   python3 main.py")
    print()
    
    print("🌐 訪問頁面:")
    print("   📋 整合藥物管理: http://localhost:8000/medicine_integrated.html")
    print("   👨‍⚕️ 醫生界面: http://localhost:8000/doctor.html")
    print("   📊 處方籤管理: http://localhost:8000/Prescription.html")
    print("   🧪 功能測試: http://localhost:8000/simple_test.html")
    print("   📖 API文檔: http://localhost:8000/docs")
    print()
    
    print("🤖 ROS2 API端點:")
    print("   GET /api/ros2/medicine/basic - 獲取基本藥物資料")
    print("   GET /api/ros2/medicine/detailed - 獲取詳細藥物資料")
    print("   GET /api/ros2/prescription - 獲取處方籤資料")
    print("   GET /api/ros2/medicine/integrated/{name} - 獲取整合資料")
    print()
    
    print("💾 數據存儲:")
    print("   📁 medicine_basic_data.json - 基本藥物資料")
    print("   📁 medicine_detailed_data.json - 詳細藥物資料")
    print("   📁 prescription_data.json - 處方籤資料")
    print()

def main():
    """主函數"""
    print("🎯 醫院藥物管理系統 - 雙JSON存儲檢查")
    print("=" * 60)
    print()
    
    check_files()
    check_json_data()
    check_api_endpoints()
    check_integration_page()
    generate_usage_report()
    
    print("🎉 檢查完成！")
    print("=" * 60)

if __name__ == "__main__":
    main()
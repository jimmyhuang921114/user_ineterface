#!/usr/bin/env python3
"""
醫院管理系統 - 模組化架構啟動腳本
Hospital Management System - Modular Architecture Startup Script
"""

import subprocess
import sys
import os
import time
import signal
from pathlib import Path

def check_dependencies():
    """檢查系統依賴"""
    print("🔍 檢查系統依賴...")
    
    required_packages = ['fastapi', 'uvicorn', 'pydantic']
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package)
            print(f"   ✅ {package}")
        except ImportError:
            missing_packages.append(package)
            print(f"   ❌ {package} (缺少)")
    
    if missing_packages:
        print(f"\n⚠️  缺少依賴套件: {', '.join(missing_packages)}")
        print("請執行: pip install fastapi uvicorn pydantic")
        return False
    
    print("✅ 所有依賴套件已安裝")
    return True

def cleanup_old_processes():
    """清理舊的進程"""
    print("🧹 清理舊的伺服器進程...")
    try:
        # 終止所有 uvicorn 進程
        subprocess.run(['pkill', '-f', 'uvicorn'], capture_output=True)
        subprocess.run(['pkill', '-f', 'modular_server'], capture_output=True)
        time.sleep(2)
        print("✅ 舊進程已清理")
    except Exception as e:
        print(f"⚠️  清理進程時出現錯誤: {e}")

def start_server():
    """啟動模組化伺服器"""
    print("🚀 啟動模組化醫院管理系統...")
    
    # 確保在正確的目錄
    script_dir = Path(__file__).parent
    os.chdir(script_dir)
    
    try:
        # 啟動伺服器
        process = subprocess.Popen([
            sys.executable, '-m', 'uvicorn',
            'modular_server:app',
            '--host', '0.0.0.0',
            '--port', '8000',
            '--reload'
        ])
        
        print(f"🔄 伺服器正在啟動... (PID: {process.pid})")
        return process
        
    except Exception as e:
        print(f"❌ 啟動失敗: {e}")
        return None

def test_api_connection():
    """測試API連接"""
    print("🔗 測試API連接...")
    
    import requests
    import time
    
    api_base = "http://localhost:8000"
    max_attempts = 10
    
    for attempt in range(1, max_attempts + 1):
        try:
            response = requests.get(f"{api_base}/api/system/status", timeout=5)
            if response.status_code == 200:
                data = response.json()
                print(f"✅ API連接成功!")
                print(f"   系統: {data.get('system', 'Unknown')}")
                print(f"   版本: {data.get('version', 'Unknown')}")
                print(f"   架構: {data.get('architecture', 'Unknown')}")
                return True
            else:
                print(f"   嘗試 {attempt}/{max_attempts}: HTTP {response.status_code}")
        except requests.exceptions.RequestException as e:
            print(f"   嘗試 {attempt}/{max_attempts}: 連接中...")
        
        if attempt < max_attempts:
            time.sleep(3)
    
    print("⚠️  API連接測試失敗，但伺服器可能仍在啟動中")
    return False

def print_access_info():
    """印出訪問資訊"""
    print("\n" + "=" * 60)
    print("🏥 醫院管理系統 - 模組化架構")
    print("=" * 60)
    print("📍 系統訪問地址:")
    print("   🏠 主頁:         http://localhost:8000")
    print("   🩺 醫生工作台:   http://localhost:8000/doctor.html")
    print("   💊 藥物管理:     http://localhost:8000/Medicine.html") 
    print("   📋 處方管理:     http://localhost:8000/Prescription.html")
    print("   📚 API文檔:      http://localhost:8000/docs")
    print("\n🎯 系統特色:")
    print("   ✨ 模組化API架構 - 藥物與處方API分離")
    print("   🎨 統一繁體中文風格")
    print("   🔧 自由格式詳細資訊輸入")
    print("   📊 處方表格支援多列藥物")
    print("=" * 60)
    print("💡 使用提示:")
    print("   1. 先在醫生工作台建立基本藥物資訊")
    print("   2. 再選擇藥物填寫詳細資訊")
    print("   3. 使用處方功能開立處方")
    print("   4. 按 Ctrl+C 停止伺服器")
    print("=" * 60)

def main():
    """主函數"""
    print("🏥 醫院管理系統 - 模組化架構啟動器")
    print("=" * 50)
    
    # 檢查依賴
    if not check_dependencies():
        sys.exit(1)
    
    # 清理舊進程
    cleanup_old_processes()
    
    # 啟動伺服器
    server_process = start_server()
    if not server_process:
        sys.exit(1)
    
    # 等待伺服器啟動
    time.sleep(5)
    
    # 測試連接
    test_api_connection()
    
    # 印出訪問資訊
    print_access_info()
    
    # 等待用戶中斷
    try:
        print("\n⌨️  按 Ctrl+C 停止伺服器...")
        server_process.wait()
    except KeyboardInterrupt:
        print("\n🛑 正在停止伺服器...")
        server_process.terminate()
        try:
            server_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server_process.kill()
        print("✅ 伺服器已停止")

if __name__ == "__main__":
    main()
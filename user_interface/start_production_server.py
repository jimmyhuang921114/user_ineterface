#!/usr/bin/env python3
"""
醫院藥物管理系統 - 正式版啟動腳本
生產環境版本，不含測試資料
"""

import os
import subprocess
import sys
import signal
import time

def check_requirements():
    """檢查系統需求"""
    print("🔍 檢查系統需求...")
    
    # 檢查 Python 版本
    if sys.version_info < (3, 7):
        print("❌ 需要 Python 3.7 或更高版本")
        return False
    
    # 檢查必要模組
    required_modules = ['fastapi', 'uvicorn', 'sqlalchemy', 'pydantic']
    missing_modules = []
    
    for module in required_modules:
        try:
            __import__(module)
            print(f"✅ {module}")
        except ImportError:
            missing_modules.append(module)
            print(f"❌ {module} - 未安裝")
    
    if missing_modules:
        print(f"\n請安裝缺少的模組: pip install {' '.join(missing_modules)}")
        return False
    
    return True

def kill_existing_servers():
    """終止現有的服務器進程"""
    print("🔄 檢查並終止現有服務器...")
    try:
        # 找到占用 8001 端口的進程
        result = subprocess.run(['lsof', '-ti:8001'], capture_output=True, text=True)
        if result.stdout.strip():
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                try:
                    os.kill(int(pid), signal.SIGTERM)
                    print(f"✅ 終止進程 {pid}")
                except ProcessLookupError:
                    pass
            time.sleep(2)
    except FileNotFoundError:
        # lsof 不可用，使用其他方法
        subprocess.run(['pkill', '-f', 'simple_server'], capture_output=True)

def start_production_server():
    """啟動正式版服務器"""
    print("\n🚀 啟動醫院藥物管理系統 - 正式版")
    print("=" * 50)
    
    if not check_requirements():
        print("❌ 系統需求檢查失敗")
        return False
    
    kill_existing_servers()
    
    print("\n🏥 初始化正式版數據庫...")
    try:
        # 初始化數據庫
        subprocess.run([sys.executable, 'database_production.py'], check=True)
        print("✅ 數據庫初始化完成")
    except subprocess.CalledProcessError:
        print("❌ 數據庫初始化失敗")
        return False
    
    print("\n🌐 啟動 Web 服務器...")
    print("注意: 此為正式版本，不含任何測試資料")
    print("=" * 50)
    
    try:
        # 啟動服務器
        subprocess.run([sys.executable, 'simple_server_production.py'])
    except KeyboardInterrupt:
        print("\n\n🛑 服務器已停止")
    except Exception as e:
        print(f"❌ 服務器啟動失敗: {e}")
        return False
    
    return True

def show_usage():
    """顯示使用說明"""
    print("醫院藥物管理系統 - 正式版")
    print("=" * 40)
    print()
    print("🌐 網頁界面:")
    print("   整合管理: http://localhost:8001/integrated_medicine_management.html")
    print("   醫生工作台: http://localhost:8001/doctor.html")
    print("   處方籤管理: http://localhost:8001/Prescription.html")
    print("   API 文檔: http://localhost:8001/docs")
    print()
    print("🤖 ROS2 功能:")
    print("   ✅ 一次處理一個訂單")
    print("   ✅ 基本藥物服務")
    print("   ✅ 詳細藥物服務")
    print()
    print("📊 特色:")
    print("   ✅ 生產環境就緒")
    print("   ✅ 不含測試資料")
    print("   ✅ 簡化表單")
    print("   ✅ 自動庫存管理")
    print()

if __name__ == "__main__":
    show_usage()
    input("按 Enter 開始啟動服務器...")
    start_production_server()
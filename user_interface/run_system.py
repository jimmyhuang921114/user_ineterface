#!/usr/bin/env python3
"""
醫院藥物管理系統 - 快速啟動腳本
Hospital Medicine Management System - Quick Start Script
"""

import subprocess
import sys
import time
import requests
import os
from pathlib import Path

def check_dependencies():
    """檢查必要的依賴"""
    print("🔍 檢查系統依賴...")
    
    required_packages = ['fastapi', 'uvicorn', 'requests']
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package)
            print(f"  ✅ {package}")
        except ImportError:
            missing_packages.append(package)
            print(f"  ❌ {package} (缺失)")
    
    if missing_packages:
        print(f"\n📦 安裝缺失的依賴: {', '.join(missing_packages)}")
        try:
            subprocess.check_call([
                sys.executable, '-m', 'pip', 'install', '--break-system-packages'
            ] + missing_packages)
            print("✅ 依賴安裝完成")
        except subprocess.CalledProcessError:
            print("❌ 依賴安裝失敗，請手動安裝")
            return False
    
    return True

def start_server():
    """啟動伺服器"""
    print("\n🚀 啟動醫院藥物管理系統...")
    
    # 確保在正確的目錄
    script_dir = Path(__file__).parent
    os.chdir(script_dir)
    
    try:
        # 啟動伺服器
        process = subprocess.Popen([
            sys.executable, 'hospital_server.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        print("⏳ 等待伺服器啟動...")
        time.sleep(5)
        
        # 測試連接
        try:
            response = requests.get('http://localhost:8000/api/test', timeout=5)
            if response.status_code == 200:
                print("✅ 伺服器啟動成功！")
                print_access_info()
                return process
            else:
                print(f"❌ 伺服器響應異常: {response.status_code}")
        except requests.exceptions.RequestException as e:
            print(f"❌ 無法連接到伺服器: {e}")
        
        # 如果測試失敗，終止進程
        process.terminate()
        return None
        
    except Exception as e:
        print(f"❌ 啟動失敗: {e}")
        return None

def print_access_info():
    """顯示訪問資訊"""
    print("\n" + "="*60)
    print("🏥 醫院藥物管理系統已啟動")
    print("="*60)
    print("🌐 訪問地址:")
    print("   主頁:         http://localhost:8000/")
    print("   藥物管理:     http://localhost:8000/Medicine.html")
    print("   處方籤:       http://localhost:8000/Prescription.html")
    print("   醫生界面:     http://localhost:8000/doctor.html")
    print("   API文檔:      http://localhost:8000/docs")
    print("\n📱 API端點:")
    print("   測試:         http://localhost:8000/api/test")
    print("   藥物列表:     http://localhost:8000/api/medicine/")
    print("   JSON導出:     http://localhost:8000/api/medicine/export/json")
    print("\n💡 功能特色:")
    print("   ✅ 前後端完整連接")
    print("   ✅ 即時資料同步")
    print("   ✅ JSON格式查詢導出")
    print("   ✅ 響應式界面設計")
    print("   ✅ 中文完整支援")
    print("="*60)

def test_system():
    """測試系統功能"""
    print("\n🧪 測試系統功能...")
    
    tests = [
        ("基本連接", "http://localhost:8000/api/test"),
        ("藥物列表", "http://localhost:8000/api/medicine/"),
        ("JSON導出", "http://localhost:8000/api/medicine/export/json")
    ]
    
    for test_name, url in tests:
        try:
            response = requests.get(url, timeout=5)
            if response.status_code == 200:
                print(f"  ✅ {test_name}: 正常")
            else:
                print(f"  ❌ {test_name}: 異常 ({response.status_code})")
        except Exception as e:
            print(f"  ❌ {test_name}: 失敗 ({e})")

def main():
    """主函數"""
    print("🏥 醫院藥物管理系統 - 快速啟動")
    print("Hospital Medicine Management System")
    print("="*50)
    
    # 檢查依賴
    if not check_dependencies():
        print("❌ 依賴檢查失敗，退出")
        sys.exit(1)
    
    # 啟動伺服器
    server_process = start_server()
    
    if server_process:
        # 測試系統
        test_system()
        
        print("\n🎉 系統準備就緒！")
        print("💡 按 Ctrl+C 停止伺服器")
        
        try:
            # 等待用戶中斷
            server_process.wait()
        except KeyboardInterrupt:
            print("\n🛑 正在停止伺服器...")
            server_process.terminate()
            print("👋 系統已停止")
    else:
        print("❌ 系統啟動失敗")
        sys.exit(1)

if __name__ == "__main__":
    main()
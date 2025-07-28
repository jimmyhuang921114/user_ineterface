#!/usr/bin/env python3
"""
醫院藥物管理系統 - 本地啟動腳本
Hospital Medicine Management System - Local Startup Script
"""

import os
import sys
import subprocess
import time
import signal
import requests
from pathlib import Path

def check_dependencies():
    """檢查並安裝依賴"""
    print("檢查系統依賴...")
    
    required_packages = ['fastapi', 'uvicorn', 'pydantic']
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package)
            print(f"  成功: {package}")
        except ImportError:
            print(f"  缺失: {package}")
            missing_packages.append(package)
    
    if missing_packages:
        print(f"\n安裝缺失的依賴: {', '.join(missing_packages)}")
        try:
            subprocess.check_call([
                sys.executable, '-m', 'pip', 'install', 
                '--user', *missing_packages, 'requests'
            ])
            print("成功: 依賴安裝完成")
        except subprocess.CalledProcessError:
            print("錯誤: 依賴安裝失敗，請手動安裝：")
            print(f"pip3 install {' '.join(missing_packages)} requests")
            return False
    
    return True

def find_server_file():
    """尋找可用的伺服器檔案"""
    current_dir = Path(__file__).parent
    
    server_files = [
        ('enhanced_server.py', '增強版伺服器（推薦）'),
        ('hospital_server.py', '醫院伺服器'),
        ('final_server.py', '最終版本伺服器'),
        ('working_server.py', '工作伺服器'),
        ('main.py', '主伺服器')
    ]
    
    for filename, description in server_files:
        file_path = current_dir / filename
        if file_path.exists():
            print(f"找到伺服器檔案: {description}")
            return file_path
    
    print("錯誤: 找不到任何伺服器檔案")
    return None

def kill_existing_servers():
    """停止現有的伺服器進程"""
    print("清理現有進程...")
    try:
        # 嘗試停止uvicorn進程
        subprocess.run(['pkill', '-f', 'uvicorn'], 
                      capture_output=True, check=False)
        subprocess.run(['pkill', '-f', 'python.*server'], 
                      capture_output=True, check=False)
        time.sleep(2)
    except Exception:
        pass

def test_server():
    """測試伺服器是否正常運行"""
    max_attempts = 10
    for attempt in range(max_attempts):
        try:
            response = requests.get('http://localhost:8000/api/test', timeout=2)
            if response.status_code == 200:
                return True
        except requests.exceptions.RequestException:
            pass
        
        if attempt < max_attempts - 1:
            print(f"  測試 {attempt + 1}/{max_attempts}...")
            time.sleep(1)
    
    return False

def main():
    """主函數"""
    print("醫院藥物管理系統 - 本地啟動")
    print("=" * 50)
    
    # 獲取當前目錄
    current_dir = Path(__file__).parent
    print(f"當前目錄: {current_dir}")
    
    # 改變工作目錄
    os.chdir(current_dir)
    
    # 檢查依賴
    if not check_dependencies():
        return False
    
    # 停止現有伺服器
    kill_existing_servers()
    
    # 尋找伺服器檔案
    server_file = find_server_file()
    if not server_file:
        return False
    
    # 啟動伺服器
    print(f"\n啟動伺服器: {server_file.name}")
    try:
        # 啟動伺服器進程
        process = subprocess.Popen([
            sys.executable, str(server_file)
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        print(f"伺服器PID: {process.pid}")
        print("等待伺服器啟動...")
        
        # 等待伺服器啟動
        time.sleep(3)
        
        # 測試連接
        print("測試API連接...")
        if test_server():
            print("成功: API連接正常")
        else:
            print("警告: API連接測試失敗，但伺服器可能仍在啟動中")
        
        # 顯示訪問資訊
        print("\n" + "=" * 50)
        print("成功: 伺服器啟動完成")
        print("\n訪問地址:")
        print("  主頁: http://localhost:8000")
        print("  API文檔: http://localhost:8000/docs")
        print("  藥物管理: http://localhost:8000/Medicine.html")
        print("  病人管理: http://localhost:8000/Patients.html")
        print("  病例記錄: http://localhost:8000/Records.html")
        print("=" * 50)
        print("按 Ctrl+C 停止伺服器")
        
        # 等待用戶中斷
        try:
            process.wait()
        except KeyboardInterrupt:
            print("\n正在停止伺服器...")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
            print("伺服器已停止")
        
        return True
        
    except Exception as e:
        print(f"錯誤: 啟動伺服器失敗: {e}")
        return False

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n用戶中斷")
        sys.exit(0)
    except Exception as e:
        print(f"未預期的錯誤: {e}")
        sys.exit(1)
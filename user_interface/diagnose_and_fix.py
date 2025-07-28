#!/usr/bin/env python3
"""
醫院藥物管理系統診斷和修復工具
"""

import os
import sys
import subprocess
import time
import requests
import socket
from pathlib import Path

def check_port(port=8000):
    """檢查端口是否被佔用"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            result = s.connect_ex(('localhost', port))
            return result == 0  # 0表示端口被佔用
    except Exception:
        return False

def kill_port_process(port=8000):
    """殺死佔用端口的進程"""
    try:
        # 查找佔用端口的進程
        result = subprocess.run(['lsof', '-ti', f':{port}'], 
                              capture_output=True, text=True)
        if result.stdout.strip():
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    print(f"殺死進程 PID: {pid}")
                    subprocess.run(['kill', '-9', pid])
            time.sleep(2)
            return True
    except Exception as e:
        print(f"無法殺死進程: {e}")
    return False

def test_server_simple():
    """簡單測試伺服器"""
    try:
        response = requests.get('http://localhost:8000/', timeout=5)
        return response.status_code == 200
    except Exception:
        return False

def test_server_api():
    """測試API端點"""
    try:
        response = requests.get('http://localhost:8000/api/test', timeout=5)
        return response.status_code == 200
    except Exception:
        return False

def run_server_with_output(server_file):
    """運行伺服器並顯示輸出"""
    print(f"啟動伺服器: {server_file}")
    print("=" * 50)
    
    try:
        # 直接運行不捕獲輸出，讓用戶看到錯誤
        process = subprocess.Popen([
            sys.executable, server_file
        ])
        
        # 等待一段時間讓伺服器啟動
        time.sleep(5)
        
        # 檢查進程是否還在運行
        if process.poll() is None:
            print("伺服器進程正在運行")
            return process
        else:
            print("伺服器進程已退出")
            return None
            
    except Exception as e:
        print(f"啟動伺服器失敗: {e}")
        return None

def diagnose_system():
    """診斷系統問題"""
    print("系統診斷報告")
    print("=" * 50)
    
    # 1. 檢查Python版本
    print(f"Python版本: {sys.version}")
    
    # 2. 檢查當前目錄
    current_dir = Path.cwd()
    print(f"當前目錄: {current_dir}")
    
    # 3. 檢查伺服器檔案
    server_files = ['enhanced_server.py', 'final_server.py', 'hospital_server.py']
    available_servers = []
    for filename in server_files:
        if (current_dir / filename).exists():
            available_servers.append(filename)
            print(f"找到伺服器: {filename}")
    
    if not available_servers:
        print("錯誤: 找不到任何伺服器檔案")
        return None
    
    # 4. 檢查依賴
    required_packages = ['fastapi', 'uvicorn', 'pydantic']
    missing_packages = []
    for package in required_packages:
        try:
            __import__(package)
            print(f"依賴檢查: {package} ✓")
        except ImportError:
            print(f"依賴檢查: {package} ✗")
            missing_packages.append(package)
    
    if missing_packages:
        print(f"缺失依賴: {missing_packages}")
        return None
    
    # 5. 檢查端口
    port_occupied = check_port(8000)
    print(f"端口8000狀態: {'被佔用' if port_occupied else '可用'}")
    
    return available_servers[0]  # 返回第一個可用的伺服器

def main():
    """主函數"""
    print("醫院藥物管理系統 - 診斷和修復工具")
    print("=" * 60)
    
    # 診斷系統
    server_file = diagnose_system()
    if not server_file:
        print("診斷失敗，請檢查上述問題")
        return False
    
    print(f"\n使用伺服器: {server_file}")
    
    # 清理端口
    if check_port(8000):
        print("清理佔用的端口...")
        kill_port_process(8000)
    
    # 啟動伺服器
    print("\n啟動伺服器...")
    process = run_server_with_output(server_file)
    
    if not process:
        print("伺服器啟動失敗")
        return False
    
    # 測試連接
    print("\n測試連接...")
    max_attempts = 15
    for i in range(max_attempts):
        print(f"  嘗試 {i+1}/{max_attempts}...")
        
        if test_server_simple():
            print("✓ 主頁連接成功")
            break
        elif test_server_api():
            print("✓ API連接成功")
            break
        elif i < max_attempts - 1:
            time.sleep(2)
    else:
        print("⚠ 連接測試失敗，但伺服器可能仍在啟動")
    
    # 顯示訪問資訊
    print("\n" + "=" * 60)
    print("伺服器狀態:")
    print(f"  PID: {process.pid}")
    print(f"  端口: 8000")
    print("\n訪問地址:")
    print("  主頁: http://localhost:8000")
    print("  API文檔: http://localhost:8000/docs")
    print("  藥物管理: http://localhost:8000/Medicine.html")
    print("=" * 60)
    print("按 Ctrl+C 停止伺服器")
    
    try:
        # 等待用戶中斷
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

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用戶中斷")
    except Exception as e:
        print(f"未預期的錯誤: {e}")
        import traceback
        traceback.print_exc()
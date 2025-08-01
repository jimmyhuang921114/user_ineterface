#!/usr/bin/env python3
"""
醫院藥物管理系統啟動腳本
Hospital Medicine Management System Startup Script
"""

import subprocess
import sys
import os

def main():
    print("醫院藥物管理系統 - 啟動腳本")
    print("=" * 50)
    
    # 檢查Python版本
    if sys.version_info < (3, 7):
        print("錯誤: 需要Python 3.7或更高版本")
        sys.exit(1)
    
    # 檢查依賴
    try:
        import fastapi
        import uvicorn
        print("✓ 依賴檢查通過")
    except ImportError as e:
        print(f"✗ 缺少依賴: {e}")
        print("請執行: pip install fastapi uvicorn")
        sys.exit(1)
    
    # 啟動伺服器
    print("\n啟動伺服器...")
    print("網址: http://localhost:8000")
    print("API文檔: http://localhost:8000/docs")
    print("藥物管理: http://localhost:8000/Medicine.html")
    print("醫生管理: http://localhost:8000/doctor.html")
    print("處方管理: http://localhost:8000/Prescription.html")
    print("\n按 Ctrl+C 停止伺服器")
    print("=" * 50)
    
    try:
        # 啟動伺服器
        subprocess.run([
            sys.executable, "-m", "uvicorn", 
            "fixed_server:app", 
            "--host", "0.0.0.0", 
            "--port", "8000", 
            "--reload"
        ])
    except KeyboardInterrupt:
        print("\n\n伺服器已停止")
    except Exception as e:
        print(f"\n啟動失敗: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
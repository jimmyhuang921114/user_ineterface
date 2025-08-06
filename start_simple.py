#!/usr/bin/env python3
"""
Simple Medicine Management System Startup Script
簡化藥物管理系統啟動腳本
"""

import subprocess
import sys
import os
from pathlib import Path

def main():
    print("🏥 簡化醫院藥物管理系統")
    print("=" * 50)
    
    # Change to user_interface directory
    user_interface_dir = Path(__file__).parent / "user_interface"
    
    if not user_interface_dir.exists():
        print("❌ user_interface 目錄不存在")
        sys.exit(1)
    
    os.chdir(user_interface_dir)
    
    try:
        print("🚀 啟動伺服器...")
        print("🌐 藥物管理: http://localhost:8000/Medicine.html")
        print("📋 處方籤管理: http://localhost:8000/Prescription.html")
        print("👨‍⚕️ 醫生界面: http://localhost:8000/doctor.html")
        print("📖 API文檔: http://localhost:8000/docs")
        print("=" * 50)
        print("按 Ctrl+C 停止系統")
        print("=" * 50)
        
        # Start the simple server
        subprocess.run([sys.executable, "simple_server.py"], check=True)
        
    except KeyboardInterrupt:
        print("\n🛑 系統已停止")
    except Exception as e:
        print(f"❌ 啟動錯誤: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
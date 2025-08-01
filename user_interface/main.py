#!/usr/bin/env python3
"""
Hospital Medicine Management System - Main Entry Point
醫院藥物管理系統 - 主要入口點
"""

from fixed_server import app
import uvicorn

if __name__ == "__main__":
    print("🏥 醫院藥物管理系統")
    print("=" * 50)
    print("🌐 網頁界面: http://localhost:8000/Medicine.html")
    print("📋 處方籤管理: http://localhost:8000/Prescription.html")
    print("👨‍⚕️ 醫生界面: http://localhost:8000/doctor.html")
    print("📖 API文檔: http://localhost:8000/docs")
    print("=" * 50)
    
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    except KeyboardInterrupt:
        print("\n🛑 系統已停止")
    except Exception as e:
        print(f"❌ 啟動錯誤: {e}")
        exit(1)
#!/usr/bin/env python3
"""
Clean Server Startup Script
乾淨版本啟動腳本
"""

import sys
import os

# 確保可以導入乾淨版本的模組
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 導入乾淨版本的伺服器
from simple_server_clean import app

if __name__ == "__main__":
    import uvicorn
    
    print("醫院藥物管理系統 - 乾淨版本")
    print("=" * 50)
    print("整合管理: http://localhost:8001/integrated_medicine_management.html")
    print("處方籤管理: http://localhost:8001/Prescription.html")
    print("醫生界面: http://localhost:8001/doctor.html")
    print("API文檔: http://localhost:8001/docs")
    print("=" * 50)
    print("注意：此版本不包含測試資料")
    print("請透過 API 或網頁界面添加藥物資料")
    print("=" * 50)
    
    uvicorn.run(app, host="0.0.0.0", port=8001, log_level="info")
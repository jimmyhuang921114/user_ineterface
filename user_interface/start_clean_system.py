#!/usr/bin/env python3
"""
Clean System Starter
乾淨的系統啟動器 - 無 emoji 版本
"""

import subprocess
import sys
import os
import time

def check_python_version():
    """檢查 Python 版本"""
    if sys.version_info < (3, 8):
        print("錯誤: 需要 Python 3.8 或更高版本")
        print(f"當前版本: {sys.version}")
        return False
    return True

def check_dependencies():
    """檢查必要的相依性"""
    required_modules = ['fastapi', 'uvicorn', 'sqlalchemy', 'requests']
    missing_modules = []
    
    for module in required_modules:
        try:
            __import__(module)
        except ImportError:
            missing_modules.append(module)
    
    if missing_modules:
        print(f"錯誤: 缺少必要模組: {', '.join(missing_modules)}")
        print("請執行: pip install fastapi uvicorn sqlalchemy requests")
        return False
    
    return True

def check_required_files():
    """檢查必要檔案"""
    required_files = [
        'simple_server_clean.py',
        'database_clean.py',
        'ros2_mock_clean.py'
    ]
    
    for file in required_files:
        if not os.path.exists(file):
            print(f"錯誤: 找不到必要檔案 {file}")
            return False
    
    return True

def start_server(server_type="full"):
    """啟動伺服器"""
    if server_type == "full":
        script = "simple_server_clean.py"
        print("啟動完整系統 (含 ROS2 模擬)")
    else:
        script = "simple_server_no_ros2.py"
        print("啟動純 API 系統 (無 ROS2)")
    
    try:
        print(f"執行: python3 {script}")
        print("=" * 50)
        print("系統資訊:")
        print("- 主頁面: http://localhost:8001/")
        print("- 整合管理: http://localhost:8001/integrated_medicine_management.html")
        print("- 處方籤管理: http://localhost:8001/Prescription.html")
        print("- 醫生工作站: http://localhost:8001/doctor.html")
        print("- API 文檔: http://localhost:8001/docs")
        print("=" * 50)
        print("按 Ctrl+C 停止系統")
        print("")
        
        subprocess.run([sys.executable, script], check=True)
        
    except KeyboardInterrupt:
        print("\n系統已停止")
    except subprocess.CalledProcessError as e:
        print(f"啟動失敗: {e}")
        return False
    except Exception as e:
        print(f"發生錯誤: {e}")
        return False
    
    return True

def main():
    """主函數"""
    print("醫院藥物管理系統")
    print("=" * 50)
    
    # 檢查系統需求
    if not check_python_version():
        sys.exit(1)
    
    if not check_dependencies():
        sys.exit(1)
    
    if not check_required_files():
        sys.exit(1)
    
    print("系統需求檢查通過")
    
    # 選擇系統類型
    print("\n請選擇系統類型:")
    print("1. 完整系統 (含 ROS2 模擬)")
    print("2. 純 API 系統 (無 ROS2)")
    
    while True:
        choice = input("請輸入選項 (1-2): ").strip()
        if choice == "1":
            if start_server("full"):
                sys.exit(0)
            else:
                sys.exit(1)
        elif choice == "2":
            if start_server("api_only"):
                sys.exit(0)
            else:
                sys.exit(1)
        else:
            print("無效選項，請輸入 1 或 2")

if __name__ == "__main__":
    main()
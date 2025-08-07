#!/usr/bin/env python3
"""
Start Hospital Medicine Management System (No ROS2 Version)
啟動醫院藥物管理系統 - 無 ROS2 版本
"""

import subprocess
import sys
import os

def main():
    print("🏥 啟動醫院藥物管理系統 (無 ROS2 版本)")
    print("=" * 50)
    
    # 檢查 Python 版本
    if sys.version_info < (3, 8):
        print("❌ 錯誤: 需要 Python 3.8 或更高版本")
        sys.exit(1)
    
    print(f"✅ Python 版本: {sys.version}")
    
    # 檢查必要檔案
    required_files = [
        "simple_server_no_ros2.py",
        "database_clean.py"
    ]
    
    for file in required_files:
        if not os.path.exists(file):
            print(f"❌ 錯誤: 找不到必要檔案 {file}")
            sys.exit(1)
    
    print("✅ 必要檔案檢查完成")
    
    # 啟動伺服器
    print("🚀 啟動無 ROS2 伺服器...")
    print("=" * 50)
    
    try:
        subprocess.run([
            sys.executable, 
            "simple_server_no_ros2.py"
        ], check=True)
    except KeyboardInterrupt:
        print("\n⏹️  伺服器已停止")
    except Exception as e:
        print(f"❌ 啟動失敗: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
🚀 Qt藥物管理系統啟動腳本
"""

import sys
import os
import subprocess
import time
from pathlib import Path

def check_dependencies():
    """檢查依賴"""
    print("🔍 檢查系統依賴...")
    
    # 檢查Python版本
    if sys.version_info < (3, 8):
        print("❌ 需要Python 3.8或更高版本")
        return False
        
    # 檢查必要的套件
    required_packages = [
        'PyQt6',
        'PyYAML',
        'rclpy',
        'numpy',
        'pandas'
    ]
    
    missing_packages = []
    for package in required_packages:
        try:
            __import__(package)
            print(f"✅ {package}")
        except ImportError:
            missing_packages.append(package)
            print(f"❌ {package}")
            
    if missing_packages:
        print(f"\n❌ 缺少以下套件: {', '.join(missing_packages)}")
        print("請執行: pip install -r requirements.txt")
        return False
        
    return True

def check_ros2():
    """檢查ROS2環境"""
    print("\n🤖 檢查ROS2環境...")
    
    try:
        # 檢查ROS2是否安裝
        result = subprocess.run(['ros2', '--version'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✅ ROS2版本: {result.stdout.strip()}")
            return True
        else:
            print("❌ ROS2未正確安裝")
            return False
    except (subprocess.TimeoutExpired, FileNotFoundError):
        print("❌ ROS2未安裝或不在PATH中")
        return False

def setup_environment():
    """設置環境"""
    print("\n⚙️ 設置環境...")
    
    # 創建必要的目錄
    directories = ['data', 'backups', 'exports', 'logs']
    for directory in directories:
        Path(directory).mkdir(exist_ok=True)
        print(f"✅ 創建目錄: {directory}")
        
    # 檢查資料檔案
    data_files = ['data/medicines.yaml', 'data/prescriptions.yaml']
    for file_path in data_files:
        if not Path(file_path).exists():
            print(f"⚠️ 資料檔案不存在: {file_path}")
        else:
            print(f"✅ 資料檔案存在: {file_path}")

def start_ros2_nodes():
    """啟動ROS2節點（可選）"""
    print("\n🚀 啟動ROS2節點...")
    
    # 這裡可以啟動相關的ROS2節點
    # 例如：訂單處理器、藥物資訊提供者等
    
    print("ℹ️ 請手動啟動相關的ROS2節點（如果需要）")
    print("例如:")
    print("  ros2 run medicine_order_processor order_processor_node")
    print("  ros2 run medicine_info_provider medicine_info_node")

def main():
    """主函數"""
    print("🏥 Qt藥物管理系統啟動器")
    print("=" * 50)
    
    # 檢查依賴
    if not check_dependencies():
        print("\n❌ 依賴檢查失敗，請安裝缺少的套件")
        return 1
        
    # 檢查ROS2
    ros2_available = check_ros2()
    if not ros2_available:
        print("\n⚠️ ROS2不可用，系統將在離線模式下運行")
        
    # 設置環境
    setup_environment()
    
    # 啟動ROS2節點（可選）
    if ros2_available:
        start_ros2_nodes()
        
    print("\n🎯 啟動Qt應用程序...")
    
    try:
        # 導入並啟動主應用程序
        from main import main as start_app
        start_app()
    except ImportError as e:
        print(f"❌ 無法導入主應用程序: {e}")
        return 1
    except Exception as e:
        print(f"❌ 應用程序啟動失敗: {e}")
        return 1
        
    return 0

if __name__ == "__main__":
    sys.exit(main())
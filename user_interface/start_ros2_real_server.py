#!/usr/bin/env python3
"""
醫院藥物管理系統 - 真實 ROS2 版本啟動器
需要真實的 ROS2 環境
"""

import os
import sys
import subprocess
import signal
import time

def check_dependencies():
    """檢查依賴套件"""
    required_packages = [
        'fastapi', 'uvicorn', 'sqlalchemy', 'requests'
    ]
    
    missing_packages = []
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            missing_packages.append(package)
    
    if missing_packages:
        print("醫院藥物管理系統 - 真實 ROS2 版本")
        print("==================================================")
        print("錯誤: 缺少必要模組:", ', '.join(missing_packages))
        print("請執行: pip install", ' '.join(missing_packages))
        return False
    
    return True

def check_ros2_environment():
    """檢查 ROS2 環境"""
    try:
        import rclpy
        print("ROS2 環境檢查通過")
        return True
    except ImportError:
        print("錯誤: 無法找到 ROS2 環境")
        print("請確保:")
        print("1. 已安裝 ROS2 (Humble/Iron/Rolling)")
        print("2. 已執行: source /opt/ros/humble/setup.bash")
        print("3. 已安裝: pip install rclpy")
        return False

def kill_existing_servers():
    """停止現有的伺服器進程"""
    try:
        # 查找並停止 Python 伺服器進程
        result = subprocess.run(['pgrep', '-f', 'python.*simple_server'], 
                              capture_output=True, text=True)
        if result.stdout:
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    try:
                        subprocess.run(['kill', '-9', pid], check=True)
                        print(f"已停止進程 PID: {pid}")
                    except subprocess.CalledProcessError:
                        pass
        
        # 查找並停止佔用 8001 端口的進程
        result = subprocess.run(['lsof', '-ti:8001'], 
                              capture_output=True, text=True)
        if result.stdout:
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    try:
                        subprocess.run(['kill', '-9', pid], check=True)
                        print(f"已停止佔用端口 8001 的進程 PID: {pid}")
                    except subprocess.CalledProcessError:
                        pass
                        
        time.sleep(2)  # 等待進程完全停止
        
    except FileNotFoundError:
        # 如果 pgrep 或 lsof 不存在，忽略錯誤
        pass

def start_server():
    """啟動真實 ROS2 版本伺服器"""
    print("醫院藥物管理系統 - 真實 ROS2 版本")
    print("==================================================")
    print("啟動中...")
    print("注意: 此版本需要真實的 ROS2 環境")
    print("==================================================")
    
    try:
        # 啟動伺服器
        process = subprocess.Popen([
            sys.executable, 'simple_server_ros2_real.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
           universal_newlines=True, bufsize=1)
        
        # 處理信號
        def signal_handler(signum, frame):
            print("\n正在關閉伺服器...")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # 即時輸出
        for line in process.stdout:
            print(line.rstrip())
            
    except KeyboardInterrupt:
        print("\n伺服器已停止")
    except Exception as e:
        print(f"啟動失敗: {e}")
        return False
    
    return True

def main():
    """主函數"""
    if not check_dependencies():
        sys.exit(1)
    
    if not check_ros2_environment():
        print("\n替代方案:")
        print("- 使用含模擬器版本: python3 start_clean_server.py")
        print("- 使用純 API 版本: python3 start_no_ros2_server.py")
        sys.exit(1)
    
    # 停止現有伺服器
    kill_existing_servers()
    
    # 啟動伺服器
    if not start_server():
        sys.exit(1)

if __name__ == "__main__":
    main()
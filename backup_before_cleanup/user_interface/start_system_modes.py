#!/usr/bin/env python3
"""
醫院藥物管理系統 - 多模式啟動腳本
支援不同的 ROS2 整合模式
"""

import os
import sys
import time
import signal
import subprocess
import threading

def start_web_server():
    """啟動 Web 服務器"""
    print("🌐 啟動 Web 服務器...")
    try:
        # 停止現有服務器
        subprocess.run(['pkill', '-f', 'simple_server_final'], capture_output=True)
        time.sleep(2)
        
        # 啟動新服務器
        server_process = subprocess.Popen([
            sys.executable, 'simple_server_final.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # 等待服務器啟動
        time.sleep(5)
        
        print("✅ Web 服務器已啟動")
        return server_process
        
    except Exception as e:
        print(f"❌ Web 服務器啟動失敗: {e}")
        return None

def start_python_integration():
    """啟動 Python 推送模式"""
    from integration_example import YourROS2System
    from ros2_order_pusher import OrderPusher
    
    print("🤖 啟動 Python 推送模式...")
    
    # 建立 ROS2 系統
    ros2_system = YourROS2System()
    
    # 建立訂單推送器
    pusher = OrderPusher(
        fastapi_base_url="http://localhost:8001",
        callback_func=ros2_system.process_order
    )
    
    # 將 pusher 引用傳遞給 ROS2 系統
    ros2_system._order_pusher = pusher
    
    # 開始監控
    pusher.start_monitoring()
    
    print("✅ Python 推送模式已啟動")
    return pusher, ros2_system

def start_ros2_services():
    """啟動 ROS2 服務模式"""
    print("🤖 啟動 ROS2 服務模式...")
    
    # 啟動 ROS2 服務節點
    ros2_process = subprocess.Popen([
        sys.executable, 'ros2_services_interface.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(3)
    
    # 啟動 ROS2 客戶端示例
    client_process = subprocess.Popen([
        sys.executable, 'ros2_client_example.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    print("✅ ROS2 服務模式已啟動")
    return ros2_process, client_process

def display_menu():
    """顯示選單"""
    print("\n" + "=" * 80)
    print("🏥 醫院藥物管理系統 - 模式選擇")
    print("=" * 80)
    print("請選擇 ROS2 整合模式:")
    print()
    print("1️⃣  Python 推送模式 (推薦)")
    print("   • 系統自動推送訂單到您的 Python 函數")
    print("   • 適合直接整合到現有 Python 程式")
    print("   • YAML 格式輸出")
    print()
    print("2️⃣  ROS2 服務模式")
    print("   • 使用標準 ROS2 服務和 Topic")
    print("   • 適合標準 ROS2 節點整合")
    print("   • YAML 格式通信")
    print()
    print("3️⃣  僅 Web 系統")
    print("   • 只啟動 Web 界面")
    print("   • 手動測試和查看")
    print()
    print("🌐 Web 界面 (所有模式都包含):")
    print("   • 藥物管理: http://localhost:8001/integrated_medicine_management.html")
    print("   • 醫生工作台: http://localhost:8001/doctor.html")
    print("   • 處方籤管理: http://localhost:8001/Prescription.html")
    print("   • API 文檔: http://localhost:8001/docs")
    print("=" * 80)

def main():
    """主函數"""
    display_menu()
    
    server_process = None
    mode_processes = []
    pusher = None
    
    try:
        # 獲取用戶選擇
        while True:
            try:
                choice = input("\n請選擇模式 (1/2/3): ").strip()
                if choice in ['1', '2', '3']:
                    break
                else:
                    print("❌ 無效選擇，請輸入 1、2 或 3")
            except KeyboardInterrupt:
                print("\n👋 再見！")
                return
        
        # 1. 啟動 Web 服務器
        server_process = start_web_server()
        if not server_process:
            print("❌ 無法啟動 Web 服務器，退出")
            return
        
        print(f"\n🚀 啟動模式 {choice}...")
        
        if choice == '1':
            # Python 推送模式
            pusher, ros2_system = start_python_integration()
            
            print("\n✅ Python 推送模式已啟動！")
            print("📋 測試流程:")
            print("   1. 到 Web 界面新增藥物")
            print("   2. 開立處方籤")
            print("   3. 查看終端機的處理日誌")
            print("   4. 系統會自動推送 YAML 格式訂單")
            
        elif choice == '2':
            # ROS2 服務模式
            ros2_process, client_process = start_ros2_services()
            mode_processes = [ros2_process, client_process]
            
            print("\n✅ ROS2 服務模式已啟動！")
            print("📋 測試流程:")
            print("   1. 到 Web 界面新增藥物和開立處方籤")
            print("   2. ROS2 客戶端會自動獲取訂單")
            print("   3. 查看 ROS2 節點的 YAML 輸出")
            print("   4. 可以使用 ros2 命令行工具測試")
            
        elif choice == '3':
            # 僅 Web 系統
            print("\n✅ Web 系統已啟動！")
            print("📋 測試流程:")
            print("   1. 使用 Web 界面管理藥物和處方籤")
            print("   2. 查看 API 文檔: http://localhost:8001/docs")
            print("   3. 手動測試各種功能")
        
        print("\n🛑 按 Ctrl+C 停止系統")
        print("⏱️ 系統運行中...")
        
        # 3. 主循環 - 顯示狀態
        while True:
            time.sleep(5)
            
            # 檢查服務器是否還在運行
            if server_process and server_process.poll() is not None:
                print("\n❌ Web 服務器已停止")
                break
            
            # 檢查其他進程
            for process in mode_processes:
                if process.poll() is not None:
                    print(f"\n❌ 進程已停止: {process}")
                    break
            
            if choice == '1' and pusher:
                status = pusher.get_status()
                print(f"\r🔄 Python 模式狀態 | " +
                      f"已處理: {status['processed_count']} | " +
                      f"ROS2: {'🔴忙碌' if status['ros2_busy'] else '🟢空閒'} | " +
                      f"當前訂單: {status['current_order_id'] or '無'}", 
                      end="", flush=True)
            else:
                print(f"\r🔄 系統運行中...", end="", flush=True)
    
    except KeyboardInterrupt:
        print("\n\n🛑 正在停止系統...")
        
    finally:
        # 清理資源
        if pusher:
            pusher.stop_monitoring()
            print("✅ Python 推送器已停止")
        
        for process in mode_processes:
            try:
                process.terminate()
                process.wait(timeout=5)
            except:
                pass
        
        if server_process:
            server_process.terminate()
            server_process.wait(timeout=5)
            print("✅ Web 服務器已停止")
        
        # 確保清理所有相關進程
        subprocess.run(['pkill', '-f', 'simple_server_final'], capture_output=True)
        subprocess.run(['pkill', '-f', 'ros2_services_interface'], capture_output=True)
        subprocess.run(['pkill', '-f', 'ros2_client_example'], capture_output=True)
        
        print("✅ 系統已完全停止")

if __name__ == "__main__":
    main()
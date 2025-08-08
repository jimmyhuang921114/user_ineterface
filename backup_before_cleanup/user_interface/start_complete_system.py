#!/usr/bin/env python3
"""
完整醫院藥物管理系統啟動腳本
同時啟動 Web 服務器和 ROS2 訂單推送器
"""

import os
import sys
import time
import signal
import subprocess
import threading
from integration_example import YourROS2System
from ros2_order_pusher import OrderPusher

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

def start_ros2_integration():
    """啟動 ROS2 整合系統"""
    print("🤖 啟動 ROS2 整合系統...")
    
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
    
    print("✅ ROS2 整合系統已啟動")
    return pusher, ros2_system

def display_system_info():
    """顯示系統資訊"""
    print("\n" + "=" * 80)
    print("🏥 醫院藥物管理系統 - 完整版")
    print("=" * 80)
    print("🌐 Web 界面:")
    print("   • 藥物管理: http://localhost:8001/integrated_medicine_management.html")
    print("   • 醫生工作台: http://localhost:8001/doctor.html")
    print("   • 處方籤管理: http://localhost:8001/Prescription.html")
    print("   • ROS2 客戶端: http://localhost:8001/ros2_client.html")
    print("   • API 文檔: http://localhost:8001/docs")
    print()
    print("🤖 ROS2 整合:")
    print("   • 自動監控新處方籤")
    print("   • 自動轉換為 YAML 格式訂單")
    print("   • 單一訂單處理（一次一個）")
    print("   • 等待 ROS2 完成後處理下一個")
    print("   • 自動狀態同步")
    print()
    print("📋 測試流程:")
    print("   1. 到藥物管理界面新增藥物")
    print("   2. 到醫生工作台開立處方籤")
    print("   3. 系統會自動推送訂單給 ROS2")
    print("   4. 查看終端機的 ROS2 處理日誌")
    print("   5. 到處方籤管理查看狀態更新")
    print()
    print("🛑 按 Ctrl+C 停止整個系統")
    print("=" * 80)

def main():
    """主函數"""
    display_system_info()
    
    server_process = None
    pusher = None
    
    try:
        # 1. 啟動 Web 服務器
        server_process = start_web_server()
        if not server_process:
            print("❌ 無法啟動 Web 服務器，退出")
            return
        
        # 2. 啟動 ROS2 整合
        pusher, ros2_system = start_ros2_integration()
        
        print("\n✅ 完整系統已啟動！")
        print("⏱️ 系統運行中...")
        
        # 3. 主循環 - 顯示狀態
        while True:
            time.sleep(5)
            
            if pusher:
                status = pusher.get_status()
                
                # 檢查服務器是否還在運行
                if server_process and server_process.poll() is not None:
                    print("\n❌ Web 服務器已停止")
                    break
                
                # 顯示狀態
                print(f"\r🔄 系統狀態 | " +
                      f"已處理訂單: {status['processed_count']} | " +
                      f"ROS2: {'🔴忙碌' if status['ros2_busy'] else '🟢空閒'} | " +
                      f"當前訂單: {status['current_order_id'] or '無'}", 
                      end="", flush=True)
    
    except KeyboardInterrupt:
        print("\n\n🛑 正在停止系統...")
        
    finally:
        # 清理資源
        if pusher:
            pusher.stop_monitoring()
            print("✅ ROS2 整合系統已停止")
        
        if server_process:
            server_process.terminate()
            server_process.wait(timeout=5)
            print("✅ Web 服務器已停止")
        
        # 確保清理所有相關進程
        subprocess.run(['pkill', '-f', 'simple_server_final'], capture_output=True)
        
        print("✅ 系統已完全停止")

if __name__ == "__main__":
    main()
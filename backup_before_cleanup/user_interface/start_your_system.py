#!/usr/bin/env python3
"""
您的自動推送訂單系統啟動器
專為您的 ROS2 節點設計

功能：
1. 啟動 Web 服務器
2. 啟動訂單推送器
3. 啟動您的 ROS2 節點
4. 完整整合，自動推送一次一個訂單
"""

import subprocess
import sys
import time
import signal
import threading
import os
from typing import List


class YourSystemLauncher:
    """您的系統啟動器"""
    
    def __init__(self):
        self.processes: List[subprocess.Popen] = []
        self.ros2_node = None
        self.order_pusher = None
        self.shutdown_event = threading.Event()
        
    def check_dependencies(self) -> bool:
        """檢查系統依賴"""
        print("🔍 檢查系統依賴...")
        
        # 檢查 ROS2
        try:
            result = subprocess.run(['ros2', '--version'], 
                                 capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print(f"✅ ROS2: {result.stdout.strip()}")
            else:
                print("❌ ROS2 未安裝或配置不正確")
                return False
        except Exception as e:
            print(f"❌ ROS2 檢查失敗: {e}")
            return False
        
        # 檢查必要檔案
        required_files = [
            'database_final.py',
            'simple_server_final.py',
            'ros2_order_pusher.py',
            'your_ros2_node.py'
        ]
        
        for file in required_files:
            if not os.path.exists(file):
                print(f"❌ 缺少必要檔案: {file}")
                return False
        
        print("✅ 所有依賴檢查通過")
        return True
    
    def start_web_server(self):
        """啟動 Web 服務器"""
        print("🌐 啟動 Web 服務器...")
        
        try:
            process = subprocess.Popen([
                sys.executable, 'simple_server_final.py'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            self.processes.append(process)
            
            # 等待服務器啟動
            time.sleep(3)
            
            if process.poll() is None:
                print("✅ Web 服務器啟動成功")
                print("🌐 Web 界面可用:")
                print("   • 藥物管理: http://localhost:8001/integrated_medicine_management.html")
                print("   • 醫生工作台: http://localhost:8001/doctor.html")
                print("   • 處方籤管理: http://localhost:8001/Prescription.html")
                return True
            else:
                print("❌ Web 服務器啟動失敗")
                return False
                
        except Exception as e:
            print(f"❌ Web 服務器啟動錯誤: {e}")
            return False
    
    def start_ros2_and_pusher(self):
        """啟動 ROS2 節點和訂單推送器"""
        print("🤖 啟動 ROS2 訂單處理系統...")
        
        try:
            # 導入模組
            import rclpy
            from your_ros2_node import YourROS2OrderHandler
            from ros2_order_pusher import OrderPusher
            
            # 初始化 ROS2
            rclpy.init()
            
            # 創建您的 ROS2 節點
            self.ros2_node = YourROS2OrderHandler()
            
            # 創建訂單推送器
            def your_order_callback(order_dict, yaml_order):
                """您的訂單處理回調函數"""
                self.ros2_node.process_order(order_dict, yaml_order)
            
            self.order_pusher = OrderPusher(callback_func=your_order_callback)
            
            # 連接節點和推送器
            self.ros2_node.set_order_pusher(self.order_pusher)
            
            # 啟動訂單監控
            self.order_pusher.start_monitoring()
            
            print("✅ ROS2 節點啟動成功")
            print("✅ 訂單推送器啟動成功")
            print("🔄 系統已準備接收和處理訂單")
            
            return True
            
        except Exception as e:
            print(f"❌ ROS2 系統啟動錯誤: {e}")
            return False
    
    def run_ros2_spin(self):
        """運行 ROS2 spin"""
        try:
            import rclpy
            while not self.shutdown_event.is_set() and rclpy.ok():
                rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
        except Exception as e:
            print(f"❌ ROS2 spin 錯誤: {e}")
    
    def print_system_status(self):
        """打印系統狀態"""
        print("\n" + "=" * 60)
        print("🎉 您的自動推送訂單系統已啟動！")
        print("=" * 60)
        
        print("\n📋 系統功能:")
        print("✅ 自動監控新處方籤")
        print("✅ 自動推送 YAML 訂單到您的 ROS2 節點")
        print("✅ 一次處理一個訂單")
        print("✅ 處理完成後自動進行下一個")
        print("✅ 完整的 Web 管理界面")
        
        print("\n🌐 Web 界面:")
        print("• 藥物管理: http://localhost:8001/integrated_medicine_management.html")
        print("• 醫生工作台: http://localhost:8001/doctor.html")
        print("• 處方籤管理: http://localhost:8001/Prescription.html")
        print("• API 文檔: http://localhost:8001/docs")
        
        print("\n🧪 測試方法:")
        print("• 基本測試: python3 test_order_flow.py basic")
        print("• 批量測試: python3 test_order_flow.py batch 3")
        print("• 查看處方籤: python3 test_order_flow.py list")
        
        print("\n📄 訂單格式 (您會收到的 YAML):")
        print("order_id: \"000001\"")
        print("prescription_id: 1")
        print("patient_name: \"張三\"")
        print("medicine:")
        print("  - name: 阿斯匹靈")
        print("    amount: 10")
        print("    locate: [2, 3]")
        print("    prompt: tablet")
        
        print("\n🔧 您的 ROS2 節點:")
        print("• 節點名稱: your_order_handler")
        print("• 自動接收訂單推送")
        print("• 在 process_medicine() 中添加您的機器人邏輯")
        print("• 完成後自動告知網站")
        
        print("\n⚠️ 重要提醒:")
        print("• 在 your_ros2_node.py 的 process_medicine() 中實現您的機器人邏輯")
        print("• 移除 time.sleep() 模擬代碼")
        print("• 確保調用 self.complete_order() 來完成訂單")
        
        print("\n🛑 停止系統: Ctrl+C")
        print("=" * 60)
    
    def signal_handler(self, signum, frame):
        """信號處理器"""
        print("\n🛑 收到停止信號，正在關閉系統...")
        self.shutdown()
    
    def shutdown(self):
        """關閉系統"""
        print("🔄 關閉系統中...")
        
        # 設置關閉事件
        self.shutdown_event.set()
        
        # 停止訂單推送器
        if self.order_pusher:
            try:
                self.order_pusher.stop_monitoring()
                print("✅ 訂單推送器已停止")
            except Exception as e:
                print(f"⚠️ 停止訂單推送器時發生錯誤: {e}")
        
        # 停止 ROS2
        if self.ros2_node:
            try:
                self.ros2_node.destroy_node()
                print("✅ ROS2 節點已關閉")
            except Exception as e:
                print(f"⚠️ 關閉 ROS2 節點時發生錯誤: {e}")
        
        try:
            import rclpy
            rclpy.shutdown()
            print("✅ ROS2 系統已關閉")
        except Exception as e:
            print(f"⚠️ 關閉 ROS2 系統時發生錯誤: {e}")
        
        # 停止所有子進程
        for process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=5)
                print("✅ Web 服務器已關閉")
            except subprocess.TimeoutExpired:
                process.kill()
                print("⚠️ 強制關閉 Web 服務器")
            except Exception as e:
                print(f"⚠️ 關閉進程時發生錯誤: {e}")
        
        print("👋 系統已完全關閉")
    
    def run(self):
        """運行系統"""
        # 設置信號處理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        print("🚀 啟動您的自動推送訂單系統")
        print("=" * 50)
        
        # 檢查依賴
        if not self.check_dependencies():
            print("❌ 依賴檢查失敗，請解決問題後重試")
            return False
        
        # 啟動 Web 服務器
        if not self.start_web_server():
            print("❌ Web 服務器啟動失敗")
            return False
        
        # 啟動 ROS2 系統
        if not self.start_ros2_and_pusher():
            print("❌ ROS2 系統啟動失敗")
            self.shutdown()
            return False
        
        # 打印系統狀態
        self.print_system_status()
        
        # 運行 ROS2 spin
        try:
            self.run_ros2_spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()
        
        return True


def main():
    """主函數"""
    launcher = YourSystemLauncher()
    
    try:
        success = launcher.run()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"❌ 系統啟動失敗: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
進階醫院藥物管理系統 - ROS2整合啟動器
Advanced Hospital Medicine Management System - ROS2 Integration Launcher

啟動完整的進階ROS2服務系統，包括：
1. 訂單處理服務節點
2. 藥物資訊提供者節點  
3. 客戶端示範節點
4. 傳統服務節點 (向後兼容)
"""

import subprocess
import sys
import time
import os
import signal
from pathlib import Path

class AdvancedROS2SystemLauncher:
    def __init__(self):
        self.processes = []
        self.base_dir = Path(__file__).parent
        
    def launch_node(self, script_path, node_name):
        """啟動單個ROS2節點"""
        try:
            if not script_path.exists():
                print(f"❌ 腳本不存在: {script_path}")
                return None
                
            print(f"🚀 啟動 {node_name}...")
            process = subprocess.Popen([
                sys.executable, str(script_path)
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            self.processes.append((process, node_name))
            print(f"✅ {node_name} 已啟動 (PID: {process.pid})")
            return process
            
        except Exception as e:
            print(f"❌ 啟動 {node_name} 失敗: {str(e)}")
            return None
    
    def launch_all_nodes(self):
        """啟動所有ROS2節點"""
        print("🏥 啟動進階醫院藥物管理系統 - ROS2服務")
        print("=" * 70)
        
        # 進階節點配置
        nodes_config = [
            {
                'path': self.base_dir / 'ros2_packages/medicine_order_processor/scripts/order_processor_node.py',
                'name': '💼 訂單處理服務節點 (新)'
            },
            {
                'path': self.base_dir / 'ros2_packages/medicine_info_provider/scripts/medicine_info_node.py',
                'name': '💊 藥物資訊提供者節點 (新)'
            },
            {
                'path': self.base_dir / 'ros2_packages/medicine_basic_provider/scripts/enhanced_basic_medicine_node.py',
                'name': '🔬 增強版基本藥物節點'
            },
            {
                'path': self.base_dir / 'ros2_packages/medicine_order_service/scripts/single_order_service_node.py',
                'name': '🔍 單筆訂單服務節點'
            },
            {
                'path': self.base_dir / 'ros2_packages/medicine_basic_provider/scripts/basic_medicine_node.py',
                'name': '🏥 基本藥物節點 (傳統)'
            },
            {
                'path': self.base_dir / 'ros2_packages/medicine_detailed_provider/scripts/detailed_medicine_node.py',
                'name': '🔬 詳細藥物節點 (傳統)'
            },
            {
                'path': self.base_dir / 'ros2_packages/medicine_order_service/scripts/order_service_node.py',
                'name': '📋 訂單服務節點 (傳統)'
            }
        ]
        
        # 啟動所有節點
        for node_config in nodes_config:
            process = self.launch_node(node_config['path'], node_config['name'])
            if process:
                time.sleep(1.5)  # 給每個節點更多啟動時間
        
        print("\n🎯 進階ROS2服務狀態:")
        print("=" * 70)
        
        # 顯示所有可用的服務和主題
        self.show_available_services()
        
        print("\n📋 可用的ROS2服務:")
        print("💼 /medicine_order             - 完整訂單處理服務 (新)")
        print("💊 /get_medicine_info          - 藥物資訊查詢服務 (新)")
        print("🔍 /get_single_order          - 獲取單筆訂單")
        print("💊 /get_all_basic_medicines   - 獲取所有基本藥物")
        print("📊 /get_medicine_info         - 獲取程式用藥物資訊")
        print("🏥 /get_basic_medicines       - 基本藥物 (傳統)")
        print("🔬 /get_detailed_medicines    - 詳細藥物")
        print("📋 /get_medicine_orders       - 藥物訂單 (傳統)")
        
        print("\n📡 可用的ROS2主題:")
        print("📤 /medicine_info_output          - 藥物資訊輸出 (新)")
        print("📤 /single_order_output           - 單筆訂單輸出")
        print("📤 /enhanced_basic_medicines_output - 增強版基本藥物輸出")
        print("📤 /structured_medicine_data      - 程式用結構化資料")
        print("📤 /basic_medicines_output        - 基本藥物輸出 (傳統)")
        print("📤 /detailed_medicines_output     - 詳細藥物輸出")
        print("📤 /medicine_order_output         - 藥物訂單輸出 (傳統)")
        
        print("\n🗂️ 輸出檔案位置:")
        print(f"📁 ~/ros2_medicine_output/")
        print(f"   ├── processed_orders/        - 訂單處理記錄")
        print(f"   ├── medicine_info/           - 藥物資訊查詢記錄")
        print(f"   └── [其他服務輸出檔案]")
        
        print("\n🔧 測試進階服務:")
        print("# 測試訂單處理服務")
        print("ros2 service call /medicine_order medicine_interfaces/srv/MedicineOrder ...")
        print("\n# 測試藥物資訊查詢")  
        print("ros2 service call /get_medicine_info medicine_interfaces/srv/GetMedicineInfo ...")
        
        print("\n📖 客戶端範例:")
        print("python3 ros2_packages/medicine_client/scripts/medicine_client_example.py")
        
    def show_available_services(self):
        """顯示可用的ROS2服務"""
        try:
            # 等待服務啟動
            time.sleep(5)
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                services = result.stdout.strip().split('\n')
                medicine_services = [s for s in services if 'medicine' in s or 'order' in s]
                
                if medicine_services:
                    print("🟢 已註冊的醫院服務:")
                    for service in medicine_services:
                        if 'medicine_order' in service:
                            print(f"   💼 {service} (進階訂單處理)")
                        elif 'get_medicine_info' in service:
                            print(f"   💊 {service} (進階藥物資訊)")
                        else:
                            print(f"   📋 {service}")
                else:
                    print("⚠️ 尚未發現醫院相關服務 (可能還在啟動中)")
            else:
                print("⚠️ 無法列出ROS2服務")
                
        except subprocess.TimeoutExpired:
            print("⚠️ 列出服務超時 (ROS2可能還在初始化)")
        except FileNotFoundError:
            print("⚠️ 找不到 ros2 命令，請確認ROS2已正確安裝")
        except Exception as e:
            print(f"⚠️ 檢查服務時發生錯誤: {str(e)}")
    
    def launch_client_demo(self):
        """啟動客戶端示範"""
        print("\n" + "="*50)
        print("🚀 啟動客戶端示範")
        print("="*50)
        
        client_script = self.base_dir / 'ros2_packages/medicine_client/scripts/medicine_client_example.py'
        
        if client_script.exists():
            print("📱 啟動客戶端示範...")
            try:
                subprocess.run([sys.executable, str(client_script)], check=True)
            except subprocess.CalledProcessError as e:
                print(f"❌ 客戶端示範執行失敗: {e}")
            except KeyboardInterrupt:
                print("🛑 客戶端示範被用戶中斷")
        else:
            print(f"❌ 客戶端腳本不存在: {client_script}")
    
    def wait_for_termination(self):
        """等待終止信號"""
        print("\n⏳ 進階系統運行中...")
        print("💡 選項:")
        print("  - 按 Ctrl+C 停止所有服務")
        print("  - 輸入 'demo' 並按 Enter 來運行客戶端示範")
        print("  - 輸入 'status' 查看服務狀態")
        print("="*70)
        
        try:
            while True:
                try:
                    # 非阻塞輸入檢查
                    import select
                    import sys
                    
                    if select.select([sys.stdin], [], [], 1) == ([sys.stdin], [], []):
                        user_input = input().strip().lower()
                        
                        if user_input == 'demo':
                            self.launch_client_demo()
                        elif user_input == 'status':
                            self.show_available_services()
                        elif user_input in ['quit', 'exit']:
                            break
                        else:
                            print("❓ 未知命令。可用命令: demo, status, quit")
                
                except EOFError:
                    break
                
                # 檢查是否有進程意外終止
                for i, (process, name) in enumerate(self.processes):
                    if process.poll() is not None:
                        print(f"⚠️ {name} 意外終止")
                        
        except KeyboardInterrupt:
            print("\n🛑 收到終止信號...")
            self.cleanup()
    
    def cleanup(self):
        """清理所有進程"""
        print("🧹 正在停止所有ROS2節點...")
        
        for process, name in self.processes:
            try:
                if process.poll() is None:  # 進程還在運行
                    print(f"🛑 停止 {name}...")
                    process.terminate()
                    
                    # 等待5秒讓進程正常終止
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        print(f"⚠️ {name} 拒絕正常終止，強制結束...")
                        process.kill()
                        
            except Exception as e:
                print(f"❌ 停止 {name} 時發生錯誤: {str(e)}")
        
        print("✅ 所有節點已停止")
        print("👋 感謝使用進階醫院藥物管理系統!")

def main():
    launcher = AdvancedROS2SystemLauncher()
    
    try:
        launcher.launch_all_nodes()
        launcher.wait_for_termination()
    except Exception as e:
        print(f"❌ 系統錯誤: {str(e)}")
        launcher.cleanup()
        sys.exit(1)

if __name__ == "__main__":
    main()
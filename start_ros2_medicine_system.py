#!/usr/bin/env python3
"""
ROS2 Medicine Management System Launcher
ROS2醫院藥物管理系統啟動器
"""

import os
import sys
import subprocess
import time
import signal
from pathlib import Path

class ROS2MedicineSystemLauncher:
    def __init__(self):
        self.processes = []
        self.system_started = False
        
    def start_web_api(self):
        """啟動Web API服務器"""
        print("🌐 啟動Web API服務器...")
        
        api_dir = Path("user_interface")
        if not api_dir.exists():
            print("❌ 找不到user_interface目錄")
            return False
            
        try:
            proc = subprocess.Popen([
                sys.executable, "main.py"
            ], cwd=api_dir)
            
            self.processes.append(proc)
            print("✅ Web API服務器已啟動 (PID: {})".format(proc.pid))
            
            # 等待API服務器啟動
            time.sleep(5)
            return True
            
        except Exception as e:
            print(f"❌ 啟動Web API服務器失敗: {e}")
            return False
    
    def start_ros2_nodes(self):
        """啟動ROS2節點"""
        print("🤖 啟動ROS2節點...")
        
        ros2_nodes = [
            {
                "name": "Medicine Order Service",
                "script": "ros2_packages/medicine_order_service/scripts/order_service_node.py",
                "description": "訂單服務節點"
            },
            {
                "name": "Basic Medicine Provider", 
                "script": "ros2_packages/medicine_basic_provider/scripts/basic_medicine_node.py",
                "description": "基本藥物提供者"
            },
            {
                "name": "Detailed Medicine Provider",
                "script": "ros2_packages/medicine_detailed_provider/scripts/detailed_medicine_node.py", 
                "description": "詳細藥物提供者"
            }
        ]
        
        for node in ros2_nodes:
            script_path = Path(node["script"])
            
            if not script_path.exists():
                print(f"⚠️  跳過 {node['name']}: 腳本不存在 ({script_path})")
                continue
                
            try:
                # 設定執行權限
                os.chmod(script_path, 0o755)
                
                proc = subprocess.Popen([
                    sys.executable, str(script_path)
                ])
                
                self.processes.append(proc)
                print(f"✅ {node['name']} 已啟動 (PID: {proc.pid}) - {node['description']}")
                
                # 節點間延遲啟動
                time.sleep(2)
                
            except Exception as e:
                print(f"❌ 啟動 {node['name']} 失敗: {e}")
    
    def start_system(self):
        """啟動整個系統"""
        print("🚀 啟動ROS2醫院藥物管理系統")
        print("=" * 60)
        
        # 1. 啟動Web API
        if not self.start_web_api():
            print("❌ Web API啟動失敗，停止啟動流程")
            return False
        
        # 2. 啟動ROS2節點
        self.start_ros2_nodes()
        
        self.system_started = True
        
        print("\n🎉 系統啟動完成!")
        print("=" * 60)
        print("🌐 Web介面:")
        print("   主頁面: http://localhost:8000")
        print("   藥物管理: http://localhost:8000/Medicine.html")
        print("   統一藥物管理: http://localhost:8000/unified_medicine.html")
        print("   處方籤管理: http://localhost:8000/Prescription.html")
        print("   醫生介面: http://localhost:8000/doctor.html")
        print("   API文檔: http://localhost:8000/docs")
        print("\n🤖 ROS2服務:")
        print("   ros2 service call /get_medicine_orders std_srvs/srv/Trigger")
        print("   ros2 service call /get_basic_medicines std_srvs/srv/Trigger")
        print("   ros2 service call /get_detailed_medicines std_srvs/srv/Trigger")
        print("\n📤 ROS2主題:")
        print("   ros2 topic echo /medicine_order_output")
        print("   ros2 topic echo /basic_medicines_output")
        print("   ros2 topic echo /detailed_medicines_output")
        print("\n📄 YAML輸出目錄: ~/ros2_medicine_output")
        print("=" * 60)
        
        return True
    
    def stop_system(self):
        """停止系統"""
        print("\n🛑 正在停止系統...")
        
        for i, proc in enumerate(self.processes):
            try:
                proc.terminate()
                proc.wait(timeout=5)
                print(f"✅ 進程 {proc.pid} 已停止")
            except subprocess.TimeoutExpired:
                proc.kill()
                print(f"⚡ 強制終止進程 {proc.pid}")
            except Exception as e:
                print(f"❌ 停止進程 {proc.pid} 時發生錯誤: {e}")
        
        self.processes.clear()
        self.system_started = False
        print("✅ 系統已完全停止")
    
    def signal_handler(self, signum, frame):
        """處理系統信號"""
        print(f"\n🔔 收到信號 {signum}")
        self.stop_system()
        sys.exit(0)
    
    def run(self):
        """運行系統"""
        # 註冊信號處理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        try:
            if self.start_system():
                print("\n⌨️  按 Ctrl+C 停止系統")
                
                # 保持系統運行
                while self.system_started:
                    time.sleep(1)
                    
                    # 檢查進程是否還在運行
                    for proc in self.processes[:]:
                        if proc.poll() is not None:
                            print(f"⚠️  進程 {proc.pid} 已意外停止")
                            self.processes.remove(proc)
        
        except KeyboardInterrupt:
            print("\n👋 收到中斷信號")
        
        finally:
            if self.system_started:
                self.stop_system()

def main():
    print("🏥 ROS2醫院藥物管理系統啟動器")
    print("版本: 1.0.0")
    print()
    
    launcher = ROS2MedicineSystemLauncher()
    launcher.run()

if __name__ == "__main__":
    main()
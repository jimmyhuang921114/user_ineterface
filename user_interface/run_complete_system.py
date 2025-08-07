#!/usr/bin/env python3
"""
Complete System Runner
完整系統運行腳本 - 啟動系統並執行測試
"""

import subprocess
import sys
import os
import time
import signal
from pathlib import Path

class SystemRunner:
    def __init__(self):
        self.server_process = None
        self.system_type = None
        
    def cleanup(self, signum=None, frame=None):
        """清理進程"""
        if self.server_process:
            print("\n🛑 正在停止伺服器...")
            self.server_process.terminate()
            try:
                self.server_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.server_process.kill()
            print("✅ 伺服器已停止")

    def check_requirements(self):
        """檢查系統需求"""
        print("🔍 檢查系統需求...")
        
        # 檢查 Python 版本
        if sys.version_info < (3, 8):
            print("❌ 需要 Python 3.8 或更高版本")
            return False
        
        # 檢查必要模組
        required_modules = ['fastapi', 'uvicorn', 'sqlalchemy', 'requests']
        missing_modules = []
        
        for module in required_modules:
            try:
                __import__(module)
            except ImportError:
                missing_modules.append(module)
        
        if missing_modules:
            print(f"❌ 缺少必要模組: {', '.join(missing_modules)}")
            print("請執行: pip install fastapi uvicorn sqlalchemy requests")
            return False
        
        print("✅ 系統需求檢查通過")
        return True

    def choose_system_type(self):
        """選擇系統類型"""
        print("\n🏥 醫院藥物管理系統")
        print("請選擇要運行的系統版本:")
        print("1. 完整版本 (含 ROS2 模擬)")
        print("2. 純 API 版本 (無 ROS2)")
        print("3. 只執行測試 (需要已運行的伺服器)")
        
        while True:
            choice = input("\n請輸入選項 (1-3): ").strip()
            if choice == "1":
                self.system_type = "full"
                return "simple_server_clean.py"
            elif choice == "2":
                self.system_type = "api_only"
                return "simple_server_no_ros2.py"
            elif choice == "3":
                self.system_type = "test_only"
                return None
            else:
                print("❌ 無效選項，請輸入 1、2 或 3")

    def start_server(self, server_script):
        """啟動伺服器"""
        if not server_script:
            return True  # 測試模式，不需要啟動伺服器
            
        print(f"🚀 啟動伺服器: {server_script}")
        
        try:
            self.server_process = subprocess.Popen(
                [sys.executable, server_script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # 等待伺服器啟動
            print("⏳ 等待伺服器啟動...")
            time.sleep(5)
            
            # 檢查伺服器是否正常運行
            if self.server_process.poll() is None:
                print("✅ 伺服器啟動成功")
                return True
            else:
                stdout, stderr = self.server_process.communicate()
                print(f"❌ 伺服器啟動失敗")
                print(f"標準輸出: {stdout}")
                print(f"錯誤輸出: {stderr}")
                return False
                
        except Exception as e:
            print(f"❌ 啟動伺服器時發生錯誤: {e}")
            return False

    def run_tests(self):
        """執行系統測試"""
        print("\n🧪 開始執行系統測試...")
        
        try:
            result = subprocess.run(
                [sys.executable, "test_complete_system.py"],
                capture_output=True,
                text=True,
                timeout=300  # 5分鐘超時
            )
            
            print(result.stdout)
            
            if result.stderr:
                print("測試過程中的警告:")
                print(result.stderr)
            
            if result.returncode == 0:
                print("🎉 所有測試通過!")
                return True
            else:
                print("❌ 部分測試失敗")
                return False
                
        except subprocess.TimeoutExpired:
            print("❌ 測試超時")
            return False
        except Exception as e:
            print(f"❌ 執行測試時發生錯誤: {e}")
            return False

    def show_system_info(self):
        """顯示系統資訊"""
        print("\n📋 系統資訊")
        print("=" * 50)
        print("🌐 網頁界面:")
        print("  - 主頁面: http://localhost:8001/")
        print("  - 整合管理: http://localhost:8001/integrated_medicine_management.html")
        print("  - 處方籤管理: http://localhost:8001/Prescription.html")
        print("  - 醫生工作站: http://localhost:8001/doctor.html")
        print("\n📖 API 文檔:")
        print("  - Swagger UI: http://localhost:8001/docs")
        print("  - ReDoc: http://localhost:8001/redoc")
        
        if self.system_type == "full":
            print("\n🤖 ROS2 功能:")
            print("  - 狀態查詢: GET /api/ros2/status")
            print("  - 待處理訂單: GET /api/ros2/pending-orders")
            print("  - 藥物查詢: POST /api/ros2/query-medicine")
            print("  - 批量查詢: POST /api/ros2/batch-query-medicines")
        
        print("\n💊 核心功能:")
        print("  - ✅ 藥物管理 (CRUD)")
        print("  - ✅ 處方籤管理")
        print("  - ✅ 庫存管理")
        print("  - ✅ 智能搜尋")
        
        if self.system_type == "full":
            print("  - ✅ ROS2 整合")
            print("  - ✅ 自動化工作流程")
        
        print("=" * 50)

    def interactive_mode(self):
        """互動模式"""
        print("\n🎮 進入互動模式")
        print("可用指令:")
        print("  test  - 執行系統測試")
        print("  info  - 顯示系統資訊") 
        print("  help  - 顯示說明")
        print("  quit  - 退出系統")
        
        while True:
            try:
                command = input("\n> ").strip().lower()
                
                if command == "test":
                    self.run_tests()
                elif command == "info":
                    self.show_system_info()
                elif command == "help":
                    print("\n📚 系統說明:")
                    print("這是一個完整的醫院藥物管理系統，包含:")
                    print("1. 網頁前端界面")
                    print("2. RESTful API 後端")
                    print("3. SQLite 資料庫")
                    if self.system_type == "full":
                        print("4. ROS2 模擬整合")
                    print("\n使用瀏覽器訪問 http://localhost:8001 開始使用")
                elif command == "quit" or command == "exit":
                    break
                elif command == "":
                    continue
                else:
                    print(f"❌ 未知指令: {command}")
                    print("輸入 'help' 查看可用指令")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break

    def run(self):
        """主要運行流程"""
        # 設置信號處理
        signal.signal(signal.SIGINT, self.cleanup)
        signal.signal(signal.SIGTERM, self.cleanup)
        
        try:
            print("🏥 醫院藥物管理系統")
            print("=" * 50)
            
            # 檢查系統需求
            if not self.check_requirements():
                return 1
            
            # 選擇系統類型
            server_script = self.choose_system_type()
            
            # 啟動伺服器
            if not self.start_server(server_script):
                return 1
            
            # 顯示系統資訊
            self.show_system_info()
            
            # 詢問是否執行測試
            if input("\n🧪 是否執行系統測試? (y/N): ").strip().lower() == 'y':
                self.run_tests()
            
            # 進入互動模式
            if self.system_type != "test_only":
                self.interactive_mode()
            
            return 0
            
        except Exception as e:
            print(f"❌ 系統運行時發生錯誤: {e}")
            return 1
        finally:
            self.cleanup()

def main():
    """主函數"""
    runner = SystemRunner()
    sys.exit(runner.run())

if __name__ == "__main__":
    main()
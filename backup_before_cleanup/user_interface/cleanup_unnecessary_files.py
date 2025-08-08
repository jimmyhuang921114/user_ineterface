#!/usr/bin/env python3
"""
檔案清理腳本 - 移除不必要的檔案
只保留自動推送 ROS2 訂單系統所需的核心檔案
"""

import os
import shutil
from pathlib import Path


class FileCleanup:
    """檔案清理工具"""
    
    def __init__(self):
        self.files_to_delete = [
            # ROS2 服務模式 (您不需要)
            'ros2_services_interface.py',
            'ros2_client_example.py',
            'ROS2_SERVICES_GUIDE.md',
            
            # 藥物查詢服務 (可選)
            'ros2_medicine_detail_service.py',
            'medicine_client_example.py',
            'MEDICINE_DETAIL_SERVICE_GUIDE.md',
            
            # 舊版檔案
            'ros2_interface_final.py',
            'integration_example.py',
            'start_final_server.py',
            'start_complete_system.py',
            'start_system_modes.py',
            
            # 文檔檔案 (可選)
            'ESSENTIAL_FILES_ANALYSIS.md',
            'FINAL_SYSTEM_GUIDE.md',
            'WEB_ROS2_ARCHITECTURE.md',
            'README.md',
            
            # 資料庫和日誌
            'hospital_medicine_final.db',
            'hospital_system_ros2_real.log',
            
            # 其他不需要的檔案
            'ESSENTIAL_FILES_FOR_ROS2.md',  # 這個也可以刪除
            'ORDER_FLOW_GUIDE.md',  # 已包含在 YOUR_ROS2_GUIDE.md
            'QUICK_REFERENCE.md',   # 已包含在 YOUR_ROS2_GUIDE.md
        ]
        
        self.essential_files = [
            # 核心檔案
            'database_final.py',
            'simple_server_final.py',
            'ros2_order_pusher.py',
            'your_ros2_node.py',
            'start_your_system.py',
            'test_order_flow.py',
            'YOUR_ROS2_GUIDE.md',
            'cleanup_unnecessary_files.py',
            
            # Web 界面 (整個 static 目錄)
            'static/',
        ]
    
    def check_files(self):
        """檢查檔案狀態"""
        print("🔍 檢查檔案狀態...")
        print("=" * 60)
        
        print("\n⭐ 必要檔案 (將保留):")
        for file in self.essential_files:
            if os.path.exists(file):
                if file.endswith('/'):
                    print(f"✅ {file} (目錄)")
                else:
                    print(f"✅ {file}")
            else:
                print(f"❌ {file} (缺失)")
        
        print("\n❌ 將刪除的檔案:")
        files_to_delete_exist = []
        for file in self.files_to_delete:
            if os.path.exists(file):
                files_to_delete_exist.append(file)
                print(f"🗑️ {file}")
        
        if not files_to_delete_exist:
            print("📭 沒有找到需要刪除的檔案")
        
        print("\n📊 總結:")
        print(f"• 必要檔案: {len([f for f in self.essential_files if os.path.exists(f)])}")
        print(f"• 將刪除: {len(files_to_delete_exist)}")
        
        return files_to_delete_exist
    
    def create_backup(self):
        """創建備份"""
        backup_dir = Path("backup_before_cleanup")
        
        if backup_dir.exists():
            print(f"⚠️ 備份目錄已存在: {backup_dir}")
            return False
        
        try:
            backup_dir.mkdir()
            
            # 備份即將刪除的檔案
            backed_up = 0
            for file in self.files_to_delete:
                if os.path.exists(file):
                    shutil.copy2(file, backup_dir)
                    backed_up += 1
            
            print(f"✅ 已備份 {backed_up} 個檔案到 {backup_dir}")
            return True
            
        except Exception as e:
            print(f"❌ 備份失敗: {e}")
            return False
    
    def delete_files(self, files_to_delete):
        """刪除檔案"""
        print("\n🗑️ 開始刪除檔案...")
        
        deleted_count = 0
        failed_count = 0
        
        for file in files_to_delete:
            try:
                if os.path.isfile(file):
                    os.remove(file)
                    print(f"✅ 已刪除: {file}")
                    deleted_count += 1
                elif os.path.isdir(file):
                    shutil.rmtree(file)
                    print(f"✅ 已刪除目錄: {file}")
                    deleted_count += 1
                else:
                    print(f"⚠️ 檔案不存在: {file}")
            except Exception as e:
                print(f"❌ 刪除失敗 {file}: {e}")
                failed_count += 1
        
        print(f"\n📊 刪除結果:")
        print(f"• 成功刪除: {deleted_count}")
        print(f"• 刪除失敗: {failed_count}")
        
        return deleted_count, failed_count
    
    def print_final_structure(self):
        """打印最終檔案結構"""
        print("\n🏗️ 清理後的檔案結構:")
        print("=" * 40)
        
        all_files = []
        for item in os.listdir('.'):
            if os.path.isfile(item) and not item.startswith('.'):
                all_files.append(f"📄 {item}")
            elif os.path.isdir(item) and not item.startswith('.'):
                all_files.append(f"📁 {item}/")
        
        all_files.sort()
        for file in all_files:
            print(file)
        
        print(f"\n總計: {len(all_files)} 項目")
    
    def run(self, backup=True, interactive=True):
        """執行清理"""
        print("🧹 ROS2 自動推送系統檔案清理工具")
        print("=" * 50)
        
        # 檢查檔案
        files_to_delete_exist = self.check_files()
        
        if not files_to_delete_exist:
            print("\n✅ 沒有需要清理的檔案")
            return True
        
        # 互動確認
        if interactive:
            print(f"\n⚠️ 即將刪除 {len(files_to_delete_exist)} 個檔案")
            choice = input("是否繼續? (y/N): ").strip().lower()
            if choice != 'y':
                print("👋 取消清理")
                return False
        
        # 創建備份
        if backup:
            if not self.create_backup():
                if interactive:
                    choice = input("備份失敗，是否繼續清理? (y/N): ").strip().lower()
                    if choice != 'y':
                        print("👋 取消清理")
                        return False
        
        # 刪除檔案
        deleted_count, failed_count = self.delete_files(files_to_delete_exist)
        
        # 顯示最終結構
        self.print_final_structure()
        
        print("\n🎉 清理完成!")
        print("=" * 30)
        print("📋 您現在擁有最小化的自動推送系統:")
        print("• 自動推送訂單給您的 ROS2 節點")
        print("• 一次一個，等結束再進行下一個")
        print("• 能夠告訴網站完成了")
        print("• 完整的 Web 管理界面")
        
        print("\n🚀 下一步:")
        print("1. python3 start_your_system.py  # 啟動系統")
        print("2. 查看 YOUR_ROS2_GUIDE.md     # 使用說明")
        print("3. 編輯 your_ros2_node.py      # 實現您的邏輯")
        
        return True


def main():
    """主函數"""
    import sys
    
    cleanup = FileCleanup()
    
    # 檢查命令行參數
    if len(sys.argv) > 1:
        if sys.argv[1] == '--check':
            # 只檢查，不刪除
            cleanup.check_files()
            return
        elif sys.argv[1] == '--force':
            # 強制清理，不互動
            cleanup.run(backup=True, interactive=False)
            return
        elif sys.argv[1] == '--no-backup':
            # 不備份
            cleanup.run(backup=False, interactive=True)
            return
    
    # 預設互動模式
    cleanup.run(backup=True, interactive=True)


if __name__ == "__main__":
    main()
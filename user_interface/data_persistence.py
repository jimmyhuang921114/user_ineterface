#!/usr/bin/env python3
"""
資料持久化模組
Data Persistence Module
使用JSON文件儲存系統資料，避免重啟後資料消失
"""

import json
import os
from pathlib import Path
from datetime import datetime
import shutil

class DataPersistence:
    def __init__(self, data_dir="data"):
        """初始化資料持久化管理器"""
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(exist_ok=True)
        
        # 資料文件路徑
        self.medicines_file = self.data_dir / "medicines.json"
        self.detailed_medicines_file = self.data_dir / "detailed_medicines.json"
        self.prescriptions_file = self.data_dir / "prescriptions.json"
        self.prescription_status_file = self.data_dir / "prescription_status.json"
        self.counters_file = self.data_dir / "counters.json"
        
        # 備份目錄
        self.backup_dir = self.data_dir / "backups"
        self.backup_dir.mkdir(exist_ok=True)
    
    def save_medicines(self, medicines_db, next_medicine_id):
        """儲存基本藥物資料"""
        try:
            data = {
                "medicines": medicines_db,
                "next_id": next_medicine_id,
                "last_updated": datetime.now().isoformat()
            }
            
            with open(self.medicines_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            
            return True
        except Exception as e:
            print(f"❌ 儲存基本藥物資料失敗: {e}")
            return False
    
    def load_medicines(self):
        """載入基本藥物資料"""
        try:
            if self.medicines_file.exists():
                with open(self.medicines_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                medicines_db = data.get("medicines", [])
                next_medicine_id = data.get("next_id", 1)
                
                print(f"✅ 載入基本藥物資料: {len(medicines_db)} 項")
                return medicines_db, next_medicine_id
            else:
                print("📝 基本藥物資料文件不存在，使用空資料")
                return [], 1
                
        except Exception as e:
            print(f"❌ 載入基本藥物資料失敗: {e}")
            return [], 1
    
    def save_detailed_medicines(self, detailed_medicines_db):
        """儲存詳細藥物資料"""
        try:
            data = {
                "detailed_medicines": detailed_medicines_db,
                "last_updated": datetime.now().isoformat()
            }
            
            with open(self.detailed_medicines_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            
            return True
        except Exception as e:
            print(f"❌ 儲存詳細藥物資料失敗: {e}")
            return False
    
    def load_detailed_medicines(self):
        """載入詳細藥物資料"""
        try:
            if self.detailed_medicines_file.exists():
                with open(self.detailed_medicines_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                detailed_medicines_db = data.get("detailed_medicines", {})
                
                print(f"✅ 載入詳細藥物資料: {len(detailed_medicines_db)} 項")
                return detailed_medicines_db
            else:
                print("📝 詳細藥物資料文件不存在，使用空資料")
                return {}
                
        except Exception as e:
            print(f"❌ 載入詳細藥物資料失敗: {e}")
            return {}
    
    def save_prescriptions(self, prescriptions_db, prescription_status_db, next_prescription_id):
        """儲存處方資料"""
        try:
            # 儲存處方主資料
            prescriptions_data = {
                "prescriptions": prescriptions_db,
                "next_id": next_prescription_id,
                "last_updated": datetime.now().isoformat()
            }
            
            with open(self.prescriptions_file, 'w', encoding='utf-8') as f:
                json.dump(prescriptions_data, f, ensure_ascii=False, indent=2)
            
            # 儲存處方狀態歷史
            status_data = {
                "prescription_status": prescription_status_db,
                "last_updated": datetime.now().isoformat()
            }
            
            with open(self.prescription_status_file, 'w', encoding='utf-8') as f:
                json.dump(status_data, f, ensure_ascii=False, indent=2)
            
            return True
        except Exception as e:
            print(f"❌ 儲存處方資料失敗: {e}")
            return False
    
    def load_prescriptions(self):
        """載入處方資料"""
        try:
            prescriptions_db = []
            prescription_status_db = []
            next_prescription_id = 1
            
            # 載入處方主資料
            if self.prescriptions_file.exists():
                with open(self.prescriptions_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                prescriptions_db = data.get("prescriptions", [])
                next_prescription_id = data.get("next_id", 1)
            
            # 載入處方狀態歷史
            if self.prescription_status_file.exists():
                with open(self.prescription_status_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                prescription_status_db = data.get("prescription_status", [])
            
            print(f"✅ 載入處方資料: {len(prescriptions_db)} 項處方, {len(prescription_status_db)} 項狀態記錄")
            return prescriptions_db, prescription_status_db, next_prescription_id
            
        except Exception as e:
            print(f"❌ 載入處方資料失敗: {e}")
            return [], [], 1
    
    def save_all_data(self, medicines_db, next_medicine_id, detailed_medicines_db, 
                      prescriptions_db, prescription_status_db, next_prescription_id):
        """儲存所有資料"""
        print("💾 正在儲存所有資料...")
        
        success = True
        success &= self.save_medicines(medicines_db, next_medicine_id)
        success &= self.save_detailed_medicines(detailed_medicines_db)
        success &= self.save_prescriptions(prescriptions_db, prescription_status_db, next_prescription_id)
        
        if success:
            print("✅ 所有資料儲存成功")
        else:
            print("❌ 部分資料儲存失敗")
        
        return success
    
    def load_all_data(self):
        """載入所有資料"""
        print("📂 正在載入所有資料...")
        
        medicines_db, next_medicine_id = self.load_medicines()
        detailed_medicines_db = self.load_detailed_medicines()
        prescriptions_db, prescription_status_db, next_prescription_id = self.load_prescriptions()
        
        return {
            'medicines_db': medicines_db,
            'next_medicine_id': next_medicine_id,
            'detailed_medicines_db': detailed_medicines_db,
            'prescriptions_db': prescriptions_db,
            'prescription_status_db': prescription_status_db,
            'next_prescription_id': next_prescription_id
        }
    
    def create_backup(self, backup_name=None):
        """創建資料備份"""
        try:
            if backup_name is None:
                backup_name = f"backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            
            backup_path = self.backup_dir / backup_name
            backup_path.mkdir(exist_ok=True)
            
            # 複製所有資料文件
            data_files = [
                self.medicines_file,
                self.detailed_medicines_file, 
                self.prescriptions_file,
                self.prescription_status_file
            ]
            
            copied_files = 0
            for data_file in data_files:
                if data_file.exists():
                    shutil.copy2(data_file, backup_path)
                    copied_files += 1
            
            if copied_files > 0:
                print(f"✅ 資料備份完成: {backup_path} ({copied_files} 個文件)")
                return str(backup_path)
            else:
                print("⚠️  沒有資料需要備份")
                return None
                
        except Exception as e:
            print(f"❌ 創建備份失敗: {e}")
            return None
    
    def restore_backup(self, backup_name):
        """還原資料備份"""
        try:
            backup_path = self.backup_dir / backup_name
            
            if not backup_path.exists():
                print(f"❌ 備份不存在: {backup_name}")
                return False
            
            # 還原資料文件
            data_files = [
                "medicines.json",
                "detailed_medicines.json",
                "prescriptions.json", 
                "prescription_status.json"
            ]
            
            restored_files = 0
            for filename in data_files:
                backup_file = backup_path / filename
                if backup_file.exists():
                    shutil.copy2(backup_file, self.data_dir)
                    restored_files += 1
            
            if restored_files > 0:
                print(f"✅ 資料還原完成: {restored_files} 個文件")
                return True
            else:
                print("⚠️  沒有可還原的資料")
                return False
                
        except Exception as e:
            print(f"❌ 還原備份失敗: {e}")
            return False
    
    def list_backups(self):
        """列出所有備份"""
        try:
            backups = []
            if self.backup_dir.exists():
                for backup_path in self.backup_dir.iterdir():
                    if backup_path.is_dir():
                        # 獲取備份資訊
                        backup_info = {
                            'name': backup_path.name,
                            'path': str(backup_path),
                            'created_time': datetime.fromtimestamp(backup_path.stat().st_mtime).isoformat(),
                            'files': len(list(backup_path.glob('*.json')))
                        }
                        backups.append(backup_info)
            
            # 按創建時間排序
            backups.sort(key=lambda x: x['created_time'], reverse=True)
            return backups
            
        except Exception as e:
            print(f"❌ 列出備份失敗: {e}")
            return []
    
    def get_data_info(self):
        """獲取資料文件資訊"""
        try:
            info = {
                'data_directory': str(self.data_dir),
                'files': {}
            }
            
            data_files = {
                'medicines': self.medicines_file,
                'detailed_medicines': self.detailed_medicines_file,
                'prescriptions': self.prescriptions_file,
                'prescription_status': self.prescription_status_file
            }
            
            for name, file_path in data_files.items():
                if file_path.exists():
                    stat = file_path.stat()
                    info['files'][name] = {
                        'exists': True,
                        'size_bytes': stat.st_size,
                        'modified_time': datetime.fromtimestamp(stat.st_mtime).isoformat()
                    }
                else:
                    info['files'][name] = {'exists': False}
            
            return info
            
        except Exception as e:
            print(f"❌ 獲取資料資訊失敗: {e}")
            return {}

# 創建全域資料持久化實例
data_persistence = DataPersistence()
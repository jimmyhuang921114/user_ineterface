#!/usr/bin/env python3
"""
Multi-Format Storage System
多格式存儲系統 - 支援JSON、YAML、SQL三種格式同步存儲
"""

import json
import yaml
import sqlite3
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Any, Optional
import logging

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MultiFormatStorage:
    """多格式存儲系統"""
    
    def __init__(self, data_dir: str = "data", db_path: str = "hospital_management.db"):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(exist_ok=True)
        self.db_path = self.data_dir / db_path
        
        # 初始化資料庫
        self._init_database()
        
    def _init_database(self):
        """初始化SQLite資料庫"""
        with sqlite3.connect(self.db_path) as conn:
            cursor = conn.cursor()
            
            # 基本藥物表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS medicine_basic (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT UNIQUE NOT NULL,
                    amount INTEGER DEFAULT 0,
                    position TEXT,
                    prompt TEXT,
                    created_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    is_active BOOLEAN DEFAULT 1
                )
            ''')
            
            # 詳細藥物表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS medicine_detailed (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    medicine_name TEXT NOT NULL,
                    description TEXT,
                    side_effects TEXT,
                    dosage TEXT,
                    contraindications TEXT,
                    created_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (medicine_name) REFERENCES medicine_basic(name)
                )
            ''')
            
            # 處方籤表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS prescriptions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    patient_name TEXT NOT NULL,
                    patient_id TEXT NOT NULL,
                    doctor_name TEXT NOT NULL,
                    medicines TEXT, -- JSON格式存儲藥物列表
                    status TEXT DEFAULT 'pending',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''')
            
            conn.commit()
            logger.info("✅ 資料庫初始化完成")

    # ========== 基本藥物管理 ==========
    
    def save_basic_medicines(self, medicines: List[Dict[str, Any]]) -> bool:
        """保存基本藥物到所有格式"""
        try:
            # 1. 保存到JSON
            json_path = self.data_dir / "medicine_basic_data.json"
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(medicines, f, ensure_ascii=False, indent=2)
            
            # 2. 保存到YAML
            yaml_path = self.data_dir / "basic_medicines.yaml"
            yaml_data = {
                "metadata": {
                    "type": "basic_medicines",
                    "created_at": datetime.now().isoformat(),
                    "total_records": len(medicines)
                },
                "medicines": medicines
            }
            with open(yaml_path, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True)
            
            # 3. 保存到SQL
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                # 清空並重新插入
                cursor.execute("DELETE FROM medicine_basic")
                for medicine in medicines:
                    cursor.execute('''
                        INSERT INTO medicine_basic (name, amount, position, prompt, created_time, updated_time)
                        VALUES (?, ?, ?, ?, ?, ?)
                    ''', (
                        medicine.get('name', ''),
                        medicine.get('amount', 0),
                        medicine.get('position', ''),
                        medicine.get('prompt', ''),
                        medicine.get('created_time', datetime.now().isoformat()),
                        medicine.get('updated_time', datetime.now().isoformat())
                    ))
                conn.commit()
            
            logger.info(f"✅ 基本藥物已保存到所有格式 ({len(medicines)} 筆)")
            return True
            
        except Exception as e:
            logger.error(f"❌ 保存基本藥物失敗: {e}")
            return False

    def load_basic_medicines(self) -> List[Dict[str, Any]]:
        """從JSON格式載入基本藥物（主要格式）"""
        try:
            json_path = self.data_dir / "medicine_basic_data.json"
            if json_path.exists():
                with open(json_path, 'r', encoding='utf-8') as f:
                    return json.load(f)
            return []
        except Exception as e:
            logger.error(f"❌ 載入基本藥物失敗: {e}")
            return []

    # ========== 詳細藥物管理 ==========
    
    def save_detailed_medicines(self, medicines: List[Dict[str, Any]]) -> bool:
        """保存詳細藥物到所有格式"""
        try:
            # 1. 保存到JSON
            json_path = self.data_dir / "medicine_detailed_data.json"
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(medicines, f, ensure_ascii=False, indent=2)
            
            # 2. 保存到YAML
            yaml_path = self.data_dir / "detailed_medicines.yaml"
            yaml_data = {
                "metadata": {
                    "type": "detailed_medicines",
                    "created_at": datetime.now().isoformat(),
                    "total_records": len(medicines)
                },
                "medicines": medicines
            }
            with open(yaml_path, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True)
            
            # 3. 保存到SQL
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                # 清空並重新插入
                cursor.execute("DELETE FROM medicine_detailed")
                for medicine in medicines:
                    cursor.execute('''
                        INSERT INTO medicine_detailed 
                        (medicine_name, description, side_effects, dosage, contraindications, created_time, updated_time)
                        VALUES (?, ?, ?, ?, ?, ?, ?)
                    ''', (
                        medicine.get('medicine_name', ''),
                        medicine.get('description', ''),
                        medicine.get('side_effects', ''),
                        medicine.get('dosage', ''),
                        medicine.get('contraindications', ''),
                        medicine.get('created_time', datetime.now().isoformat()),
                        medicine.get('updated_time', datetime.now().isoformat())
                    ))
                conn.commit()
            
            logger.info(f"✅ 詳細藥物已保存到所有格式 ({len(medicines)} 筆)")
            return True
            
        except Exception as e:
            logger.error(f"❌ 保存詳細藥物失敗: {e}")
            return False

    def load_detailed_medicines(self) -> List[Dict[str, Any]]:
        """從JSON格式載入詳細藥物（主要格式）"""
        try:
            json_path = self.data_dir / "medicine_detailed_data.json"
            if json_path.exists():
                with open(json_path, 'r', encoding='utf-8') as f:
                    return json.load(f)
            return []
        except Exception as e:
            logger.error(f"❌ 載入詳細藥物失敗: {e}")
            return []

    # ========== 處方籤管理 ==========
    
    def save_prescriptions(self, prescriptions: List[Dict[str, Any]]) -> bool:
        """保存處方籤到所有格式"""
        try:
            # 1. 保存到JSON
            json_path = self.data_dir / "prescription_data.json"
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(prescriptions, f, ensure_ascii=False, indent=2)
            
            # 2. 保存到YAML
            yaml_path = self.data_dir / "prescriptions.yaml"
            yaml_data = {
                "metadata": {
                    "type": "prescriptions",
                    "created_at": datetime.now().isoformat(),
                    "total_records": len(prescriptions)
                },
                "prescriptions": prescriptions
            }
            with open(yaml_path, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True)
            
            # 3. 保存到SQL
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                # 清空並重新插入
                cursor.execute("DELETE FROM prescriptions")
                for prescription in prescriptions:
                    cursor.execute('''
                        INSERT INTO prescriptions 
                        (patient_name, patient_id, doctor_name, medicines, status, created_at, updated_at)
                        VALUES (?, ?, ?, ?, ?, ?, ?)
                    ''', (
                        prescription.get('patient_name', ''),
                        prescription.get('patient_id', ''),
                        prescription.get('doctor_name', ''),
                        json.dumps(prescription.get('medicines', []), ensure_ascii=False),
                        prescription.get('status', 'pending'),
                        prescription.get('created_at', datetime.now().isoformat()),
                        prescription.get('updated_at', datetime.now().isoformat())
                    ))
                conn.commit()
            
            logger.info(f"✅ 處方籤已保存到所有格式 ({len(prescriptions)} 筆)")
            return True
            
        except Exception as e:
            logger.error(f"❌ 保存處方籤失敗: {e}")
            return False

    def load_prescriptions(self) -> List[Dict[str, Any]]:
        """從JSON格式載入處方籤（主要格式）"""
        try:
            json_path = self.data_dir / "prescription_data.json"
            if json_path.exists():
                with open(json_path, 'r', encoding='utf-8') as f:
                    return json.load(f)
            return []
        except Exception as e:
            logger.error(f"❌ 載入處方籤失敗: {e}")
            return []

    # ========== 數據同步 ==========
    
    def sync_all_formats(self) -> bool:
        """同步所有格式的數據"""
        try:
            logger.info("🔄 開始同步所有格式數據...")
            
            # 從JSON載入數據並同步到其他格式
            basic_medicines = self.load_basic_medicines()
            detailed_medicines = self.load_detailed_medicines()
            prescriptions = self.load_prescriptions()
            
            # 同步所有格式
            self.save_basic_medicines(basic_medicines)
            self.save_detailed_medicines(detailed_medicines)
            self.save_prescriptions(prescriptions)
            
            logger.info("✅ 所有格式數據同步完成")
            return True
            
        except Exception as e:
            logger.error(f"❌ 數據同步失敗: {e}")
            return False

    # ========== 數據驗證 ==========
    
    def validate_data_consistency(self) -> Dict[str, bool]:
        """驗證各格式數據一致性"""
        results = {
            "basic_medicines": True,
            "detailed_medicines": True,
            "prescriptions": True
        }
        
        try:
            # 檢查基本藥物
            json_basic = self.load_basic_medicines()
            yaml_path = self.data_dir / "basic_medicines.yaml"
            if yaml_path.exists():
                with open(yaml_path, 'r', encoding='utf-8') as f:
                    yaml_data = yaml.safe_load(f)
                    yaml_basic = yaml_data.get('medicines', [])
                    results["basic_medicines"] = len(json_basic) == len(yaml_basic)
            
            # 檢查詳細藥物
            json_detailed = self.load_detailed_medicines()
            yaml_path = self.data_dir / "detailed_medicines.yaml"
            if yaml_path.exists():
                with open(yaml_path, 'r', encoding='utf-8') as f:
                    yaml_data = yaml.safe_load(f)
                    yaml_detailed = yaml_data.get('medicines', [])
                    results["detailed_medicines"] = len(json_detailed) == len(yaml_detailed)
            
            # 檢查處方籤
            json_prescriptions = self.load_prescriptions()
            yaml_path = self.data_dir / "prescriptions.yaml"
            if yaml_path.exists():
                with open(yaml_path, 'r', encoding='utf-8') as f:
                    yaml_data = yaml.safe_load(f)
                    yaml_prescriptions = yaml_data.get('prescriptions', [])
                    results["prescriptions"] = len(json_prescriptions) == len(yaml_prescriptions)
            
        except Exception as e:
            logger.error(f"❌ 數據驗證失敗: {e}")
        
        return results

    # ========== 清理無用文件 ==========
    
    def cleanup_unused_files(self) -> List[str]:
        """清理無用的檔案"""
        unused_files = []
        
        # 可能的無用檔案列表
        potential_unused = [
            "medicines.json",  # 舊的藥物檔案
            "medicine_data.json",
            "cases.json",  # 舊的病例檔案
            "case_data.json",
            "patients.json",  # 如果有病患檔案但不再使用
            "old_prescriptions.json",
            "backup_*.json",
        ]
        
        for filename in potential_unused:
            file_path = self.data_dir / filename
            if file_path.exists():
                try:
                    file_path.unlink()
                    unused_files.append(filename)
                    logger.info(f"🗑️ 已刪除無用檔案: {filename}")
                except Exception as e:
                    logger.error(f"❌ 刪除檔案失敗 {filename}: {e}")
        
        return unused_files

# 全域實例
storage = MultiFormatStorage()

def get_storage() -> MultiFormatStorage:
    """獲取存儲系統實例"""
    return storage

if __name__ == "__main__":
    # 測試多格式存儲系統
    print("🧪 測試多格式存儲系統")
    
    # 同步所有格式
    storage.sync_all_formats()
    
    # 驗證數據一致性
    consistency = storage.validate_data_consistency()
    print(f"📊 數據一致性檢查: {consistency}")
    
    # 清理無用檔案
    cleaned = storage.cleanup_unused_files()
    print(f"🗑️ 已清理檔案: {cleaned}")
    
    print("✅ 多格式存儲系統測試完成")
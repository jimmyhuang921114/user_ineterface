#!/usr/bin/env python3
"""
💾 資料管理模組
處理YAML格式的藥物和處方籤資料
"""

import os
import yaml
import json
import shutil
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any
import uuid


class DataManager:
    """YAML資料管理器"""
    
    def __init__(self, data_dir: str = "data"):
        self.data_dir = data_dir
        self.medicines_file = os.path.join(data_dir, "medicines.yaml")
        self.prescriptions_file = os.path.join(data_dir, "prescriptions.yaml")
        self.config_file = os.path.join(data_dir, "config.yaml")
        
        # 確保資料目錄存在
        os.makedirs(data_dir, exist_ok=True)
        
        # 初始化資料結構
        self.medicines = []
        self.prescriptions = []
        self.config = {
            'auto_save_interval': 300,
            'backup_enabled': True,
            'backup_interval': 86400,
            'max_prescription_history': 1000,
            'low_stock_threshold': 50,
            'expiry_warning_days': 30
        }
        
    def load_data(self):
        """載入所有資料"""
        self.load_medicines()
        self.load_prescriptions()
        self.load_config()
        
    def save_data(self):
        """儲存所有資料"""
        self.save_medicines()
        self.save_prescriptions()
        self.save_config()
        
    def load_medicines(self):
        """載入藥物資料"""
        try:
            if os.path.exists(self.medicines_file):
                with open(self.medicines_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    self.medicines = data.get('medicines', [])
            else:
                self.medicines = []
                self.save_medicines()
        except Exception as e:
            print(f"載入藥物資料失敗: {e}")
            self.medicines = []
            
    def save_medicines(self):
        """儲存藥物資料"""
        try:
            data = {
                'medicines': self.medicines,
                'categories': self.get_categories(),
                'dosage_forms': self.get_dosage_forms()
            }
            with open(self.medicines_file, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True, indent=2)
        except Exception as e:
            print(f"儲存藥物資料失敗: {e}")
            
    def load_prescriptions(self):
        """載入處方籤資料"""
        try:
            if os.path.exists(self.prescriptions_file):
                with open(self.prescriptions_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    self.prescriptions = data.get('prescriptions', [])
            else:
                self.prescriptions = []
                self.save_prescriptions()
        except Exception as e:
            print(f"載入處方籤資料失敗: {e}")
            self.prescriptions = []
            
    def save_prescriptions(self):
        """儲存處方籤資料"""
        try:
            data = {
                'prescriptions': self.prescriptions,
                'system_config': self.config
            }
            with open(self.prescriptions_file, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True, indent=2)
        except Exception as e:
            print(f"儲存處方籤資料失敗: {e}")
            
    def load_config(self):
        """載入系統配置"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config_data = yaml.safe_load(f)
                    self.config.update(config_data)
        except Exception as e:
            print(f"載入配置失敗: {e}")
            
    def save_config(self):
        """儲存系統配置"""
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                yaml.dump(self.config, f, default_flow_style=False, allow_unicode=True, indent=2)
        except Exception as e:
            print(f"儲存配置失敗: {e}")
            
    def get_medicines(self) -> List[Dict]:
        """獲取所有藥物"""
        return self.medicines.copy()
        
    def get_medicine_by_id(self, medicine_id: str) -> Optional[Dict]:
        """根據ID獲取藥物"""
        for medicine in self.medicines:
            if medicine['id'] == medicine_id:
                return medicine.copy()
        return None
        
    def add_medicine(self, medicine_data: Dict):
        """新增藥物"""
        # 生成唯一ID
        if 'id' not in medicine_data or not medicine_data['id']:
            medicine_data['id'] = f"M{str(uuid.uuid4())[:8].upper()}"
            
        # 檢查ID是否已存在
        if self.get_medicine_by_id(medicine_data['id']):
            raise ValueError(f"藥物ID '{medicine_data['id']}' 已存在")
            
        # 設置預設值
        medicine_data.setdefault('stock_quantity', 0)
        medicine_data.setdefault('price', 0.0)
        medicine_data.setdefault('expiry_date', '2025-12-31')
        
        self.medicines.append(medicine_data)
        self.save_medicines()
        
    def update_medicine(self, medicine_id: str, medicine_data: Dict):
        """更新藥物"""
        for i, medicine in enumerate(self.medicines):
            if medicine['id'] == medicine_id:
                medicine_data['id'] = medicine_id  # 保持ID不變
                self.medicines[i] = medicine_data
                self.save_medicines()
                return
        raise ValueError(f"找不到藥物ID '{medicine_id}'")
        
    def delete_medicine(self, medicine_id: str):
        """刪除藥物"""
        for i, medicine in enumerate(self.medicines):
            if medicine['id'] == medicine_id:
                del self.medicines[i]
                self.save_medicines()
                return
        raise ValueError(f"找不到藥物ID '{medicine_id}'")
        
    def update_medicine_stock(self, medicine_id: str, quantity: int):
        """更新藥物庫存"""
        for medicine in self.medicines:
            if medicine['id'] == medicine_id:
                medicine['stock_quantity'] = max(0, medicine['stock_quantity'] + quantity)
                self.save_medicines()
                return
        raise ValueError(f"找不到藥物ID '{medicine_id}'")
        
    def get_prescriptions(self) -> List[Dict]:
        """獲取所有處方籤"""
        return self.prescriptions.copy()
        
    def get_prescription_by_id(self, prescription_id: str) -> Optional[Dict]:
        """根據ID獲取處方籤"""
        for prescription in self.prescriptions:
            if prescription['id'] == prescription_id:
                return prescription.copy()
        return None
        
    def add_prescription(self, prescription_data: Dict):
        """新增處方籤"""
        # 生成唯一ID
        if 'id' not in prescription_data or not prescription_data['id']:
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            prescription_data['id'] = f"P{timestamp}_{prescription_data.get('patient_name', 'Unknown')}"
            
        # 檢查ID是否已存在
        if self.get_prescription_by_id(prescription_data['id']):
            raise ValueError(f"處方籤ID '{prescription_data['id']}' 已存在")
            
        # 設置預設值
        prescription_data.setdefault('timestamp', datetime.now().isoformat())
        prescription_data.setdefault('status', 'pending')
        prescription_data.setdefault('total_price', 0.0)
        prescription_data.setdefault('medicines', [])
        
        # 計算總價
        total_price = 0.0
        for medicine in prescription_data['medicines']:
            medicine_info = self.get_medicine_by_id(medicine['medicine_id'])
            if medicine_info:
                total_price += medicine_info['price'] * medicine['quantity']
        prescription_data['total_price'] = total_price
        
        self.prescriptions.append(prescription_data)
        self.save_prescriptions()
        
    def update_prescription_status(self, prescription_id: str, status: str):
        """更新處方籤狀態"""
        for prescription in self.prescriptions:
            if prescription['id'] == prescription_id:
                prescription['status'] = status
                if status == 'completed':
                    # 更新庫存
                    for medicine in prescription['medicines']:
                        self.update_medicine_stock(medicine['medicine_id'], -medicine['quantity'])
                self.save_prescriptions()
                return
        raise ValueError(f"找不到處方籤ID '{prescription_id}'")
        
    def delete_prescription(self, prescription_id: str):
        """刪除處方籤"""
        for i, prescription in enumerate(self.prescriptions):
            if prescription['id'] == prescription_id:
                del self.prescriptions[i]
                self.save_prescriptions()
                return
        raise ValueError(f"找不到處方籤ID '{prescription_id}'")
        
    def get_categories(self) -> List[str]:
        """獲取所有藥物分類"""
        categories = set()
        for medicine in self.medicines:
            if 'category' in medicine:
                categories.add(medicine['category'])
        return sorted(list(categories))
        
    def get_dosage_forms(self) -> List[str]:
        """獲取所有劑型"""
        forms = set()
        for medicine in self.medicines:
            if 'dosage_form' in medicine:
                forms.add(medicine['dosage_form'])
        return sorted(list(forms))
        
    def search_medicines(self, query: str) -> List[Dict]:
        """搜尋藥物"""
        query = query.lower()
        results = []
        for medicine in self.medicines:
            if (query in medicine['name'].lower() or 
                query in medicine['english_name'].lower() or
                query in medicine['category'].lower()):
                results.append(medicine)
        return results
        
    def get_low_stock_medicines(self, threshold: int = None) -> List[Dict]:
        """獲取低庫存藥物"""
        if threshold is None:
            threshold = self.config.get('low_stock_threshold', 50)
        return [m for m in self.medicines if m['stock_quantity'] <= threshold]
        
    def get_expiring_medicines(self, days: int = None) -> List[Dict]:
        """獲取即將過期的藥物"""
        if days is None:
            days = self.config.get('expiry_warning_days', 30)
        warning_date = datetime.now() + timedelta(days=days)
        expiring = []
        for medicine in self.medicines:
            try:
                expiry_date = datetime.strptime(medicine['expiry_date'], '%Y-%m-%d')
                if expiry_date <= warning_date:
                    expiring.append(medicine)
            except:
                pass
        return expiring
        
    def get_inventory_summary(self) -> Dict:
        """獲取庫存摘要"""
        total_medicines = len(self.medicines)
        total_value = sum(m['stock_quantity'] * m['price'] for m in self.medicines)
        low_stock_count = len(self.get_low_stock_medicines())
        expiring_count = len(self.get_expiring_medicines())
        
        return {
            'total_medicines': total_medicines,
            'total_value': total_value,
            'low_stock_count': low_stock_count,
            'expiring_count': expiring_count
        }
        
    def export_data(self, export_dir: str = "exports"):
        """匯出資料"""
        os.makedirs(export_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 匯出YAML格式
        yaml_export = {
            'export_timestamp': datetime.now().isoformat(),
            'medicines': self.medicines,
            'prescriptions': self.prescriptions,
            'config': self.config
        }
        
        yaml_file = os.path.join(export_dir, f"medicine_system_export_{timestamp}.yaml")
        with open(yaml_file, 'w', encoding='utf-8') as f:
            yaml.dump(yaml_export, f, default_flow_style=False, allow_unicode=True, indent=2)
            
        # 匯出JSON格式
        json_file = os.path.join(export_dir, f"medicine_system_export_{timestamp}.json")
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(yaml_export, f, ensure_ascii=False, indent=2)
            
        return yaml_file, json_file
        
    def import_data(self, import_file: str = None):
        """匯入資料"""
        if import_file is None:
            # 尋找最新的匯出檔案
            export_dir = "exports"
            if os.path.exists(export_dir):
                files = [f for f in os.listdir(export_dir) if f.endswith('.yaml')]
                if files:
                    import_file = os.path.join(export_dir, sorted(files)[-1])
                    
        if not import_file or not os.path.exists(import_file):
            raise FileNotFoundError("找不到匯入檔案")
            
        with open(import_file, 'r', encoding='utf-8') as f:
            if import_file.endswith('.yaml'):
                data = yaml.safe_load(f)
            else:
                data = json.load(f)
                
        if 'medicines' in data:
            self.medicines = data['medicines']
        if 'prescriptions' in data:
            self.prescriptions = data['prescriptions']
        if 'config' in data:
            self.config.update(data['config'])
            
        self.save_data()
        
    def backup_data(self, backup_dir: str = "backups"):
        """建立備份"""
        os.makedirs(backup_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 備份原始檔案
        backup_files = []
        for file_path in [self.medicines_file, self.prescriptions_file, self.config_file]:
            if os.path.exists(file_path):
                backup_name = f"{os.path.basename(file_path)}_{timestamp}"
                backup_path = os.path.join(backup_dir, backup_name)
                shutil.copy2(file_path, backup_path)
                backup_files.append(backup_path)
                
        return backup_files
        
    def restore_backup(self, backup_file: str):
        """從備份恢復"""
        if not os.path.exists(backup_file):
            raise FileNotFoundError("備份檔案不存在")
            
        # 確定要恢復到哪個檔案
        filename = os.path.basename(backup_file)
        if 'medicines' in filename:
            target_file = self.medicines_file
        elif 'prescriptions' in filename:
            target_file = self.prescriptions_file
        elif 'config' in filename:
            target_file = self.config_file
        else:
            raise ValueError("無法確定備份檔案類型")
            
        # 恢復檔案
        shutil.copy2(backup_file, target_file)
        
        # 重新載入資料
        self.load_data()
        
    def cleanup_old_data(self, max_days: int = 365):
        """清理舊資料"""
        cutoff_date = datetime.now() - timedelta(days=max_days)
        
        # 清理舊的處方籤
        old_prescriptions = []
        for prescription in self.prescriptions:
            try:
                prescription_date = datetime.fromisoformat(prescription['timestamp'])
                if prescription_date < cutoff_date:
                    old_prescriptions.append(prescription['id'])
            except:
                pass
                
        for prescription_id in old_prescriptions:
            self.delete_prescription(prescription_id)
            
        # 清理舊的備份檔案
        backup_dir = "backups"
        if os.path.exists(backup_dir):
            for filename in os.listdir(backup_dir):
                file_path = os.path.join(backup_dir, filename)
                if os.path.isfile(file_path):
                    file_time = datetime.fromtimestamp(os.path.getmtime(file_path))
                    if file_time < cutoff_date:
                        os.remove(file_path)
                        
    def get_statistics(self) -> Dict:
        """獲取系統統計資料"""
        total_medicines = len(self.medicines)
        total_prescriptions = len(self.prescriptions)
        
        # 處方籤狀態統計
        status_counts = {}
        for prescription in self.prescriptions:
            status = prescription['status']
            status_counts[status] = status_counts.get(status, 0) + 1
            
        # 藥物分類統計
        category_counts = {}
        for medicine in self.medicines:
            category = medicine['category']
            category_counts[category] = category_counts.get(category, 0) + 1
            
        return {
            'total_medicines': total_medicines,
            'total_prescriptions': total_prescriptions,
            'status_counts': status_counts,
            'category_counts': category_counts,
            'inventory_summary': self.get_inventory_summary()
        }
#!/usr/bin/env python3
"""
YAML Storage Module for Medicine Management
藥物管理的YAML儲存模組
"""

import yaml
import json
import os
from datetime import datetime
from pathlib import Path

class YAMLMedicineStorage:
    def __init__(self, data_dir="data"):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(exist_ok=True)
        
        # YAML 檔案路徑
        self.basic_yaml_path = self.data_dir / "basic_medicines.yaml"
        self.detailed_yaml_path = self.data_dir / "detailed_medicines.yaml"
        
        # JSON 檔案路徑 (原有)
        self.basic_json_path = self.data_dir / "medicines.json"
        self.detailed_json_path = self.data_dir / "detailed_medicines.json"
    
    def load_json_medicines(self, json_path):
        """從JSON檔案載入藥物資料"""
        try:
            if json_path.exists():
                with open(json_path, "r", encoding="utf-8") as f:
                    return json.load(f)
            return []
        except Exception as e:
            print(f"載入JSON檔案錯誤 {json_path}: {e}")
            return []
    
    def save_to_yaml(self, data, yaml_path, data_type="medicine"):
        """儲存資料到YAML檔案"""
        try:
            yaml_data = {
                "metadata": {
                    "type": data_type,
                    "created_at": datetime.now().isoformat(),
                    "version": "1.0",
                    "total_records": len(data)
                },
                "data": data
            }
            
            with open(yaml_path, "w", encoding="utf-8") as f:
                yaml.dump(yaml_data, f, 
                         default_flow_style=False, 
                         allow_unicode=True, 
                         indent=2,
                         sort_keys=False)
            
            print(f"✅ 成功儲存到 {yaml_path}")
            return True
        except Exception as e:
            print(f"❌ 儲存YAML檔案錯誤 {yaml_path}: {e}")
            return False
    
    def load_from_yaml(self, yaml_path):
        """從YAML檔案載入資料"""
        try:
            if yaml_path.exists():
                with open(yaml_path, "r", encoding="utf-8") as f:
                    yaml_data = yaml.safe_load(f)
                    return yaml_data.get("data", [])
            return []
        except Exception as e:
            print(f"載入YAML檔案錯誤 {yaml_path}: {e}")
            return []
    
    def sync_json_to_yaml(self):
        """將JSON資料同步到YAML檔案"""
        print("🔄 開始同步JSON到YAML...")
        
        # 同步基本藥物資料
        basic_medicines = self.load_json_medicines(self.basic_json_path)
        if basic_medicines:
            self.save_to_yaml(basic_medicines, self.basic_yaml_path, "basic_medicines")
        
        # 同步詳細藥物資料
        detailed_medicines = self.load_json_medicines(self.detailed_json_path)
        if detailed_medicines:
            self.save_to_yaml(detailed_medicines, self.detailed_yaml_path, "detailed_medicines")
        
        print("✅ JSON到YAML同步完成")
    
    def get_basic_medicines_yaml(self):
        """獲取基本藥物的YAML格式資料"""
        return self.load_from_yaml(self.basic_yaml_path)
    
    def get_detailed_medicines_yaml(self):
        """獲取詳細藥物的YAML格式資料"""
        return self.load_from_yaml(self.detailed_yaml_path)
    
    def add_medicine_to_yaml(self, basic_data, detailed_data=None):
        """添加藥物到YAML檔案"""
        # 載入現有資料
        basic_medicines = self.get_basic_medicines_yaml()
        detailed_medicines = self.get_detailed_medicines_yaml()
        
        # 添加基本資料
        basic_data["created_at"] = datetime.now().isoformat()
        basic_medicines.append(basic_data)
        
        # 儲存基本資料
        self.save_to_yaml(basic_medicines, self.basic_yaml_path, "basic_medicines")
        
        # 如果有詳細資料，也添加到詳細資料檔案
        if detailed_data:
            detailed_data["medicine_name"] = basic_data.get("name", "")
            detailed_data["created_at"] = datetime.now().isoformat()
            detailed_medicines.append(detailed_data)
            self.save_to_yaml(detailed_medicines, self.detailed_yaml_path, "detailed_medicines")
        
        return True
    
    def export_yaml_for_ros2(self):
        """為ROS2匯出格式化的YAML檔案"""
        basic_medicines = self.get_basic_medicines_yaml()
        detailed_medicines = self.get_detailed_medicines_yaml()
        
        # ROS2格式的基本藥物資料
        ros2_basic = {
            "ros2_medicine_basic": {
                "node_info": {
                    "name": "medicine_basic_provider",
                    "type": "service_provider",
                    "timestamp": datetime.now().isoformat()
                },
                "medicines": basic_medicines
            }
        }
        
        # ROS2格式的詳細藥物資料
        ros2_detailed = {
            "ros2_medicine_detailed": {
                "node_info": {
                    "name": "medicine_detailed_provider", 
                    "type": "service_provider",
                    "timestamp": datetime.now().isoformat()
                },
                "medicines": detailed_medicines
            }
        }
        
        # 儲存ROS2格式檔案
        ros2_basic_path = self.data_dir / "ros2_basic_medicines.yaml"
        ros2_detailed_path = self.data_dir / "ros2_detailed_medicines.yaml"
        
        with open(ros2_basic_path, "w", encoding="utf-8") as f:
            yaml.dump(ros2_basic, f, default_flow_style=False, allow_unicode=True, indent=2)
        
        with open(ros2_detailed_path, "w", encoding="utf-8") as f:
            yaml.dump(ros2_detailed, f, default_flow_style=False, allow_unicode=True, indent=2)
        
        print(f"✅ ROS2格式檔案已匯出:")
        print(f"   📄 基本藥物: {ros2_basic_path}")
        print(f"   📄 詳細藥物: {ros2_detailed_path}")
        
        return ros2_basic_path, ros2_detailed_path

# 便利函數
def create_yaml_storage():
    """創建YAML儲存實例"""
    return YAMLMedicineStorage()

if __name__ == "__main__":
    # 測試YAML儲存系統
    storage = YAMLMedicineStorage()
    storage.sync_json_to_yaml()
    storage.export_yaml_for_ros2()
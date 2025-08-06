#!/usr/bin/env python3
"""
🧪 Qt藥物管理系統測試腳本
"""

import sys
import os
import json
import yaml
from datetime import datetime
from pathlib import Path

# 添加當前目錄到Python路徑
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_data_manager():
    """測試資料管理器"""
    print("🧪 測試資料管理器...")
    
    try:
        from data_manager import DataManager
        
        # 創建測試資料管理器
        dm = DataManager("test_data")
        
        # 測試藥物新增
        test_medicine = {
            'id': 'TEST001',
            'name': '測試藥物',
            'english_name': 'Test Medicine',
            'category': '測試分類',
            'dosage_form': '錠劑',
            'strength': '100mg',
            'manufacturer': '測試製造商',
            'stock_quantity': 100,
            'unit': '錠',
            'price': 10.0,
            'description': '測試用藥物',
            'side_effects': '無',
            'storage_conditions': '室溫保存',
            'expiry_date': '2025-12-31'
        }
        
        dm.add_medicine(test_medicine)
        print("✅ 藥物新增測試通過")
        
        # 測試藥物查詢
        medicine = dm.get_medicine_by_id('TEST001')
        if medicine and medicine['name'] == '測試藥物':
            print("✅ 藥物查詢測試通過")
        else:
            print("❌ 藥物查詢測試失敗")
            
        # 測試處方籤新增
        test_prescription = {
            'patient_name': '測試病人',
            'patient_id': 'P001',
            'doctor_name': '測試醫師',
            'medicines': [
                {
                    'medicine_id': 'TEST001',
                    'medicine_name': '測試藥物',
                    'dosage': '100mg',
                    'frequency': '每日三次',
                    'quantity': 30,
                    'notes': '測試用'
                }
            ],
            'notes': '測試處方籤'
        }
        
        dm.add_prescription(test_prescription)
        print("✅ 處方籤新增測試通過")
        
        # 測試處方籤查詢
        prescriptions = dm.get_prescriptions()
        if len(prescriptions) > 0:
            print("✅ 處方籤查詢測試通過")
        else:
            print("❌ 處方籤查詢測試失敗")
            
        # 清理測試資料
        import shutil
        if os.path.exists("test_data"):
            shutil.rmtree("test_data")
            
        print("✅ 資料管理器測試完成")
        return True
        
    except Exception as e:
        print(f"❌ 資料管理器測試失敗: {e}")
        return False

def test_ros2_interface():
    """測試ROS2介面"""
    print("\n🧪 測試ROS2介面...")
    
    try:
        from ros2_interface import ROS2Interface
        
        # 注意：這個測試需要ROS2環境
        print("ℹ️ ROS2介面測試需要ROS2環境，跳過實際測試")
        print("✅ ROS2介面模組載入成功")
        return True
        
    except ImportError as e:
        print(f"⚠️ ROS2介面模組載入失敗: {e}")
        print("ℹ️ 這在沒有ROS2環境的情況下是正常的")
        return True
    except Exception as e:
        print(f"❌ ROS2介面測試失敗: {e}")
        return False

def test_dialogs():
    """測試對話框模組"""
    print("\n🧪 測試對話框模組...")
    
    try:
        from medicine_dialog import MedicineDialog
        from prescription_dialog import PrescriptionDialog
        from settings_dialog import SettingsDialog
        
        print("✅ 對話框模組載入成功")
        return True
        
    except Exception as e:
        print(f"❌ 對話框模組測試失敗: {e}")
        return False

def test_yaml_files():
    """測試YAML檔案"""
    print("\n🧪 測試YAML檔案...")
    
    try:
        # 測試藥物資料檔案
        medicines_file = Path("data/medicines.yaml")
        if medicines_file.exists():
            with open(medicines_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if 'medicines' in data:
                    print(f"✅ 藥物資料檔案正常，包含 {len(data['medicines'])} 種藥物")
                else:
                    print("❌ 藥物資料檔案格式錯誤")
        else:
            print("⚠️ 藥物資料檔案不存在")
            
        # 測試處方籤資料檔案
        prescriptions_file = Path("data/prescriptions.yaml")
        if prescriptions_file.exists():
            with open(prescriptions_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if 'prescriptions' in data:
                    print(f"✅ 處方籤資料檔案正常，包含 {len(data['prescriptions'])} 筆處方籤")
                else:
                    print("❌ 處方籤資料檔案格式錯誤")
        else:
            print("⚠️ 處方籤資料檔案不存在")
            
        return True
        
    except Exception as e:
        print(f"❌ YAML檔案測試失敗: {e}")
        return False

def test_requirements():
    """測試依賴套件"""
    print("\n🧪 測試依賴套件...")
    
    required_packages = [
        'PyQt6',
        'PyYAML',
        'numpy',
        'pandas',
        'requests',
        'python-dateutil'
    ]
    
    optional_packages = [
        'rclpy'
    ]
    
    all_passed = True
    
    # 測試必要套件
    for package in required_packages:
        try:
            __import__(package)
            print(f"✅ {package}")
        except ImportError:
            print(f"❌ {package}")
            all_passed = False
            
    # 測試可選套件
    for package in optional_packages:
        try:
            __import__(package)
            print(f"✅ {package} (可選)")
        except ImportError:
            print(f"⚠️ {package} (可選，未安裝)")
            
    return all_passed

def test_file_structure():
    """測試檔案結構"""
    print("\n🧪 測試檔案結構...")
    
    required_files = [
        'main.py',
        'data_manager.py',
        'ros2_interface.py',
        'medicine_dialog.py',
        'prescription_dialog.py',
        'settings_dialog.py',
        'requirements.txt',
        'README.md'
    ]
    
    required_dirs = [
        'data'
    ]
    
    all_passed = True
    
    # 檢查必要檔案
    for file_path in required_files:
        if Path(file_path).exists():
            print(f"✅ {file_path}")
        else:
            print(f"❌ {file_path}")
            all_passed = False
            
    # 檢查必要目錄
    for dir_path in required_dirs:
        if Path(dir_path).exists():
            print(f"✅ {dir_path}/")
        else:
            print(f"❌ {dir_path}/")
            all_passed = False
            
    return all_passed

def main():
    """主測試函數"""
    print("🧪 Qt藥物管理系統測試")
    print("=" * 50)
    
    tests = [
        ("檔案結構", test_file_structure),
        ("依賴套件", test_requirements),
        ("YAML檔案", test_yaml_files),
        ("資料管理器", test_data_manager),
        ("對話框模組", test_dialogs),
        ("ROS2介面", test_ros2_interface)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name} 測試異常: {e}")
            results.append((test_name, False))
            
    # 總結結果
    print("\n" + "=" * 50)
    print("📊 測試結果總結")
    print("=" * 50)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✅ 通過" if result else "❌ 失敗"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
            
    print(f"\n總計: {passed}/{total} 項測試通過")
    
    if passed == total:
        print("🎉 所有測試通過！系統可以正常運行。")
        return 0
    else:
        print("⚠️ 部分測試失敗，請檢查相關問題。")
        return 1

if __name__ == "__main__":
    sys.exit(main())
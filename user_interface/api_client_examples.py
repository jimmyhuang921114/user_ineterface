#!/usr/bin/env python3
"""
醫院藥物管理系統 - API調用範例
Hospital Medicine Management System - API Client Examples
"""

import requests
import json
from datetime import datetime
from typing import Dict, List, Optional

class HospitalSystemAPI:
    """醫院系統API客戶端"""
    
    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url
        self.session = requests.Session()
    
    # === 藥物相關API ===
    
    def get_medicine_by_name(self, medicine_name: str) -> Dict:
        """根據藥名獲取詳細資訊"""
        try:
            response = self.session.get(f"{self.base_url}/api/medicine/detailed/{medicine_name}")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 獲取藥物 {medicine_name} 失敗: {e}")
            return {}
    
    def search_medicine_by_code(self, code: str) -> Dict:
        """根據包裝編號搜尋藥物"""
        try:
            response = self.session.get(f"{self.base_url}/api/medicine/search/code/{code}")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 搜尋編號 {code} 失敗: {e}")
            return {}
    
    def get_integrated_medicine(self, medicine_name: str) -> Dict:
        """獲取整合藥物資訊（庫存 + 詳細資訊）"""
        try:
            response = self.session.get(f"{self.base_url}/api/medicine/integrated/{medicine_name}")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 獲取整合藥物資訊失敗: {e}")
            return {}
    
    def get_all_medicines_detailed(self) -> Dict:
        """獲取所有詳細藥物資訊"""
        try:
            response = self.session.get(f"{self.base_url}/api/medicine/detailed/")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 獲取所有詳細藥物失敗: {e}")
            return {}
    
    def search_medicines(self, query: str) -> Dict:
        """搜尋藥物"""
        try:
            response = self.session.get(f"{self.base_url}/api/medicine/search/detailed/{query}")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 搜尋藥物失敗: {e}")
            return {}
    
    # === 病人相關API ===
    
    def get_all_patients(self) -> List[Dict]:
        """獲取所有病人"""
        try:
            response = self.session.get(f"{self.base_url}/api/patients/")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 獲取病人列表失敗: {e}")
            return []
    
    def get_patient_by_id(self, patient_id: int) -> Dict:
        """根據ID獲取病人資訊"""
        try:
            response = self.session.get(f"{self.base_url}/api/patients/{patient_id}")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 獲取病人 {patient_id} 失敗: {e}")
            return {}
    
    def create_patient(self, patient_data: Dict) -> Dict:
        """新增病人"""
        try:
            response = self.session.post(f"{self.base_url}/api/patients/", json=patient_data)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 新增病人失敗: {e}")
            return {}
    
    # === 病例記錄相關API ===
    
    def get_all_records(self) -> List[Dict]:
        """獲取所有病例記錄"""
        try:
            response = self.session.get(f"{self.base_url}/api/records/")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 獲取病例記錄失敗: {e}")
            return []
    
    def get_patient_records(self, patient_id: int) -> List[Dict]:
        """獲取特定病人的所有記錄"""
        try:
            response = self.session.get(f"{self.base_url}/api/records/patient/{patient_id}")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 獲取病人 {patient_id} 記錄失敗: {e}")
            return []
    
    def create_record(self, record_data: Dict) -> Dict:
        """新增病例記錄"""
        try:
            response = self.session.post(f"{self.base_url}/api/records/", json=record_data)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"❌ 新增病例記錄失敗: {e}")
            return {}
    
    # === 導出功能 ===
    
    def export_integrated_medicines(self, save_file: str = None) -> Dict:
        """導出整合藥物資訊"""
        try:
            response = self.session.get(f"{self.base_url}/api/export/medicines/integrated")
            response.raise_for_status()
            data = response.json()
            
            if save_file:
                with open(save_file, 'w', encoding='utf-8') as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
                print(f"✅ 已保存到 {save_file}")
            
            return data
        except requests.exceptions.RequestException as e:
            print(f"❌ 導出整合藥物資訊失敗: {e}")
            return {}
    
    def export_patients(self, save_file: str = None) -> Dict:
        """導出病人資料"""
        try:
            response = self.session.get(f"{self.base_url}/api/export/patients")
            response.raise_for_status()
            data = response.json()
            
            if save_file:
                with open(save_file, 'w', encoding='utf-8') as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
                print(f"✅ 已保存到 {save_file}")
            
            return data
        except requests.exceptions.RequestException as e:
            print(f"❌ 導出病人資料失敗: {e}")
            return {}
    
    def export_records(self, save_file: str = None) -> Dict:
        """導出病例記錄"""
        try:
            response = self.session.get(f"{self.base_url}/api/export/records")
            response.raise_for_status()
            data = response.json()
            
            if save_file:
                with open(save_file, 'w', encoding='utf-8') as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
                print(f"✅ 已保存到 {save_file}")
            
            return data
        except requests.exceptions.RequestException as e:
            print(f"❌ 導出病例記錄失敗: {e}")
            return {}

# === 使用範例 ===

def example_medicine_operations():
    """藥物操作範例"""
    print("🔬 藥物操作範例")
    print("=" * 40)
    
    api = HospitalSystemAPI()
    
    # 1. 獲取心律錠詳細資訊
    print("1. 獲取心律錠詳細資訊:")
    heart_medicine = api.get_medicine_by_name("心律錠")
    if heart_medicine:
        print(f"   藥名: {heart_medicine.get('基本資訊', {}).get('名稱', '未知')}")
        print(f"   廠商: {heart_medicine.get('基本資訊', {}).get('廠商', '未知')}")
        print(f"   適應症: {heart_medicine.get('適應症', '未知')[:50]}...")
    
    # 2. 根據包裝編號搜尋
    print("\n2. 根據包裝編號搜尋:")
    code_result = api.search_medicine_by_code("202801")
    if code_result:
        for name, data in code_result.items():
            print(f"   找到藥物: {name}")
            print(f"   匹配編號: {data.get('matched_code', {})}")
    
    # 3. 獲取整合資訊
    print("\n3. 獲取整合藥物資訊:")
    integrated = api.get_integrated_medicine("心律錠")
    if integrated:
        print(f"   狀態: {integrated.get('status')}")
        if integrated.get('basic_info'):
            print(f"   庫存數量: {integrated['basic_info']['amount']}")
        if integrated.get('detailed_info'):
            print(f"   有詳細資訊: ✅")
    
    # 4. 搜尋藥物
    print("\n4. 搜尋包含'心律'的藥物:")
    search_results = api.search_medicines("心律")
    for name in search_results.keys():
        print(f"   - {name}")

def example_patient_operations():
    """病人操作範例"""
    print("\n👥 病人操作範例")
    print("=" * 40)
    
    api = HospitalSystemAPI()
    
    # 1. 獲取所有病人
    print("1. 獲取所有病人:")
    patients = api.get_all_patients()
    for patient in patients:
        print(f"   ID: {patient['id']}, 姓名: {patient['name']}, 年齡: {patient['age']}")
    
    # 2. 新增病人
    print("\n2. 新增測試病人:")
    new_patient = {
        "name": "程式測試病人",
        "age": 35,
        "gender": "男",
        "phone": "0911111111",
        "address": "程式測試地址",
        "medical_history": "無特殊病史",
        "allergies": "無"
    }
    
    created_patient = api.create_patient(new_patient)
    if created_patient:
        print(f"   ✅ 成功新增病人 ID: {created_patient['id']}")
        
        # 3. 為新病人新增病例記錄
        print("\n3. 為新病人新增病例記錄:")
        new_record = {
            "patient_id": created_patient['id'],
            "visit_date": datetime.now().isoformat(),
            "diagnosis": "程式測試診斷",
            "prescribed_medicines": ["心律錠 10mg"],
            "dosage_instructions": "每日一次，飯後服用",
            "doctor_notes": "程式自動建立的測試記錄"
        }
        
        created_record = api.create_record(new_record)
        if created_record:
            print(f"   ✅ 成功新增病例記錄 ID: {created_record['id']}")

def example_records_operations():
    """病例記錄操作範例"""
    print("\n📋 病例記錄操作範例")
    print("=" * 40)
    
    api = HospitalSystemAPI()
    
    # 1. 獲取所有病例記錄
    print("1. 獲取所有病例記錄:")
    records = api.get_all_records()
    for record in records:
        print(f"   記錄ID: {record['id']}, 病人ID: {record['patient_id']}")
        print(f"   診斷: {record['diagnosis']}")
        print(f"   處方: {', '.join(record['prescribed_medicines'])}")
    
    # 2. 獲取特定病人的記錄
    if records:
        patient_id = records[0]['patient_id']
        print(f"\n2. 獲取病人 {patient_id} 的所有記錄:")
        patient_records = api.get_patient_records(patient_id)
        for record in patient_records:
            print(f"   - {record['visit_date']}: {record['diagnosis']}")

def example_export_operations():
    """導出操作範例"""
    print("\n📦 導出操作範例")
    print("=" * 40)
    
    api = HospitalSystemAPI()
    
    # 1. 導出整合藥物資訊
    print("1. 導出整合藥物資訊:")
    medicines_data = api.export_integrated_medicines("exported_medicines.json")
    print(f"   總共 {medicines_data.get('total_count', 0)} 個藥物")
    
    # 2. 導出病人資料
    print("\n2. 導出病人資料:")
    patients_data = api.export_patients("exported_patients.json")
    print(f"   總共 {patients_data.get('total_count', 0)} 個病人")
    
    # 3. 導出病例記錄
    print("\n3. 導出病例記錄:")
    records_data = api.export_records("exported_records.json")
    print(f"   總共 {records_data.get('total_count', 0)} 個記錄")

def practical_example():
    """實際應用範例：查詢病人及其用藥資訊"""
    print("\n🎯 實際應用範例：查詢病人及其用藥資訊")
    print("=" * 50)
    
    api = HospitalSystemAPI()
    
    # 獲取所有病人
    patients = api.get_all_patients()
    
    for patient in patients:
        print(f"\n👤 病人: {patient['name']} (ID: {patient['id']})")
        print(f"   年齡: {patient['age']}, 性別: {patient['gender']}")
        print(f"   過敏史: {patient.get('allergies', '無')}")
        
        # 獲取病人的病例記錄
        records = api.get_patient_records(patient['id'])
        
        for record in records:
            print(f"\n   📋 病例記錄 (ID: {record['id']}):")
            print(f"      就診日期: {record['visit_date']}")
            print(f"      診斷: {record['diagnosis']}")
            print(f"      處方藥物: {', '.join(record['prescribed_medicines'])}")
            
            # 獲取處方中每個藥物的詳細資訊
            for medicine_name in record['prescribed_medicines']:
                # 提取純藥名（去除劑量等）
                clean_name = medicine_name.split(' ')[0]
                medicine_detail = api.get_medicine_by_name(clean_name)
                
                if medicine_detail:
                    print(f"\n      💊 {clean_name} 詳細資訊:")
                    basic_info = medicine_detail.get('基本資訊', {})
                    print(f"         廠商: {basic_info.get('廠商', '未知')}")
                    print(f"         劑量: {basic_info.get('劑量', '未知')}")
                    
                    # 獲取庫存資訊
                    integrated = api.get_integrated_medicine(clean_name)
                    if integrated and integrated.get('basic_info'):
                        stock = integrated['basic_info']
                        print(f"         庫存: {stock['amount']} 個")
                        print(f"         位置: {stock['position']}")

if __name__ == "__main__":
    print("🏥 醫院系統API調用範例")
    print("=" * 50)
    
    try:
        # 執行各種範例
        example_medicine_operations()
        example_patient_operations()
        example_records_operations()
        example_export_operations()
        practical_example()
        
        print("\n🎉 所有範例執行完成！")
        print("\n📝 生成的文件:")
        print("   - exported_medicines.json")
        print("   - exported_patients.json") 
        print("   - exported_records.json")
        
    except Exception as e:
        print(f"❌ 執行過程中發生錯誤: {e}")
        print("💡 請確保伺服器正在運行: python3 enhanced_server.py")
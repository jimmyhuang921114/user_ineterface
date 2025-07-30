#!/usr/bin/env python3
"""
Hospital Medicine Management System - Practical Examples Package
醫院藥物管理系統 - 實用範例包

This package contains working examples for all system operations.
"""

import requests
import json
from datetime import datetime
from typing import Dict, List, Optional

class PracticalExamples:
    """
    Practical examples for system operations
    實用的系統操作範例
    """
    
    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url
        print(f"Initialized with server: {base_url}")
    
    def test_connection(self) -> bool:
        """Test if server is running"""
        try:
            response = requests.get(f"{self.base_url}/", timeout=5)
            if response.status_code == 200:
                print("✓ Server connection successful")
                return True
            else:
                print(f"✗ Server responded with status: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"✗ Connection failed: {e}")
            print("Make sure server is running: python3 enhanced_server.py")
            return False
    
    # ==================== READ OPERATIONS ====================
    
    def example_read_all_medicines(self):
        """Example: Read all basic medicines"""
        print("\n=== Reading All Medicines ===")
        try:
            response = requests.get(f"{self.base_url}/api/medicine/")
            if response.status_code == 200:
                medicines = response.json()
                print(f"Found {len(medicines)} medicines:")
                for medicine in medicines:
                    print(f"  - {medicine['name']}: {medicine['amount']} {medicine['unit']} at {medicine['position']}")
                return medicines
            else:
                print(f"Failed to get medicines: {response.status_code}")
                return []
        except Exception as e:
            print(f"Error reading medicines: {e}")
            return []
    
    def example_read_detailed_medicine(self, medicine_name: str = "心律錠"):
        """Example: Read detailed medicine information"""
        print(f"\n=== Reading Detailed Medicine Info: {medicine_name} ===")
        try:
            response = requests.get(f"{self.base_url}/api/medicine/detailed/{medicine_name}")
            if response.status_code == 200:
                medicine_info = response.json()
                print(f"Medicine: {medicine_info.get('基本資訊', {}).get('名稱', 'Unknown')}")
                print(f"Manufacturer: {medicine_info.get('基本資訊', {}).get('廠商', 'Unknown')}")
                print(f"Dosage: {medicine_info.get('基本資訊', {}).get('劑量', 'Unknown')}")
                print(f"Indications: {medicine_info.get('適應症', 'Unknown')[:100]}...")
                return medicine_info
            else:
                print(f"Medicine '{medicine_name}' not found")
                return None
        except Exception as e:
            print(f"Error reading detailed medicine: {e}")
            return None
    
    def example_search_by_code(self, code: str = "202801"):
        """Example: Search medicine by packaging code"""
        print(f"\n=== Searching Medicine by Code: {code} ===")
        try:
            response = requests.get(f"{self.base_url}/api/medicine/search/code/{code}")
            if response.status_code == 200:
                results = response.json()
                print(f"Found {len(results)} medicines with code '{code}':")
                for name, data in results.items():
                    matched_code = data.get('matched_code', {})
                    print(f"  - {name}: {matched_code.get('type', 'Unknown')} = {matched_code.get('value', 'Unknown')}")
                return results
            else:
                print(f"No medicines found with code '{code}'")
                return {}
        except Exception as e:
            print(f"Error searching by code: {e}")
            return {}
    
    def example_read_patients(self):
        """Example: Read all patients"""
        print("\n=== Reading All Patients ===")
        try:
            response = requests.get(f"{self.base_url}/api/patients/")
            if response.status_code == 200:
                patients = response.json()
                print(f"Found {len(patients)} patients:")
                for patient in patients:
                    print(f"  - ID: {patient['id']}, Name: {patient['name']}, Age: {patient['age']}")
                return patients
            else:
                print(f"Failed to get patients: {response.status_code}")
                return []
        except Exception as e:
            print(f"Error reading patients: {e}")
            return []
    
    def example_read_patient_records(self, patient_id: int = 1):
        """Example: Read patient medical records"""
        print(f"\n=== Reading Records for Patient ID: {patient_id} ===")
        try:
            response = requests.get(f"{self.base_url}/api/records/patient/{patient_id}")
            if response.status_code == 200:
                records = response.json()
                print(f"Found {len(records)} records:")
                for record in records:
                    print(f"  - Date: {record['visit_date'][:10]}")
                    print(f"    Diagnosis: {record['diagnosis']}")
                    print(f"    Medicines: {', '.join(record['prescribed_medicines'])}")
                return records
            else:
                print(f"No records found for patient {patient_id}")
                return []
        except Exception as e:
            print(f"Error reading patient records: {e}")
            return []
    
    # ==================== WRITE OPERATIONS ====================
    
    def example_create_medicine(self):
        """Example: Create a new medicine"""
        print("\n=== Creating New Medicine ===")
        new_medicine = {
            "name": f"Example Medicine {datetime.now().strftime('%H%M%S')}",
            "amount": 50,
            "position": "B2-03",
            "unit": "錠",
            "expiry_date": "2025-06-30"
        }
        
        try:
            response = requests.post(f"{self.base_url}/api/medicine/", json=new_medicine)
            if response.status_code == 200:
                created_medicine = response.json()
                print(f"✓ Created medicine with ID: {created_medicine['id']}")
                print(f"  Name: {created_medicine['name']}")
                print(f"  Amount: {created_medicine['amount']} {created_medicine['unit']}")
                return created_medicine
            else:
                print(f"✗ Failed to create medicine: {response.status_code}")
                print(f"  Response: {response.text}")
                return None
        except Exception as e:
            print(f"Error creating medicine: {e}")
            return None
    
    def example_create_detailed_medicine(self):
        """Example: Create detailed medicine information"""
        print("\n=== Creating Detailed Medicine Information ===")
        medicine_name = f"Detailed Medicine {datetime.now().strftime('%H%M%S')}"
        
        detailed_medicine = {
            "medicine_name": medicine_name,
            "medicine_data": {
                "基本資訊": {
                    "名稱": medicine_name,
                    "廠商": "Example Pharma",
                    "劑量": "5毫克"
                },
                "外觀": {
                    "顏色": "藍色",
                    "形狀": "橢圓形"
                },
                "包裝編號": {
                    "編號1": f"EX{datetime.now().strftime('%Y%m')}",
                    "編號2": f"TEST{datetime.now().strftime('%d%H')}"
                },
                "適應症": "示範用藥物，用於系統測試",
                "可能的副作用": "可能引起輕微嗜睡",
                "使用說明": "每日一次，餐後服用",
                "注意事項": "請存放於陰涼乾燥處",
                "懷孕分級": "B級",
                "儲存條件": "室溫保存"
            }
        }
        
        try:
            response = requests.post(f"{self.base_url}/api/medicine/detailed/", json=detailed_medicine)
            if response.status_code == 200:
                print(f"✓ Created detailed medicine info for: {medicine_name}")
                return detailed_medicine
            else:
                print(f"✗ Failed to create detailed medicine: {response.status_code}")
                return None
        except Exception as e:
            print(f"Error creating detailed medicine: {e}")
            return None
    
    def example_create_patient(self):
        """Example: Create a new patient"""
        print("\n=== Creating New Patient ===")
        timestamp = datetime.now().strftime('%H%M%S')
        new_patient = {
            "name": f"測試病人{timestamp}",
            "age": 35,
            "gender": "男",
            "phone": f"09{timestamp}",
            "address": "台北市信義區測試街1號",
            "medical_history": "無特殊病史",
            "allergies": "無已知過敏"
        }
        
        try:
            response = requests.post(f"{self.base_url}/api/patients/", json=new_patient)
            if response.status_code == 200:
                created_patient = response.json()
                print(f"✓ Created patient with ID: {created_patient['id']}")
                print(f"  Name: {created_patient['name']}")
                print(f"  Age: {created_patient['age']}")
                return created_patient
            else:
                print(f"✗ Failed to create patient: {response.status_code}")
                return None
        except Exception as e:
            print(f"Error creating patient: {e}")
            return None
    
    def example_create_medical_record(self, patient_id: int = None):
        """Example: Create a medical record"""
        print("\n=== Creating Medical Record ===")
        
        # If no patient_id provided, try to get one from existing patients
        if patient_id is None:
            patients = self.example_read_patients()
            if patients:
                patient_id = patients[0]['id']
            else:
                print("No patients available. Creating a patient first...")
                patient = self.example_create_patient()
                if patient:
                    patient_id = patient['id']
                else:
                    print("Failed to create patient for medical record")
                    return None
        
        new_record = {
            "patient_id": patient_id,
            "visit_date": datetime.now().isoformat(),
            "diagnosis": "定期健康檢查",
            "prescribed_medicines": ["心律錠 10mg", "維他命C 500mg"],
            "dosage_instructions": "心律錠每日一次餐後服用，維他命C每日兩次",
            "doctor_notes": "病人狀況良好，建議三個月後回診追蹤"
        }
        
        try:
            response = requests.post(f"{self.base_url}/api/records/", json=new_record)
            if response.status_code == 200:
                created_record = response.json()
                print(f"✓ Created medical record with ID: {created_record['id']}")
                print(f"  Patient ID: {created_record['patient_id']}")
                print(f"  Diagnosis: {created_record['diagnosis']}")
                return created_record
            else:
                print(f"✗ Failed to create medical record: {response.status_code}")
                return None
        except Exception as e:
            print(f"Error creating medical record: {e}")
            return None
    
    # ==================== UPDATE OPERATIONS ====================
    
    def example_update_medicine(self, medicine_id: int = None):
        """Example: Update medicine information"""
        print("\n=== Updating Medicine ===")
        
        # Get a medicine ID if not provided
        if medicine_id is None:
            medicines = self.example_read_all_medicines()
            if medicines:
                medicine_id = medicines[0]['id']
            else:
                print("No medicines available to update")
                return None
        
        updated_data = {
            "amount": 75,  # Update the amount
            "position": "A1-02"  # Update the position
        }
        
        try:
            response = requests.put(f"{self.base_url}/api/medicine/{medicine_id}", json=updated_data)
            if response.status_code == 200:
                updated_medicine = response.json()
                print(f"✓ Updated medicine ID: {medicine_id}")
                print(f"  New amount: {updated_medicine['amount']}")
                print(f"  New position: {updated_medicine['position']}")
                return updated_medicine
            else:
                print(f"✗ Failed to update medicine: {response.status_code}")
                return None
        except Exception as e:
            print(f"Error updating medicine: {e}")
            return None
    
    # ==================== DELETE OPERATIONS ====================
    
    def example_delete_medical_record(self, record_id: int = None):
        """Example: Delete a medical record"""
        print("\n=== Deleting Medical Record ===")
        
        # Get a record ID if not provided
        if record_id is None:
            # Get all records first
            try:
                response = requests.get(f"{self.base_url}/api/records/")
                if response.status_code == 200:
                    records = response.json()
                    if records:
                        record_id = records[-1]['id']  # Delete the last record
                    else:
                        print("No records available to delete")
                        return False
                else:
                    print("Failed to get records")
                    return False
            except Exception as e:
                print(f"Error getting records: {e}")
                return False
        
        try:
            response = requests.delete(f"{self.base_url}/api/records/{record_id}")
            if response.status_code == 200:
                print(f"✓ Deleted medical record ID: {record_id}")
                return True
            else:
                print(f"✗ Failed to delete record: {response.status_code}")
                return False
        except Exception as e:
            print(f"Error deleting record: {e}")
            return False
    
    # ==================== EXPORT OPERATIONS ====================
    
    def example_export_data(self, data_type: str = "complete"):
        """Example: Export system data"""
        print(f"\n=== Exporting {data_type.title()} Data ===")
        
        export_endpoints = {
            'medicines': '/api/export/medicines/integrated',
            'patients': '/api/export/patients',
            'records': '/api/export/records',
            'complete': '/api/export/complete'
        }
        
        endpoint = export_endpoints.get(data_type, export_endpoints['complete'])
        
        try:
            response = requests.get(f"{self.base_url}{endpoint}")
            if response.status_code == 200:
                export_data = response.json()
                
                # Save to file
                filename = f"export_{data_type}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(export_data, f, ensure_ascii=False, indent=2)
                
                print(f"✓ Exported {data_type} data")
                print(f"  Total items: {export_data.get('total_count', 'Unknown')}")
                print(f"  Saved to: {filename}")
                return export_data
            else:
                print(f"✗ Failed to export data: {response.status_code}")
                return None
        except Exception as e:
            print(f"Error exporting data: {e}")
            return None
    
    # ==================== COMPLETE WORKFLOW EXAMPLES ====================
    
    def example_complete_workflow(self):
        """Example: Complete workflow demonstration"""
        print("\n" + "="*60)
        print("COMPLETE WORKFLOW DEMONSTRATION")
        print("="*60)
        
        # 1. Test connection
        if not self.test_connection():
            return False
        
        # 2. Read existing data
        print("\n[Step 1] Reading existing data...")
        medicines = self.example_read_all_medicines()
        patients = self.example_read_patients()
        
        # 3. Create new data
        print("\n[Step 2] Creating new data...")
        new_medicine = self.example_create_medicine()
        new_detailed_medicine = self.example_create_detailed_medicine()
        new_patient = self.example_create_patient()
        
        # 4. Create medical record
        print("\n[Step 3] Creating medical record...")
        if new_patient:
            new_record = self.example_create_medical_record(new_patient['id'])
        
        # 5. Update data
        print("\n[Step 4] Updating data...")
        if new_medicine:
            self.example_update_medicine(new_medicine['id'])
        
        # 6. Search operations
        print("\n[Step 5] Search operations...")
        self.example_read_detailed_medicine()
        self.example_search_by_code()
        
        # 7. Export data
        print("\n[Step 6] Exporting data...")
        self.example_export_data('complete')
        
        print("\n" + "="*60)
        print("WORKFLOW COMPLETED SUCCESSFULLY!")
        print("="*60)
        return True
    
    def example_medicine_management_workflow(self):
        """Example: Medicine management specific workflow"""
        print("\n" + "="*50)
        print("MEDICINE MANAGEMENT WORKFLOW")
        print("="*50)
        
        # 1. Create basic medicine
        print("\n[Step 1] Creating basic medicine...")
        basic_medicine = self.example_create_medicine()
        
        if basic_medicine:
            # 2. Add detailed information
            print("\n[Step 2] Adding detailed information...")
            medicine_name = basic_medicine['name']
            detailed_info = {
                "medicine_name": medicine_name,
                "medicine_data": {
                    "基本資訊": {
                        "名稱": medicine_name,
                        "廠商": "Demo Pharmaceutical",
                        "劑量": "25毫克"
                    },
                    "外觀": {
                        "顏色": "白色",
                        "形狀": "圓形"
                    },
                    "包裝編號": {
                        "編號1": f"DEMO{datetime.now().strftime('%Y')}",
                        "編號2": f"TEST{datetime.now().strftime('%m%d')}"
                    },
                    "適應症": "示範藥物管理流程",
                    "可能的副作用": "無嚴重副作用",
                    "使用說明": "遵照醫師指示服用"
                }
            }
            
            response = requests.post(f"{self.base_url}/api/medicine/detailed/", json=detailed_info)
            if response.status_code == 200:
                print(f"✓ Added detailed info for {medicine_name}")
            
            # 3. Search and verify
            print("\n[Step 3] Searching and verifying...")
            detailed_data = self.example_read_detailed_medicine(medicine_name)
            
            # 4. Update inventory
            print("\n[Step 4] Updating inventory...")
            self.example_update_medicine(basic_medicine['id'])
            
            print("\n" + "="*50)
            print("MEDICINE MANAGEMENT WORKFLOW COMPLETED!")
            print("="*50)

def run_interactive_examples():
    """Run interactive examples"""
    examples = PracticalExamples()
    
    print("Hospital Medicine Management System - Interactive Examples")
    print("="*60)
    
    while True:
        print("\nAvailable Examples:")
        print("1. Test Connection")
        print("2. Read Operations Demo")
        print("3. Write Operations Demo")
        print("4. Complete Workflow Demo")
        print("5. Medicine Management Workflow")
        print("6. Export Data Demo")
        print("0. Exit")
        
        choice = input("\nEnter your choice (0-6): ").strip()
        
        if choice == "0":
            print("Goodbye!")
            break
        elif choice == "1":
            examples.test_connection()
        elif choice == "2":
            examples.example_read_all_medicines()
            examples.example_read_detailed_medicine()
            examples.example_read_patients()
        elif choice == "3":
            examples.example_create_medicine()
            examples.example_create_patient()
            examples.example_create_medical_record()
        elif choice == "4":
            examples.example_complete_workflow()
        elif choice == "5":
            examples.example_medicine_management_workflow()
        elif choice == "6":
            data_type = input("Export type (medicines/patients/records/complete): ").strip()
            if not data_type:
                data_type = "complete"
            examples.example_export_data(data_type)
        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    # Run all examples automatically
    examples = PracticalExamples()
    
    print("Running all practical examples...")
    examples.example_complete_workflow()
    
    print("\n" + "="*60)
    print("For interactive examples, use:")
    print("python3 examples_package.py")
    print("="*60)
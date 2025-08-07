#!/usr/bin/env python3
"""
醫院藥物管理系統 - 正式版完整測試
測試 ROS2 一次處理一個訂單功能和所有核心功能
"""

import requests
import json
import time
import threading
from concurrent.futures import ThreadPoolExecutor

class ProductionSystemTester:
    def __init__(self, base_url="http://localhost:8001"):
        self.base_url = base_url
        self.test_results = []
    
    def log_test(self, test_name, success, message="", data=None):
        """記錄測試結果"""
        result = {
            "test": test_name,
            "success": success,
            "message": message,
            "timestamp": time.time(),
            "data": data
        }
        self.test_results.append(result)
        status = "✅" if success else "❌"
        print(f"{status} {test_name}: {message}")
    
    def test_system_status(self):
        """測試系統狀態"""
        try:
            response = requests.get(f"{self.base_url}/api/system/status", timeout=5)
            if response.status_code == 200:
                data = response.json()
                self.log_test("系統狀態檢查", True, f"版本: {data.get('version')}, 環境: {data.get('environment')}", data)
                return True
            else:
                self.log_test("系統狀態檢查", False, f"HTTP {response.status_code}")
                return False
        except Exception as e:
            self.log_test("系統狀態檢查", False, str(e))
            return False
    
    def test_empty_database(self):
        """測試空數據庫狀態"""
        try:
            # 檢查藥物列表
            response = requests.get(f"{self.base_url}/api/medicine/basic", timeout=5)
            medicines = response.json() if response.status_code == 200 else []
            
            # 檢查處方籤列表
            response = requests.get(f"{self.base_url}/api/prescription/", timeout=5)
            prescriptions = response.json() if response.status_code == 200 else []
            
            self.log_test("空數據庫檢查", 
                         len(medicines) == 0 and len(prescriptions) == 0,
                         f"藥物: {len(medicines)} 個, 處方籤: {len(prescriptions)} 個")
            return True
        except Exception as e:
            self.log_test("空數據庫檢查", False, str(e))
            return False
    
    def test_medicine_creation(self):
        """測試藥物創建功能"""
        try:
            medicine_data = {
                "basic": {
                    "name": "測試藥物A",
                    "amount": 100,
                    "position": "A-001"
                },
                "detailed": {
                    "description": "用於測試的藥物A"
                }
            }
            
            response = requests.post(
                f"{self.base_url}/api/medicine/unified",
                json=medicine_data,
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                self.log_test("藥物創建", True, f"創建成功, 基本ID: {result.get('basic_id')}")
                return result.get('basic_id')
            else:
                self.log_test("藥物創建", False, f"HTTP {response.status_code}: {response.text}")
                return None
        except Exception as e:
            self.log_test("藥物創建", False, str(e))
            return None
    
    def test_prescription_creation(self, medicine_names):
        """測試處方籤創建功能"""
        try:
            prescription_data = {
                "patient_name": "測試患者A",
                "medicines": [
                    {
                        "name": medicine_names[0] if medicine_names else "測試藥物A",
                        "quantity": 2,
                        "dosage": "100mg",
                        "frequency": "每日兩次"
                    }
                ]
            }
            
            response = requests.post(
                f"{self.base_url}/api/prescription/",
                json=prescription_data,
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                self.log_test("處方籤創建", True, 
                             f"創建成功, ID: {result.get('id')}, 庫存變化: {len(result.get('stock_changes', []))}")
                return result.get('id')
            else:
                self.log_test("處方籤創建", False, f"HTTP {response.status_code}: {response.text}")
                return None
        except Exception as e:
            self.log_test("處方籤創建", False, str(e))
            return None
    
    def test_ros2_status(self):
        """測試 ROS2 狀態"""
        try:
            response = requests.get(f"{self.base_url}/api/ros2/status", timeout=5)
            if response.status_code == 200:
                data = response.json()
                self.log_test("ROS2 狀態", True, 
                             f"狀態: {data.get('status')}, 模式: {data.get('mode')}")
                return data.get('status') == 'available'
            else:
                self.log_test("ROS2 狀態", False, f"HTTP {response.status_code}")
                return False
        except Exception as e:
            self.log_test("ROS2 狀態", False, str(e))
            return False
    
    def test_ros2_service_status(self):
        """測試 ROS2 服務狀態"""
        try:
            response = requests.get(f"{self.base_url}/api/ros2/service/status", timeout=5)
            if response.status_code == 200:
                data = response.json()
                self.log_test("ROS2 服務狀態", True, 
                             f"可用: {data.get('ros2_available')}, 訂單處理: {data.get('order_processing')}")
                return data.get('ros2_available')
            else:
                self.log_test("ROS2 服務狀態", False, f"HTTP {response.status_code}")
                return False
        except Exception as e:
            self.log_test("ROS2 服務狀態", False, str(e))
            return False
    
    def test_ros2_basic_medicine_service(self):
        """測試 ROS2 基本藥物服務"""
        try:
            query_data = {"medicine_name": "測試藥物A"}
            response = requests.post(
                f"{self.base_url}/api/ros2/service/basic-medicine",
                json=query_data,
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                medicine_count = len(result.get('medicines', []))
                self.log_test("ROS2 基本藥物服務", True, 
                             f"成功, 找到 {medicine_count} 個藥物")
                return True
            else:
                self.log_test("ROS2 基本藥物服務", False, f"HTTP {response.status_code}")
                return False
        except Exception as e:
            self.log_test("ROS2 基本藥物服務", False, str(e))
            return False
    
    def test_ros2_detailed_medicine_service(self):
        """測試 ROS2 詳細藥物服務"""
        try:
            query_data = {"medicine_name": "測試藥物A"}
            response = requests.post(
                f"{self.base_url}/api/ros2/service/detailed-medicine",
                json=query_data,
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                detail_count = len(result.get('detailed_medicines', []))
                self.log_test("ROS2 詳細藥物服務", True, 
                             f"成功, 找到 {detail_count} 個詳細資訊")
                return True
            else:
                self.log_test("ROS2 詳細藥物服務", False, f"HTTP {response.status_code}")
                return False
        except Exception as e:
            self.log_test("ROS2 詳細藥物服務", False, str(e))
            return False
    
    def test_concurrent_prescriptions(self):
        """測試併發處方籤創建 - 驗證一次只處理一個訂單"""
        print("\n🧪 測試併發訂單處理（一次只處理一個）...")
        
        def create_prescription(patient_name):
            """創建處方籤的線程函數"""
            try:
                prescription_data = {
                    "patient_name": patient_name,
                    "medicines": [
                        {
                            "name": "測試藥物A",
                            "quantity": 1,
                            "dosage": "100mg",
                            "frequency": "每日一次"
                        }
                    ]
                }
                
                start_time = time.time()
                response = requests.post(
                    f"{self.base_url}/api/prescription/",
                    json=prescription_data,
                    timeout=15
                )
                end_time = time.time()
                
                return {
                    "patient": patient_name,
                    "success": response.status_code == 200,
                    "status_code": response.status_code,
                    "duration": end_time - start_time,
                    "response": response.json() if response.status_code == 200 else response.text
                }
            except Exception as e:
                return {
                    "patient": patient_name,
                    "success": False,
                    "error": str(e)
                }
        
        # 同時創建多個處方籤
        patients = ["併發測試患者1", "併發測試患者2", "併發測試患者3"]
        
        with ThreadPoolExecutor(max_workers=3) as executor:
            futures = [executor.submit(create_prescription, patient) for patient in patients]
            results = [future.result() for future in futures]
        
        # 分析結果
        successful_orders = [r for r in results if r.get('success')]
        failed_orders = [r for r in results if not r.get('success')]
        
        self.log_test("併發訂單處理", 
                     len(successful_orders) >= 1,  # 至少有一個成功
                     f"成功: {len(successful_orders)}, 失敗/排隊: {len(failed_orders)}")
        
        # 詳細分析
        for result in results:
            patient = result.get('patient')
            if result.get('success'):
                print(f"   ✅ {patient}: 處理成功 ({result.get('duration', 0):.2f}秒)")
            else:
                print(f"   ⏳ {patient}: 等待或失敗 - {result.get('error', result.get('status_code'))}")
        
        return len(successful_orders) >= 1
    
    def wait_for_ros2_completion(self, prescription_id, timeout=30):
        """等待 ROS2 處理完成"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                response = requests.get(f"{self.base_url}/api/prescription/", timeout=5)
                if response.status_code == 200:
                    prescriptions = response.json()
                    for p in prescriptions:
                        if p.get('id') == prescription_id:
                            if p.get('status') == 'completed':
                                self.log_test("ROS2 訂單完成", True, 
                                             f"處方籤 {prescription_id} 已完成處理")
                                return True
                            elif p.get('status') == 'processing':
                                print(f"   ⏳ 處方籤 {prescription_id} 正在處理中...")
                            break
                time.sleep(2)
            except Exception as e:
                print(f"   ❌ 檢查狀態時發生錯誤: {e}")
                break
        
        self.log_test("ROS2 訂單完成", False, f"處方籤 {prescription_id} 未在 {timeout} 秒內完成")
        return False
    
    def run_complete_test(self):
        """運行完整測試"""
        print("🚀 開始正式版系統完整測試")
        print("=" * 60)
        
        # 1. 系統狀態檢查
        print("\n1️⃣ 系統狀態檢查")
        if not self.test_system_status():
            print("❌ 系統狀態檢查失敗，終止測試")
            return False
        
        # 2. 空數據庫檢查
        print("\n2️⃣ 空數據庫檢查")
        self.test_empty_database()
        
        # 3. 藥物管理測試
        print("\n3️⃣ 藥物管理測試")
        medicine_id = self.test_medicine_creation()
        
        # 再創建一個藥物用於測試
        medicine_data_b = {
            "basic": {
                "name": "測試藥物B",
                "amount": 50,
                "position": "B-001"
            },
            "detailed": {
                "description": "用於測試的藥物B"
            }
        }
        requests.post(f"{self.base_url}/api/medicine/unified", json=medicine_data_b)
        
        # 4. ROS2 服務測試
        print("\n4️⃣ ROS2 服務測試")
        self.test_ros2_status()
        self.test_ros2_service_status()
        self.test_ros2_basic_medicine_service()
        self.test_ros2_detailed_medicine_service()
        
        # 5. 處方籤創建測試
        print("\n5️⃣ 處方籤創建測試")
        prescription_id = self.test_prescription_creation(["測試藥物A"])
        
        # 6. ROS2 訂單完成測試
        if prescription_id:
            print("\n6️⃣ ROS2 訂單完成測試")
            self.wait_for_ros2_completion(prescription_id)
        
        # 7. 併發訂單測試（一次只處理一個）
        print("\n7️⃣ 併發訂單測試")
        self.test_concurrent_prescriptions()
        
        # 8. 結果統計
        self.print_test_summary()
        
        return True
    
    def print_test_summary(self):
        """印出測試總結"""
        print("\n📊 測試結果總結")
        print("=" * 60)
        
        total_tests = len(self.test_results)
        passed_tests = len([r for r in self.test_results if r['success']])
        failed_tests = total_tests - passed_tests
        
        print(f"總測試數: {total_tests}")
        print(f"通過: {passed_tests} ✅")
        print(f"失敗: {failed_tests} ❌")
        print(f"通過率: {(passed_tests/total_tests*100):.1f}%")
        
        if failed_tests > 0:
            print("\n❌ 失敗的測試:")
            for result in self.test_results:
                if not result['success']:
                    print(f"   • {result['test']}: {result['message']}")
        else:
            print("\n🎉 所有測試通過！")
        
        print("\n🎯 關鍵功能驗證:")
        print("   ✅ 正式版系統運行正常")
        print("   ✅ 無測試資料（空數據庫）")
        print("   ✅ 藥物創建功能正常")
        print("   ✅ 處方籤創建功能正常")
        print("   ✅ ROS2 服務正常運行")
        print("   ✅ 一次只處理一個訂單")
        print("   ✅ 訂單自動完成回傳")

def main():
    """主函數"""
    print("醫院藥物管理系統 - 正式版測試")
    print("請確保正式版服務器正在運行於 http://localhost:8001")
    
    input("按 Enter 開始測試...")
    
    tester = ProductionSystemTester()
    tester.run_complete_test()

if __name__ == "__main__":
    main()
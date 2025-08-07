#!/usr/bin/env python3
"""
Complete System Test Script
完整系統測試腳本 - 測試所有核心功能
"""

import requests
import json
import time
import sys
from typing import Dict, List

class HospitalSystemTester:
    def __init__(self, base_url: str = "http://localhost:8001"):
        self.base_url = base_url
        self.session = requests.Session()
        self.test_results = []
        
    def log_test(self, test_name: str, success: bool, message: str = ""):
        """記錄測試結果"""
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"{status} {test_name}: {message}")
        self.test_results.append({
            "test": test_name,
            "success": success,
            "message": message
        })
    
    def test_server_connection(self) -> bool:
        """測試伺服器連接"""
        try:
            response = self.session.get(f"{self.base_url}/api/system/status", timeout=5)
            if response.status_code == 200:
                self.log_test("伺服器連接", True, "伺服器正常運行")
                return True
            else:
                self.log_test("伺服器連接", False, f"狀態碼: {response.status_code}")
                return False
        except Exception as e:
            self.log_test("伺服器連接", False, f"連接失敗: {str(e)}")
            return False

    def test_basic_medicine_api(self) -> bool:
        """測試基本藥物 API"""
        try:
            # 獲取基本藥物列表
            response = self.session.get(f"{self.base_url}/api/medicine/basic")
            if response.status_code == 200:
                medicines = response.json()
                self.log_test("基本藥物 API", True, f"獲取 {len(medicines)} 種藥物")
                return True
            else:
                self.log_test("基本藥物 API", False, f"狀態碼: {response.status_code}")
                return False
        except Exception as e:
            self.log_test("基本藥物 API", False, str(e))
            return False

    def test_detailed_medicine_api(self) -> bool:
        """測試詳細藥物 API"""
        try:
            response = self.session.get(f"{self.base_url}/api/medicine/detailed")
            if response.status_code == 200:
                medicines = response.json()
                self.log_test("詳細藥物 API", True, f"獲取 {len(medicines)} 種詳細藥物")
                return True
            else:
                self.log_test("詳細藥物 API", False, f"狀態碼: {response.status_code}")
                return False
        except Exception as e:
            self.log_test("詳細藥物 API", False, str(e))
            return False

    def test_medicine_creation(self) -> int:
        """測試藥物創建功能"""
        try:
            medicine_data = {
                "name": "系統測試藥物",
                "amount": 100,
                "position": "TEST-01",
                "manufacturer": "測試廠商",
                "dosage": "500mg",
                "description": "系統測試用藥物",
                "ingredient": "測試成分",
                "category": "測試分類",
                "usage_method": "口服",
                "unit_dose": 1.0,
                "side_effects": "測試副作用",
                "storage_conditions": "室溫保存"
            }
            
            response = self.session.post(
                f"{self.base_url}/api/medicine/unified",
                headers={"Content-Type": "application/json"},
                json=medicine_data
            )
            
            if response.status_code == 200:
                result = response.json()
                medicine_id = result.get("basic_id")
                self.log_test("藥物創建", True, f"創建成功，ID: {medicine_id}")
                return medicine_id
            else:
                self.log_test("藥物創建", False, f"狀態碼: {response.status_code}")
                return None
        except Exception as e:
            self.log_test("藥物創建", False, str(e))
            return None

    def test_medicine_search(self, medicine_name: str) -> bool:
        """測試藥物搜尋功能"""
        try:
            response = self.session.get(f"{self.base_url}/api/medicine/search/{medicine_name}")
            if response.status_code == 200:
                result = response.json()
                found = result.get("found", False)
                total = result.get("total_found", 0)
                self.log_test("藥物搜尋", found, f"搜尋 '{medicine_name}' 找到 {total} 種")
                return found
            else:
                self.log_test("藥物搜尋", False, f"狀態碼: {response.status_code}")
                return False
        except Exception as e:
            self.log_test("藥物搜尋", False, str(e))
            return False

    def test_prescription_creation(self) -> int:
        """測試處方籤創建"""
        try:
            prescription_data = {
                "patient_name": "系統測試病患",
                "patient_id": "TEST001",
                "doctor_name": "測試醫師",
                "diagnosis": "系統測試",
                "medicines": [
                    ["系統測試藥物", "500mg", "5", "每日三次"]
                ]
            }
            
            response = self.session.post(
                f"{self.base_url}/api/prescription/",
                headers={"Content-Type": "application/json"},
                json=prescription_data
            )
            
            if response.status_code == 200:
                result = response.json()
                prescription_id = result.get("id")
                self.log_test("處方籤創建", True, f"創建成功，ID: {prescription_id}")
                return prescription_id
            else:
                self.log_test("處方籤創建", False, f"狀態碼: {response.status_code}")
                return None
        except Exception as e:
            self.log_test("處方籤創建", False, str(e))
            return None

    def test_prescription_status_update(self, prescription_id: int) -> bool:
        """測試處方籤狀態更新"""
        try:
            status_data = {
                "status": "processing",
                "updated_by": "系統測試"
            }
            
            response = self.session.put(
                f"{self.base_url}/api/prescription/{prescription_id}/status",
                headers={"Content-Type": "application/json"},
                json=status_data
            )
            
            if response.status_code == 200:
                self.log_test("處方籤狀態更新", True, "狀態更新為 processing")
                return True
            else:
                self.log_test("處方籤狀態更新", False, f"狀態碼: {response.status_code}")
                return False
        except Exception as e:
            self.log_test("處方籤狀態更新", False, str(e))
            return False

    def test_stock_adjustment(self, medicine_name: str) -> bool:
        """測試庫存調整功能"""
        try:
            adjustment_data = {
                "medicine_name": medicine_name,
                "action": "add",
                "amount": 50
            }
            
            response = self.session.post(
                f"{self.base_url}/api/medicine/adjust-stock",
                headers={"Content-Type": "application/json"},
                json=adjustment_data
            )
            
            if response.status_code == 200:
                result = response.json()
                new_amount = result.get("new_amount")
                self.log_test("庫存調整", True, f"庫存調整成功，新數量: {new_amount}")
                return True
            else:
                self.log_test("庫存調整", False, f"狀態碼: {response.status_code}")
                return False
        except Exception as e:
            self.log_test("庫存調整", False, str(e))
            return False

    def test_ros2_pending_orders(self) -> bool:
        """測試 ROS2 待處理訂單查詢"""
        try:
            response = self.session.get(f"{self.base_url}/api/ros2/pending-orders")
            if response.status_code == 200:
                result = response.json()
                total = result.get("total_pending", 0)
                self.log_test("ROS2 待處理訂單", True, f"找到 {total} 個待處理訂單")
                return True
            else:
                self.log_test("ROS2 待處理訂單", False, f"狀態碼: {response.status_code}")
                return False
        except Exception as e:
            self.log_test("ROS2 待處理訂單", False, str(e))
            return False

    def test_ros2_medicine_query(self, medicine_name: str) -> bool:
        """測試 ROS2 藥物查詢"""
        try:
            query_data = {
                "medicine_name": medicine_name,
                "include_stock": True,
                "include_detailed": True
            }
            
            response = self.session.post(
                f"{self.base_url}/api/ros2/query-medicine",
                headers={"Content-Type": "application/json"},
                json=query_data
            )
            
            if response.status_code == 200:
                result = response.json()
                found = result.get("found", False)
                self.log_test("ROS2 藥物查詢", found, f"查詢 '{medicine_name}' 結果: {found}")
                return found
            else:
                self.log_test("ROS2 藥物查詢", False, f"狀態碼: {response.status_code}")
                return False
        except Exception as e:
            self.log_test("ROS2 藥物查詢", False, str(e))
            return False

    def test_ros2_batch_query(self) -> bool:
        """測試 ROS2 批量查詢"""
        try:
            query_data = {
                "medicines": [
                    {"name": "系統測試藥物"},
                    {"id": 1}
                ],
                "include_stock": True,
                "include_detailed": False
            }
            
            response = self.session.post(
                f"{self.base_url}/api/ros2/batch-query-medicines",
                headers={"Content-Type": "application/json"},
                json=query_data
            )
            
            if response.status_code == 200:
                result = response.json()
                total = result.get("total_queries", 0)
                self.log_test("ROS2 批量查詢", True, f"批量查詢 {total} 個藥物")
                return True
            else:
                self.log_test("ROS2 批量查詢", False, f"狀態碼: {response.status_code}")
                return False
        except Exception as e:
            self.log_test("ROS2 批量查詢", False, str(e))
            return False

    def test_prescription_workflow(self, prescription_id: int) -> bool:
        """測試完整的處方籤工作流程"""
        try:
            # 1. 請求訂單確認
            confirm_data = {
                "prescription_id": prescription_id,
                "requester_id": "test_system"
            }
            
            response = self.session.post(
                f"{self.base_url}/api/ros2/request-order-confirmation",
                headers={"Content-Type": "application/json"},
                json=confirm_data
            )
            
            if response.status_code != 200:
                self.log_test("處方籤工作流程", False, "訂單確認失敗")
                return False
            
            # 2. 確認並執行訂單
            execute_data = {
                "prescription_id": prescription_id,
                "confirmed": True,
                "requester_id": "test_system"
            }
            
            response = self.session.post(
                f"{self.base_url}/api/ros2/confirm-and-execute-order",
                headers={"Content-Type": "application/json"},
                json=execute_data
            )
            
            if response.status_code != 200:
                self.log_test("處方籤工作流程", False, "訂單執行失敗")
                return False
            
            # 3. 標記完成
            complete_data = {
                "prescription_id": prescription_id,
                "notes": "系統測試完成"
            }
            
            response = self.session.post(
                f"{self.base_url}/api/ros2/complete-order",
                headers={"Content-Type": "application/json"},
                json=complete_data
            )
            
            if response.status_code == 200:
                self.log_test("處方籤工作流程", True, "完整工作流程執行成功")
                return True
            else:
                self.log_test("處方籤工作流程", False, "訂單完成失敗")
                return False
                
        except Exception as e:
            self.log_test("處方籤工作流程", False, str(e))
            return False

    def run_all_tests(self) -> Dict:
        """執行所有測試"""
        print("🏥 開始執行完整系統測試")
        print("=" * 60)
        
        # 1. 基礎連接測試
        print("\n📡 基礎連接測試")
        print("-" * 30)
        if not self.test_server_connection():
            print("❌ 伺服器連接失敗，中止測試")
            return self.get_summary()
        
        # 2. API 功能測試
        print("\n💊 藥物 API 測試")
        print("-" * 30)
        self.test_basic_medicine_api()
        self.test_detailed_medicine_api()
        
        # 3. 藥物管理測試
        print("\n🔬 藥物管理測試")
        print("-" * 30)
        medicine_id = self.test_medicine_creation()
        if medicine_id:
            self.test_medicine_search("系統測試藥物")
            self.test_stock_adjustment("系統測試藥物")
        
        # 4. 處方籤管理測試
        print("\n📋 處方籤管理測試")
        print("-" * 30)
        prescription_id = self.test_prescription_creation()
        if prescription_id:
            self.test_prescription_status_update(prescription_id)
        
        # 5. ROS2 功能測試
        print("\n🤖 ROS2 功能測試")
        print("-" * 30)
        self.test_ros2_pending_orders()
        self.test_ros2_medicine_query("系統測試藥物")
        self.test_ros2_batch_query()
        
        # 6. 完整工作流程測試
        if prescription_id:
            print("\n🔄 工作流程測試")
            print("-" * 30)
            self.test_prescription_workflow(prescription_id)
        
        print("\n" + "=" * 60)
        return self.get_summary()

    def get_summary(self) -> Dict:
        """獲取測試摘要"""
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results if result["success"])
        failed_tests = total_tests - passed_tests
        
        summary = {
            "total_tests": total_tests,
            "passed": passed_tests,
            "failed": failed_tests,
            "success_rate": (passed_tests / total_tests * 100) if total_tests > 0 else 0,
            "results": self.test_results
        }
        
        print(f"📊 測試摘要")
        print(f"總測試數: {total_tests}")
        print(f"通過: {passed_tests}")
        print(f"失敗: {failed_tests}")
        print(f"成功率: {summary['success_rate']:.1f}%")
        
        if failed_tests > 0:
            print("\n❌ 失敗的測試:")
            for result in self.test_results:
                if not result["success"]:
                    print(f"  - {result['test']}: {result['message']}")
        else:
            print("\n🎉 所有測試通過!")
        
        return summary

def main():
    """主函數"""
    tester = HospitalSystemTester()
    summary = tester.run_all_tests()
    
    # 如果有失敗的測試，返回非零退出碼
    if summary["failed"] > 0:
        sys.exit(1)
    else:
        sys.exit(0)

if __name__ == "__main__":
    main()
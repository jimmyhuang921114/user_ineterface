#!/usr/bin/env python3
"""
Clean System Test Script
完整系統測試腳本 - 無 emoji 版本
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
        status = "PASS" if success else "FAIL"
        print(f"[{status}] {test_name}: {message}")
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

    def test_ros2_functionality(self) -> bool:
        """測試 ROS2 功能"""
        test_results = []
        
        # 測試 ROS2 狀態
        try:
            response = self.session.get(f"{self.base_url}/api/ros2/status")
            if response.status_code == 200:
                self.log_test("ROS2 狀態查詢", True, "狀態查詢成功")
                test_results.append(True)
            else:
                self.log_test("ROS2 狀態查詢", False, f"狀態碼: {response.status_code}")
                test_results.append(False)
        except Exception as e:
            self.log_test("ROS2 狀態查詢", False, str(e))
            test_results.append(False)
        
        # 測試待處理訂單查詢
        try:
            response = self.session.get(f"{self.base_url}/api/ros2/pending-orders")
            if response.status_code == 200:
                result = response.json()
                total = result.get("total_pending", 0)
                self.log_test("ROS2 待處理訂單", True, f"找到 {total} 個待處理訂單")
                test_results.append(True)
            else:
                self.log_test("ROS2 待處理訂單", False, f"狀態碼: {response.status_code}")
                test_results.append(False)
        except Exception as e:
            self.log_test("ROS2 待處理訂單", False, str(e))
            test_results.append(False)
        
        # 測試藥物查詢
        try:
            query_data = {
                "medicine_name": "系統測試藥物",
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
                self.log_test("ROS2 藥物查詢", found, f"查詢結果: {found}")
                test_results.append(found)
            else:
                self.log_test("ROS2 藥物查詢", False, f"狀態碼: {response.status_code}")
                test_results.append(False)
        except Exception as e:
            self.log_test("ROS2 藥物查詢", False, str(e))
            test_results.append(False)
        
        return all(test_results)

    def run_all_tests(self) -> Dict:
        """執行所有測試"""
        print("醫院藥物管理系統 - 完整測試")
        print("=" * 60)
        
        # 基礎連接測試
        print("\n基礎連接測試")
        print("-" * 30)
        if not self.test_server_connection():
            print("伺服器連接失敗，中止測試")
            return self.get_summary()
        
        # API 功能測試
        print("\n藥物 API 測試")
        print("-" * 30)
        self.test_basic_medicine_api()
        self.test_detailed_medicine_api()
        
        # 藥物管理測試
        print("\n藥物管理測試")
        print("-" * 30)
        medicine_id = self.test_medicine_creation()
        
        # ROS2 功能測試
        print("\nROS2 功能測試")
        print("-" * 30)
        self.test_ros2_functionality()
        
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
        
        print(f"測試摘要")
        print(f"總測試數: {total_tests}")
        print(f"通過: {passed_tests}")
        print(f"失敗: {failed_tests}")
        print(f"成功率: {summary['success_rate']:.1f}%")
        
        if failed_tests > 0:
            print("\n失敗的測試:")
            for result in self.test_results:
                if not result["success"]:
                    print(f"  - {result['test']}: {result['message']}")
        else:
            print("\n所有測試通過!")
        
        return summary

def main():
    """主函數"""
    tester = HospitalSystemTester()
    summary = tester.run_all_tests()
    
    if summary["failed"] > 0:
        sys.exit(1)
    else:
        sys.exit(0)

if __name__ == "__main__":
    main()
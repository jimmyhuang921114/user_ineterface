#!/usr/bin/env python3
"""
ROS2 Services Test Script
ROS2 服務測試腳本 - 測試 ROS2 服務功能
"""

import requests
import json
import sys
import time
from typing import Dict, List

class ROS2ServiceTester:
    def __init__(self, base_url: str = "http://localhost:8001"):
        self.base_url = base_url
        self.session = requests.Session()
        
    def test_ros2_status(self) -> bool:
        """測試 ROS2 狀態"""
        print("🤖 測試 ROS2 狀態...")
        try:
            response = self.session.get(f"{self.base_url}/api/ros2/status")
            if response.status_code == 200:
                result = response.json()
                print(f"✅ ROS2 狀態: {result}")
                return True
            else:
                print(f"❌ ROS2 狀態查詢失敗: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ ROS2 狀態查詢錯誤: {e}")
            return False

    def test_basic_medicine_service(self) -> bool:
        """測試基本藥物服務模擬"""
        print("\n💊 測試基本藥物查詢服務...")
        
        test_cases = [
            # 獲取所有藥物
            {
                "name": "獲取所有基本藥物",
                "method": "GET",
                "url": f"{self.base_url}/api/medicine/basic",
                "expected_keys": ["id", "name", "amount"]
            },
            # 按名稱搜尋
            {
                "name": "按名稱搜尋藥物",
                "method": "GET", 
                "url": f"{self.base_url}/api/medicine/search/普拿疼",
                "expected_keys": ["found", "medicines"]
            },
            # ROS2 查詢介面
            {
                "name": "ROS2 藥物查詢",
                "method": "POST",
                "url": f"{self.base_url}/api/ros2/query-medicine",
                "data": {
                    "medicine_name": "普拿疼",
                    "include_stock": True,
                    "include_detailed": False
                },
                "expected_keys": ["found", "medicine"]
            }
        ]
        
        success_count = 0
        for test in test_cases:
            try:
                if test["method"] == "GET":
                    response = self.session.get(test["url"])
                else:
                    response = self.session.post(
                        test["url"],
                        headers={"Content-Type": "application/json"},
                        json=test["data"]
                    )
                
                if response.status_code == 200:
                    result = response.json()
                    # 檢查預期的鍵值
                    has_keys = all(key in result for key in test["expected_keys"])
                    if has_keys:
                        print(f"  ✅ {test['name']}: 成功")
                        success_count += 1
                    else:
                        print(f"  ❌ {test['name']}: 回應格式不正確")
                else:
                    print(f"  ❌ {test['name']}: HTTP {response.status_code}")
                    
            except Exception as e:
                print(f"  ❌ {test['name']}: {e}")
        
        print(f"\n基本藥物服務測試結果: {success_count}/{len(test_cases)} 通過")
        return success_count == len(test_cases)

    def test_detailed_medicine_service(self) -> bool:
        """測試詳細藥物服務模擬"""
        print("\n🔬 測試詳細藥物查詢服務...")
        
        test_cases = [
            # 獲取所有詳細藥物
            {
                "name": "獲取所有詳細藥物",
                "method": "GET",
                "url": f"{self.base_url}/api/medicine/detailed",
                "expected_keys": ["basic_info", "detailed_info"]
            },
            # ROS2 詳細查詢
            {
                "name": "ROS2 詳細藥物查詢",
                "method": "POST",
                "url": f"{self.base_url}/api/ros2/query-medicine",
                "data": {
                    "medicine_name": "普拿疼",
                    "include_stock": True,
                    "include_detailed": True
                },
                "expected_keys": ["found", "medicine"]
            },
            # 批量查詢
            {
                "name": "ROS2 批量藥物查詢",
                "method": "POST",
                "url": f"{self.base_url}/api/ros2/batch-query-medicines",
                "data": {
                    "medicines": [
                        {"name": "普拿疼"},
                        {"id": 1}
                    ],
                    "include_stock": True,
                    "include_detailed": True
                },
                "expected_keys": ["total_queries", "results"]
            }
        ]
        
        success_count = 0
        for test in test_cases:
            try:
                if test["method"] == "GET":
                    response = self.session.get(test["url"])
                else:
                    response = self.session.post(
                        test["url"],
                        headers={"Content-Type": "application/json"},
                        json=test["data"]
                    )
                
                if response.status_code == 200:
                    result = response.json()
                    
                    # 特殊處理批量查詢
                    if "batch" in test["name"].lower():
                        has_keys = all(key in result for key in test["expected_keys"])
                        if has_keys and len(result.get("results", [])) > 0:
                            print(f"  ✅ {test['name']}: 成功 (查詢 {result['total_queries']} 個)")
                            success_count += 1
                        else:
                            print(f"  ❌ {test['name']}: 回應格式不正確或無結果")
                    
                    # 一般檢查
                    elif test["url"].endswith("/detailed"):
                        if isinstance(result, list) and len(result) > 0:
                            has_structure = all(key in result[0] for key in test["expected_keys"]) if result else False
                            if has_structure:
                                print(f"  ✅ {test['name']}: 成功 (獲取 {len(result)} 種)")
                                success_count += 1
                            else:
                                print(f"  ❌ {test['name']}: 回應結構不正確")
                        else:
                            print(f"  ❌ {test['name']}: 無數據")
                    
                    else:
                        has_keys = all(key in result for key in test["expected_keys"])
                        if has_keys:
                            print(f"  ✅ {test['name']}: 成功")
                            success_count += 1
                        else:
                            print(f"  ❌ {test['name']}: 回應格式不正確")
                else:
                    print(f"  ❌ {test['name']}: HTTP {response.status_code}")
                    
            except Exception as e:
                print(f"  ❌ {test['name']}: {e}")
        
        print(f"\n詳細藥物服務測試結果: {success_count}/{len(test_cases)} 通過")
        return success_count == len(test_cases)

    def test_ros2_workflow_services(self) -> bool:
        """測試 ROS2 工作流程服務"""
        print("\n🔄 測試 ROS2 工作流程服務...")
        
        # 1. 創建測試處方籤
        print("  📋 創建測試處方籤...")
        prescription_data = {
            "patient_name": "ROS2測試病患",
            "patient_id": "ROS2001",
            "doctor_name": "ROS2測試醫師",
            "diagnosis": "ROS2服務測試",
            "medicines": [
                ["普拿疼", "500mg", "3", "每日三次"]
            ]
        }
        
        try:
            response = self.session.post(
                f"{self.base_url}/api/prescription/",
                headers={"Content-Type": "application/json"},
                json=prescription_data
            )
            
            if response.status_code != 200:
                print("  ❌ 創建測試處方籤失敗")
                return False
            
            prescription_id = response.json().get("id")
            print(f"  ✅ 創建測試處方籤成功 (ID: {prescription_id})")
            
        except Exception as e:
            print(f"  ❌ 創建處方籤錯誤: {e}")
            return False
        
        # 2. 測試 ROS2 工作流程 API
        workflow_tests = [
            {
                "name": "查詢待處理訂單",
                "method": "GET",
                "url": f"{self.base_url}/api/ros2/pending-orders"
            },
            {
                "name": "請求訂單確認",
                "method": "POST",
                "url": f"{self.base_url}/api/ros2/request-order-confirmation",
                "data": {
                    "prescription_id": prescription_id,
                    "requester_id": "ros2_test_service"
                }
            },
            {
                "name": "確認並執行訂單",
                "method": "POST",
                "url": f"{self.base_url}/api/ros2/confirm-and-execute-order",
                "data": {
                    "prescription_id": prescription_id,
                    "confirmed": True,
                    "requester_id": "ros2_test_service"
                }
            },
            {
                "name": "標記訂單完成",
                "method": "POST",
                "url": f"{self.base_url}/api/ros2/complete-order",
                "data": {
                    "prescription_id": prescription_id,
                    "notes": "ROS2 服務測試完成"
                }
            }
        ]
        
        success_count = 0
        for test in workflow_tests:
            try:
                if test["method"] == "GET":
                    response = self.session.get(test["url"])
                else:
                    response = self.session.post(
                        test["url"],
                        headers={"Content-Type": "application/json"},
                        json=test["data"]
                    )
                
                if response.status_code == 200:
                    result = response.json()
                    if result.get("success", True):  # 某些 API 沒有 success 字段
                        print(f"  ✅ {test['name']}: 成功")
                        success_count += 1
                    else:
                        print(f"  ❌ {test['name']}: {result.get('message', '未知錯誤')}")
                else:
                    print(f"  ❌ {test['name']}: HTTP {response.status_code}")
                    
            except Exception as e:
                print(f"  ❌ {test['name']}: {e}")
        
        print(f"\nROS2 工作流程服務測試結果: {success_count}/{len(workflow_tests)} 通過")
        return success_count == len(workflow_tests)

    def run_comprehensive_test(self):
        """執行綜合測試"""
        print("🚀 開始 ROS2 服務綜合測試")
        print("=" * 60)
        
        # 檢查伺服器連接
        print("📡 檢查伺服器連接...")
        try:
            response = self.session.get(f"{self.base_url}/api/system/status", timeout=5)
            if response.status_code != 200:
                print("❌ 伺服器連接失敗，中止測試")
                return False
            print("✅ 伺服器連接正常")
        except Exception as e:
            print(f"❌ 伺服器連接錯誤: {e}")
            return False
        
        # 執行各項測試
        test_results = []
        
        test_results.append(self.test_ros2_status())
        test_results.append(self.test_basic_medicine_service())
        test_results.append(self.test_detailed_medicine_service())
        test_results.append(self.test_ros2_workflow_services())
        
        # 統計結果
        passed = sum(test_results)
        total = len(test_results)
        
        print("\n" + "=" * 60)
        print("📊 ROS2 服務測試摘要")
        print(f"總測試項目: {total}")
        print(f"通過項目: {passed}")
        print(f"失敗項目: {total - passed}")
        print(f"成功率: {(passed/total*100):.1f}%")
        
        if passed == total:
            print("\n🎉 所有 ROS2 服務測試通過!")
            print("✅ 系統已準備好接入真實 ROS2 節點")
            return True
        else:
            print(f"\n{total - passed} 個測試項目失敗")
            print("❌ 請檢查系統配置")
            return False

def main():
    """主函數"""
    print("🤖 ROS2 服務測試工具")
    print("此工具測試系統的 ROS2 相關 API 功能")
    print("確保這些 API 可以被真實的 ROS2 節點調用")
    print("-" * 50)
    
    tester = ROS2ServiceTester()
    success = tester.run_comprehensive_test()
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
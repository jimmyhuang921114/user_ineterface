#!/usr/bin/env python3
"""
Medicine Detail Service Node
Provides ROS2 service for querying medicine detailed information

Service:
# 您的用法：
"""

import os
import json
import yaml
import rclpy
from typing import Dict, Any, List, Optional
from rclpy.node import Node
from tm_robot_if.srv import MedicineDetail  # Request.name -> Response.detail (YAML)

# Optional: HTTP backend query
try:
    import requests
except Exception:
    requests = None


class MedicineDetailServiceNode(Node):
    """
# 藥物詳細資訊查詢服務節點
    """

    def __init__(self):
        super().__init__('medicine_detail_service')
# # HTTP 設定 =====
        self.base_url = os.getenv('MEDICINE_BASE_URL', 'http://...:')
        self.detail_endpoint = '/api/medicine/detailed'
        self.http_timeout = float(os.getenv('MEDICINE_HTTP_TIMEOUT', ''))
        self.auth_token = os.getenv('MEDICINE_HTTP_TOKEN', '')

        if requests is None:
# self.get_logger().warn("requests 不可用：將使用模擬資料。")

         # ROS2 Services
        self.detail_srv = self.create_service(
            MedicineDetail, '/hospital/get_medicine_detail', self._get_medicine_detail_cb
        )
        
        self.all_medicines_srv = self.create_service(
            MedicineDetail, '/hospital/get_all_medicines', self._get_all_medicines_cb
        )
        
        self.search_srv = self.create_service(
            MedicineDetail, '/hospital/search_medicines', self._search_medicines_cb
        )
# # 快取 =====
        self._medicine_cache = {}
        self._cache_timestamp = 
# self._cache_timeout =     秒快取

        self.get_logger().info(" MedicineDetailServiceNode ready.")
# self.get_logger().info(" 可用服務:")
        self.get_logger().info("   /hospital/get_medicine_detail")
        self.get_logger().info("   /hospital/get_all_medicines")
        self.get_logger().info("   /hospital/search_medicines")

     ================# Service Callbacks =====================
    def _get_medicine_detail_cb(self, request: MedicineDetail.Request, response: MedicineDetail.Response):
# """查詢單個藥物詳細資訊"""
        medicine_name = request.name.strip()
        
        if not medicine_name:
            response.success = False
# response.detail = "藥物名稱不能為空"
            return response

        try:
# 獲取所有藥物資料
            all_medicines = self._fetch_all_medicines()
# 尋找匹配的藥物
            found_medicine = self._find_medicine_by_name(all_medicines, medicine_name)
            
            if found_medicine:
# 格式化為 YAML
                detail_yaml = self._format_medicine_detail_yaml(found_medicine, medicine_name)
                response.success = True
                response.detail = detail_yaml
# self.get_logger().info(f" 找到藥物: {medicine_name}")
            else:
# 沒找到，回傳可用藥物列表
                available_list = self._get_available_medicines_list(all_medicines)
                not_found_yaml = self._format_not_found_yaml(medicine_name, available_list)
                response.success = False
                response.detail = not_found_yaml
# self.get_logger().warn(f" 找不到藥物: {medicine_name}")

        except Exception as e:
# error_msg = f"查詢藥物時發生錯誤: {e}"
            self.get_logger().error(error_msg)
            response.success = False
            response.detail = self._format_error_yaml(medicine_name, error_msg)

        return response

    def _get_all_medicines_cb(self, request: MedicineDetail.Request, response: MedicineDetail.Response):
# """獲取所有可用藥物列表"""
        try:
            all_medicines = self._fetch_all_medicines()
            medicines_yaml = self._format_all_medicines_yaml(all_medicines)
            
            response.success = True
            response.detail = medicines_yaml
# self.get_logger().info(f" 返回 {len(all_medicines)} 種藥物清單")

        except Exception as e:
# error_msg = f"獲取藥物清單時發生錯誤: {e}"
            self.get_logger().error(error_msg)
            response.success = False
            response.detail = self._format_error_yaml("all_medicines", error_msg)

        return response

    def _search_medicines_cb(self, request: MedicineDetail.Request, response: MedicineDetail.Response):
# """搜尋藥物 (模糊匹配)"""
        search_term = request.name.strip().lower()
        
        if not search_term:
            response.success = False
# response.detail = "搜尋關鍵字不能為空"
            return response

        try:
            all_medicines = self._fetch_all_medicines()
            matched_medicines = self._search_medicines_fuzzy(all_medicines, search_term)
            
            search_yaml = self._format_search_results_yaml(search_term, matched_medicines)
            response.success = True
            response.detail = search_yaml
# self.get_logger().info(f" 搜尋 '{search_term}' 找到 {len(matched_medicines)} 個結果")

        except Exception as e:
# error_msg = f"搜尋藥物時發生錯誤: {e}"
            self.get_logger().error(error_msg)
            response.success = False
            response.detail = self._format_error_yaml(search_term, error_msg)

        return response

     ================# Data Fetching =====================
    def _fetch_all_medicines(self) -> List[Dict[str, Any]]:
# """獲取所有藥物資料，支援快取"""
        import time
        current_time = time.time()
# 檢查快取
        if (current_time - self._cache_timestamp) < self._cache_timeout and self._medicine_cache:
            return list(self._medicine_cache.values())
# 從後端獲取
        if requests is None:
            return self._get_demo_medicines()
        
        try:
            url = self.base_url.rstrip('/') + self.detail_endpoint
            headers = self._get_headers()
            
            response = requests.get(url, headers=headers, timeout=self.http_timeout)
            response.raise_for_status()
            
            medicines = response.json()
# 更新快取
            self._medicine_cache = {med.get('name', f"medicine_{i}"): med for i, med in enumerate(medicines)}
            self._cache_timestamp = current_time
# self.get_logger().info(f" 從後端獲取了 {len(medicines)} 種藥物")
            return medicines
            
        except Exception as e:
# self.get_logger().warn(f"從後端獲取藥物失敗，使用演示資料: {e}")
            return self._get_demo_medicines()

    def _get_demo_medicines(self) -> List[Dict[str, Any]]:
# """演示用藥物資料"""
        return [
            {
# "name": "阿斯匹靈",
# "description": "解熱鎮痛藥，用於緩解頭痛、發燒等症狀",
# "category": "解熱鎮痛藥",
                "unit_dose": "mg",
                "stock_quantity": ,
# "manufacturer": "示範藥廠",
                "expiry_date": "--"
            },
            {
# "name": "維他命C",
# "description": "維生素C補充劑，增強免疫力",
# "category": "維生素",
                "unit_dose": "mg",
                "stock_quantity": ,
# "manufacturer": "健康藥廠",
                "expiry_date": "--"
            },
            {
# "name": "感冒膠囊",
# "description": "綜合感冒症狀緩解藥物",
# "category": "感冒藥",
                "unit_dose": " capsule",
                "stock_quantity": ,
# "manufacturer": "康復藥廠",
                "expiry_date": "--"
            }
        ]

     ================# Search & Match =====================
    def _find_medicine_by_name(self, medicines: List[Dict[str, Any]], name: str) -> Optional[Dict[str, Any]]:
# """根據名稱查找藥物 (精確匹配優先，然後模糊匹配)"""
        name_lower = name.lower().strip()
# 精確匹配
        for med in medicines:
            if med.get('name', '').lower().strip() == name_lower:
                return med
# 模糊匹配
        for med in medicines:
            med_name = med.get('name', '').lower()
            if name_lower in med_name or med_name in name_lower:
                return med
        
        return None

    def _search_medicines_fuzzy(self, medicines: List[Dict[str, Any]], search_term: str) -> List[Dict[str, Any]]:
# """模糊搜尋藥物"""
        matched = []
        search_lower = search_term.lower()
        
        for med in medicines:
            name = med.get('name', '').lower()
            description = med.get('description', '').lower()
            category = med.get('category', '').lower()
# 檢查名稱、描述、分類是否包含搜尋關鍵字
            if (search_lower in name or 
                search_lower in description or 
                search_lower in category):
                matched.append(med)
        
        return matched

    def _get_available_medicines_list(self, medicines: List[Dict[str, Any]]) -> List[str]:
# """獲取可用藥物名稱列表"""
        return [med.get('name', 'Unknown') for med in medicines]

     ================# YAML Formatting =====================
    def _format_medicine_detail_yaml(self, medicine: Dict[str, Any], query_name: str) -> str:
# """格式化單個藥物詳細資訊為 YAML"""
        detail = {
            "query": query_name,
            "found": True,
            "medicine": {
                "name": medicine.get('name', 'Unknown'),
                "description": medicine.get('description', 'No description available'),
                "category": medicine.get('category', 'Unknown'),
                "unit_dose": medicine.get('unit_dose', 'Unknown'),
                "stock_quantity": medicine.get('stock_quantity', ),
                "manufacturer": medicine.get('manufacturer', 'Unknown'),
                "expiry_date": medicine.get('expiry_date', 'Unknown')
            },
            "timestamp": self._get_timestamp()
        }
        return yaml.safe_dump(detail, allow_unicode=True, default_flow_style=False)

    def _format_not_found_yaml(self, query_name: str, available_medicines: List[str]) -> str:
# """格式化找不到藥物的回應"""
        result = {
            "query": query_name,
            "found": False,
# "message": f"找不到藥物: {query_name}",
            "available_medicines": available_medicines,
            "total_available": len(available_medicines),
# "suggestion": "請檢查藥物名稱是否正確，或使用 /hospital/search_medicines 進行模糊搜尋",
            "timestamp": self._get_timestamp()
        }
        return yaml.safe_dump(result, allow_unicode=True, default_flow_style=False)

    def _format_all_medicines_yaml(self, medicines: List[Dict[str, Any]]) -> str:
# """格式化所有藥物清單為 YAML"""
        result = {
            "total_medicines": len(medicines),
            "medicines": []
        }
        
        for med in medicines:
            med_info = {
                "name": med.get('name', 'Unknown'),
                "category": med.get('category', 'Unknown'),
                "stock_quantity": med.get('stock_quantity', ),
                "unit_dose": med.get('unit_dose', 'Unknown')
            }
            result["medicines"].append(med_info)
        
        result["timestamp"] = self._get_timestamp()
        return yaml.safe_dump(result, allow_unicode=True, default_flow_style=False)

    def _format_search_results_yaml(self, search_term: str, matched_medicines: List[Dict[str, Any]]) -> str:
# """格式化搜尋結果為 YAML"""
        result = {
            "search_term": search_term,
            "total_results": len(matched_medicines),
            "results": []
        }
        
        for med in matched_medicines:
            med_info = {
                "name": med.get('name', 'Unknown'),
                "description": med.get('description', 'No description'),
                "category": med.get('category', 'Unknown'),
                "stock_quantity": med.get('stock_quantity', )
            }
            result["results"].append(med_info)
        
        result["timestamp"] = self._get_timestamp()
        return yaml.safe_dump(result, allow_unicode=True, default_flow_style=False)

    def _format_error_yaml(self, query: str, error_message: str) -> str:
# """格式化錯誤訊息為 YAML"""
        error = {
            "query": query,
            "error": True,
            "message": error_message,
            "timestamp": self._get_timestamp()
        }
        return yaml.safe_dump(error, allow_unicode=True, default_flow_style=False)

     ================# Utils =====================
    def _get_headers(self) -> Dict[str, str]:
# """獲取 HTTP 請求標頭"""
        headers = {"Content-Type": "application/json"}
        if self.auth_token:
            headers["Authorization"] = f"Bearer {self.auth_token}"
        return headers

    def _get_timestamp(self) -> str:
# """獲取當前時間戳"""
        import datetime
        return datetime.datetime.now().isoformat()


def main(args=None):
    rclpy.init(args=args)
    node = MedicineDetailServiceNode()
    
    try:
# print(" Medicine Detail Service Node 已啟動")
# print(" 可用服務:")
# print("   ros service call /hospital/get_medicine_detail tm_robot_if/srv/MedicineDetail \"{name: '阿斯匹靈'}\"")
        print("   ros service call /hospital/get_all_medicines tm_robot_if/srv/MedicineDetail \"{name: ''}\"")
# print("   ros service call /hospital/search_medicines tm_robot_if/srv/MedicineDetail \"{name: '感冒'}\"")
# print(" 按 Ctrl+C 停止")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
# print("\n Medicine Detail Service Node 停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
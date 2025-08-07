#!/usr/bin/env python3
"""
ROS2 Medicine Service Node
ROS2 藥物服務節點 - 提供基本和詳細藥物資訊查詢服務
"""

import rclpy
from rclpy.node import Node
import requests
import json
from typing import List, Dict, Any

# 假設您的消息和服務定義在 hospital_medicine_msgs 包中
# from hospital_medicine_msgs.msg import MedicineBasicInfo, MedicineDetailedInfo
# from hospital_medicine_msgs.srv import GetBasicMedicine, GetDetailedMedicine

# 由於目前沒有編譯的消息，我們先用字典模擬
class MedicineBasicInfo:
    def __init__(self, **kwargs):
        self.id = kwargs.get('id', 0)
        self.name = kwargs.get('name', '')
        self.amount = kwargs.get('amount', 0)
        self.position = kwargs.get('position', '')
        self.manufacturer = kwargs.get('manufacturer', '')
        self.dosage = kwargs.get('dosage', '')
        self.is_active = kwargs.get('is_active', True)
        self.created_at = kwargs.get('created_at', '')
        self.updated_at = kwargs.get('updated_at', '')

class MedicineDetailedInfo:
    def __init__(self, **kwargs):
        self.id = kwargs.get('id', 0)
        self.medicine_id = kwargs.get('medicine_id', 0)
        self.description = kwargs.get('description', '')
        self.ingredient = kwargs.get('ingredient', '')
        self.category = kwargs.get('category', '')
        self.usage_method = kwargs.get('usage_method', '')
        self.unit_dose = kwargs.get('unit_dose', 0.0)
        self.side_effects = kwargs.get('side_effects', '')
        self.storage_conditions = kwargs.get('storage_conditions', '')
        self.expiry_date = kwargs.get('expiry_date', '')
        self.barcode = kwargs.get('barcode', '')
        self.appearance_type = kwargs.get('appearance_type', '')
        self.notes = kwargs.get('notes', '')

class MedicineServiceNode(Node):
    def __init__(self):
        super().__init__('medicine_service_node')
        
        # API 基礎 URL
        self.api_base = "http://localhost:8001"
        
        # 創建服務
        self.basic_medicine_service = self.create_service(
            # GetBasicMedicine,
            dict,  # 臨時使用 dict
            'get_basic_medicine',
            self.get_basic_medicine_callback
        )
        
        self.detailed_medicine_service = self.create_service(
            # GetDetailedMedicine,
            dict,  # 臨時使用 dict
            'get_detailed_medicine', 
            self.get_detailed_medicine_callback
        )
        
        self.get_logger().info('藥物服務節點已啟動')
        self.get_logger().info('提供的服務:')
        self.get_logger().info('  - get_basic_medicine: 獲取基本藥物資訊')
        self.get_logger().info('  - get_detailed_medicine: 獲取詳細藥物資訊')

    def get_basic_medicine_callback(self, request, response):
        """處理基本藥物資訊請求"""
        self.get_logger().info('收到基本藥物資訊請求')
        
        try:
            # 解析請求參數
            medicine_name = getattr(request, 'medicine_name', '')
            medicine_id = getattr(request, 'medicine_id', 0)
            get_all = getattr(request, 'get_all', False)
            
            medicines = []
            
            if get_all:
                # 獲取所有基本藥物
                api_url = f"{self.api_base}/api/medicine/basic"
                response_data = self._make_api_request(api_url)
                
                if response_data:
                    for med_data in response_data:
                        medicine = self._create_basic_medicine_info(med_data)
                        medicines.append(medicine)
                        
            elif medicine_name:
                # 按名稱搜尋
                api_url = f"{self.api_base}/api/medicine/search/{medicine_name}"
                response_data = self._make_api_request(api_url)
                
                if response_data and response_data.get('found'):
                    for med_data in response_data.get('medicines', []):
                        basic_info = med_data.get('basic_info', {})
                        medicine = self._create_basic_medicine_info(basic_info)
                        medicines.append(medicine)
                        
            elif medicine_id > 0:
                # 按 ID 查詢 (通過 ROS2 查詢 API)
                api_url = f"{self.api_base}/api/ros2/query-medicine"
                post_data = {
                    "medicine_id": medicine_id,
                    "include_stock": True,
                    "include_detailed": False
                }
                response_data = self._make_api_request(api_url, method='POST', data=post_data)
                
                if response_data and response_data.get('found'):
                    basic_info = response_data.get('medicine', {}).get('basic_info', {})
                    stock_info = response_data.get('medicine', {}).get('stock_info', {})
                    
                    # 合併基本資訊和庫存資訊
                    basic_info.update({
                        'amount': stock_info.get('current_amount', basic_info.get('amount', 0)),
                        'updated_at': stock_info.get('last_updated', basic_info.get('updated_at', ''))
                    })
                    
                    medicine = self._create_basic_medicine_info(basic_info)
                    medicines.append(medicine)
            
            # 設置回應
            response.success = True
            response.message = f"成功獲取 {len(medicines)} 種基本藥物資訊"
            response.medicines = medicines
            
            self.get_logger().info(f'返回 {len(medicines)} 種基本藥物資訊')
            
        except Exception as e:
            self.get_logger().error(f'獲取基本藥物資訊失敗: {str(e)}')
            response.success = False
            response.message = f"獲取失敗: {str(e)}"
            response.medicines = []
        
        return response

    def get_detailed_medicine_callback(self, request, response):
        """處理詳細藥物資訊請求"""
        self.get_logger().info('收到詳細藥物資訊請求')
        
        try:
            # 解析請求參數
            medicine_name = getattr(request, 'medicine_name', '')
            medicine_id = getattr(request, 'medicine_id', 0)
            get_all = getattr(request, 'get_all', False)
            include_basic = getattr(request, 'include_basic', True)
            
            detailed_medicines = []
            basic_medicines = []
            
            if get_all:
                # 獲取所有詳細藥物
                api_url = f"{self.api_base}/api/medicine/detailed"
                response_data = self._make_api_request(api_url)
                
                if response_data:
                    for med_data in response_data:
                        detailed_info = med_data.get('detailed_info', {})
                        basic_info = med_data.get('basic_info', {})
                        
                        detailed_medicine = self._create_detailed_medicine_info(detailed_info)
                        detailed_medicines.append(detailed_medicine)
                        
                        if include_basic:
                            basic_medicine = self._create_basic_medicine_info(basic_info)
                            basic_medicines.append(basic_medicine)
                            
            elif medicine_name:
                # 按名稱搜尋
                api_url = f"{self.api_base}/api/medicine/search/{medicine_name}"
                response_data = self._make_api_request(api_url)
                
                if response_data and response_data.get('found'):
                    for med_data in response_data.get('medicines', []):
                        detailed_info = med_data.get('detailed_info')
                        basic_info = med_data.get('basic_info', {})
                        
                        if detailed_info:
                            detailed_medicine = self._create_detailed_medicine_info(detailed_info)
                            detailed_medicines.append(detailed_medicine)
                        
                        if include_basic:
                            basic_medicine = self._create_basic_medicine_info(basic_info)
                            basic_medicines.append(basic_medicine)
                            
            elif medicine_id > 0:
                # 按 ID 查詢詳細資訊
                api_url = f"{self.api_base}/api/ros2/query-medicine"
                post_data = {
                    "medicine_id": medicine_id,
                    "include_stock": include_basic,
                    "include_detailed": True
                }
                response_data = self._make_api_request(api_url, method='POST', data=post_data)
                
                if response_data and response_data.get('found'):
                    medicine_data = response_data.get('medicine', {})
                    detailed_info = medicine_data.get('detailed_info')
                    
                    if detailed_info:
                        detailed_medicine = self._create_detailed_medicine_info(detailed_info)
                        detailed_medicines.append(detailed_medicine)
                    
                    if include_basic:
                        basic_info = medicine_data.get('basic_info', {})
                        stock_info = medicine_data.get('stock_info', {})
                        
                        # 合併庫存資訊
                        if stock_info:
                            basic_info.update({
                                'amount': stock_info.get('current_amount', basic_info.get('amount', 0)),
                                'updated_at': stock_info.get('last_updated', basic_info.get('updated_at', ''))
                            })
                        
                        basic_medicine = self._create_basic_medicine_info(basic_info)
                        basic_medicines.append(basic_medicine)
            
            # 設置回應
            response.success = True
            response.message = f"成功獲取 {len(detailed_medicines)} 種詳細藥物資訊"
            response.detailed_medicines = detailed_medicines
            response.basic_medicines = basic_medicines if include_basic else []
            
            self.get_logger().info(f'返回 {len(detailed_medicines)} 種詳細藥物資訊')
            if include_basic:
                self.get_logger().info(f'包含 {len(basic_medicines)} 種基本藥物資訊')
            
        except Exception as e:
            self.get_logger().error(f'獲取詳細藥物資訊失敗: {str(e)}')
            response.success = False
            response.message = f"獲取失敗: {str(e)}"
            response.detailed_medicines = []
            response.basic_medicines = []
        
        return response

    def _make_api_request(self, url: str, method: str = 'GET', data: Dict = None) -> Dict[str, Any]:
        """發送 API 請求"""
        try:
            if method == 'GET':
                response = requests.get(url, timeout=10)
            elif method == 'POST':
                response = requests.post(
                    url, 
                    headers={'Content-Type': 'application/json'},
                    json=data,
                    timeout=10
                )
            else:
                raise ValueError(f"不支援的 HTTP 方法: {method}")
            
            response.raise_for_status()
            return response.json()
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'API 請求失敗 ({url}): {str(e)}')
            return None

    def _create_basic_medicine_info(self, data: Dict) -> MedicineBasicInfo:
        """創建基本藥物資訊物件"""
        return MedicineBasicInfo(
            id=data.get('id', 0),
            name=data.get('name', ''),
            amount=data.get('amount', 0),
            position=data.get('position', ''),
            manufacturer=data.get('manufacturer', ''),
            dosage=data.get('dosage', ''),
            is_active=data.get('is_active', True),
            created_at=data.get('created_at', ''),
            updated_at=data.get('updated_at', '')
        )

    def _create_detailed_medicine_info(self, data: Dict) -> MedicineDetailedInfo:
        """創建詳細藥物資訊物件"""
        return MedicineDetailedInfo(
            id=data.get('id', 0),
            medicine_id=data.get('medicine_id', 0),
            description=data.get('description', ''),
            ingredient=data.get('ingredient', ''),
            category=data.get('category', ''),
            usage_method=data.get('usage_method', ''),
            unit_dose=data.get('unit_dose', 0.0),
            side_effects=data.get('side_effects', ''),
            storage_conditions=data.get('storage_conditions', ''),
            expiry_date=data.get('expiry_date', ''),
            barcode=data.get('barcode', ''),
            appearance_type=data.get('appearance_type', ''),
            notes=data.get('notes', '')
        )

def main(args=None):
    rclpy.init(args=args)
    
    medicine_service_node = MedicineServiceNode()
    
    try:
        rclpy.spin(medicine_service_node)
    except KeyboardInterrupt:
        medicine_service_node.get_logger().info('收到停止信號，正在關閉...')
    finally:
        medicine_service_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
ROS2 Medicine Order Service Node
藥物訂單服務節點
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import json
import yaml
import requests
from datetime import datetime
import os
import sys

class MedicineOrderServiceNode(Node):
    def __init__(self):
        super().__init__('medicine_order_service')
        
        # 服務端點
        self.order_service = self.create_service(
            Trigger, 
            'get_medicine_orders', 
            self.get_orders_callback
        )
        
        # 發布者用於輸出結果
        self.order_publisher = self.create_publisher(
            String, 
            'medicine_order_output', 
            10
        )
        
        # HTTP API 基礎URL
        self.api_base_url = "http://localhost:8000/api"
        
        self.get_logger().info('🏥 Medicine Order Service Node Started')
        self.get_logger().info('📋 Service: /get_medicine_orders')
        self.get_logger().info('📤 Publisher: /medicine_order_output')
    
    def get_orders_callback(self, request, response):
        """處理訂單獲取請求"""
        try:
            self.get_logger().info('📨 收到訂單查詢請求')
            
            # 獲取訂單資料
            orders_data = self.fetch_orders_from_api()
            
            if orders_data:
                # 生成YAML輸出
                yaml_output = self.generate_yaml_output(orders_data)
                
                # 發布結果
                msg = String()
                msg.data = yaml_output
                self.order_publisher.publish(msg)
                
                # 儲存到檔案
                self.save_output_to_file(yaml_output)
                
                response.success = True
                response.message = f"✅ 成功獲取 {len(orders_data)} 筆訂單資料"
                
                self.get_logger().info(f'✅ 成功處理訂單查詢，共 {len(orders_data)} 筆')
            else:
                response.success = False
                response.message = "❌ 無法獲取訂單資料"
                self.get_logger().warn('❌ 無法獲取訂單資料')
                
        except Exception as e:
            response.success = False
            response.message = f"❌ 服務錯誤: {str(e)}"
            self.get_logger().error(f'❌ 服務錯誤: {str(e)}')
        
        return response
    
    def fetch_orders_from_api(self):
        """從API獲取訂單資料"""
        try:
            # 獲取處方籤資料 (作為訂單)
            response = requests.get(f"{self.api_base_url}/prescription/", timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                return data.get('prescriptions', [])
            else:
                self.get_logger().warn(f'API回應錯誤: {response.status_code}')
                return []
                
        except requests.RequestException as e:
            self.get_logger().error(f'API請求錯誤: {str(e)}')
            return []
    
    def generate_yaml_output(self, orders_data):
        """生成YAML格式的輸出"""
        output_data = {
            'medicine_order_service': {
                'timestamp': datetime.now().isoformat(),
                'total_orders': len(orders_data),
                'node_info': {
                    'name': self.get_name(),
                    'namespace': self.get_namespace(),
                    'service_name': 'get_medicine_orders'
                },
                'orders': []
            }
        }
        
        # 處理每個訂單
        for idx, order in enumerate(orders_data):
            processed_order = {
                'order_id': idx + 1,
                'patient_name': order.get('patient_name', 'N/A'),
                'doctor_name': order.get('doctor_name', 'N/A'),
                'created_at': order.get('created_at', 'N/A'),
                'medicines_count': len(order.get('medicines', [])),
                'medicines': order.get('medicines', []),
                'status': 'active'
            }
            output_data['medicine_order_service']['orders'].append(processed_order)
        
        return yaml.dump(output_data, default_flow_style=False, allow_unicode=True, indent=2)
    
    def save_output_to_file(self, yaml_content):
        """儲存輸出到檔案"""
        try:
            output_dir = os.path.expanduser("~/ros2_medicine_output")
            os.makedirs(output_dir, exist_ok=True)
            
            filename = f"medicine_orders_{datetime.now().strftime('%Y%m%d_%H%M%S')}.yaml"
            filepath = os.path.join(output_dir, filename)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(yaml_content)
            
            self.get_logger().info(f'📄 輸出已儲存至: {filepath}')
            
        except Exception as e:
            self.get_logger().error(f'儲存檔案錯誤: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MedicineOrderServiceNode()
        
        print("🚀 Medicine Order Service Node 已啟動")
        print("=" * 50)
        print("📋 可用服務:")
        print("   ros2 service call /get_medicine_orders std_srvs/srv/Trigger")
        print()
        print("📤 輸出主題:")
        print("   ros2 topic echo /medicine_order_output")
        print()
        print("🔍 檢查服務:")
        print("   ros2 service list | grep medicine")
        print("=" * 50)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 正在關閉 Medicine Order Service...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
ROS2 Basic Medicine Provider Node
基本藥物資料提供者節點
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

class BasicMedicineProviderNode(Node):
    def __init__(self):
        super().__init__('basic_medicine_provider')
        
        # 服務端點
        self.basic_service = self.create_service(
            Trigger, 
            'get_basic_medicines', 
            self.get_basic_medicines_callback
        )
        
        # 發布者
        self.basic_publisher = self.create_publisher(
            String, 
            'basic_medicines_output', 
            10
        )
        
        # API基礎URL
        self.api_base_url = "http://localhost:8000/api"
        
        # 定時發布器 (每30秒)
        self.timer = self.create_timer(30.0, self.publish_basic_medicines)
        
        self.get_logger().info('💊 Basic Medicine Provider Node Started')
        self.get_logger().info('📋 Service: /get_basic_medicines')
        self.get_logger().info('📤 Publisher: /basic_medicines_output')
    
    def get_basic_medicines_callback(self, request, response):
        """處理基本藥物查詢請求"""
        try:
            self.get_logger().info('📨 收到基本藥物查詢請求')
            
            medicines_data = self.fetch_basic_medicines()
            
            if medicines_data:
                yaml_output = self.generate_yaml_output(medicines_data)
                
                # 發布結果
                msg = String()
                msg.data = yaml_output
                self.basic_publisher.publish(msg)
                
                # 儲存到檔案
                self.save_yaml_file(yaml_output, "basic_medicines")
                
                response.success = True
                response.message = f"✅ 成功獲取 {len(medicines_data)} 種基本藥物資料"
                
                self.get_logger().info(f'✅ 成功處理基本藥物查詢，共 {len(medicines_data)} 種')
            else:
                response.success = False
                response.message = "❌ 無法獲取基本藥物資料"
                self.get_logger().warn('❌ 無法獲取基本藥物資料')
                
        except Exception as e:
            response.success = False
            response.message = f"❌ 服務錯誤: {str(e)}"
            self.get_logger().error(f'❌ 服務錯誤: {str(e)}')
        
        return response
    
    def fetch_basic_medicines(self):
        """從API獲取基本藥物資料"""
        try:
            response = requests.get(f"{self.api_base_url}/medicine/basic", timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                return data.get('medicines', [])
            else:
                self.get_logger().warn(f'API回應錯誤: {response.status_code}')
                return []
                
        except requests.RequestException as e:
            self.get_logger().error(f'API請求錯誤: {str(e)}')
            return []
    
    def generate_yaml_output(self, medicines_data):
        """生成YAML格式輸出"""
        output_data = {
            'basic_medicine_provider': {
                'timestamp': datetime.now().isoformat(),
                'total_medicines': len(medicines_data),
                'node_info': {
                    'name': self.get_name(),
                    'type': 'basic_medicine_provider',
                    'service_name': 'get_basic_medicines'
                },
                'medicines': []
            }
        }
        
        # 處理每種藥物
        for medicine in medicines_data:
            processed_medicine = {
                'name': medicine.get('name', 'N/A'),
                'amount': medicine.get('amount', 0),
                'usage_days': medicine.get('usage_days', 0),
                'position': medicine.get('position', 'N/A'),
                'manufacturer': medicine.get('manufacturer', ''),
                'dosage': medicine.get('dosage', ''),
                'created_time': medicine.get('created_time', ''),
                'stock_status': 'sufficient' if medicine.get('amount', 0) > 10 else 'low'
            }
            output_data['basic_medicine_provider']['medicines'].append(processed_medicine)
        
        return yaml.dump(output_data, default_flow_style=False, allow_unicode=True, indent=2)
    
    def publish_basic_medicines(self):
        """定時發布基本藥物資料"""
        medicines_data = self.fetch_basic_medicines()
        if medicines_data:
            yaml_output = self.generate_yaml_output(medicines_data)
            msg = String()
            msg.data = yaml_output
            self.basic_publisher.publish(msg)
            self.get_logger().debug(f'📤 定時發布基本藥物資料: {len(medicines_data)} 種')
    
    def save_yaml_file(self, yaml_content, filename_prefix):
        """儲存YAML到檔案"""
        try:
            output_dir = os.path.expanduser("~/ros2_medicine_output")
            os.makedirs(output_dir, exist_ok=True)
            
            filename = f"{filename_prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.yaml"
            filepath = os.path.join(output_dir, filename)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(yaml_content)
            
            self.get_logger().info(f'📄 基本藥物資料已儲存至: {filepath}')
            
        except Exception as e:
            self.get_logger().error(f'儲存檔案錯誤: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BasicMedicineProviderNode()
        
        print("🚀 Basic Medicine Provider Node 已啟動")
        print("=" * 50)
        print("💊 基本藥物服務:")
        print("   ros2 service call /get_basic_medicines std_srvs/srv/Trigger")
        print()
        print("📤 輸出主題:")
        print("   ros2 topic echo /basic_medicines_output")
        print()
        print("🔄 自動發布: 每30秒發布一次基本藥物資料")
        print("=" * 50)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 正在關閉 Basic Medicine Provider...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
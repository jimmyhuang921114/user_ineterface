#!/usr/bin/env python3
"""
單筆訂單服務節點 - ROS2
Single Order Service Node - ROS2

提供服務來調用單筆處方籤/訂單的詳細資訊
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import json
import yaml
import requests
import os
from pathlib import Path
from datetime import datetime

class SingleOrderServiceNode(Node):
    def __init__(self):
        super().__init__('single_order_service_node')
        
        # 建立服務
        self.order_service = self.create_service(
            Trigger,
            '/get_single_order',
            self.get_single_order_callback
        )
        
        # 建立發布者
        self.order_publisher = self.create_publisher(
            String,
            '/single_order_output',
            10
        )
        
        # API配置
        self.api_base_url = "http://localhost:8000/api"
        
        # 輸出目錄配置
        self.output_dir = Path.home() / "ros2_medicine_output"
        self.output_dir.mkdir(exist_ok=True)
        
        self.get_logger().info('🔍 單筆訂單服務節點已啟動')
        self.get_logger().info(f'📁 輸出目錄: {self.output_dir}')

    def get_single_order_callback(self, request, response):
        """處理單筆訂單服務請求"""
        try:
            self.get_logger().info('📋 收到單筆訂單請求...')
            
            # 獲取最新的處方籤
            prescription_data = self.fetch_latest_prescription()
            
            if not prescription_data:
                response.success = False
                response.message = "❌ 未找到處方籤資料"
                return response
            
            # 生成YAML輸出
            yaml_output = self.generate_single_order_yaml(prescription_data)
            
            # 發布到topic
            msg = String()
            msg.data = yaml_output
            self.order_publisher.publish(msg)
            
            # 保存到檔案
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"single_order_{timestamp}.yaml"
            file_path = self.output_dir / filename
            
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(yaml_output)
            
            response.success = True
            response.message = f"✅ 單筆訂單資料已生成: {filename}"
            
            self.get_logger().info(f'✅ 單筆訂單處理完成: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 單筆訂單服務錯誤: {str(e)}')
            response.success = False
            response.message = f"❌ 服務錯誤: {str(e)}"
        
        return response

    def fetch_latest_prescription(self):
        """獲取最新的處方籤資料"""
        try:
            # 獲取所有處方籤
            response = requests.get(f"{self.api_base_url}/prescription/", timeout=5)
            response.raise_for_status()
            
            prescriptions = response.json()
            
            # 如果是列表格式，取第一個（最新的）
            if isinstance(prescriptions, list) and prescriptions:
                return prescriptions[0]
            elif isinstance(prescriptions, dict) and 'prescriptions' in prescriptions:
                prescriptions_list = prescriptions['prescriptions']
                return prescriptions_list[0] if prescriptions_list else None
            
            return None
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'❌ API請求失敗: {str(e)}')
            return None
        except Exception as e:
            self.get_logger().error(f'❌ 處方籤資料處理錯誤: {str(e)}')
            return None

    def fetch_prescription_medicines(self, prescription_id):
        """獲取處方籤的藥物清單"""
        try:
            # 如果有處方藥物API端點，使用它
            # 否則從處方籤資料中提取
            return []
            
        except Exception as e:
            self.get_logger().error(f'❌ 獲取處方藥物錯誤: {str(e)}')
            return []

    def generate_single_order_yaml(self, prescription):
        """生成單筆訂單的YAML格式"""
        try:
            # 構建訂單資料結構
            order_data = {
                'order_info': {
                    'prescription_id': prescription.get('id', 'N/A'),
                    'patient_name': prescription.get('patient_name', 'N/A'),
                    'patient_id': prescription.get('patient_id', 'N/A'),
                    'doctor_name': prescription.get('doctor_name', 'N/A'),
                    'diagnosis': prescription.get('diagnosis', 'N/A'),
                    'status': prescription.get('status', 'N/A'),
                    'created_at': prescription.get('created_at', 'N/A'),
                    'updated_at': prescription.get('updated_at', 'N/A')
                },
                'prescription_medicines': [],
                'order_summary': {
                    'total_medicines': 0,
                    'order_status': prescription.get('status', 'N/A'),
                    'generated_at': datetime.now().isoformat(),
                    'generated_by': 'ROS2_Single_Order_Service'
                },
                'additional_info': {
                    'service_node': 'single_order_service_node',
                    'api_source': self.api_base_url,
                    'format_version': '1.0'
                }
            }
            
            # 如果處方籤包含藥物資訊，添加到輸出
            if 'medicines' in prescription:
                medicines = prescription['medicines']
                if isinstance(medicines, list):
                    for medicine in medicines:
                        medicine_data = {
                            'medicine_name': medicine.get('name', 'N/A'),
                            'dosage': medicine.get('dosage', 'N/A'),
                            'frequency': medicine.get('frequency', 'N/A'),
                            'duration': medicine.get('duration', 'N/A'),
                            'instructions': medicine.get('instructions', 'N/A')
                        }
                        order_data['prescription_medicines'].append(medicine_data)
                
                order_data['order_summary']['total_medicines'] = len(order_data['prescription_medicines'])
            
            # 轉換為YAML格式
            yaml_output = yaml.dump(order_data, 
                                   default_flow_style=False, 
                                   allow_unicode=True, 
                                   sort_keys=False)
            
            return yaml_output
            
        except Exception as e:
            self.get_logger().error(f'❌ YAML生成錯誤: {str(e)}')
            return f"# Error generating YAML: {str(e)}\n"

def main(args=None):
    rclpy.init(args=args)
    
    node = SingleOrderServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 節點正在關閉...')
    except Exception as e:
        node.get_logger().error(f'❌ 節點錯誤: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
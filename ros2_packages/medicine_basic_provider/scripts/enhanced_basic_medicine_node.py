#!/usr/bin/env python3
"""
增強版基本藥物提供者節點 - ROS2
Enhanced Basic Medicine Provider Node - ROS2

提供增強的基本藥物資訊服務，支援:
1. 查詢所有藥物
2. 查詢特定藥物
3. 按條件篩選藥物
4. 程式使用的結構化資料
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

class EnhancedBasicMedicineNode(Node):
    def __init__(self):
        super().__init__('enhanced_basic_medicine_node')
        
        # 建立多個服務
        self.all_medicines_service = self.create_service(
            Trigger,
            '/get_all_basic_medicines',
            self.get_all_medicines_callback
        )
        
        self.medicine_info_service = self.create_service(
            Trigger,
            '/get_medicine_info',
            self.get_medicine_info_callback
        )
        
        # 建立發布者
        self.basic_publisher = self.create_publisher(
            String,
            '/enhanced_basic_medicines_output',
            10
        )
        
        self.structured_publisher = self.create_publisher(
            String,
            '/structured_medicine_data',
            10
        )
        
        # API配置
        self.api_base_url = "http://localhost:8000/api"
        
        # 輸出目錄配置
        self.output_dir = Path.home() / "ros2_medicine_output"
        self.output_dir.mkdir(exist_ok=True)
        
        # 定時發布器 (每30秒)
        self.timer = self.create_timer(30.0, self.publish_basic_medicines_periodically)
        
        self.get_logger().info('🔬 增強版基本藥物節點已啟動')
        self.get_logger().info(f'📁 輸出目錄: {self.output_dir}')

    def get_all_medicines_callback(self, request, response):
        """處理獲取所有基本藥物的服務請求"""
        try:
            self.get_logger().info('💊 收到獲取所有基本藥物請求...')
            
            # 獲取基本藥物資料
            medicines_data = self.fetch_basic_medicines()
            
            if not medicines_data:
                response.success = False
                response.message = "❌ 未找到基本藥物資料"
                return response
            
            # 生成YAML輸出
            yaml_output = self.generate_all_medicines_yaml(medicines_data)
            
            # 發布到topic
            msg = String()
            msg.data = yaml_output
            self.basic_publisher.publish(msg)
            
            # 保存到檔案
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"all_basic_medicines_{timestamp}.yaml"
            file_path = self.output_dir / filename
            
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(yaml_output)
            
            response.success = True
            response.message = f"✅ 所有基本藥物資料已生成: {filename}"
            
            self.get_logger().info(f'✅ 所有基本藥物處理完成: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 獲取所有基本藥物錯誤: {str(e)}')
            response.success = False
            response.message = f"❌ 服務錯誤: {str(e)}"
        
        return response

    def get_medicine_info_callback(self, request, response):
        """處理獲取程式用藥物資訊的服務請求"""
        try:
            self.get_logger().info('📊 收到獲取程式用藥物資訊請求...')
            
            # 獲取基本和詳細藥物資料
            basic_medicines = self.fetch_basic_medicines()
            detailed_medicines = self.fetch_detailed_medicines()
            
            if not basic_medicines:
                response.success = False
                response.message = "❌ 未找到藥物資料"
                return response
            
            # 生成程式用結構化資料
            structured_data = self.generate_structured_medicine_data(basic_medicines, detailed_medicines)
            
            # 發布到專門的topic
            msg = String()
            msg.data = json.dumps(structured_data, ensure_ascii=False, indent=2)
            self.structured_publisher.publish(msg)
            
            # 保存為JSON檔案
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            json_filename = f"structured_medicines_{timestamp}.json"
            json_file_path = self.output_dir / json_filename
            
            with open(json_file_path, 'w', encoding='utf-8') as f:
                json.dump(structured_data, f, ensure_ascii=False, indent=2)
            
            # 同時保存為YAML檔案
            yaml_filename = f"structured_medicines_{timestamp}.yaml"
            yaml_file_path = self.output_dir / yaml_filename
            
            with open(yaml_file_path, 'w', encoding='utf-8') as f:
                yaml.dump(structured_data, f, default_flow_style=False, allow_unicode=True)
            
            response.success = True
            response.message = f"✅ 程式用藥物資料已生成: {json_filename}, {yaml_filename}"
            
            self.get_logger().info(f'✅ 程式用藥物資料處理完成')
            
        except Exception as e:
            self.get_logger().error(f'❌ 獲取程式用藥物資訊錯誤: {str(e)}')
            response.success = False
            response.message = f"❌ 服務錯誤: {str(e)}"
        
        return response

    def publish_basic_medicines_periodically(self):
        """定期發布基本藥物資料"""
        try:
            medicines_data = self.fetch_basic_medicines()
            if medicines_data:
                yaml_output = self.generate_all_medicines_yaml(medicines_data)
                
                msg = String()
                msg.data = yaml_output
                self.basic_publisher.publish(msg)
                
                self.get_logger().info('📢 定期發布基本藥物資料完成')
                
        except Exception as e:
            self.get_logger().error(f'❌ 定期發布錯誤: {str(e)}')

    def fetch_basic_medicines(self):
        """獲取基本藥物資料"""
        try:
            response = requests.get(f"{self.api_base_url}/medicine/basic", timeout=5)
            response.raise_for_status()
            
            data = response.json()
            
            # 處理不同的API響應格式
            if isinstance(data, list):
                return data
            elif isinstance(data, dict) and 'medicines' in data:
                return data['medicines']
            elif isinstance(data, dict):
                return [data]
            
            return []
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'❌ 基本藥物API請求失敗: {str(e)}')
            return []
        except Exception as e:
            self.get_logger().error(f'❌ 基本藥物資料處理錯誤: {str(e)}')
            return []

    def fetch_detailed_medicines(self):
        """獲取詳細藥物資料"""
        try:
            response = requests.get(f"{self.api_base_url}/medicine/detailed", timeout=5)
            response.raise_for_status()
            
            data = response.json()
            
            # 處理不同的API響應格式
            if isinstance(data, list):
                return data
            elif isinstance(data, dict) and 'medicines' in data:
                return data['medicines']
            elif isinstance(data, dict):
                return [data]
            
            return []
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'❌ 詳細藥物API請求失敗: {str(e)}')
            return []
        except Exception as e:
            self.get_logger().error(f'❌ 詳細藥物資料處理錯誤: {str(e)}')
            return []

    def generate_all_medicines_yaml(self, medicines):
        """生成所有基本藥物的YAML格式"""
        try:
            medicines_data = {
                'basic_medicines': {
                    'count': len(medicines),
                    'generated_at': datetime.now().isoformat(),
                    'medicines': []
                },
                'metadata': {
                    'service_node': 'enhanced_basic_medicine_node',
                    'api_source': self.api_base_url,
                    'format_version': '2.0'
                }
            }
            
            for medicine in medicines:
                medicine_entry = {
                    'id': medicine.get('id', 'N/A'),
                    'name': medicine.get('name', 'N/A'),
                    'amount': medicine.get('amount', 0),
                    'position': medicine.get('position', 'N/A'),
                    'manufacturer': medicine.get('manufacturer', ''),
                    'dosage': medicine.get('dosage', ''),
                    'prompt': medicine.get('prompt', ''),  # 包含AI提示詞
                    'created_time': medicine.get('created_time', ''),
                    'stock_status': self.determine_stock_status(medicine.get('amount', 0)),
                    'usage_days': medicine.get('usage_days', None)
                }
                medicines_data['basic_medicines']['medicines'].append(medicine_entry)
            
            return yaml.dump(medicines_data, default_flow_style=False, allow_unicode=True, sort_keys=False)
            
        except Exception as e:
            self.get_logger().error(f'❌ YAML生成錯誤: {str(e)}')
            return f"# Error generating YAML: {str(e)}\n"

    def generate_structured_medicine_data(self, basic_medicines, detailed_medicines):
        """生成程式用的結構化藥物資料"""
        try:
            # 建立詳細藥物的索引 (按medicine_id或medicine_name)
            detailed_index = {}
            for detailed in detailed_medicines:
                medicine_id = detailed.get('medicine_id')
                if medicine_id:
                    detailed_index[medicine_id] = detailed
            
            structured_data = {
                'medicine_database': {
                    'total_medicines': len(basic_medicines),
                    'last_updated': datetime.now().isoformat(),
                    'medicines': []
                },
                'statistics': {
                    'low_stock_count': 0,
                    'normal_stock_count': 0,
                    'with_ai_prompt_count': 0,
                    'with_detailed_info_count': len(detailed_medicines)
                },
                'metadata': {
                    'generated_by': 'enhanced_basic_medicine_node',
                    'data_source': 'SQL_Database',
                    'format': 'structured_json',
                    'version': '2.0'
                }
            }
            
            for medicine in basic_medicines:
                medicine_id = medicine.get('id')
                detailed_info = detailed_index.get(medicine_id, {})
                
                # 統計庫存狀態
                stock_amount = medicine.get('amount', 0)
                if stock_amount < 10:
                    structured_data['statistics']['low_stock_count'] += 1
                else:
                    structured_data['statistics']['normal_stock_count'] += 1
                
                # 統計AI提示詞
                if medicine.get('prompt'):
                    structured_data['statistics']['with_ai_prompt_count'] += 1
                
                medicine_entry = {
                    'basic_info': {
                        'id': medicine_id,
                        'name': medicine.get('name', ''),
                        'amount': stock_amount,
                        'position': medicine.get('position', ''),
                        'manufacturer': medicine.get('manufacturer', ''),
                        'dosage': medicine.get('dosage', ''),
                        'usage_days': medicine.get('usage_days'),
                        'ai_prompt': medicine.get('prompt', ''),
                        'created_time': medicine.get('created_time', ''),
                        'stock_status': self.determine_stock_status(stock_amount)
                    },
                    'detailed_info': {
                        'has_detailed': bool(detailed_info),
                        'description': detailed_info.get('description', ''),
                        'ingredient': detailed_info.get('ingredient', ''),
                        'category': detailed_info.get('category', ''),
                        'usage_method': detailed_info.get('usage_method', ''),
                        'side_effects': detailed_info.get('side_effects', ''),
                        'storage_conditions': detailed_info.get('storage_conditions', ''),
                        'expiry_date': detailed_info.get('expiry_date', ''),
                        'barcode': detailed_info.get('barcode', ''),
                        'appearance_type': detailed_info.get('appearance_type', ''),
                        'notes': detailed_info.get('notes', '')
                    },
                    'for_programming': {
                        'unique_key': f"med_{medicine_id}",
                        'search_keywords': [
                            medicine.get('name', '').lower(),
                            medicine.get('manufacturer', '').lower(),
                            detailed_info.get('category', '').lower()
                        ],
                        'is_available': stock_amount > 0,
                        'requires_prescription': True,  # 可根據藥物類型調整
                        'last_updated': medicine.get('updated_time', medicine.get('created_time', ''))
                    }
                }
                
                structured_data['medicine_database']['medicines'].append(medicine_entry)
            
            return structured_data
            
        except Exception as e:
            self.get_logger().error(f'❌ 結構化資料生成錯誤: {str(e)}')
            return {'error': str(e)}

    def determine_stock_status(self, amount):
        """判斷庫存狀態"""
        if amount <= 0:
            return 'out_of_stock'
        elif amount < 10:
            return 'low_stock'
        elif amount < 50:
            return 'normal'
        else:
            return 'sufficient'

def main(args=None):
    rclpy.init(args=args)
    
    node = EnhancedBasicMedicineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 增強版基本藥物節點正在關閉...')
    except Exception as e:
        node.get_logger().error(f'❌ 節點錯誤: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
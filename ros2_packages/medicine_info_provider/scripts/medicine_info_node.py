#!/usr/bin/env python3
"""
藥物資訊提供者節點 - ROS2
Medicine Information Provider Node - ROS2

專門呼叫基礎和詳細藥物的資訊
"""

import rclpy
from rclpy.node import Node
from medicine_interfaces.srv import GetMedicineInfo
from medicine_interfaces.msg import MedicineBasic, MedicineDetailed
from std_msgs.msg import String
import requests
import json
from datetime import datetime
from pathlib import Path

class MedicineInfoProviderNode(Node):
    def __init__(self):
        super().__init__('medicine_info_provider_node')
        
        # 創建服務
        self.medicine_info_service = self.create_service(
            GetMedicineInfo,
            'get_medicine_info',
            self.get_medicine_info_callback
        )
        
        # 創建發布者
        self.medicine_info_publisher = self.create_publisher(
            String,
            'medicine_info_output',
            10
        )
        
        # API配置
        self.api_base_url = "http://localhost:8000/api"
        
        # 快取配置
        self.cache_timeout = 30  # 30秒快取
        self.basic_medicines_cache = {'data': None, 'timestamp': 0}
        self.detailed_medicines_cache = {'data': None, 'timestamp': 0}
        
        # 輸出目錄配置
        self.output_dir = Path.home() / "ros2_medicine_output" / "medicine_info"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 定時發布器 (每60秒)
        self.timer = self.create_timer(60.0, self.publish_medicine_info_periodically)
        
        self.get_logger().info('💊 藥物資訊提供者節點已啟動')
        self.get_logger().info(f'📁 輸出目錄: {self.output_dir}')

    def get_medicine_info_callback(self, request, response):
        """處理藥物資訊查詢請求"""
        try:
            self.get_logger().info(f'🔍 收到藥物資訊查詢請求')
            self.get_logger().info(f'📝 查詢參數: 藥物名稱="{request.medicine_name}", 基本資訊={request.include_basic}, 詳細資訊={request.include_detailed}')
            
            basic_medicines = []
            detailed_medicines = []
            
            # 獲取基本藥物資訊
            if request.include_basic:
                basic_result = self.fetch_basic_medicines(request.medicine_name)
                if basic_result['success']:
                    basic_medicines = self.convert_to_basic_message_list(basic_result['data'])
                else:
                    response.success = False
                    response.message = f"獲取基本藥物資訊失敗: {basic_result['error']}"
                    return response
            
            # 獲取詳細藥物資訊
            if request.include_detailed:
                detailed_result = self.fetch_detailed_medicines(request.medicine_name)
                if detailed_result['success']:
                    detailed_medicines = self.convert_to_detailed_message_list(detailed_result['data'])
                else:
                    response.success = False
                    response.message = f"獲取詳細藥物資訊失敗: {detailed_result['error']}"
                    return response
            
            # 檢查可用性
            if request.check_availability:
                self.check_medicine_availability(basic_medicines)
            
            # 生成綜合資訊
            combined_info = self.generate_combined_info(basic_medicines, detailed_medicines)
            
            # 發布到主題
            info_json = json.dumps(combined_info, ensure_ascii=False, indent=2)
            msg = String()
            msg.data = info_json
            self.medicine_info_publisher.publish(msg)
            
            # 保存到檔案
            self.save_medicine_info(combined_info, request.medicine_name)
            
            # 設置回應
            response.success = True
            response.message = f"成功獲取藥物資訊，基本藥物: {len(basic_medicines)}, 詳細藥物: {len(detailed_medicines)}"
            response.basic_medicines = basic_medicines
            response.detailed_medicines = detailed_medicines
            response.total_count = len(basic_medicines) + len(detailed_medicines)
            response.query_time = datetime.now().isoformat()
            
            self.get_logger().info(f'✅ 藥物資訊查詢完成，返回 {len(basic_medicines)} 基本藥物和 {len(detailed_medicines)} 詳細藥物')
            
        except Exception as e:
            self.get_logger().error(f'❌ 藥物資訊查詢錯誤: {str(e)}')
            response.success = False
            response.message = f"查詢過程發生錯誤: {str(e)}"
            response.basic_medicines = []
            response.detailed_medicines = []
            response.total_count = 0
            response.query_time = datetime.now().isoformat()
        
        return response

    def fetch_basic_medicines(self, medicine_name=""):
        """獲取基本藥物資訊"""
        try:
            # 檢查快取
            current_time = datetime.now().timestamp()
            if (self.basic_medicines_cache['data'] is not None and 
                current_time - self.basic_medicines_cache['timestamp'] < self.cache_timeout):
                cached_data = self.basic_medicines_cache['data']
                
                # 如果指定了藥物名稱，進行篩選
                if medicine_name:
                    filtered_data = [med for med in cached_data if med['name'] == medicine_name]
                    return {'success': True, 'data': filtered_data, 'error': ''}
                else:
                    return {'success': True, 'data': cached_data, 'error': ''}
            
            # 從API獲取
            response = requests.get(f"{self.api_base_url}/medicine/basic", timeout=10)
            response.raise_for_status()
            
            medicine_data = response.json()
            medicines = medicine_data.get('medicines', medicine_data) if isinstance(medicine_data, dict) else medicine_data
            
            # 更新快取
            self.basic_medicines_cache = {
                'data': medicines,
                'timestamp': current_time
            }
            
            # 如果指定了藥物名稱，進行篩選
            if medicine_name:
                medicines = [med for med in medicines if med['name'] == medicine_name]
            
            return {'success': True, 'data': medicines, 'error': ''}
            
        except Exception as e:
            return {'success': False, 'data': [], 'error': str(e)}

    def fetch_detailed_medicines(self, medicine_name=""):
        """獲取詳細藥物資訊"""
        try:
            # 檢查快取
            current_time = datetime.now().timestamp()
            if (self.detailed_medicines_cache['data'] is not None and 
                current_time - self.detailed_medicines_cache['timestamp'] < self.cache_timeout):
                cached_data = self.detailed_medicines_cache['data']
                
                # 如果指定了藥物名稱，進行篩選
                if medicine_name:
                    filtered_data = [med for med in cached_data if med.get('medicine_name') == medicine_name]
                    return {'success': True, 'data': filtered_data, 'error': ''}
                else:
                    return {'success': True, 'data': cached_data, 'error': ''}
            
            # 從API獲取
            response = requests.get(f"{self.api_base_url}/medicine/detailed", timeout=10)
            response.raise_for_status()
            
            medicine_data = response.json()
            medicines = medicine_data.get('medicines', medicine_data) if isinstance(medicine_data, dict) else medicine_data
            
            # 更新快取
            self.detailed_medicines_cache = {
                'data': medicines,
                'timestamp': current_time
            }
            
            # 如果指定了藥物名稱，進行篩選
            if medicine_name:
                medicines = [med for med in medicines if med.get('medicine_name') == medicine_name]
            
            return {'success': True, 'data': medicines, 'error': ''}
            
        except Exception as e:
            return {'success': False, 'data': [], 'error': str(e)}

    def convert_to_basic_message_list(self, medicine_list):
        """轉換為基本藥物訊息列表"""
        message_list = []
        
        for med in medicine_list:
            msg = MedicineBasic()
            msg.id = med.get('id', 0)
            msg.name = med.get('name', '')
            msg.amount = med.get('amount', 0)
            msg.position = med.get('position', '')
            msg.manufacturer = med.get('manufacturer', '')
            msg.dosage = med.get('dosage', '')
            msg.prompt = med.get('prompt', '')
            msg.usage_days = med.get('usage_days', 0) or 0
            msg.created_time = med.get('created_time', '')
            msg.updated_time = med.get('updated_time', '')
            msg.is_active = med.get('is_active', True)
            
            message_list.append(msg)
        
        return message_list

    def convert_to_detailed_message_list(self, medicine_list):
        """轉換為詳細藥物訊息列表"""
        message_list = []
        
        for med in medicine_list:
            msg = MedicineDetailed()
            msg.id = med.get('id', 0)
            msg.medicine_id = med.get('medicine_id', 0)
            msg.medicine_name = med.get('medicine_name', '')
            msg.description = med.get('description', '')
            msg.ingredient = med.get('ingredient', '')
            msg.category = med.get('category', '')
            msg.usage_method = med.get('usage_method', '')
            msg.unit_dose = med.get('unit_dose', '')
            msg.side_effects = med.get('side_effects', '')
            msg.storage_conditions = med.get('storage_conditions', '')
            msg.expiry_date = med.get('expiry_date', '')
            msg.barcode = med.get('barcode', '')
            msg.appearance_type = med.get('appearance_type', '')
            msg.notes = med.get('notes', '')
            msg.created_time = med.get('created_time', '')
            msg.updated_time = med.get('updated_time', '')
            
            message_list.append(msg)
        
        return message_list

    def check_medicine_availability(self, basic_medicines):
        """檢查藥物可用性並標記"""
        for med in basic_medicines:
            if med.amount <= 0:
                med.is_active = False  # 標記為不可用
            elif med.amount < 10:
                # 可以在這裡添加低庫存警告邏輯
                pass

    def generate_combined_info(self, basic_medicines, detailed_medicines):
        """生成綜合藥物資訊"""
        combined_info = {
            'query_info': {
                'timestamp': datetime.now().isoformat(),
                'basic_count': len(basic_medicines),
                'detailed_count': len(detailed_medicines),
                'total_count': len(basic_medicines) + len(detailed_medicines)
            },
            'basic_medicines': [],
            'detailed_medicines': [],
            'statistics': {
                'available_medicines': 0,
                'low_stock_medicines': 0,
                'out_of_stock_medicines': 0,
                'with_ai_prompt': 0
            }
        }
        
        # 處理基本藥物
        for med in basic_medicines:
            basic_info = {
                'id': med.id,
                'name': med.name,
                'amount': med.amount,
                'position': med.position,
                'manufacturer': med.manufacturer,
                'dosage': med.dosage,
                'prompt': med.prompt,
                'usage_days': med.usage_days,
                'is_active': med.is_active,
                'created_time': med.created_time,
                'updated_time': med.updated_time
            }
            combined_info['basic_medicines'].append(basic_info)
            
            # 統計
            if med.amount > 0:
                combined_info['statistics']['available_medicines'] += 1
                if med.amount < 10:
                    combined_info['statistics']['low_stock_medicines'] += 1
            else:
                combined_info['statistics']['out_of_stock_medicines'] += 1
            
            if med.prompt:
                combined_info['statistics']['with_ai_prompt'] += 1
        
        # 處理詳細藥物
        for med in detailed_medicines:
            detailed_info = {
                'id': med.id,
                'medicine_id': med.medicine_id,
                'medicine_name': med.medicine_name,
                'description': med.description,
                'ingredient': med.ingredient,
                'category': med.category,
                'usage_method': med.usage_method,
                'unit_dose': med.unit_dose,
                'side_effects': med.side_effects,
                'storage_conditions': med.storage_conditions,
                'expiry_date': med.expiry_date,
                'barcode': med.barcode,
                'appearance_type': med.appearance_type,
                'notes': med.notes,
                'created_time': med.created_time,
                'updated_time': med.updated_time
            }
            combined_info['detailed_medicines'].append(detailed_info)
        
        return combined_info

    def save_medicine_info(self, info, medicine_name=""):
        """保存藥物資訊到檔案"""
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            
            if medicine_name:
                filename = f"medicine_info_{medicine_name}_{timestamp}.json"
            else:
                filename = f"all_medicines_info_{timestamp}.json"
            
            file_path = self.output_dir / filename
            
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(info, f, ensure_ascii=False, indent=2)
            
            self.get_logger().info(f'📄 藥物資訊已保存: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 保存藥物資訊失敗: {str(e)}')

    def publish_medicine_info_periodically(self):
        """定期發布藥物資訊"""
        try:
            # 獲取所有基本和詳細藥物資訊
            basic_result = self.fetch_basic_medicines()
            detailed_result = self.fetch_detailed_medicines()
            
            if basic_result['success'] and detailed_result['success']:
                basic_medicines = self.convert_to_basic_message_list(basic_result['data'])
                detailed_medicines = self.convert_to_detailed_message_list(detailed_result['data'])
                
                combined_info = self.generate_combined_info(basic_medicines, detailed_medicines)
                
                # 發布到主題
                info_json = json.dumps(combined_info, ensure_ascii=False, indent=2)
                msg = String()
                msg.data = info_json
                self.medicine_info_publisher.publish(msg)
                
                self.get_logger().info(f'📢 定期發布藥物資訊完成 (基本: {len(basic_medicines)}, 詳細: {len(detailed_medicines)})')
                
        except Exception as e:
            self.get_logger().error(f'❌ 定期發布錯誤: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = MedicineInfoProviderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 藥物資訊提供者節點正在關閉...')
    except Exception as e:
        node.get_logger().error(f'❌ 節點錯誤: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
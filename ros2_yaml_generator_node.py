#!/usr/bin/env python3
"""
ROS2 YAML 藥物訂單生成節點
根據訂單需求生成指定格式的YAML文件，並在新訂單時覆蓋
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import yaml
import json
from datetime import datetime
import os

class YAMLMedicineOrderNode(Node):
    def __init__(self):
        super().__init__('yaml_medicine_order_generator')
        
        # API配置
        self.api_base = "http://localhost:8000"
        
        # 創建訂單接收訂閱者
        self.order_subscription = self.create_subscription(
            String,
            'medicine_order_request',
            self.handle_order_request,
            10
        )
        
        # 創建YAML發布者
        self.yaml_publisher = self.create_publisher(String, 'medicine_yaml_output', 10)
        
        # 創建定時器，每10秒檢查一次新訂單
        self.timer = self.create_timer(10.0, self.check_for_new_orders)
        
        # 訂單狀態
        self.current_order_id = None
        self.yaml_output_path = "medicine_order_output.yaml"
        
        self.get_logger().info('🤖 YAML藥物訂單生成節點已啟動')
        self.get_logger().info(f'📁 YAML輸出路徑: {self.yaml_output_path}')

    def handle_order_request(self, msg):
        """處理訂單請求"""
        try:
            order_data = json.loads(msg.data)
            order_id = order_data.get('id', 'unknown')
            
            self.get_logger().info(f'📦 收到新訂單: {order_id}')
            
            # 生成YAML
            yaml_content = self.generate_yaml_from_order(order_data)
            
            if yaml_content:
                # 覆蓋寫入YAML文件
                self.write_yaml_file(yaml_content)
                
                # 發布YAML內容
                yaml_msg = String()
                yaml_msg.data = yaml_content
                self.yaml_publisher.publish(yaml_msg)
                
                self.current_order_id = order_id
                self.get_logger().info(f'✅ 訂單 {order_id} 的YAML已生成並覆蓋')
            
        except Exception as e:
            self.get_logger().error(f'❌ 處理訂單失敗: {e}')

    def generate_yaml_from_order(self, order_data):
        """根據訂單生成YAML格式"""
        try:
            order_id = order_data.get('id', '000001')
            medicines_in_order = order_data.get('order_data', {})
            
            # 獲取所有藥物資料
            basic_medicines = self.get_medicine_data('basic')
            detailed_medicines = self.get_medicine_data('detailed')
            
            if not basic_medicines or not detailed_medicines:
                self.get_logger().warn('⚠️ 無法獲取藥物資料')
                return None
            
            # 構建YAML數據結構
            yaml_data = {}
            
            # 處理訂單中的每種藥物
            for medicine_key, order_info in medicines_in_order.items():
                amount = order_info.get('amount', 0)
                locate = order_info.get('locate', [1, 1])
                
                # 查找對應的藥物資料
                # 這裡我們根據位置或其他條件來匹配藥物
                medicine_name = self.find_medicine_by_criteria(basic_medicines, detailed_medicines, locate)
                
                if medicine_name:
                    # 獲取基本和詳細資料
                    basic_data = self.find_medicine_by_name(basic_medicines, medicine_name)
                    detailed_data = self.find_medicine_by_name(detailed_medicines, medicine_name, is_detailed=True)
                    
                    if basic_data and detailed_data:
                        yaml_data[medicine_name] = self.create_medicine_yaml_structure(
                            basic_data, detailed_data, locate
                        )
            
            # 如果沒有找到匹配的藥物，使用示例數據
            if not yaml_data:
                yaml_data = self.create_sample_yaml_data()
            
            # 轉換為YAML字符串
            yaml_content = yaml.dump(yaml_data, default_flow_style=False, allow_unicode=True, sort_keys=False)
            
            # 添加訂單標題
            header = f"# 訂單ID: {order_id}\n# 生成時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n# 精神好製藥\n\n"
            
            return header + yaml_content
            
        except Exception as e:
            self.get_logger().error(f'❌ 生成YAML失敗: {e}')
            return None

    def create_medicine_yaml_structure(self, basic_data, detailed_data, locate):
        """創建單個藥物的YAML結構"""
        return {
            "藥物基本資料": {
                "外觀類型": detailed_data.get('appearance', {}).get('shape', '藥片'),
                "存取位置": locate
            },
            "藥物詳細資料": {
                "名稱": basic_data.get('name', ''),
                "成分": detailed_data.get('ingredient', detailed_data.get('notes', '')),
                "分類": detailed_data.get('category', '一般藥物'),
                "劑量": basic_data.get('dosage', detailed_data.get('dosage', '')),
                "服用方式": detailed_data.get('usage_method', '口服 (Oral use)'),
                "單位劑量": detailed_data.get('unit_dose', '1 special pill'),
                "有效日期": detailed_data.get('expiry_date', ''),
                "適應症": detailed_data.get('indications', detailed_data.get('description', '')),
                "可能副作用": detailed_data.get('side_effects', ''),
                "條碼編號": detailed_data.get('barcode', 'N/A'),
                "外觀": {
                    "顏色": detailed_data.get('appearance', {}).get('color', ''),
                    "形狀": detailed_data.get('appearance', {}).get('shape', '')
                }
            }
        }

    def create_sample_yaml_data(self):
        """創建示例YAML資料"""
        return {
            "Antipsychotics": {
                "藥物基本資料": {
                    "外觀類型": "藥片",
                    "存取位置": [1, 1]
                },
                "藥物詳細資料": {
                    "名稱": "Antipsychotics",
                    "成分": "Antipsychotic compounds",
                    "分類": "精神科藥物",
                    "劑量": "5 毫克",
                    "服用方式": "口服 (Oral use)",
                    "單位劑量": "1 special pill",
                    "有效日期": "2027/08/02",
                    "適應症": "消除精神分裂症的陽性病徵，例如幻覺、妄想、思想混亂等。有助眠功效，適用於對其他藥物不適應者。",
                    "可能副作用": "嗜睡、頭暈、體重增加、口乾、便秘、姿勢性低血壓",
                    "條碼編號": "ANTI-123456789",
                    "外觀": {
                        "顏色": "紅色條紋 白色外觀",
                        "形狀": "圓扁形"
                    }
                }
            },
            "Andsodhcd": {
                "藥物基本資料": {
                    "外觀類型": "藥片",
                    "存取位置": [1, 2]
                },
                "藥物詳細資料": {
                    "名稱": "Andsodhcd",
                    "成分": "Nirmatrelvir",
                    "分類": "3CL proteinase inhibitors",
                    "劑量": "1 special pill",
                    "服用方式": "口服 (Oral use)",
                    "單位劑量": "1 special pill",
                    "有效日期": "2027/11/09",
                    "適應症": "適用於12歲以上、體重至少40公斤，於5天內確診輕度至中度COVID-19，且具嚴重疾病風險因子的成人與兒童",
                    "可能副作用": "味覺異常、腹瀉、噁心、嘔吐、頭痛",
                    "條碼編號": "TEST-367842394",
                    "外觀": {
                        "顏色": "藍色條紋 白色外觀",
                        "形狀": "圓扁形"
                    }
                }
            }
        }

    def get_medicine_data(self, data_type):
        """獲取藥物資料"""
        try:
            if data_type == 'basic':
                response = requests.get(f"{self.api_base}/api/ros2/medicine/basic", timeout=5)
            else:
                response = requests.get(f"{self.api_base}/api/ros2/medicine/detailed", timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                return data.get('data', [])
            else:
                self.get_logger().warn(f'⚠️ API回應錯誤: {response.status_code}')
                return []
                
        except Exception as e:
            self.get_logger().warn(f'⚠️ 無法連接API: {e}')
            return []

    def find_medicine_by_criteria(self, basic_medicines, detailed_medicines, locate):
        """根據條件查找藥物"""
        # 首先嘗試根據位置查找
        for medicine in basic_medicines:
            if medicine.get('position') == f"{locate[0]}-{locate[1]}":
                return medicine.get('name')
        
        # 如果沒找到，返回第一個藥物的名稱
        if basic_medicines:
            return basic_medicines[0].get('name')
        
        return None

    def find_medicine_by_name(self, medicines, name, is_detailed=False):
        """根據名稱查找藥物"""
        for medicine in medicines:
            if is_detailed:
                if medicine.get('medicine_name') == name:
                    return medicine
            else:
                if medicine.get('name') == name:
                    return medicine
        return None

    def write_yaml_file(self, yaml_content):
        """寫入YAML文件（覆蓋模式）"""
        try:
            with open(self.yaml_output_path, 'w', encoding='utf-8') as f:
                f.write(yaml_content)
            self.get_logger().info(f'📝 YAML文件已更新: {self.yaml_output_path}')
        except Exception as e:
            self.get_logger().error(f'❌ 寫入YAML文件失敗: {e}')

    def check_for_new_orders(self):
        """定期檢查新訂單"""
        try:
            # 檢查是否有新的處方籤（可以視為訂單）
            response = requests.get(f"{self.api_base}/api/ros2/prescription", timeout=5)
            if response.status_code == 200:
                data = response.json()
                prescriptions = data.get('data', [])
                
                if prescriptions:
                    # 獲取最新的處方籤
                    latest_prescription = prescriptions[-1]
                    prescription_id = latest_prescription.get('id', 'unknown')
                    
                    # 如果是新訂單，處理它
                    if prescription_id != self.current_order_id:
                        self.get_logger().info(f'🔄 發現新處方籤: {prescription_id}')
                        
                        # 轉換處方籤格式為訂單格式
                        order_data = self.convert_prescription_to_order(latest_prescription)
                        
                        # 生成YAML
                        yaml_content = self.generate_yaml_from_order(order_data)
                        
                        if yaml_content:
                            self.write_yaml_file(yaml_content)
                            
                            # 發布YAML內容
                            yaml_msg = String()
                            yaml_msg.data = yaml_content
                            self.yaml_publisher.publish(yaml_msg)
                            
                            self.current_order_id = prescription_id
                            self.get_logger().info(f'✅ 新處方籤 {prescription_id} 的YAML已生成')
                        
        except Exception as e:
            # 靜默處理連接錯誤，避免過多日誌
            pass

    def convert_prescription_to_order(self, prescription):
        """將處方籤轉換為訂單格式"""
        order_data = {
            "id": prescription.get('id', '000001'),
            "order_data": {}
        }
        
        medicines = prescription.get('medicines', [])
        for i, medicine in enumerate(medicines):
            medicine_key = f"medicine_{i+1}"
            order_data["order_data"][medicine_key] = {
                "amount": 1,  # 預設數量
                "locate": [1, i+1],  # 預設位置
                "name": medicine.get('medicine_name', '')
            }
        
        return order_data

    def publish_manual_order(self, order_id="000001", medicines_data=None):
        """手動發布訂單（用於測試）"""
        if medicines_data is None:
            medicines_data = {
                "medicine_1": {
                    "amount": 87,
                    "locate": [1, 1],
                    "name": "Antipsychotics"
                }
            }
        
        order_data = {
            "id": order_id,
            "order_data": medicines_data
        }
        
        yaml_content = self.generate_yaml_from_order(order_data)
        
        if yaml_content:
            self.write_yaml_file(yaml_content)
            
            # 發布YAML內容
            yaml_msg = String()
            yaml_msg.data = yaml_content
            self.yaml_publisher.publish(yaml_msg)
            
            self.current_order_id = order_id
            self.get_logger().info(f'📤 手動訂單 {order_id} 已發布')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YAMLMedicineOrderNode()
        
        # 啟動時發布一個示例訂單
        node.get_logger().info('🚀 發布示例訂單...')
        node.publish_manual_order()
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
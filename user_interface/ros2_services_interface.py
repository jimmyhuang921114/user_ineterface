#!/usr/bin/env python3
"""
ROS2 服務接口 - 醫院藥物管理系統
提供訂單處理和藥物查詢的 ROS2 服務
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
import yaml
import requests
import threading
import time
import logging
from typing import Dict, Any, Optional

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ros2_services")

# 自定義服務消息類型（如果沒有，我們使用 String 類型）
try:
    from custom_interfaces.srv import GetOrder, GetMedicineDetail, CompleteOrder
except ImportError:
    # 如果沒有自定義接口，我們創建簡單的替代方案
    logger.warning("未找到自定義接口，使用 String 服務")
    from std_srvs.srv import Empty
    GetOrder = Empty
    GetMedicineDetail = Empty  
    CompleteOrder = Empty

class HospitalROS2Services(Node):
    """醫院 ROS2 服務節點"""
    
    def __init__(self):
        super().__init__('hospital_services')
        
        # 配置
        self.web_api_url = "http://localhost:8001"
        self.current_order = None
        self.processing_lock = threading.Lock()
        
        # 創建服務
        self.get_order_service = self.create_service(
            Empty, 
            'hospital/get_order', 
            self.get_order_callback
        )
        
        self.get_medicine_detail_service = self.create_service(
            Empty,
            'hospital/get_medicine_detail',
            self.get_medicine_detail_callback
        )
        
        self.complete_order_service = self.create_service(
            Empty,
            'hospital/complete_order', 
            self.complete_order_callback
        )
        
        # 發布者 - 用於發送數據
        self.order_publisher = self.create_publisher(String, 'hospital/order_data', 10)
        self.medicine_publisher = self.create_publisher(String, 'hospital/medicine_data', 10)
        self.status_publisher = self.create_publisher(String, 'hospital/status', 10)
        
        # 訂閱者 - 用於接收請求
        self.medicine_request_sub = self.create_subscription(
            String,
            'hospital/medicine_request',
            self.medicine_request_callback,
            10
        )
        
        self.get_logger().info("🏥 醫院 ROS2 服務已啟動")
        self.get_logger().info("📋 可用服務:")
        self.get_logger().info("   • hospital/get_order - 獲取新訂單")
        self.get_logger().info("   • hospital/get_medicine_detail - 查詢藥物詳細資訊")
        self.get_logger().info("   • hospital/complete_order - 標記訂單完成")
        self.get_logger().info("📡 Topic:")
        self.get_logger().info("   • hospital/order_data - 訂單數據")
        self.get_logger().info("   • hospital/medicine_data - 藥物數據")
        self.get_logger().info("   • hospital/medicine_request - 藥物查詢請求")
        self.get_logger().info("   • hospital/status - 系統狀態")

    def get_order_callback(self, request, response):
        """獲取新訂單的服務回調"""
        try:
            self.get_logger().info("📋 收到獲取訂單請求")
            
            with self.processing_lock:
                if self.current_order:
                    self.get_logger().warn("⚠️ 已有訂單正在處理中")
                    # 發送當前訂單
                    order_msg = String()
                    order_msg.data = json.dumps(self.current_order)
                    self.order_publisher.publish(order_msg)
                    return response
                
                # 從 Web API 獲取新訂單
                order_data = self._fetch_new_order()
                
                if order_data:
                    self.current_order = order_data
                    self.get_logger().info(f"✅ 獲取到新訂單: {order_data['order_id']}")
                    
                    # 發布訂單數據 (YAML 格式)
                    order_yaml = self._format_order_yaml(order_data)
                    order_msg = String()
                    order_msg.data = order_yaml
                    self.order_publisher.publish(order_msg)
                    
                    # 發布狀態
                    status_msg = String()
                    status_msg.data = yaml.dump({
                        "status": "order_received", 
                        "order_id": order_data['order_id']
                    }, default_flow_style=False, allow_unicode=True)
                    self.status_publisher.publish(status_msg)
                    
                else:
                    self.get_logger().info("📭 目前沒有新訂單")
                    
        except Exception as e:
            self.get_logger().error(f"❌ 獲取訂單時發生錯誤: {e}")
            
        return response

    def get_medicine_detail_callback(self, request, response):
        """獲取藥物詳細資訊的服務回調（已棄用，請使用 Topic）"""
        self.get_logger().warn("⚠️ 請使用 hospital/medicine_request Topic 查詢藥物詳細資訊")
        return response

    def medicine_request_callback(self, msg):
        """處理藥物查詢請求"""
        try:
            medicine_name = msg.data
            self.get_logger().info(f"💊 收到藥物查詢請求: {medicine_name}")
            
            # 查詢藥物詳細資訊
            medicine_detail = self._fetch_medicine_detail(medicine_name)
            
            if medicine_detail:
                # 發布藥物詳細資訊 (YAML 格式)
                medicine_yaml = self._format_medicine_yaml(medicine_detail)
                detail_msg = String()
                detail_msg.data = medicine_yaml
                self.medicine_publisher.publish(detail_msg)
                
                self.get_logger().info(f"✅ 已發送 {medicine_name} 的詳細資訊")
            else:
                # 發送錯誤訊息
                error_msg = String()
                error_msg.data = yaml.dump({
                    "error": f"未找到藥物: {medicine_name}",
                    "medicine_name": medicine_name
                }, default_flow_style=False, allow_unicode=True)
                self.medicine_publisher.publish(error_msg)
                
                self.get_logger().warn(f"⚠️ 未找到藥物: {medicine_name}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 查詢藥物時發生錯誤: {e}")

    def complete_order_callback(self, request, response):
        """標記訂單完成的服務回調"""
        try:
            with self.processing_lock:
                if not self.current_order:
                    self.get_logger().warn("⚠️ 沒有正在處理的訂單")
                    return response
                
                order_id = self.current_order['order_id']
                self.get_logger().info(f"✅ 標記訂單完成: {order_id}")
                
                # 通知 Web 系統訂單完成
                success = self._notify_order_complete(order_id)
                
                if success:
                    self.get_logger().info(f"✅ 已通知 Web 系統訂單 {order_id} 完成")
                    self.current_order = None
                    
                    # 發布完成狀態
                    status_msg = String()
                    status_msg.data = yaml.dump({
                        "status": "order_completed", 
                        "order_id": order_id
                    }, default_flow_style=False, allow_unicode=True)
                    self.status_publisher.publish(status_msg)
                    
                else:
                    self.get_logger().error(f"❌ 通知 Web 系統失敗")
                    
        except Exception as e:
            self.get_logger().error(f"❌ 完成訂單時發生錯誤: {e}")
            
        return response

    def _fetch_new_order(self) -> Optional[Dict[str, Any]]:
        """從 Web API 獲取新訂單"""
        try:
            # 檢查是否有待處理的處方籤
            response = requests.get(f"{self.web_api_url}/api/prescription/", timeout=5)
            if response.status_code != 200:
                return None
                
            prescriptions = response.json()
            pending_prescriptions = [p for p in prescriptions if p.get('status') == 'pending']
            
            if not pending_prescriptions:
                return None
                
            # 獲取最早的處方籤
            earliest = min(pending_prescriptions, key=lambda p: p.get('id'))
            prescription_id = earliest['id']
            
            # 更新處方籤狀態為處理中
            update_response = requests.put(
                f"{self.web_api_url}/api/prescription/{prescription_id}/status",
                json={"status": "processing"},
                timeout=5
            )
            
            if update_response.status_code != 200:
                self.get_logger().warn(f"⚠️ 無法更新處方籤 {prescription_id} 狀態")
                return None
            
            # 轉換為訂單格式
            order_data = {
                "order_id": f"{prescription_id:06d}",
                "prescription_id": prescription_id,
                "patient_name": earliest.get('patient_name', ''),
                "medicines": []
            }
            
            # 解析藥物列表
            medicines_data = earliest.get('medicines', [])
            for med in medicines_data:
                medicine_info = {
                    "name": med.get('name', ''),
                    "amount": med.get('amount', 0),
                    "locate": self._get_medicine_location(med.get('name', '')),
                    "prompt": self._get_medicine_prompt(med.get('name', ''))
                }
                order_data["medicines"].append(medicine_info)
            
            return order_data
            
        except Exception as e:
            self.get_logger().error(f"❌ 獲取新訂單失敗: {e}")
            return None

    def _fetch_medicine_detail(self, medicine_name: str) -> Optional[Dict[str, Any]]:
        """查詢藥物詳細資訊"""
        try:
            # 查詢藥物詳細資訊
            response = requests.get(f"{self.web_api_url}/api/medicine/detailed", timeout=5)
            if response.status_code != 200:
                return None
                
            medicines = response.json()
            
            # 查找指定藥物
            for med in medicines:
                if med.get('name') == medicine_name:
                    # 只返回藥物描述（按您的要求）
                    return {
                        "name": medicine_name,
                        "description": med.get('description', ''),  # 藥物詳細資料
                        "found": True
                    }
            
            return {
                "name": medicine_name,
                "found": False,
                "error": "藥物未找到"
            }
            
        except Exception as e:
            self.get_logger().error(f"❌ 查詢藥物詳細資訊失敗: {e}")
            return None

    def _notify_order_complete(self, order_id: str) -> bool:
        """通知 Web 系統訂單完成"""
        try:
            prescription_id = int(order_id)
            response = requests.put(
                f"{self.web_api_url}/api/prescription/{prescription_id}/status",
                json={"status": "completed"},
                timeout=5
            )
            return response.status_code == 200
            
        except Exception as e:
            self.get_logger().error(f"❌ 通知訂單完成失敗: {e}")
            return False

    def _get_medicine_location(self, medicine_name: str) -> list:
        """獲取藥物位置（模擬）"""
        # 簡單的雜湊函數來生成一致的位置
        hash_value = hash(medicine_name) % 100
        row = (hash_value // 10) + 1
        col = (hash_value % 10) + 1
        return [row, col]

    def _get_medicine_prompt(self, medicine_name: str) -> str:
        """獲取藥物類型提示（模擬）"""
        # 簡單的類型判斷
        name_lower = medicine_name.lower()
        if 'tablet' in name_lower or '片' in medicine_name:
            return 'tablet'
        elif 'capsule' in name_lower or '膠囊' in medicine_name:
            return 'capsule'
        elif 'box' in name_lower or '盒' in medicine_name:
            return 'white_circle_box'
        else:
            return 'tablet'  # 預設

    def _format_order_yaml(self, order_data: Dict[str, Any]) -> str:
        """將訂單數據格式化為 YAML"""
        try:
            yaml_content = f"""order_id: "{order_data['order_id']}"
prescription_id: {order_data.get('prescription_id', '')}
patient_name: "{order_data.get('patient_name', '')}"
medicine:
"""
            
            for med in order_data.get('medicines', []):
                yaml_content += f"""  - name: {med.get('name', '')}
    amount: {med.get('amount', 0)}
    locate: {med.get('locate', [1, 1])}
    prompt: {med.get('prompt', 'tablet')}
"""
            
            return yaml_content.rstrip()
            
        except Exception as e:
            self.get_logger().error(f"格式化訂單 YAML 失敗: {e}")
            return yaml.dump(order_data, default_flow_style=False, allow_unicode=True)

    def _format_medicine_yaml(self, medicine_data: Dict[str, Any]) -> str:
        """將藥物詳細資訊格式化為 YAML"""
        try:
            if not medicine_data.get('found', False):
                return yaml.dump(medicine_data, default_flow_style=False, allow_unicode=True)
            
            yaml_content = f"""name: {medicine_data.get('name', '')}
description: "{medicine_data.get('description', '')}"
found: true
"""
            return yaml_content.rstrip()
            
        except Exception as e:
            self.get_logger().error(f"格式化藥物 YAML 失敗: {e}")
            return yaml.dump(medicine_data, default_flow_style=False, allow_unicode=True)


def main():
    """主函數"""
    rclpy.init()
    
    try:
        node = HospitalROS2Services()
        
        print("🏥 醫院 ROS2 服務節點已啟動")
        print("=" * 50)
        print("📋 可用服務:")
        print("   ros2 service call /hospital/get_order std_srvs/srv/Empty")
        print("   ros2 service call /hospital/complete_order std_srvs/srv/Empty")
        print("")
        print("📡 Topic 使用方式:")
        print("   # 查詢藥物詳細資訊")
        print("   ros2 topic pub /hospital/medicine_request std_msgs/msg/String 'data: \"藥物名稱\"'")
        print("")
        print("   # 訂閱訂單數據")
        print("   ros2 topic echo /hospital/order_data")
        print("")
        print("   # 訂閱藥物詳細資訊")
        print("   ros2 topic echo /hospital/medicine_data")
        print("")
        print("   # 訂閱系統狀態")
        print("   ros2 topic echo /hospital/status")
        print("")
        print("🛑 按 Ctrl+C 停止服務")
        print("=" * 50)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 正在停止 ROS2 服務...")
    except Exception as e:
        print(f"❌ ROS2 服務發生錯誤: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("✅ ROS2 服務已停止")


if __name__ == '__main__':
    main()
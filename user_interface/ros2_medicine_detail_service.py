#!/usr/bin/env python3
"""
ROS2 藥物詳細資料服務
提供藥物詳細資訊查詢的專用 ROS2 服務
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
import yaml
import requests
import logging
from typing import Dict, Any, Optional

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("medicine_detail_service")

class MedicineDetailService(Node):
    """藥物詳細資料服務節點"""
    
    def __init__(self):
        super().__init__('medicine_detail_service')
        
        # 配置
        self.web_api_url = "http://localhost:8001"
        
        # 創建發布者 - 用於發送詳細資料
        self.detail_publisher = self.create_publisher(String, 'medicine/detail_response', 10)
        
        # 創建訂閱者 - 用於接收查詢請求
        self.request_sub = self.create_subscription(
            String,
            'medicine/detail_request',
            self.detail_request_callback,
            10
        )
        
        # 創建服務 - 同步查詢方式
        self.detail_service = self.create_service(
            Empty,
            'medicine/get_detail',
            self.get_detail_service_callback
        )
        
        # 儲存最後一次查詢的結果
        self.last_query_name = ""
        self.last_query_result = None
        
        self.get_logger().info("💊 藥物詳細資料服務已啟動")
        self.get_logger().info("📡 Topic:")
        self.get_logger().info("   • medicine/detail_request - 發送藥物名稱查詢")
        self.get_logger().info("   • medicine/detail_response - 接收詳細資料回應")
        self.get_logger().info("🔧 Service:")
        self.get_logger().info("   • medicine/get_detail - 同步查詢服務")
        self.get_logger().info("")
        self.get_logger().info("📋 使用方式:")
        self.get_logger().info("   # Topic 方式:")
        self.get_logger().info("   ros2 topic pub /medicine/detail_request std_msgs/msg/String 'data: \"藥物名稱\"'")
        self.get_logger().info("   ros2 topic echo /medicine/detail_response")
        self.get_logger().info("")
        self.get_logger().info("   # Service 方式 (需要先設置查詢名稱):")
        self.get_logger().info("   ros2 service call /medicine/get_detail std_srvs/srv/Empty")

    def detail_request_callback(self, msg):
        """處理藥物詳細資料查詢請求"""
        try:
            medicine_name = msg.data.strip()
            if not medicine_name:
                self.get_logger().warn("⚠️ 收到空的藥物名稱")
                return
                
            self.get_logger().info(f"💊 收到藥物詳細資料查詢: {medicine_name}")
            
            # 儲存查詢資訊供服務使用
            self.last_query_name = medicine_name
            
            # 查詢藥物詳細資訊
            detail_data = self._fetch_medicine_detail(medicine_name)
            
            if detail_data:
                # 格式化為 YAML 並發布
                detail_yaml = self._format_medicine_detail_yaml(detail_data, medicine_name)
                
                detail_msg = String()
                detail_msg.data = detail_yaml
                self.detail_publisher.publish(detail_msg)
                
                # 儲存結果供服務使用
                self.last_query_result = detail_yaml
                
                self.get_logger().info(f"✅ 已發送 {medicine_name} 的詳細資訊")
                
            else:
                # 發送錯誤訊息
                error_yaml = f"""name: {medicine_name}
found: false
error: "藥物未找到"
description: ""
"""
                
                error_msg = String()
                error_msg.data = error_yaml
                self.detail_publisher.publish(error_msg)
                
                # 儲存錯誤結果
                self.last_query_result = error_yaml
                
                self.get_logger().warn(f"⚠️ 未找到藥物: {medicine_name}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 處理藥物查詢時發生錯誤: {e}")

    def get_detail_service_callback(self, request, response):
        """同步查詢服務回調"""
        try:
            if not self.last_query_name:
                self.get_logger().warn("⚠️ 沒有設置查詢的藥物名稱")
                self.get_logger().warn("   請先使用 Topic 發送查詢請求")
                return response
                
            self.get_logger().info(f"🔧 服務查詢: {self.last_query_name}")
            
            # 重新查詢以確保是最新資料
            detail_data = self._fetch_medicine_detail(self.last_query_name)
            
            if detail_data:
                detail_yaml = self._format_medicine_detail_yaml(detail_data, self.last_query_name)
                
                # 發布結果
                detail_msg = String()
                detail_msg.data = detail_yaml
                self.detail_publisher.publish(detail_msg)
                
                self.last_query_result = detail_yaml
                
                self.get_logger().info(f"✅ 服務回應: {self.last_query_name} 詳細資訊已發送")
            else:
                error_yaml = f"""name: {self.last_query_name}
found: false
error: "藥物未找到"
description: ""
"""
                error_msg = String()
                error_msg.data = error_yaml
                self.detail_publisher.publish(error_msg)
                
                self.last_query_result = error_yaml
                
                self.get_logger().warn(f"⚠️ 服務回應: 未找到藥物 {self.last_query_name}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 服務查詢時發生錯誤: {e}")
            
        return response

    def _fetch_medicine_detail(self, medicine_name: str) -> Optional[Dict[str, Any]]:
        """從 Web API 查詢藥物詳細資訊"""
        try:
            # 查詢藥物詳細資訊
            response = requests.get(f"{self.web_api_url}/api/medicine/detailed", timeout=5)
            if response.status_code != 200:
                self.get_logger().error(f"❌ Web API 回應錯誤: {response.status_code}")
                return None
                
            medicines = response.json()
            
            # 查找指定藥物 (支援模糊匹配)
            exact_match = None
            partial_matches = []
            
            for med in medicines:
                med_name = med.get('name', '')
                if med_name == medicine_name:
                    # 精確匹配
                    exact_match = med
                    break
                elif medicine_name.lower() in med_name.lower() or med_name.lower() in medicine_name.lower():
                    # 部分匹配
                    partial_matches.append(med)
            
            if exact_match:
                self.get_logger().info(f"🎯 找到精確匹配: {exact_match.get('name')}")
                return exact_match
            elif partial_matches:
                # 使用第一個部分匹配
                match = partial_matches[0]
                self.get_logger().info(f"🔍 找到部分匹配: {match.get('name')} (查詢: {medicine_name})")
                return match
            else:
                self.get_logger().warn(f"❌ 未找到匹配的藥物: {medicine_name}")
                return None
            
        except Exception as e:
            self.get_logger().error(f"❌ 查詢藥物詳細資訊失敗: {e}")
            return None

    def _format_medicine_detail_yaml(self, medicine_data: Dict[str, Any], query_name: str) -> str:
        """將藥物詳細資訊格式化為 YAML"""
        try:
            name = medicine_data.get('name', query_name)
            description = medicine_data.get('description', '')
            
            # 獲取其他可能的欄位
            category = medicine_data.get('category', '')
            unit_dose = medicine_data.get('unit_dose', '')
            stock_quantity = medicine_data.get('stock_quantity', 0)
            
            yaml_content = f"""name: {name}
found: true
description: "{description}"
category: "{category}"
unit_dose: "{unit_dose}"
stock_quantity: {stock_quantity}
query_name: "{query_name}"
"""
            
            # 如果有額外資訊，也包含進去
            if 'manufacturer' in medicine_data:
                yaml_content += f'manufacturer: "{medicine_data["manufacturer"]}"\n'
            
            if 'expiry_date' in medicine_data:
                yaml_content += f'expiry_date: "{medicine_data["expiry_date"]}"\n'
                
            return yaml_content.rstrip()
            
        except Exception as e:
            self.get_logger().error(f"❌ 格式化藥物 YAML 失敗: {e}")
            # 返回基本格式
            return f"""name: {query_name}
found: false
error: "格式化失敗"
description: ""
"""

    def get_available_medicines(self) -> list:
        """獲取所有可用的藥物列表"""
        try:
            response = requests.get(f"{self.web_api_url}/api/medicine/detailed", timeout=5)
            if response.status_code == 200:
                medicines = response.json()
                return [med.get('name', '') for med in medicines if med.get('name')]
            else:
                return []
        except Exception as e:
            self.get_logger().error(f"❌ 獲取藥物列表失敗: {e}")
            return []

    def list_medicines(self):
        """列出所有可用的藥物"""
        medicines = self.get_available_medicines()
        if medicines:
            self.get_logger().info("📋 可用的藥物:")
            for i, med in enumerate(medicines, 1):
                self.get_logger().info(f"   {i}. {med}")
        else:
            self.get_logger().warn("⚠️ 沒有找到可用的藥物")


def main():
    """主函數"""
    rclpy.init()
    
    try:
        service = MedicineDetailService()
        
        print("💊 藥物詳細資料 ROS2 服務已啟動")
        print("=" * 60)
        print("📡 使用方式:")
        print()
        print("1️⃣ Topic 方式 (推薦):")
        print("   # 查詢藥物詳細資料")
        print("   ros2 topic pub /medicine/detail_request std_msgs/msg/String 'data: \"阿斯匹靈\"'")
        print()
        print("   # 接收詳細資料")
        print("   ros2 topic echo /medicine/detail_response")
        print()
        print("2️⃣ Service 方式:")
        print("   # 先設置要查詢的藥物 (使用 Topic)")
        print("   ros2 topic pub /medicine/detail_request std_msgs/msg/String 'data: \"維他命C\"'")
        print()
        print("   # 然後調用服務")
        print("   ros2 service call /medicine/get_detail std_srvs/srv/Empty")
        print()
        print("3️⃣ Python 程式使用:")
        print("""
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   
   class MedicineClient(Node):
       def __init__(self):
           super().__init__('medicine_client')
           self.pub = self.create_publisher(String, 'medicine/detail_request', 10)
           self.sub = self.create_subscription(String, 'medicine/detail_response', 
                                              self.response_callback, 10)
       
       def query_medicine(self, name):
           msg = String()
           msg.data = name
           self.pub.publish(msg)
       
       def response_callback(self, msg):
           print("收到藥物詳細資料:")
           print(msg.data)
""")
        print("=" * 60)
        print("🛑 按 Ctrl+C 停止服務")
        
        # 啟動時列出可用藥物
        service.list_medicines()
        
        rclpy.spin(service)
        
    except KeyboardInterrupt:
        print("\n🛑 正在停止藥物詳細資料服務...")
    except Exception as e:
        print(f"❌ 服務發生錯誤: {e}")
    finally:
        try:
            service.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("✅ 藥物詳細資料服務已停止")


if __name__ == '__main__':
    main()
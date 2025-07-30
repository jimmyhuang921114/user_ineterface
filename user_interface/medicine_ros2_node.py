#!/usr/bin/env python3
"""
ROS2 醫院藥物管理服務節點
Hospital Medicine Management ROS2 Service Node
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import requests
import json
from datetime import datetime

class MedicineServiceNode(Node):
    def __init__(self):
        super().__init__('medicine_service_node')
        
        # 配置
        self.api_base = "http://localhost:8000"
        
        # 創建服務
        self.medicine_service = self.create_service(
            Empty, 
            'get_all_medicines', 
            self.get_all_medicines_callback
        )
        
        self.get_logger().info('醫院藥物管理ROS2服務已啟動')
    
    def get_all_medicines_callback(self, request, response):
        """獲取所有藥物資訊的服務回調"""
        try:
            # 獲取基本藥物
            basic_response = requests.get(f"{self.api_base}/api/medicine/")
            basic_medicines = basic_response.json() if basic_response.status_code == 200 else []
            
            # 獲取詳細藥物
            detailed_response = requests.get(f"{self.api_base}/api/medicine/detailed/")
            detailed_medicines = detailed_response.json() if detailed_response.status_code == 200 else {}
            
            # 組合資料
            result = {
                "timestamp": datetime.now().isoformat(),
                "basic_medicines": basic_medicines,
                "detailed_medicines": detailed_medicines,
                "total_basic": len(basic_medicines),
                "total_detailed": len(detailed_medicines)
            }
            
            self.get_logger().info(f'成功獲取藥物資訊: {len(basic_medicines)} 基本, {len(detailed_medicines)} 詳細')
            
            # 保存到文件供其他ROS2節點使用
            with open('/tmp/medicine_data_ros2.json', 'w', encoding='utf-8') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'獲取藥物資訊失敗: {str(e)}')
            return response

def main(args=None):
    rclpy.init(args=args)
    node = MedicineServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

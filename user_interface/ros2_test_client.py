#!/usr/bin/env python3
"""
ROS2 Test Client
ROS2測試客戶端
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import time
from typing import Dict, List

class MedicineOrderProcessor(Node):
    """藥物訂單處理節點"""
    
    def __init__(self):
        super().__init__('medicine_order_processor')
        
        # 訂閱藥物資料
        self.medicine_subscriber = self.create_subscription(
            String,
            'medicine_data',
            self.medicine_data_callback,
            10
        )
        
        # 訂閱訂單狀態
        self.order_status_subscriber = self.create_subscription(
            String,
            'order_status',
            self.order_status_callback,
            10
        )
        
        # 發布訂單狀態
        self.status_publisher = self.create_publisher(
            String,
            'order_status',
            10
        )
        
        # 服務器 - 處理藥物訂單
        self.order_service = self.create_service(
            Trigger,
            'process_medicine_order',
            self.process_order_callback
        )
        
        self.get_logger().info('🤖 藥物訂單處理節點已啟動')
    
    def medicine_data_callback(self, msg: String):
        """藥物資料回調"""
        try:
            medicine_data = json.loads(msg.data)
            self.get_logger().info(f'💊 收到藥物資料: {medicine_data.get("name", "Unknown")}')
            
            # 這裡可以添加藥物資料處理邏輯
            # 例如：更新本地資料庫、記錄日誌等
            
        except json.JSONDecodeError:
            self.get_logger().error(f'❌ 無法解析藥物資料: {msg.data}')
    
    def order_status_callback(self, msg: String):
        """訂單狀態回調"""
        try:
            status_data = json.loads(msg.data)
            self.get_logger().info(f'📊 收到訂單狀態: {status_data}')
            
        except json.JSONDecodeError:
            self.get_logger().error(f'❌ 無法解析訂單狀態: {msg.data}')
    
    def process_order_callback(self, request, response):
        """處理訂單服務回調"""
        try:
            self.get_logger().info('🔄 開始處理藥物訂單')
            
            # 模擬處理時間
            time.sleep(2)
            
            # 發布處理完成狀態
            status_data = {
                "order_id": f"ORDER_{int(time.time())}",
                "status": "completed",
                "message": "訂單處理完成",
                "timestamp": time.time()
            }
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.status_publisher.publish(status_msg)
            
            response.success = True
            response.message = "訂單處理成功"
            
            self.get_logger().info('✅ 訂單處理完成')
            
        except Exception as e:
            self.get_logger().error(f'❌ 處理訂單時發生錯誤: {e}')
            response.success = False
            response.message = f"處理失敗: {str(e)}"
        
        return response

def main():
    """主函數"""
    rclpy.init()
    
    # 創建節點
    processor = MedicineOrderProcessor()
    
    try:
        print("🤖 藥物訂單處理節點已啟動")
        print("📡 等待訂單...")
        rclpy.spin(processor)
    except KeyboardInterrupt:
        print("\n🛑 節點已停止")
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
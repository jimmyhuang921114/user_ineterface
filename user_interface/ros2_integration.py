#!/usr/bin/env python3
"""
ROS2 Integration Module
ROS2整合模組
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String
import json
import time
import threading
from typing import Dict, List, Optional
from queue import Queue
import asyncio

class HospitalROS2Node(Node):
    """醫院ROS2節點"""
    
    def __init__(self):
        super().__init__('hospital_medicine_system')
        
        # 訂單處理狀態
        self.order_queue = Queue()
        self.current_order = None
        self.processing = False
        self.order_lock = threading.Lock()
        
        # 回調群組
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        
        # 服務客戶端
        self.order_service_client = self.create_client(
            Trigger, 
            'process_medicine_order',
            callback_group=self.service_group
        )
        
        # 訂閱者
        self.order_status_subscriber = self.create_subscription(
            String,
            'order_status',
            self.order_status_callback,
            10
        )
        
        # 發布者
        self.medicine_data_publisher = self.create_publisher(
            String,
            'medicine_data',
            10
        )
        
        # 定時器 - 處理訂單佇列
        self.order_timer = self.create_timer(
            2.0,  # 每2秒檢查一次
            self.process_order_queue,
            callback_group=self.timer_group
        )
        
        self.get_logger().info('🏥 醫院藥物管理ROS2節點已啟動')
    
    def add_order(self, order_data: Dict):
        """添加訂單到佇列"""
        with self.order_lock:
            self.order_queue.put(order_data)
            self.get_logger().info(f'📋 訂單已加入佇列: {order_data.get("order_id", "Unknown")}')
    
    def process_order_queue(self):
        """處理訂單佇列"""
        if self.processing:
            return
        
        with self.order_lock:
            if not self.order_queue.empty() and not self.processing:
                self.current_order = self.order_queue.get()
                self.processing = True
                self.get_logger().info(f'🔄 開始處理訂單: {self.current_order.get("order_id", "Unknown")}')
                
                # 在新線程中處理訂單
                threading.Thread(target=self.process_single_order, args=(self.current_order,)).start()
    
    def process_single_order(self, order_data: Dict):
        """處理單個訂單"""
        try:
            # 準備訂單資料
            order_msg = String()
            order_msg.data = json.dumps(order_data)
            
            # 發送訂單到ROS2服務
            self.get_logger().info(f'📤 發送訂單到ROS2服務: {order_data.get("order_id")}')
            
            # 這裡應該調用實際的ROS2服務
            # 目前使用模擬處理
            self.simulate_order_processing(order_data)
            
        except Exception as e:
            self.get_logger().error(f'❌ 處理訂單時發生錯誤: {e}')
            self.mark_order_completed()
    
    def simulate_order_processing(self, order_data: Dict):
        """模擬訂單處理（實際環境中會調用真實的ROS2服務）"""
        order_id = order_data.get("order_id")
        self.get_logger().info(f'🤖 模擬處理訂單: {order_id}')
        
        # 模擬處理時間
        time.sleep(3)
        
        # 模擬處理結果
        result = {
            "order_id": order_id,
            "status": "completed",
            "message": "訂單處理完成",
            "timestamp": time.time()
        }
        
        # 發布處理結果
        status_msg = String()
        status_msg.data = json.dumps(result)
        self.order_status_subscriber.get_logger().info(f'📢 發布訂單狀態: {result}')
        
        # 標記訂單完成
        self.mark_order_completed()
    
    def mark_order_completed(self):
        """標記訂單處理完成"""
        with self.order_lock:
            self.processing = False
            self.current_order = None
            self.get_logger().info('✅ 訂單處理完成，準備處理下一個訂單')
    
    def order_status_callback(self, msg: String):
        """訂單狀態回調"""
        try:
            status_data = json.loads(msg.data)
            self.get_logger().info(f'📊 收到訂單狀態: {status_data}')
            
            # 如果收到完成狀態，標記當前訂單完成
            if status_data.get("status") in ["completed", "failed"]:
                self.mark_order_completed()
                
        except json.JSONDecodeError:
            self.get_logger().error(f'❌ 無法解析訂單狀態: {msg.data}')
    
    def publish_medicine_data(self, medicine_data: Dict):
        """發布藥物資料"""
        try:
            msg = String()
            msg.data = json.dumps(medicine_data)
            self.medicine_data_publisher.publish(msg)
            self.get_logger().info(f'💊 發布藥物資料: {medicine_data.get("name", "Unknown")}')
        except Exception as e:
            self.get_logger().error(f'❌ 發布藥物資料時發生錯誤: {e}')
    
    def get_queue_status(self) -> Dict:
        """獲取佇列狀態"""
        with self.order_lock:
            return {
                "queue_size": self.order_queue.qsize(),
                "processing": self.processing,
                "current_order": self.current_order
            }

# 全域ROS2節點實例
ros2_node = None

def init_ros2_node():
    """初始化ROS2節點"""
    global ros2_node
    try:
        rclpy.init()
        ros2_node = HospitalROS2Node()
        return ros2_node
    except Exception as e:
        print(f"❌ ROS2節點初始化失敗: {e}")
        return None

def get_ros2_node() -> Optional[HospitalROS2Node]:
    """獲取ROS2節點實例"""
    return ros2_node

def spin_ros2_node():
    """運行ROS2節點"""
    if ros2_node:
        try:
            executor = MultiThreadedExecutor()
            executor.add_node(ros2_node)
            executor.spin()
        except KeyboardInterrupt:
            print("🛑 ROS2節點已停止")
        finally:
            ros2_node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    node = init_ros2_node()
    if node:
        spin_ros2_node()
#!/usr/bin/env python3
"""
ROS2 Auto Order Pusher
自動推送訂單給您的ROS2系統

此檔案會：
1. 定期檢查是否有新的處方籤待處理
2. 自動推送一筆訂單給您
3. 支援查詢基礎和詳細藥物資訊
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
import yaml
import time
import threading
from typing import Dict, Any, Optional

class AutoOrderPusher(Node):
    """
    自動訂單推送器
    - 定期從醫院系統拉取新訂單
    - 自動推送給您的ROS2系統
    - 提供藥物資訊查詢服務
    """
    
    def __init__(self):
        super().__init__('auto_order_pusher')
        
        # 醫院系統設定
        self.hospital_base_url = 'http://localhost:8001'
        self.check_interval = 5.0  # 每5秒檢查一次新訂單
        
        # ROS2 發布器
        self.order_publisher = self.create_publisher(
            String, 
            '/hospital/new_order', 
            10
        )
        
        # 藥物查詢發布器
        self.medicine_basic_pub = self.create_publisher(
            String,
            '/hospital/medicine_basic_info',
            10
        )
        
        self.medicine_detail_pub = self.create_publisher(
            String,
            '/hospital/medicine_detail_info', 
            10
        )
        
        # 狀態發布器
        self.status_publisher = self.create_publisher(
            String,
            '/hospital/pusher_status',
            10
        )
        
        # 定時器
        self.order_timer = self.create_timer(self.check_interval, self.check_and_push_order)
        
        # 狀態追蹤
        self.last_pushed_order_id = None
        self.is_running = True
        
        self.get_logger().info("🚀 自動訂單推送器已啟動")
        self.get_logger().info(f"📡 連接醫院系統: {self.hospital_base_url}")
        self.get_logger().info(f"⏱️ 檢查間隔: {self.check_interval} 秒")
        
        # 發布啟動狀態
        self.publish_status("啟動", "自動訂單推送器已準備就緒")
    
    def check_and_push_order(self):
        """檢查並推送新訂單"""
        try:
            # 檢查系統狀態
            response = requests.get(f"{self.hospital_base_url}/api/system/status", timeout=3)
            if response.status_code != 200:
                self.get_logger().warn("醫院系統不可用")
                return
            
            # 拉取下一個訂單
            response = requests.get(f"{self.hospital_base_url}/api/ros2/order/next", timeout=5)
            
            if response.status_code == 204:
                # 沒有新訂單
                return
            elif response.status_code == 200:
                # 有新訂單
                order_data = response.json()
                order = order_data['order']
                order_yaml = order_data['yaml']
                
                # 檢查是否為新訂單
                if order['order_id'] != self.last_pushed_order_id:
                    self.push_order_to_ros2(order, order_yaml)
                    self.last_pushed_order_id = order['order_id']
                    
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"網路錯誤: {e}")
        except Exception as e:
            self.get_logger().error(f"推送訂單時發生錯誤: {e}")
    
    def push_order_to_ros2(self, order: Dict[str, Any], order_yaml: str):
        """推送訂單到ROS2"""
        order_id = order['order_id']
        patient_name = order['patient_name']
        medicine_count = len(order['medicine'])
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📦 推送新訂單: {order_id}")
        self.get_logger().info(f"👤 病患: {patient_name}")
        self.get_logger().info(f"💊 藥物數量: {medicine_count}")
        
        # 顯示藥物詳情
        for i, med in enumerate(order['medicine'], 1):
            self.get_logger().info(f"  {i}. {med['name']} - 位置:{med['position']} - 數量:{med['amount']}")
        
        self.get_logger().info("=" * 60)
        
        # 發布到ROS2 Topic
        msg = String()
        msg.data = order_yaml
        self.order_publisher.publish(msg)
        
        # 發布狀態
        self.publish_status("推送訂單", f"已推送訂單 {order_id} ({patient_name})")
        
        self.get_logger().info(f"✅ 訂單 {order_id} 已推送到 /hospital/new_order topic")
    
    def publish_status(self, stage: str, message: str):
        """發布狀態資訊"""
        status_msg = String()
        status_data = {
            "timestamp": time.time(),
            "stage": stage,
            "message": message,
            "node": "auto_order_pusher"
        }
        status_msg.data = yaml.safe_dump(status_data, allow_unicode=True)
        self.status_publisher.publish(status_msg)
    
    def query_medicine_basic(self, medicine_name: str) -> Optional[Dict[str, Any]]:
        """查詢基礎藥物資訊"""
        try:
            response = requests.get(
                f"{self.hospital_base_url}/api/ros2/medicine/basic/{medicine_name}",
                timeout=3
            )
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().warn(f"找不到藥物: {medicine_name}")
                return None
        except Exception as e:
            self.get_logger().error(f"查詢基礎藥物資訊錯誤: {e}")
            return None
    
    def query_medicine_detail(self, medicine_name: str) -> Optional[Dict[str, Any]]:
        """查詢詳細藥物資訊"""
        try:
            response = requests.get(
                f"{self.hospital_base_url}/api/ros2/medicine/detailed/{medicine_name}",
                timeout=3
            )
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().warn(f"找不到藥物詳細資訊: {medicine_name}")
                return None
        except Exception as e:
            self.get_logger().error(f"查詢詳細藥物資訊錯誤: {e}")
            return None
    
    def publish_medicine_basic_info(self, medicine_name: str):
        """發布基礎藥物資訊到ROS2"""
        info = self.query_medicine_basic(medicine_name)
        if info:
            msg = String()
            msg.data = info['yaml']
            self.medicine_basic_pub.publish(msg)
            self.get_logger().info(f"📊 已發布 {medicine_name} 基礎資訊到 /hospital/medicine_basic_info")
        else:
            self.get_logger().warn(f"❌ 無法取得 {medicine_name} 基礎資訊")
    
    def publish_medicine_detail_info(self, medicine_name: str):
        """發布詳細藥物資訊到ROS2"""
        info = self.query_medicine_detail(medicine_name)
        if info:
            msg = String()
            msg.data = info['yaml']
            self.medicine_detail_pub.publish(msg)
            self.get_logger().info(f"📝 已發布 {medicine_name} 詳細資訊到 /hospital/medicine_detail_info")
        else:
            self.get_logger().warn(f"❌ 無法取得 {medicine_name} 詳細資訊")
    
    def complete_order(self, order_id: str, status: str = "success", details: str = ""):
        """回報訂單完成"""
        try:
            payload = {
                "order_id": order_id,
                "status": status,
                "details": details
            }
            response = requests.post(
                f"{self.hospital_base_url}/api/ros2/order/complete",
                json=payload,
                timeout=5
            )
            if response.status_code == 200:
                self.get_logger().info(f"✅ 訂單 {order_id} 完成狀態已回報")
                self.publish_status("完成訂單", f"訂單 {order_id} 已完成")
                return True
            else:
                self.get_logger().error(f"❌ 回報訂單完成失敗: {response.status_code}")
                return False
        except Exception as e:
            self.get_logger().error(f"回報訂單完成時發生錯誤: {e}")
            return False
    
    def report_progress(self, order_id: str, stage: str, message: str):
        """回報訂單進度"""
        try:
            payload = {
                "order_id": order_id,
                "stage": stage,
                "message": message
            }
            response = requests.post(
                f"{self.hospital_base_url}/api/ros2/order/progress",
                json=payload,
                timeout=3
            )
            if response.status_code == 200:
                self.get_logger().info(f"📈 訂單 {order_id} 進度已回報: {stage} - {message}")
        except Exception as e:
            self.get_logger().debug(f"回報進度時發生錯誤: {e}")


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    
    pusher = AutoOrderPusher()
    
    print("\n" + "="*60)
    print("🏥 醫院自動訂單推送器")
    print("="*60)
    print("📡 ROS2 Topics:")
    print("  - 訂單推送: /hospital/new_order")
    print("  - 基礎藥物: /hospital/medicine_basic_info")
    print("  - 詳細藥物: /hospital/medicine_detail_info")
    print("  - 狀態更新: /hospital/pusher_status")
    print()
    print("🔧 使用範例:")
    print("  # 在另一個終端監聽訂單")
    print("  ros2 topic echo /hospital/new_order")
    print()
    print("  # 監聽基礎藥物資訊")
    print("  ros2 topic echo /hospital/medicine_basic_info")
    print()
    print("  # 監聽詳細藥物資訊")
    print("  ros2 topic echo /hospital/medicine_detail_info")
    print()
    print("💡 推送器會自動檢查新訂單並推送給您")
    print("🔄 請確保醫院系統正在運行 (http://localhost:8001)")
    print("="*60)
    
    try:
        rclpy.spin(pusher)
    except KeyboardInterrupt:
        pusher.get_logger().info("🛑 推送器已停止")
    finally:
        pusher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
醫院藥物管理系統 - ROS2 服務接口
提供三個專門的服務：訂單、基本藥物、詳細藥物
"""

import requests
import json
import time
import threading
from typing import Dict, List, Optional, Any
from datetime import datetime

# 嘗試導入 ROS2
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
    print("✅ ROS2 環境可用")
except ImportError:
    ROS2_AVAILABLE = False
    print("⚠️ ROS2 不可用，將使用 HTTP 模式")
    Node = object  # 佔位符


class OrderServiceInterface:
    """訂單服務接口 - 每筆訂單結束後再送下一筆"""
    
    def __init__(self, base_url="http://localhost:8001"):
        self.base_url = base_url
        self.current_order = None
        self.order_queue = []
        self.processing = False
        self.use_ros2 = ROS2_AVAILABLE
        
        # ROS2 設置
        if self.use_ros2:
            self.node = None
            self.order_publisher = None
            self.order_subscriber = None
            self._init_ros2()
        
        print("📦 訂單服務接口初始化完成")
    
    def _init_ros2(self):
        """初始化 ROS2"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = rclpy.create_node('order_service_interface')
            
            # 發布器 - 發送訂單請求
            self.order_publisher = self.node.create_publisher(
                String, 'hospital/order_request', 10
            )
            
            # 訂閱器 - 接收訂單完成通知
            self.order_subscriber = self.node.create_subscription(
                String, 'hospital/order_completed', 
                self._order_completed_callback, 10
            )
            
            # 在後台執行 ROS2
            self.ros2_thread = threading.Thread(target=self._ros2_spin, daemon=True)
            self.ros2_thread.start()
            
            print("🤖 訂單服務 ROS2 節點啟動成功")
            
        except Exception as e:
            print(f"❌ ROS2 初始化失敗: {e}")
            self.use_ros2 = False
    
    def _ros2_spin(self):
        """ROS2 事件循環"""
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS2 事件循環錯誤: {e}")
    
    def _order_completed_callback(self, msg):
        """訂單完成回調"""
        try:
            data = json.loads(msg.data)
            order_id = data.get('order_id')
            print(f"📦 訂單完成通知: {order_id}")
            
            # 標記當前訂單完成
            if self.current_order:
                print(f"✅ 訂單 {self.current_order.get('order_id')} 已完成")
                self.current_order = None
                self.processing = False
                
                # 處理下一筆訂單
                self._process_next_order()
                
        except Exception as e:
            print(f"❌ 處理訂單完成通知失敗: {e}")
    
    def send_order(self, medicines: List[Dict[str, Any]], patient_info: Dict[str, str] = None):
        """
        發送訂單
        
        Args:
            medicines: 藥物列表 [{"name": "藥物名", "quantity": 數量}, ...]
            patient_info: 患者資訊 {"patient_name": "姓名", "doctor_name": "醫生"}
        """
        order_data = {
            "order_id": f"ORDER_{int(time.time() * 1000) % 1000000:06d}",
            "medicines": medicines,
            "patient_info": patient_info or {},
            "timestamp": datetime.now().isoformat(),
            "status": "pending"
        }
        
        # 加入訂單佇列
        self.order_queue.append(order_data)
        print(f"📦 訂單已加入佇列: {order_data['order_id']} ({len(medicines)} 種藥物)")
        
        # 如果沒有正在處理的訂單，立即處理
        if not self.processing:
            self._process_next_order()
        else:
            print(f"⏳ 訂單排隊中，當前正在處理: {self.current_order.get('order_id') if self.current_order else 'Unknown'}")
    
    def _process_next_order(self):
        """處理下一筆訂單"""
        if self.order_queue and not self.processing:
            self.current_order = self.order_queue.pop(0)
            self.processing = True
            
            print(f"🚀 開始處理訂單: {self.current_order['order_id']}")
            
            if self.use_ros2:
                # ROS2 方式
                msg = String()
                msg.data = json.dumps(self.current_order)
                self.order_publisher.publish(msg)
                print(f"🤖 [ROS2] 訂單已發送: {self.current_order['order_id']}")
            else:
                # HTTP 方式
                self._send_order_http()
    
    def _send_order_http(self):
        """HTTP 方式發送訂單"""
        try:
            # 創建處方籤
            prescription_data = {
                "patient_name": self.current_order["patient_info"].get("patient_name", "ROS2患者"),
                "doctor_name": self.current_order["patient_info"].get("doctor_name", "ROS2醫生"),
                "medicines": [
                    {
                        "name": med["name"],
                        "quantity": med["quantity"],
                        "dosage": med.get("dosage", ""),
                        "frequency": med.get("frequency", ""),
                        "duration": med.get("duration", "7天")
                    }
                    for med in self.current_order["medicines"]
                ]
            }
            
            response = requests.post(
                f"{self.base_url}/api/prescription/",
                json=prescription_data,
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                prescription_id = result.get('id')
                print(f"🌐 [HTTP] 處方籤已創建: {prescription_id}")
                
                # 模擬處理完成
                time.sleep(2)  # 模擬處理時間
                # 創建簡單的數據結構來模擬回調
                mock_msg = type('MockString', (), {'data': json.dumps({
                    "order_id": self.current_order["order_id"],
                    "prescription_id": prescription_id
                })})()
                self._order_completed_callback(mock_msg)
            else:
                print(f"❌ [HTTP] 處方籤創建失敗: {response.status_code}")
                self.processing = False
                
        except Exception as e:
            print(f"❌ HTTP 訂單處理失敗: {e}")
            self.processing = False
    
    def get_order_status(self):
        """獲取訂單狀態"""
        return {
            "current_order": self.current_order,
            "queue_length": len(self.order_queue),
            "processing": self.processing
        }


class BasicMedicineServiceInterface:
    """基本藥物服務接口 - 持續獲取基本藥物資訊"""
    
    def __init__(self, base_url="http://localhost:8001"):
        self.base_url = base_url
        self.use_ros2 = ROS2_AVAILABLE
        self.running = False
        self.request_queue = []
        
        # ROS2 設置
        if self.use_ros2:
            self.node = None
            self.request_publisher = None
            self.response_subscriber = None
            self._init_ros2()
        
        print("💊 基本藥物服務接口初始化完成")
    
    def _init_ros2(self):
        """初始化 ROS2"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = rclpy.create_node('basic_medicine_service_interface')
            
            # 發布器 - 發送藥物查詢請求
            self.request_publisher = self.node.create_publisher(
                String, 'hospital/basic_medicine_request', 10
            )
            
            # 訂閱器 - 接收藥物資訊響應
            self.response_subscriber = self.node.create_subscription(
                String, 'hospital/basic_medicine_response', 
                self._medicine_response_callback, 10
            )
            
            # 在後台執行 ROS2
            self.ros2_thread = threading.Thread(target=self._ros2_spin, daemon=True)
            self.ros2_thread.start()
            
            print("🤖 基本藥物服務 ROS2 節點啟動成功")
            
        except Exception as e:
            print(f"❌ ROS2 初始化失敗: {e}")
            self.use_ros2 = False
    
    def _ros2_spin(self):
        """ROS2 事件循環"""
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS2 事件循環錯誤: {e}")
    
    def _medicine_response_callback(self, msg):
        """藥物資訊響應回調"""
        try:
            data = json.loads(msg.data)
            print(f"💊 收到基本藥物資訊: {data.get('medicine_name', 'Unknown')}")
            # 這裡可以添加回調處理邏輯
            
        except Exception as e:
            print(f"❌ 處理藥物響應失敗: {e}")
    
    def query_medicine(self, medicine_name: str):
        """
        查詢基本藥物資訊
        
        Args:
            medicine_name: 藥物名稱
        """
        query_data = {
            "medicine_name": medicine_name,
            "query_type": "basic",
            "timestamp": datetime.now().isoformat()
        }
        
        if self.use_ros2:
            # ROS2 方式
            msg = String()
            msg.data = json.dumps(query_data)
            self.request_publisher.publish(msg)
            print(f"🤖 [ROS2] 基本藥物查詢已發送: {medicine_name}")
        else:
            # HTTP 方式
            return self._query_medicine_http(medicine_name)
    
    def _query_medicine_http(self, medicine_name: str):
        """HTTP 方式查詢基本藥物"""
        try:
            response = requests.post(
                f"{self.base_url}/api/ros2/service/basic-medicine",
                json={"medicine_name": medicine_name},
                timeout=5
            )
            
            if response.status_code == 200:
                result = response.json()
                print(f"🌐 [HTTP] 基本藥物查詢成功: {medicine_name}")
                return result
            else:
                print(f"❌ [HTTP] 基本藥物查詢失敗: {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ HTTP 基本藥物查詢失敗: {e}")
            return None
    
    def start_continuous_service(self):
        """啟動持續服務模式"""
        self.running = True
        print("🔄 基本藥物持續服務已啟動")
    
    def stop_continuous_service(self):
        """停止持續服務模式"""
        self.running = False
        print("⏹️ 基本藥物持續服務已停止")


class DetailedMedicineServiceInterface:
    """詳細藥物服務接口 - 持續獲取詳細藥物資訊"""
    
    def __init__(self, base_url="http://localhost:8001"):
        self.base_url = base_url
        self.use_ros2 = ROS2_AVAILABLE
        self.running = False
        self.request_queue = []
        
        # ROS2 設置
        if self.use_ros2:
            self.node = None
            self.request_publisher = None
            self.response_subscriber = None
            self._init_ros2()
        
        print("🔬 詳細藥物服務接口初始化完成")
    
    def _init_ros2(self):
        """初始化 ROS2"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = rclpy.create_node('detailed_medicine_service_interface')
            
            # 發布器 - 發送藥物查詢請求
            self.request_publisher = self.node.create_publisher(
                String, 'hospital/detailed_medicine_request', 10
            )
            
            # 訂閱器 - 接收藥物資訊響應
            self.response_subscriber = self.node.create_subscription(
                String, 'hospital/detailed_medicine_response', 
                self._medicine_response_callback, 10
            )
            
            # 在後台執行 ROS2
            self.ros2_thread = threading.Thread(target=self._ros2_spin, daemon=True)
            self.ros2_thread.start()
            
            print("🤖 詳細藥物服務 ROS2 節點啟動成功")
            
        except Exception as e:
            print(f"❌ ROS2 初始化失敗: {e}")
            self.use_ros2 = False
    
    def _ros2_spin(self):
        """ROS2 事件循環"""
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS2 事件循環錯誤: {e}")
    
    def _medicine_response_callback(self, msg):
        """藥物資訊響應回調"""
        try:
            data = json.loads(msg.data)
            print(f"🔬 收到詳細藥物資訊: {data.get('medicine_name', 'Unknown')}")
            # 這裡可以添加回調處理邏輯
            
        except Exception as e:
            print(f"❌ 處理藥物響應失敗: {e}")
    
    def query_medicine(self, medicine_name: str):
        """
        查詢詳細藥物資訊
        
        Args:
            medicine_name: 藥物名稱
        """
        query_data = {
            "medicine_name": medicine_name,
            "query_type": "detailed",
            "timestamp": datetime.now().isoformat()
        }
        
        if self.use_ros2:
            # ROS2 方式
            msg = String()
            msg.data = json.dumps(query_data)
            self.request_publisher.publish(msg)
            print(f"🤖 [ROS2] 詳細藥物查詢已發送: {medicine_name}")
        else:
            # HTTP 方式
            return self._query_medicine_http(medicine_name)
    
    def _query_medicine_http(self, medicine_name: str):
        """HTTP 方式查詢詳細藥物"""
        try:
            response = requests.post(
                f"{self.base_url}/api/ros2/service/detailed-medicine",
                json={"medicine_name": medicine_name},
                timeout=5
            )
            
            if response.status_code == 200:
                result = response.json()
                print(f"🌐 [HTTP] 詳細藥物查詢成功: {medicine_name}")
                return result
            else:
                print(f"❌ [HTTP] 詳細藥物查詢失敗: {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ HTTP 詳細藥物查詢失敗: {e}")
            return None
    
    def start_continuous_service(self):
        """啟動持續服務模式"""
        self.running = True
        print("🔄 詳細藥物持續服務已啟動")
    
    def stop_continuous_service(self):
        """停止持續服務模式"""
        self.running = False
        print("⏹️ 詳細藥物持續服務已停止")


class HospitalROS2ServiceManager:
    """醫院 ROS2 服務管理器 - 統一管理三個服務接口"""
    
    def __init__(self, base_url="http://localhost:8001"):
        self.base_url = base_url
        
        # 初始化三個服務接口
        self.order_service = OrderServiceInterface(base_url)
        self.basic_service = BasicMedicineServiceInterface(base_url)
        self.detailed_service = DetailedMedicineServiceInterface(base_url)
        
        print("🏥 醫院 ROS2 服務管理器初始化完成")
    
    def send_order(self, medicines: List[Dict[str, Any]], patient_info: Dict[str, str] = None):
        """發送訂單（每筆完成後再送下一筆）"""
        return self.order_service.send_order(medicines, patient_info)
    
    def query_basic_medicine(self, medicine_name: str):
        """查詢基本藥物資訊（持續）"""
        return self.basic_service.query_medicine(medicine_name)
    
    def query_detailed_medicine(self, medicine_name: str):
        """查詢詳細藥物資訊（持續）"""
        return self.detailed_service.query_medicine(medicine_name)
    
    def start_continuous_services(self):
        """啟動持續服務模式"""
        self.basic_service.start_continuous_service()
        self.detailed_service.start_continuous_service()
        print("🔄 所有持續服務已啟動")
    
    def stop_continuous_services(self):
        """停止持續服務模式"""
        self.basic_service.stop_continuous_service()
        self.detailed_service.stop_continuous_service()
        print("⏹️ 所有持續服務已停止")
    
    def get_service_status(self):
        """獲取所有服務狀態"""
        return {
            "order_service": self.order_service.get_order_status(),
            "basic_service": {"running": self.basic_service.running},
            "detailed_service": {"running": self.detailed_service.running}
        }


def example_usage():
    """使用範例"""
    print("醫院 ROS2 服務接口範例")
    print("=" * 60)
    
    # 創建服務管理器
    manager = HospitalROS2ServiceManager()
    
    print("\n🚀 1. 啟動持續服務")
    manager.start_continuous_services()
    
    print("\n💊 2. 查詢基本藥物資訊")
    manager.query_basic_medicine("阿司匹林")
    manager.query_basic_medicine("布洛芬")
    
    print("\n🔬 3. 查詢詳細藥物資訊")
    manager.query_detailed_medicine("維他命C")
    manager.query_detailed_medicine("胃藥")
    
    print("\n📦 4. 發送訂單")
    # 訂單1
    manager.send_order([
        {"name": "阿司匹林", "quantity": 2},
        {"name": "維他命C", "quantity": 1}
    ], {
        "patient_name": "張三",
        "doctor_name": "李醫師"
    })
    
    # 訂單2（會排隊等待訂單1完成）
    manager.send_order([
        {"name": "布洛芬", "quantity": 1}
    ], {
        "patient_name": "李四",
        "doctor_name": "王醫師"
    })
    
    print("\n📊 5. 檢查服務狀態")
    status = manager.get_service_status()
    print(f"訂單服務狀態: {status['order_service']}")
    
    # 等待一段時間看處理結果
    print("\n⏳ 等待處理結果...")
    time.sleep(5)
    
    print("\n🎯 範例完成！")
    print("\n💡 使用說明:")
    print("   • order_service: 每筆訂單完成後自動處理下一筆")
    print("   • basic_service: 持續提供基本藥物資訊查詢")
    print("   • detailed_service: 持續提供詳細藥物資訊查詢")


if __name__ == "__main__":
    example_usage()
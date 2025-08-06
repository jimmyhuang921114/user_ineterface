#!/usr/bin/env python3
"""
🤖 ROS2通訊介面模組
處理Qt應用程序與ROS2系統之間的通訊
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
from typing import Dict, List, Optional, Any, Callable
from datetime import datetime

# ROS2標準訊息
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState

# 自定義訊息 (如果有的話)
try:
    from medicine_interfaces.msg import MedicineOrder, OrderResponse
    from medicine_interfaces.srv import GetMedicineInfo
    CUSTOM_MESSAGES_AVAILABLE = True
except ImportError:
    CUSTOM_MESSAGES_AVAILABLE = False
    print("⚠️ 自定義ROS2訊息不可用，使用標準訊息")


class ROS2Interface(Node):
    """ROS2通訊介面"""
    
    def __init__(self, node_name: str = "qt_medicine_interface"):
        super().__init__(node_name)
        
        # 初始化狀態
        self.is_connected = False
        self.node_count = 0
        self.topic_count = 0
        
        # 串行訂單處理狀態
        self.processing_order = False
        self.pending_orders = []
        self.current_order = None
        self.order_processing_lock = threading.Lock()
        
        # 設置QoS配置
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 回調群組
        self.callback_group = ReentrantCallbackGroup()
        
        # 初始化發布者和訂閱者
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_services()
        
        # 訊息處理回調
        self.message_callbacks = {}
        self.status_callbacks = []
        
        # 執行器
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self)
        
        # 啟動執行器線程
        self.executor_thread = threading.Thread(target=self._run_executor, daemon=True)
        self.executor_thread.start()
        
        # 啟動訂單處理監控線程
        self.order_monitor_thread = threading.Thread(target=self._monitor_orders, daemon=True)
        self.order_monitor_thread.start()
        
        self.get_logger().info("ROS2介面初始化完成")
        
    def setup_publishers(self):
        """設置發布者"""
        # 處方籤處理請求
        self.prescription_pub = self.create_publisher(
            String, 
            '/medicine/prescription_request', 
            self.qos_profile,
            callback_group=self.callback_group
        )
        
        # 測試訊息
        self.test_pub = self.create_publisher(
            String, 
            '/medicine/test_message', 
            self.qos_profile,
            callback_group=self.callback_group
        )
        
        # 系統狀態
        self.status_pub = self.create_publisher(
            String, 
            '/medicine/system_status', 
            self.qos_profile,
            callback_group=self.callback_group
        )
        
        # 藥物資訊請求
        self.medicine_info_pub = self.create_publisher(
            String, 
            '/medicine/info_request', 
            self.qos_profile,
            callback_group=self.callback_group
        )
        
    def setup_subscribers(self):
        """設置訂閱者"""
        # 處方籤處理回應
        self.prescription_response_sub = self.create_subscription(
            String,
            '/medicine/prescription_response',
            self._prescription_response_callback,
            self.qos_profile,
            callback_group=self.callback_group
        )
        
        # 系統狀態更新
        self.status_update_sub = self.create_subscription(
            String,
            '/medicine/status_update',
            self._status_update_callback,
            self.qos_profile,
            callback_group=self.callback_group
        )
        
        # 藥物資訊回應
        self.medicine_info_response_sub = self.create_subscription(
            String,
            '/medicine/info_response',
            self._medicine_info_response_callback,
            self.qos_profile,
            callback_group=self.callback_group
        )
        
        # 錯誤訊息
        self.error_sub = self.create_subscription(
            String,
            '/medicine/error',
            self._error_callback,
            self.qos_profile,
            callback_group=self.callback_group
        )
        
    def setup_services(self):
        """設置服務"""
        # 藥物資訊查詢服務
        if CUSTOM_MESSAGES_AVAILABLE:
            self.medicine_info_service = self.create_service(
                GetMedicineInfo,
                '/medicine/get_info',
                self._medicine_info_service_callback,
                callback_group=self.callback_group
            )
        else:
            # 使用標準服務作為替代
            pass
            
        # 訂單處理服務客戶端
        self.order_processor_client = self.create_client(
            String, 
            '/medicine/order_processor',
            callback_group=self.callback_group
        )
        
        # 藥物資訊服務客戶端
        self.medicine_info_client = self.create_client(
            String,
            '/medicine/info_service',
            callback_group=self.callback_group
        )
            
    def _run_executor(self):
        """執行器線程"""
        try:
            self.executor.spin()
        except Exception as e:
            self.get_logger().error(f"執行器錯誤: {e}")
            
    def connect(self):
        """連接ROS2系統"""
        try:
            # 檢查ROS2環境
            if not rclpy.ok():
                rclpy.init()
                
            # 發布連接狀態
            self.publish_status("connected")
            self.is_connected = True
            
            # 更新節點和主題計數
            self._update_counts()
            
            self.get_logger().info("ROS2連接成功")
            return True
            
        except Exception as e:
            self.get_logger().error(f"ROS2連接失敗: {e}")
            return False
            
    def disconnect(self):
        """斷開ROS2連接"""
        try:
            self.publish_status("disconnected")
            self.is_connected = False
            self.get_logger().info("ROS2連接已斷開")
        except Exception as e:
            self.get_logger().error(f"斷開連接失敗: {e}")
            
    def cleanup(self):
        """清理資源"""
        try:
            self.disconnect()
            self.executor.shutdown()
            if self.executor_thread.is_alive():
                self.executor_thread.join(timeout=1.0)
        except Exception as e:
            self.get_logger().error(f"清理失敗: {e}")
            
    def send_prescription(self, prescription_id: str):
        """發送處方籤處理請求（串行處理）"""
        with self.order_processing_lock:
            # 添加到待處理列表
            if prescription_id not in [order['id'] for order in self.pending_orders]:
                self.pending_orders.append({
                    'id': prescription_id,
                    'timestamp': datetime.now().isoformat(),
                    'status': 'pending'
                })
                self.get_logger().info(f"添加處方籤到待處理列表: {prescription_id}")
            else:
                self.get_logger().warning(f"處方籤已在待處理列表中: {prescription_id}")
                
    def process_next_order(self):
        """處理下一個訂單"""
        with self.order_processing_lock:
            if self.processing_order:
                self.get_logger().info("當前有訂單正在處理中，等待完成...")
                return False
                
            if not self.pending_orders:
                self.get_logger().info("沒有待處理的訂單")
                return False
                
            # 取出第一個待處理訂單
            self.current_order = self.pending_orders.pop(0)
            self.processing_order = True
            
            self.get_logger().info(f"開始處理訂單: {self.current_order['id']}")
            
            # 發送訂單處理請求
            try:
                msg = String()
                msg.data = self.current_order['id']
                self.prescription_pub.publish(msg)
                
                # 通知回調函數
                for callback in self.status_callbacks:
                    try:
                        callback('order_processing_started', self.current_order)
                    except Exception as e:
                        self.get_logger().error(f"回調函數錯誤: {e}")
                        
            except Exception as e:
                self.get_logger().error(f"發送訂單處理請求失敗: {e}")
                self.processing_order = False
                self.current_order = None
                return False
                
            return True
            
    def complete_current_order(self, result: str):
        """完成當前訂單處理"""
        with self.order_processing_lock:
            if self.current_order:
                self.current_order['status'] = 'completed'
                self.current_order['result'] = result
                self.current_order['completion_time'] = datetime.now().isoformat()
                
                self.get_logger().info(f"訂單處理完成: {self.current_order['id']} - {result}")
                
                # 通知回調函數
                for callback in self.status_callbacks:
                    try:
                        callback('order_completed', self.current_order)
                    except Exception as e:
                        self.get_logger().error(f"回調函數錯誤: {e}")
                        
                self.processing_order = False
                self.current_order = None
                
    def _monitor_orders(self):
        """監控訂單處理狀態"""
        while rclpy.ok():
            try:
                if not self.processing_order and self.pending_orders:
                    self.process_next_order()
                time.sleep(1.0)  # 每秒檢查一次
            except Exception as e:
                self.get_logger().error(f"訂單監控錯誤: {e}")
                time.sleep(5.0)  # 錯誤時等待更長時間
            
    def send_test_message(self, message: str):
        """發送測試訊息"""
        try:
            msg = String()
            msg.data = message
            self.test_pub.publish(msg)
            self.get_logger().info(f"發送測試訊息: {message}")
        except Exception as e:
            self.get_logger().error(f"發送測試訊息失敗: {e}")
            
    def publish_status(self, status: str):
        """發布系統狀態"""
        try:
            msg = String()
            msg.data = f"{datetime.now().isoformat()}: {status}"
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"發布狀態失敗: {e}")
            
    def request_medicine_info(self, medicine_id: str):
        """請求藥物資訊"""
        try:
            msg = String()
            msg.data = medicine_id
            self.medicine_info_pub.publish(msg)
            self.get_logger().info(f"請求藥物資訊: {medicine_id}")
        except Exception as e:
            self.get_logger().error(f"請求藥物資訊失敗: {e}")
            
    def get_medicine_info_via_service(self, medicine_id: str, timeout: float = 10.0):
        """通過服務獲取藥物資訊"""
        try:
            if not self.medicine_info_client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error("藥物資訊服務不可用")
                return None
                
            request = String()
            request.data = medicine_id
            
            future = self.medicine_info_client.call_async(request)
            
            # 等待回應
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f"收到藥物資訊回應: {response.data}")
                return response.data
            else:
                self.get_logger().error("藥物資訊服務請求超時")
                return None
                
        except Exception as e:
            self.get_logger().error(f"獲取藥物資訊失敗: {e}")
            return None
            
    def get_all_medicines_via_service(self, timeout: float = 30.0):
        """通過服務獲取所有藥物資訊"""
        try:
            if not self.medicine_info_client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error("藥物資訊服務不可用")
                return None
                
            request = String()
            request.data = "ALL"  # 特殊標識符表示請求所有藥物
            
            future = self.medicine_info_client.call_async(request)
            
            # 等待回應
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                self.get_logger().info("收到所有藥物資訊回應")
                return response.data
            else:
                self.get_logger().error("藥物資訊服務請求超時")
                return None
                
        except Exception as e:
            self.get_logger().error(f"獲取所有藥物資訊失敗: {e}")
            return None
            
    def create_medicine_via_service(self, medicine_data: dict, timeout: float = 10.0):
        """通過服務創建藥物"""
        try:
            if not self.medicine_info_client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error("藥物資訊服務不可用")
                return None
                
            request = String()
            request.data = f"CREATE:{json.dumps(medicine_data, ensure_ascii=False)}"
            
            future = self.medicine_info_client.call_async(request)
            
            # 等待回應
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f"藥物創建回應: {response.data}")
                return response.data
            else:
                self.get_logger().error("藥物創建服務請求超時")
                return None
                
        except Exception as e:
            self.get_logger().error(f"創建藥物失敗: {e}")
            return None
            
    def update_medicine_via_service(self, medicine_id: str, medicine_data: dict, timeout: float = 10.0):
        """通過服務更新藥物"""
        try:
            if not self.medicine_info_client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error("藥物資訊服務不可用")
                return None
                
            request = String()
            request.data = f"UPDATE:{medicine_id}:{json.dumps(medicine_data, ensure_ascii=False)}"
            
            future = self.medicine_info_client.call_async(request)
            
            # 等待回應
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f"藥物更新回應: {response.data}")
                return response.data
            else:
                self.get_logger().error("藥物更新服務請求超時")
                return None
                
        except Exception as e:
            self.get_logger().error(f"更新藥物失敗: {e}")
            return None
            
    def delete_medicine_via_service(self, medicine_id: str, timeout: float = 10.0):
        """通過服務刪除藥物"""
        try:
            if not self.medicine_info_client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error("藥物資訊服務不可用")
                return None
                
            request = String()
            request.data = f"DELETE:{medicine_id}"
            
            future = self.medicine_info_client.call_async(request)
            
            # 等待回應
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f"藥物刪除回應: {response.data}")
                return response.data
            else:
                self.get_logger().error("藥物刪除服務請求超時")
                return None
                
        except Exception as e:
            self.get_logger().error(f"刪除藥物失敗: {e}")
            return None
            
    def _prescription_response_callback(self, msg: String):
        """處方籤處理回應回調"""
        try:
            response_data = msg.data
            self.get_logger().info(f"收到處方籤處理回應: {response_data}")
            
            # 完成當前訂單處理
            self.complete_current_order(response_data)
            
            # 通知回調函數
            for callback in self.status_callbacks:
                try:
                    callback('prescription_response', response_data)
                except Exception as e:
                    self.get_logger().error(f"回調函數錯誤: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"處理處方籤回應失敗: {e}")
            # 即使出錯也要完成當前訂單
            self.complete_current_order("error")
            
    def _status_update_callback(self, msg: String):
        """狀態更新回調"""
        try:
            status_data = msg.data
            self.get_logger().info(f"收到狀態更新: {status_data}")
            
            # 通知回調函數
            for callback in self.status_callbacks:
                try:
                    callback('status_update', status_data)
                except Exception as e:
                    self.get_logger().error(f"回調函數錯誤: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"處理狀態更新失敗: {e}")
            
    def _medicine_info_response_callback(self, msg: String):
        """藥物資訊回應回調"""
        try:
            info_data = msg.data
            self.get_logger().info(f"收到藥物資訊回應: {info_data}")
            
            # 通知回調函數
            for callback in self.status_callbacks:
                try:
                    callback('medicine_info_response', info_data)
                except Exception as e:
                    self.get_logger().error(f"回調函數錯誤: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"處理藥物資訊回應失敗: {e}")
            
    def _error_callback(self, msg: String):
        """錯誤訊息回調"""
        try:
            error_data = msg.data
            self.get_logger().error(f"收到錯誤訊息: {error_data}")
            
            # 通知回調函數
            for callback in self.status_callbacks:
                try:
                    callback('error', error_data)
                except Exception as e:
                    self.get_logger().error(f"回調函數錯誤: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"處理錯誤訊息失敗: {e}")
            
    def _medicine_info_service_callback(self, request, response):
        """藥物資訊服務回調"""
        try:
            medicine_id = request.medicine_id
            self.get_logger().info(f"收到藥物資訊服務請求: {medicine_id}")
            
            # 這裡應該查詢藥物資料庫
            # 暫時返回預設回應
            if CUSTOM_MESSAGES_AVAILABLE:
                response.medicine_name = f"藥物_{medicine_id}"
                response.category = "未知分類"
                response.stock_quantity = 0
                response.price = 0.0
            else:
                response = String()
                response.data = f"藥物資訊: {medicine_id}"
                
            return response
            
        except Exception as e:
            self.get_logger().error(f"藥物資訊服務錯誤: {e}")
            return None
            
    def _update_counts(self):
        """更新節點和主題計數"""
        try:
            # 獲取節點列表
            node_names = self.get_node_names()
            self.node_count = len(node_names)
            
            # 獲取主題列表
            topic_names = self.get_topic_names_and_types()
            self.topic_count = len(topic_names)
            
        except Exception as e:
            self.get_logger().error(f"更新計數失敗: {e}")
            
    def get_node_count(self) -> int:
        """獲取節點數量"""
        return self.node_count
        
    def get_topic_count(self) -> int:
        """獲取主題數量"""
        return self.topic_count
        
    def add_status_callback(self, callback: Callable):
        """添加狀態回調函數"""
        self.status_callbacks.append(callback)
        
    def remove_status_callback(self, callback: Callable):
        """移除狀態回調函數"""
        if callback in self.status_callbacks:
            self.status_callbacks.remove(callback)
            
    def get_connection_status(self) -> bool:
        """獲取連接狀態"""
        return self.is_connected
        
    def get_order_status(self) -> dict:
        """獲取訂單處理狀態"""
        with self.order_processing_lock:
            return {
                'processing_order': self.processing_order,
                'current_order': self.current_order,
                'pending_count': len(self.pending_orders),
                'pending_orders': self.pending_orders.copy()
            }
            
    def get_processing_status(self) -> str:
        """獲取處理狀態描述"""
        with self.order_processing_lock:
            if self.processing_order:
                return f"處理中: {self.current_order['id'] if self.current_order else 'Unknown'}"
            elif self.pending_orders:
                return f"等待中: {len(self.pending_orders)} 筆訂單"
            else:
                return "閒置中"
        
    def get_available_topics(self) -> List[str]:
        """獲取可用的主題列表"""
        try:
            topics = self.get_topic_names_and_types()
            return [topic[0] for topic in topics]
        except Exception as e:
            self.get_logger().error(f"獲取主題列表失敗: {e}")
            return []
            
    def get_available_services(self) -> List[str]:
        """獲取可用的服務列表"""
        try:
            services = self.get_service_names_and_types()
            return [service[0] for service in services]
        except Exception as e:
            self.get_logger().error(f"獲取服務列表失敗: {e}")
            return []
            
    def publish_twist_command(self, linear_x: float = 0.0, angular_z: float = 0.0):
        """發布Twist命令（用於機械手臂控制）"""
        try:
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.create_publisher(Twist, '/cmd_vel', self.qos_profile).publish(msg)
            self.get_logger().info(f"發布Twist命令: linear_x={linear_x}, angular_z={angular_z}")
        except Exception as e:
            self.get_logger().error(f"發布Twist命令失敗: {e}")
            
    def publish_joint_state(self, joint_names: List[str], positions: List[float]):
        """發布關節狀態（用於機械手臂控制）"""
        try:
            msg = JointState()
            msg.name = joint_names
            msg.position = positions
            msg.header.stamp = self.get_clock().now().to_msg()
            self.create_publisher(JointState, '/joint_states', self.qos_profile).publish(msg)
            self.get_logger().info(f"發布關節狀態: {joint_names}")
        except Exception as e:
            self.get_logger().error(f"發布關節狀態失敗: {e}")
            
    def send_custom_message(self, topic: str, message: str):
        """發送自定義訊息到指定主題"""
        try:
            msg = String()
            msg.data = message
            publisher = self.create_publisher(String, topic, self.qos_profile)
            publisher.publish(msg)
            self.get_logger().info(f"發送自定義訊息到 {topic}: {message}")
        except Exception as e:
            self.get_logger().error(f"發送自定義訊息失敗: {e}")
            
    def get_system_info(self) -> Dict[str, Any]:
        """獲取系統資訊"""
        try:
            return {
                'node_name': self.get_name(),
                'node_count': self.node_count,
                'topic_count': self.topic_count,
                'is_connected': self.is_connected,
                'available_topics': self.get_available_topics(),
                'available_services': self.get_available_services(),
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            self.get_logger().error(f"獲取系統資訊失敗: {e}")
            return {}


class ROS2Manager:
    """ROS2管理器（單例模式）"""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance
        
    def __init__(self):
        if not hasattr(self, 'initialized'):
            self.interface = None
            self.initialized = True
            
    def initialize(self, node_name: str = "qt_medicine_manager"):
        """初始化ROS2介面"""
        try:
            if self.interface is None:
                self.interface = ROS2Interface(node_name)
            return self.interface
        except Exception as e:
            print(f"初始化ROS2管理器失敗: {e}")
            return None
            
    def get_interface(self) -> Optional[ROS2Interface]:
        """獲取ROS2介面"""
        return self.interface
        
    def cleanup(self):
        """清理資源"""
        if self.interface:
            self.interface.cleanup()
            self.interface = None


# 便捷函數
def create_ros2_interface(node_name: str = "qt_medicine_interface") -> Optional[ROS2Interface]:
    """創建ROS2介面"""
    try:
        return ROS2Interface(node_name)
    except Exception as e:
        print(f"創建ROS2介面失敗: {e}")
        return None
        
def get_ros2_manager() -> ROS2Manager:
    """獲取ROS2管理器實例"""
    return ROS2Manager()
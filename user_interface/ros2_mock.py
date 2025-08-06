#!/usr/bin/env python3
"""
Mock ROS2 Integration Module
模擬ROS2整合模組 - 用於沒有安裝完整ROS2的環境
"""

import json
import time
import threading
from typing import Dict, List, Optional
from queue import Queue
import logging

logger = logging.getLogger("ros2_mock")

class MockHospitalROS2Node:
    """模擬醫院ROS2節點"""
    
    def __init__(self):
        logger.info("🤖 初始化模擬ROS2節點...")
        
        # 訂單處理狀態
        self.order_queue = Queue()
        self.current_order = None
        self.processing = False
        self.order_lock = threading.Lock()
        self.order_history = []
        
        # 模擬硬體狀態
        self.robot_available = True
        self.medicine_dispenser_status = "ready"
        
        # 啟動背景處理執行緒
        self.processing_thread = threading.Thread(target=self._background_processor, daemon=True)
        self.processing_thread.start()
        
        logger.info("✅ 模擬ROS2節點初始化完成")
    
    def _background_processor(self):
        """背景訂單處理器"""
        while True:
            try:
                if not self.order_queue.empty() and not self.processing:
                    with self.order_lock:
                        if not self.processing and not self.order_queue.empty():
                            self.current_order = self.order_queue.get()
                            self.processing = True
                            logger.info(f"🤖 開始處理訂單: {self.current_order.get('order_id', 'Unknown')}")
                    
                    # 模擬處理時間
                    self._simulate_order_processing()
                    
                    with self.order_lock:
                        self.order_history.append({
                            **self.current_order,
                            "completed_at": time.time(),
                            "status": "completed"
                        })
                        self.current_order = None
                        self.processing = False
                        logger.info("✅ 訂單處理完成")
                
                time.sleep(1)  # 檢查間隔
            except Exception as e:
                logger.error(f"❌ 背景處理器錯誤: {e}")
                with self.order_lock:
                    self.processing = False
                    self.current_order = None
    
    def _simulate_order_processing(self):
        """模擬訂單處理過程"""
        # 模擬各種處理階段
        stages = [
            ("驗證處方籤", 2),
            ("定位藥物", 3),
            ("移動機器人", 4),
            ("分配藥物", 5),
            ("包裝藥物", 3),
            ("品質檢查", 2)
        ]
        
        for stage, duration in stages:
            logger.info(f"🤖 {stage}中...")
            time.sleep(duration)
    
    def add_order(self, order_data: Dict):
        """添加訂單到處理佇列"""
        try:
            # 添加時間戳和狀態
            order_data.update({
                "timestamp": time.time(),
                "status": "pending",
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S")
            })
            
            self.order_queue.put(order_data)
            logger.info(f"📦 訂單已加入佇列: {order_data.get('order_id', 'Unknown')}")
            logger.debug(f"📦 訂單詳情: {json.dumps(order_data, ensure_ascii=False, indent=2)}")
            
            return {
                "success": True,
                "message": "訂單已加入處理佇列",
                "queue_position": self.order_queue.qsize()
            }
        except Exception as e:
            logger.error(f"❌ 添加訂單失敗: {e}")
            return {
                "success": False,
                "message": f"添加訂單失敗: {str(e)}"
            }
    
    def get_queue_status(self) -> Dict:
        """獲取佇列狀態"""
        with self.order_lock:
            return {
                "queue_size": self.order_queue.qsize(),
                "processing": self.processing,
                "current_order": self.current_order.get('order_id') if self.current_order else None,
                "robot_available": self.robot_available,
                "dispenser_status": self.medicine_dispenser_status,
                "total_completed": len(self.order_history)
            }
    
    def get_order_history(self, limit: int = 10) -> List[Dict]:
        """獲取訂單歷史"""
        return self.order_history[-limit:]
    
    def publish_medicine_data(self, medicine_data: Dict):
        """發布藥物資料更新"""
        logger.debug(f"📡 發布藥物資料: {medicine_data.get('name', 'Unknown')}")
        # 在真實環境中，這會發布到ROS2主題
        # 這裡只是記錄日誌
    
    def emergency_stop(self):
        """緊急停止"""
        with self.order_lock:
            logger.warning("🚨 緊急停止觸發")
            self.processing = False
            self.robot_available = False
            # 清空佇列
            while not self.order_queue.empty():
                self.order_queue.get()
    
    def reset_system(self):
        """重置系統"""
        with self.order_lock:
            logger.info("🔄 重置ROS2系統")
            self.processing = False
            self.current_order = None
            self.robot_available = True
            self.medicine_dispenser_status = "ready"
            while not self.order_queue.empty():
                self.order_queue.get()

def init_ros2_node() -> MockHospitalROS2Node:
    """初始化模擬ROS2節點"""
    try:
        logger.info("🚀 啟動模擬ROS2整合...")
        node = MockHospitalROS2Node()
        logger.info("✅ 模擬ROS2整合啟動成功")
        return node
    except Exception as e:
        logger.error(f"❌ 模擬ROS2整合啟動失敗: {e}")
        return None

def get_ros2_node() -> Optional[MockHospitalROS2Node]:
    """獲取ROS2節點實例"""
    # 在這個模擬版本中，我們返回None
    # 實際的ROS2實現會返回真實的節點
    return None
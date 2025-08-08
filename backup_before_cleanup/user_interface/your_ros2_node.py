#!/usr/bin/env python3
"""
您的 ROS2 節點 - 接收自動推送的訂單
基於 integration_example.py 改寫，專為您的需求設計

功能：
1. 自動接收推送的 YAML 訂單
2. 處理訂單 (一次一個)
3. 完成後告知網站，自動進行下一個
"""

import rclpy
from rclpy.node import Node
import yaml
import time
import threading
from typing import Dict, Any, Optional


class YourROS2OrderHandler(Node):
    """您的 ROS2 訂單處理節點"""
    
    def __init__(self):
        super().__init__('your_order_handler')
        
        # 訂單推送器 (由啟動腳本設置)
        self._order_pusher = None
        
        # 當前處理的訂單
        self.current_order = None
        
        # 處理狀態
        self.is_processing = False
        
        self.get_logger().info("🤖 您的 ROS2 訂單處理節點已啟動")
        self.get_logger().info("📋 等待訂單推送...")
    
    def set_order_pusher(self, order_pusher):
        """設置訂單推送器 (由啟動腳本調用)"""
        self._order_pusher = order_pusher
        self.get_logger().info("✅ 訂單推送器已連接")
    
    def process_order(self, order_dict: Dict[str, Any], yaml_order: str):
        """
        處理推送的訂單 (由 ros2_order_pusher 調用)
        
        Args:
            order_dict: 解析後的訂單字典
            yaml_order: 原始 YAML 訂單字符串
        """
        if self.is_processing:
            self.get_logger().warning("⚠️ 正在處理其他訂單，跳過此訂單")
            return
        
        self.is_processing = True
        self.current_order = order_dict
        
        order_id = order_dict.get('order_id', 'unknown')
        patient_name = order_dict.get('patient_name', 'unknown')
        medicines = order_dict.get('medicine', [])
        
        self.get_logger().info(f"📋 收到新訂單: {order_id}")
        self.get_logger().info(f"👤 病患: {patient_name}")
        self.get_logger().info(f"💊 藥物數量: {len(medicines)}")
        
        # 顯示完整訂單內容
        self.print_order_details(order_dict, yaml_order)
        
        # 開始處理訂單
        threading.Thread(target=self._process_order_async, args=(order_dict,)).start()
    
    def print_order_details(self, order_dict: Dict[str, Any], yaml_order: str):
        """打印詳細訂單內容"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("📋 訂單詳細內容:")
        self.get_logger().info("=" * 50)
        
        # 打印 YAML 格式
        self.get_logger().info("🔖 YAML 格式:")
        for line in yaml_order.split('\n'):
            if line.strip():
                self.get_logger().info(f"   {line}")
        
        self.get_logger().info("-" * 30)
        
        # 打印結構化信息
        order_id = order_dict.get('order_id', 'N/A')
        prescription_id = order_dict.get('prescription_id', 'N/A')
        patient_name = order_dict.get('patient_name', 'N/A')
        
        self.get_logger().info(f"🆔 訂單 ID: {order_id}")
        self.get_logger().info(f"📝 處方籤 ID: {prescription_id}")
        self.get_logger().info(f"👤 病患姓名: {patient_name}")
        
        medicines = order_dict.get('medicine', [])
        if medicines:
            self.get_logger().info(f"💊 藥物清單 ({len(medicines)} 項):")
            for i, med in enumerate(medicines, 1):
                name = med.get('name', 'N/A')
                amount = med.get('amount', 'N/A')
                locate = med.get('locate', [0, 0])
                prompt = med.get('prompt', 'N/A')
                
                self.get_logger().info(f"   {i}. 藥物: {name}")
                self.get_logger().info(f"      數量: {amount}")
                self.get_logger().info(f"      位置: 第{locate[0]}排第{locate[1]}列")
                self.get_logger().info(f"      類型: {prompt}")
        else:
            self.get_logger().info("💊 無藥物項目")
        
        self.get_logger().info("=" * 50)
    
    def _process_order_async(self, order_dict: Dict[str, Any]):
        """異步處理訂單"""
        order_id = order_dict.get('order_id', 'unknown')
        medicines = order_dict.get('medicine', [])
        
        try:
            self.get_logger().info(f"🤖 開始處理訂單: {order_id}")
            
            # 處理每個藥物
            for i, medicine in enumerate(medicines, 1):
                self.process_medicine(medicine, i, len(medicines))
            
            # 模擬處理時間 (您可以移除這個)
            self.get_logger().info("🔄 模擬處理中...")
            time.sleep(3)  # 移除此行以實際使用
            
            # 處理完成
            self.get_logger().info(f"✅ 訂單 {order_id} 處理完成")
            
            # 告知網站完成
            self.complete_order(order_id)
            
        except Exception as e:
            self.get_logger().error(f"❌ 處理訂單時發生錯誤: {e}")
            # 即使發生錯誤也要告知完成，避免卡住
            self.complete_order(order_id)
        
        finally:
            self.is_processing = False
            self.current_order = None
    
    def process_medicine(self, medicine: Dict[str, Any], index: int, total: int):
        """
        處理單個藥物
        
        在這裡實現您的機器人邏輯！
        """
        name = medicine.get('name', 'N/A')
        amount = medicine.get('amount', 0)
        locate = medicine.get('locate', [0, 0])
        prompt = medicine.get('prompt', 'unknown')
        
        self.get_logger().info(f"🔧 處理藥物 ({index}/{total}): {name}")
        self.get_logger().info(f"   數量: {amount}")
        self.get_logger().info(f"   位置: 第{locate[0]}排第{locate[1]}列")
        self.get_logger().info(f"   類型: {prompt}")
        
        # ============================================
        # 在這裡添加您的機器人控制代碼！
        # ============================================
        
        # 例如：
        # 1. 移動到指定位置
        # self.move_robot_to_position(locate[0], locate[1])
        
        # 2. 根據類型選擇不同的抓取方式
        # if prompt == 'tablet':
        #     self.pick_tablet(amount)
        # elif prompt == 'capsule':
        #     self.pick_capsule(amount)
        # elif prompt == 'white_circle_box':
        #     self.pick_box(amount)
        
        # 3. 放置到分配區域
        # self.place_medicine()
        
        # 模擬處理時間 (移除此行)
        time.sleep(1)
        
        self.get_logger().info(f"✅ 藥物 {name} 處理完成")
    
    def complete_order(self, order_id: str):
        """告知網站訂單已完成"""
        if hasattr(self, '_order_pusher') and self._order_pusher:
            try:
                success = self._order_pusher.complete_order(order_id)
                if success:
                    self.get_logger().info(f"✅ 已告知網站訂單 {order_id} 完成")
                    self.get_logger().info("🔄 系統將自動處理下一個訂單...")
                else:
                    self.get_logger().error(f"❌ 告知網站完成失敗: {order_id}")
            except Exception as e:
                self.get_logger().error(f"❌ 完成訂單時發生錯誤: {e}")
        else:
            self.get_logger().error("❌ 訂單推送器未連接")
    
    def get_current_order_info(self) -> Optional[Dict[str, Any]]:
        """獲取當前處理的訂單信息"""
        return self.current_order
    
    def is_busy(self) -> bool:
        """檢查是否正在處理訂單"""
        return self.is_processing


def main(args=None):
    """主函數 - 單獨運行此節點用於測試"""
    rclpy.init(args=args)
    
    node = YourROS2OrderHandler()
    
    try:
        # 提示信息
        node.get_logger().info("🔧 您的 ROS2 節點正在運行")
        node.get_logger().info("💡 要接收訂單，請使用 start_system_modes.py 啟動完整系統")
        node.get_logger().info("📋 或者運行 test_order_flow.py 進行測試")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("👋 節點關閉中...")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
ROS2 訂單主動推送器
監控資料庫中的新處方籤，自動轉換為訂單並推送給 ROS2 系統
"""

import json
import time
import threading
import requests
import logging
from typing import Dict, List, Optional, Callable
from datetime import datetime

logger = logging.getLogger("ros2_order_pusher")

class OrderPusher:
    """訂單主動推送器"""
    
    def __init__(self, fastapi_base_url: str = "http://localhost:8001", 
                 callback_func: Optional[Callable] = None):
        self.base_url = fastapi_base_url
        self.callback_func = callback_func  # 用戶提供的回調函數
        self.is_running = False
        self.processed_prescriptions = set()  # 已處理的處方籤ID
        self.monitor_thread = None
        self.check_interval = 3  # 每3秒檢查一次
        
        logger.info("訂單推送器已初始化")
    
    def start_monitoring(self):
        """開始監控新處方籤"""
        if self.is_running:
            logger.warning("監控已在運行中")
            return
        
        self.is_running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        logger.info("開始監控新處方籤...")
    
    def stop_monitoring(self):
        """停止監控"""
        self.is_running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5)
        logger.info("監控已停止")
    
    def _monitor_loop(self):
        """監控循環"""
        while self.is_running:
            try:
                self._check_for_new_prescriptions()
                time.sleep(self.check_interval)
            except Exception as e:
                logger.error(f"監控循環錯誤: {e}")
                time.sleep(self.check_interval)
    
    def _check_for_new_prescriptions(self):
        """檢查新的待處理處方籤"""
        try:
            response = requests.get(f"{self.base_url}/api/prescription/", timeout=5)
            if response.status_code != 200:
                return
            
            prescriptions = response.json()
            
            for prescription in prescriptions:
                prescription_id = prescription.get('id')
                status = prescription.get('status')
                
                # 檢查是否為新的 pending 處方籤
                if (status == 'pending' and 
                    prescription_id not in self.processed_prescriptions):
                    
                    self._process_new_prescription(prescription_id)
                    
        except Exception as e:
            logger.error(f"檢查處方籤時發生錯誤: {e}")
    
    def _process_new_prescription(self, prescription_id: int):
        """處理新的處方籤，轉換為訂單"""
        try:
            # 獲取處方籤詳細資訊
            response = requests.get(f"{self.base_url}/api/prescription/{prescription_id}", timeout=5)
            if response.status_code != 200:
                logger.error(f"無法獲取處方籤 {prescription_id} 詳細資訊")
                return
            
            prescription_detail = response.json()
            
            # 轉換為訂單格式
            order = self._convert_prescription_to_order(prescription_detail)
            
            # 標記處方籤為處理中
            self._update_prescription_status(prescription_id, "processing")
            
            # 記錄已處理
            self.processed_prescriptions.add(prescription_id)
            
            # 推送訂單
            self._push_order_to_ros2(order, prescription_id)
            
            logger.info(f"處方籤 {prescription_id} 已轉換為訂單並推送")
            
        except Exception as e:
            logger.error(f"處理處方籤 {prescription_id} 時發生錯誤: {e}")
    
    def _convert_prescription_to_order(self, prescription: Dict) -> Dict:
        """將處方籤轉換為訂單格式"""
        prescription_id = prescription.get('id')
        patient_name = prescription.get('patient_name', 'Unknown')
        medicines = prescription.get('medicines', [])
        
        # 生成訂單ID
        order_id = f"{prescription_id:06d}"
        
        # 轉換藥物列表
        order_medicines = []
        for medicine in medicines:
            medicine_name = medicine.get('medicine_name', '')
            quantity = medicine.get('quantity', 1)
            
            # 獲取藥物位置和類型
            locate = self._get_medicine_location(medicine_name)
            prompt = self._get_medicine_prompt(medicine_name)
            
            order_medicines.append({
                'name': medicine_name,
                'amount': quantity,
                'locate': locate,
                'prompt': prompt
            })
        
        return {
            'order_id': order_id,
            'prescription_id': prescription_id,
            'patient_name': patient_name,
            'medicines': order_medicines,
            'created_at': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
    
    def _get_medicine_location(self, medicine_name: str) -> List[int]:
        """獲取藥物位置"""
        # 使用哈希算法分配位置
        hash_value = hash(medicine_name) % 100
        row = (hash_value // 10) + 1
        col = (hash_value % 10) + 1
        return [row, col]
    
    def _get_medicine_prompt(self, medicine_name: str) -> str:
        """獲取藥物提示符"""
        name_lower = medicine_name.lower()
        if 'tablet' in name_lower or '錠' in medicine_name:
            return 'tablet'
        elif 'capsule' in name_lower or '膠囊' in medicine_name:
            return 'capsule'
        elif 'syrup' in name_lower or '糖漿' in medicine_name:
            return 'liquid'
        elif 'injection' in name_lower or '注射' in medicine_name:
            return 'injection'
        else:
            return 'white_circle_box'
    
    def _update_prescription_status(self, prescription_id: int, status: str):
        """更新處方籤狀態"""
        try:
            response = requests.put(
                f"{self.base_url}/api/prescription/{prescription_id}/status",
                json={"status": status},
                timeout=5
            )
            if response.status_code == 200:
                logger.info(f"處方籤 {prescription_id} 狀態已更新為: {status}")
            else:
                logger.warning(f"更新處方籤 {prescription_id} 狀態失敗: {response.status_code}")
        except Exception as e:
            logger.error(f"更新處方籤狀態時發生錯誤: {e}")
    
    def _push_order_to_ros2(self, order: Dict, prescription_id: int):
        """推送訂單到 ROS2 系統"""
        try:
            # 格式化為您要求的 YAML 格式
            yaml_order = self._format_order_as_yaml(order)
            
            # 如果有回調函數，調用它
            if self.callback_func:
                try:
                    self.callback_func(order, yaml_order)
                except Exception as e:
                    logger.error(f"回調函數執行失敗: {e}")
            
            # 同時在日誌中顯示
            logger.info(f"🚀 新訂單推送:")
            logger.info(f"\n{yaml_order}")
            
            # 您可以在這裡添加實際的 ROS2 發送邏輯
            # 例如: ros2_publisher.publish(yaml_order)
            
        except Exception as e:
            logger.error(f"推送訂單到 ROS2 時發生錯誤: {e}")
    
    def _format_order_as_yaml(self, order: Dict) -> str:
        """格式化訂單為 YAML 格式"""
        order_id = order.get('order_id', '000000')
        medicines = order.get('medicines', [])
        
        yaml_content = f'order_id: "{order_id}"\nmedicine:\n'
        
        for medicine in medicines:
            name = medicine.get('name', 'Unknown')
            amount = medicine.get('amount', 0)
            locate = medicine.get('locate', [1, 1])
            prompt = medicine.get('prompt', 'tablet')
            
            yaml_content += f"""  - name: {name}
    amount: {amount}
    locate: {locate}
    prompt: {prompt}

"""
        
        return yaml_content.rstrip()
    
    def complete_order(self, order_id: str) -> bool:
        """標記訂單完成"""
        try:
            # 從訂單ID推算處方籤ID
            prescription_id = int(order_id)
            
            # 更新處方籤狀態為已完成
            self._update_prescription_status(prescription_id, "completed")
            
            logger.info(f"訂單 {order_id} 已標記為完成")
            return True
            
        except Exception as e:
            logger.error(f"完成訂單 {order_id} 時發生錯誤: {e}")
            return False
    
    def get_status(self) -> Dict:
        """獲取推送器狀態"""
        return {
            "monitoring": self.is_running,
            "processed_count": len(self.processed_prescriptions),
            "check_interval": self.check_interval,
            "base_url": self.base_url
        }

# ==================== 使用範例 ====================

def example_ros2_callback(order_dict: Dict, yaml_order: str):
    """
    範例回調函數 - 您可以在這裡處理收到的訂單
    
    Args:
        order_dict: 訂單字典格式
        yaml_order: YAML 格式訂單字符串
    """
    print("🔔 收到新訂單!")
    print("=" * 50)
    print(yaml_order)
    print("=" * 50)
    
    # 在這裡添加您的 ROS2 處理邏輯
    # 例如：
    # ros2_node.send_order(yaml_order)
    # robot_controller.process_order(order_dict)

def main():
    """主函數 - 示範如何使用"""
    logging.basicConfig(level=logging.INFO, 
                       format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    
    print("🚀 ROS2 訂單主動推送器")
    print("=" * 50)
    print("功能：自動監控新處方籤並推送訂單")
    print("")
    
    # 創建推送器實例，帶回調函數
    pusher = OrderPusher(callback_func=example_ros2_callback)
    
    try:
        # 開始監控
        pusher.start_monitoring()
        
        print("✅ 監控已啟動，等待新處方籤...")
        print("📋 創建一些測試處方籤來查看效果")
        print("🛑 按 Ctrl+C 停止監控")
        
        # 保持運行
        while True:
            status = pusher.get_status()
            print(f"\r⏱️ 監控中... 已處理: {status['processed_count']} 個處方籤", end="", flush=True)
            time.sleep(5)
            
    except KeyboardInterrupt:
        print("\n\n🛑 停止監控...")
        pusher.stop_monitoring()
        print("✅ 監控已停止")

if __name__ == "__main__":
    main()
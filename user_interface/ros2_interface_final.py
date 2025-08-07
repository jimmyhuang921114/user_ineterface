#!/usr/bin/env python3
"""
醫院藥物管理系統 - 最終版 ROS2 接口
不自動模擬，但提供所有必要接口供您整合使用
"""

import json
import time
import threading
import requests
from typing import Dict, List, Optional, Any
import logging

logger = logging.getLogger("ros2_interface_final")

class MedicineDetailFormatter:
    """藥物詳細資訊格式化器"""
    
    def __init__(self, data: Dict[str, Any]):
        self.raw_data = data
        
    def to_yaml_format(self, medicine_name: str) -> str:
        """轉換為 YAML 格式"""
        basic_info = self.raw_data.get('basic', {})
        detailed_info = self.raw_data.get('detailed', {})
        
        # 處理外觀資訊
        appearance_parts = detailed_info.get('appearance_type', '').split()
        color = ' '.join(appearance_parts[:-1]) if len(appearance_parts) > 1 else "白色"
        shape = appearance_parts[-1] if appearance_parts else "圓形"
        
        yaml_content = f"""name: {medicine_name}
constant:
  名稱: "{basic_info.get('name', medicine_name)}"
  成分: "{detailed_info.get('ingredient', 'N/A')}"
  分類: "{detailed_info.get('category', 'N/A')}"
  劑量: "{basic_info.get('dosage', detailed_info.get('unit_dose', 'N/A'))}"
  服用方式: "{detailed_info.get('usage_method', '依醫師指示')}"
  有效日期: "{detailed_info.get('expiry_date', 'N/A')}"
  適應症: "{detailed_info.get('description', 'N/A')}"
  可能副作用: "{detailed_info.get('side_effects', 'N/A')}"
  條碼編號: "{detailed_info.get('barcode', 'N/A')}"
  外觀:
    顏色: "{color}"
    形狀: "{shape}"
"""
        return yaml_content

class OrderFormatter:
    """訂單格式化器"""
    
    def __init__(self, order_id: str, medicines: List[Dict]):
        self.order_id = order_id
        self.medicines = medicines
        
    def to_yaml_format(self) -> str:
        """轉換為 YAML 格式"""
        yaml_content = f'order_id: "{self.order_id}"\nmedicine:\n'
        
        for med in self.medicines:
            yaml_content += f"""  - name: {med.get('name', 'Unknown')}
    amount: {med.get('amount', 0)}
    locate: {med.get('locate', [1, 1])}
    prompt: {med.get('prompt', 'tablet')}

"""
        return yaml_content.rstrip()

class ROS2InterfaceFinal:
    """最終版 ROS2 接口 - 不自動模擬，但提供完整接口"""
    
    def __init__(self, fastapi_base_url: str = "http://localhost:8001"):
        self.base_url = fastapi_base_url
        self.is_running = True
        self.current_order = None
        self.processing_lock = threading.Lock()
        
        # 不啟動自動模擬，僅初始化接口
        logger.info("ROS2 接口已初始化 - 等待您的整合")
    
    def query_medicine_detail(self, medicine_name: str) -> str:
        """查詢藥物詳細資訊並返回 YAML 格式"""
        try:
            # 查詢基本藥物資訊
            basic_response = requests.post(
                f"{self.base_url}/api/ros2/service/basic-medicine",
                json={"medicine_name": medicine_name},
                timeout=10
            )
            
            if basic_response.status_code != 200:
                return f"錯誤: 無法查詢基本藥物資訊 - {basic_response.status_code}"
            
            basic_data = basic_response.json()
            basic_medicines = basic_data.get('medicines', [])
            
            if not basic_medicines:
                return f"錯誤: 找不到藥物 '{medicine_name}'"
            
            # 查詢詳細藥物資訊
            detailed_response = requests.post(
                f"{self.base_url}/api/ros2/service/detailed-medicine",
                json={"medicine_name": medicine_name, "include_detailed": True},
                timeout=10
            )
            
            detailed_data = {}
            if detailed_response.status_code == 200:
                detailed_result = detailed_response.json()
                detailed_medicines = detailed_result.get('detailed_medicines', [])
                if detailed_medicines:
                    detailed_data = detailed_medicines[0]
            
            # 組合資料
            combined_data = {
                'basic': basic_medicines[0],
                'detailed': detailed_data
            }
            
            # 創建格式化器並轉換格式
            formatter = MedicineDetailFormatter(combined_data)
            return formatter.to_yaml_format(medicine_name)
            
        except Exception as e:
            logger.error(f"查詢藥物詳細資訊失敗: {e}")
            return f"錯誤: 查詢失敗 - {str(e)}"
    
    def process_order(self, order_data: Dict) -> str:
        """處理訂單請求 - 不自動模擬，但格式化訂單"""
        with self.processing_lock:
            if self.current_order is not None:
                return "錯誤: 目前正在處理其他訂單，請稍後再試"
            
            try:
                order_id = order_data.get('order_id', f"ORDER_{int(time.time())}")
                
                logger.info(f"接收訂單: {order_id} - 等待 ROS2 處理")
                
                # 處理訂單中的每個藥物
                processed_medicines = []
                for medicine in order_data.get('medicines', []):
                    medicine_name = medicine.get('name', '')
                    quantity = medicine.get('quantity', 1)
                    
                    # 獲取藥物位置和類型
                    locate = self._get_medicine_location(medicine_name)
                    prompt = self._get_medicine_prompt(medicine_name)
                    
                    processed_medicines.append({
                        'name': medicine_name,
                        'amount': quantity,
                        'locate': locate,
                        'prompt': prompt
                    })
                
                # 設置當前訂單（但不自動處理）
                self.current_order = {
                    "order_id": order_id,
                    "medicines": processed_medicines,
                    "status": "waiting_for_ros2",
                    "created_at": time.strftime("%Y-%m-%d %H:%M:%S")
                }
                
                # 創建訂單格式化器
                formatter = OrderFormatter(order_id, processed_medicines)
                order_yaml = formatter.to_yaml_format()
                
                return f"訂單已接收，等待 ROS2 處理:\n\n{order_yaml}"
                
            except Exception as e:
                logger.error(f"處理訂單失敗: {e}")
                return f"錯誤: 處理訂單失敗 - {str(e)}"
    
    def _get_medicine_location(self, medicine_name: str) -> List[int]:
        """獲取藥物位置（可供您自定義）"""
        # 使用哈希算法分配位置，您可以替換為實際的庫存查詢
        hash_value = hash(medicine_name) % 100
        row = (hash_value // 10) + 1
        col = (hash_value % 10) + 1
        return [row, col]
    
    def _get_medicine_prompt(self, medicine_name: str) -> str:
        """獲取藥物提示符（可供您自定義）"""
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
    
    def complete_order(self, order_id: str) -> Dict:
        """標記訂單完成 - 供您的 ROS2 系統調用"""
        with self.processing_lock:
            if self.current_order and self.current_order.get('order_id') == order_id:
                logger.info(f"訂單 {order_id} 已完成")
                completed_order = self.current_order.copy()
                self.current_order = None
                
                # 通知 FastAPI 更新狀態（如果有 prescription_id）
                if 'prescription_id' in completed_order:
                    self._notify_prescription_completion(completed_order['prescription_id'])
                
                return {
                    "success": True,
                    "message": f"訂單 {order_id} 已完成",
                    "completed_order": completed_order
                }
            else:
                return {
                    "success": False,
                    "message": f"找不到訂單 {order_id} 或不是當前訂單"
                }
    
    def _notify_prescription_completion(self, prescription_id: int):
        """通知 FastAPI 更新處方籤狀態"""
        try:
            response = requests.post(
                f"{self.base_url}/api/ros2/complete-order",
                json={
                    "prescription_id": prescription_id,
                    "status": "completed",
                    "completed_at": time.strftime("%Y-%m-%d %H:%M:%S")
                },
                timeout=5
            )
            
            if response.status_code == 200:
                logger.info(f"處方籤 {prescription_id} 狀態已更新")
            else:
                logger.warning(f"更新處方籤狀態失敗: {response.status_code}")
                
        except Exception as e:
            logger.error(f"通知處方籤完成時發生錯誤: {e}")
    
    def get_current_order(self) -> Optional[Dict]:
        """獲取當前訂單 - 供您的 ROS2 系統查詢"""
        with self.processing_lock:
            return self.current_order.copy() if self.current_order else None
    
    def get_status(self) -> Dict:
        """獲取接口狀態"""
        return {
            "service_name": "ROS2 接口 - 最終版",
            "status": "ready",
            "current_order": self.current_order.get('order_id') if self.current_order else None,
            "processing": self.current_order is not None,
            "base_url": self.base_url,
            "mode": "interface_only",
            "message": "等待 ROS2 整合"
        }
    
    def stop(self):
        """停止接口"""
        self.is_running = False
        logger.info("ROS2 接口已停止")

# 全域接口實例
_interface_instance = None

def get_ros2_interface() -> ROS2InterfaceFinal:
    """獲取 ROS2 接口實例（單例模式）"""
    global _interface_instance
    if _interface_instance is None:
        _interface_instance = ROS2InterfaceFinal()
    return _interface_instance

# ==================== 供您整合使用的函數 ====================

def ros2_query_medicine(medicine_name: str) -> str:
    """
    ROS2 藥物查詢接口
    
    Args:
        medicine_name: 藥物名稱
        
    Returns:
        YAML 格式的藥物詳細資訊
    """
    interface = get_ros2_interface()
    return interface.query_medicine_detail(medicine_name)

def ros2_process_order(order_id: str, medicines: List[Dict]) -> str:
    """
    ROS2 訂單處理接口
    
    Args:
        order_id: 訂單編號
        medicines: 藥物列表 [{"name": "藥名", "quantity": 數量}, ...]
        
    Returns:
        YAML 格式的訂單詳情
    """
    interface = get_ros2_interface()
    order_data = {
        "order_id": order_id,
        "medicines": medicines
    }
    return interface.process_order(order_data)

def ros2_complete_order(order_id: str) -> Dict:
    """
    ROS2 訂單完成接口 - 您的 ROS2 系統完成處理後調用
    
    Args:
        order_id: 訂單編號
        
    Returns:
        完成狀態資訊
    """
    interface = get_ros2_interface()
    return interface.complete_order(order_id)

def ros2_get_current_order() -> Optional[Dict]:
    """
    獲取當前待處理訂單 - 您的 ROS2 系統查詢用
    
    Returns:
        當前訂單資訊或 None
    """
    interface = get_ros2_interface()
    return interface.get_current_order()

def ros2_get_status() -> Dict:
    """
    獲取 ROS2 接口狀態
    
    Returns:
        狀態資訊
    """
    interface = get_ros2_interface()
    return interface.get_status()

def main():
    """測試接口功能"""
    logging.basicConfig(level=logging.INFO)
    
    print("🔌 ROS2 接口 - 最終版測試")
    print("=" * 50)
    print("這個版本不會自動模擬，但提供完整接口供您整合")
    print()
    
    # 顯示可用接口
    print("📋 可用接口函數:")
    print("   • ros2_query_medicine(medicine_name)")
    print("   • ros2_process_order(order_id, medicines)")
    print("   • ros2_complete_order(order_id)")
    print("   • ros2_get_current_order()")
    print("   • ros2_get_status()")
    print()
    
    # 顯示狀態
    status = ros2_get_status()
    print("🔍 當前狀態:")
    for key, value in status.items():
        print(f"   {key}: {value}")
    
    print("\n✅ 接口已準備完成，等待您的 ROS2 整合！")

if __name__ == "__main__":
    main()
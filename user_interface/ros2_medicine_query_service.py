#!/usr/bin/env python3
"""
ROS2 藥物詳細查詢服務
根據藥物名稱返回詳細的藥物資訊，格式化為您指定的格式
"""

import json
import time
import threading
import requests
from typing import Dict, List, Optional, Any
import logging

logger = logging.getLogger("ros2_medicine_query")

class MedicineDetail:
    """藥物詳細資訊類"""
    
    def __init__(self, data: Dict[str, Any]):
        self.raw_data = data
        
    def to_yaml_format(self, medicine_name: str) -> str:
        """轉換為指定的 YAML 格式"""
        # 從 raw_data 提取資訊
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

class OrderDetail:
    """訂單詳細資訊類"""
    
    def __init__(self, order_id: str, medicines: List[Dict]):
        self.order_id = order_id
        self.medicines = medicines
        
    def to_yaml_format(self) -> str:
        """轉換為指定的訂單 YAML 格式"""
        yaml_content = f'order_id: "{self.order_id}"\nmedicine:\n'
        
        for med in self.medicines:
            yaml_content += f"""  - name: {med.get('name', 'Unknown')}
    amount: {med.get('amount', 0)}
    locate: {med.get('locate', [1, 1])}
    prompt: {med.get('prompt', 'tablet')}

"""
        return yaml_content.rstrip()

class ROS2MedicineQueryService:
    """ROS2 藥物查詢服務"""
    
    def __init__(self, fastapi_base_url: str = "http://localhost:8001"):
        self.base_url = fastapi_base_url
        self.is_running = True
        self.current_order = None
        self.processing_lock = threading.Lock()
        
        # 啟動背景服務
        self.service_thread = threading.Thread(target=self._background_service, daemon=True)
        self.service_thread.start()
        
        logger.info("ROS2 藥物查詢服務已啟動")
    
    def query_medicine_detail(self, medicine_name: str) -> str:
        """查詢藥物詳細資訊並返回 YAML 格式"""
        try:
            # 首先查詢基本藥物資訊
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
            
            # 創建 MedicineDetail 物件並轉換格式
            medicine_detail = MedicineDetail(combined_data)
            return medicine_detail.to_yaml_format(medicine_name)
            
        except Exception as e:
            logger.error(f"查詢藥物詳細資訊失敗: {e}")
            return f"錯誤: 查詢失敗 - {str(e)}"
    
    def process_order(self, order_data: Dict) -> str:
        """處理訂單請求"""
        with self.processing_lock:
            if self.current_order is not None:
                return "錯誤: 目前正在處理其他訂單，請稍後再試"
            
            try:
                # 設置當前訂單
                self.current_order = order_data
                order_id = order_data.get('order_id', f"ORDER_{int(time.time())}")
                
                logger.info(f"開始處理訂單: {order_id}")
                
                # 處理訂單中的每個藥物
                processed_medicines = []
                for medicine in order_data.get('medicines', []):
                    medicine_name = medicine.get('name', '')
                    quantity = medicine.get('quantity', 1)
                    
                    # 查詢藥物位置資訊（模擬）
                    locate = self._get_medicine_location(medicine_name)
                    prompt = self._get_medicine_prompt(medicine_name)
                    
                    processed_medicines.append({
                        'name': medicine_name,
                        'amount': quantity,
                        'locate': locate,
                        'prompt': prompt
                    })
                
                # 創建訂單物件
                order_detail = OrderDetail(order_id, processed_medicines)
                order_yaml = order_detail.to_yaml_format()
                
                # 模擬處理時間
                threading.Thread(target=self._process_order_async, args=(order_id,), daemon=True).start()
                
                return f"訂單已接收並開始處理:\n\n{order_yaml}"
                
            except Exception as e:
                self.current_order = None
                logger.error(f"處理訂單失敗: {e}")
                return f"錯誤: 處理訂單失敗 - {str(e)}"
    
    def _get_medicine_location(self, medicine_name: str) -> List[int]:
        """獲取藥物位置（模擬）"""
        # 這裡可以連接到實際的庫存管理系統
        # 目前使用簡單的哈希算法模擬位置
        hash_value = hash(medicine_name) % 100
        row = (hash_value // 10) + 1
        col = (hash_value % 10) + 1
        return [row, col]
    
    def _get_medicine_prompt(self, medicine_name: str) -> str:
        """獲取藥物提示符（模擬）"""
        # 根據藥物名稱返回不同的提示符
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
    
    def _process_order_async(self, order_id: str):
        """異步處理訂單"""
        try:
            # 模擬處理時間（10-20秒）
            processing_time = 15
            logger.info(f"訂單 {order_id} 開始處理，預計需要 {processing_time} 秒")
            
            for i in range(processing_time):
                if not self.is_running:
                    break
                time.sleep(1)
                if i % 3 == 0:
                    logger.info(f"訂單 {order_id} 處理進度: {((i+1)/processing_time)*100:.1f}%")
            
            # 處理完成，通知 FastAPI 更新狀態
            self._complete_order(order_id)
            
        except Exception as e:
            logger.error(f"異步處理訂單 {order_id} 失敗: {e}")
        finally:
            with self.processing_lock:
                self.current_order = None
    
    def _complete_order(self, order_id: str):
        """完成訂單處理"""
        try:
            # 提取處方籤 ID
            prescription_id = None
            if self.current_order:
                prescription_id = self.current_order.get('prescription_id')
            
            if prescription_id:
                # 通知 FastAPI 更新處方籤狀態
                response = requests.post(
                    f"{self.base_url}/api/ros2/complete-order",
                    json={
                        "prescription_id": prescription_id,
                        "order_id": order_id,
                        "status": "completed",
                        "completed_at": time.strftime("%Y-%m-%d %H:%M:%S")
                    },
                    timeout=5
                )
                
                if response.status_code == 200:
                    logger.info(f"訂單 {order_id} 已完成，處方籤 {prescription_id} 狀態已更新")
                else:
                    logger.warning(f"更新處方籤 {prescription_id} 狀態失敗: {response.status_code}")
            
        except Exception as e:
            logger.error(f"完成訂單 {order_id} 時發生錯誤: {e}")
    
    def _background_service(self):
        """背景服務循環"""
        while self.is_running:
            try:
                # 這裡可以添加其他背景任務
                time.sleep(1)
            except Exception as e:
                logger.error(f"背景服務錯誤: {e}")
    
    def get_status(self) -> Dict:
        """獲取服務狀態"""
        return {
            "service_name": "ROS2 藥物查詢服務",
            "status": "running" if self.is_running else "stopped",
            "current_order": self.current_order.get('order_id') if self.current_order else None,
            "processing": self.current_order is not None,
            "base_url": self.base_url
        }
    
    def stop(self):
        """停止服務"""
        self.is_running = False
        logger.info("ROS2 藥物查詢服務已停止")

# 全域服務實例
_service_instance = None

def get_medicine_query_service() -> ROS2MedicineQueryService:
    """獲取服務實例（單例模式）"""
    global _service_instance
    if _service_instance is None:
        _service_instance = ROS2MedicineQueryService()
    return _service_instance

def main():
    """主函數 - 用於測試"""
    logging.basicConfig(level=logging.INFO)
    
    print("🚀 啟動 ROS2 藥物查詢服務測試")
    print("=" * 50)
    
    service = get_medicine_query_service()
    
    # 測試藥物查詢
    print("\n1️⃣ 測試藥物詳細查詢:")
    medicine_name = "測試藥物A"  # 您可以改為實際存在的藥物名稱
    result = service.query_medicine_detail(medicine_name)
    print(result)
    
    # 測試訂單處理
    print("\n2️⃣ 測試訂單處理:")
    test_order = {
        "order_id": "TEST_ORDER_001",
        "prescription_id": 1,
        "medicines": [
            {
                "name": "Antipsychotics",
                "quantity": 87
            },
            {
                "name": "測試藥物B",
                "quantity": 212
            }
        ]
    }
    
    order_result = service.process_order(test_order)
    print(order_result)
    
    # 顯示服務狀態
    print("\n3️⃣ 服務狀態:")
    status = service.get_status()
    for key, value in status.items():
        print(f"   {key}: {value}")
    
    print("\n✅ 測試完成！")
    print("服務將在背景繼續運行...")
    
    try:
        while True:
            time.sleep(10)
            status = service.get_status()
            if status['current_order']:
                print(f"⏳ 正在處理訂單: {status['current_order']}")
            else:
                print("💤 等待訂單中...")
    except KeyboardInterrupt:
        print("\n🛑 停止服務...")
        service.stop()

if __name__ == "__main__":
    main()
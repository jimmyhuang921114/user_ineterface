#!/usr/bin/env python3
"""
醫院藥物管理系統 ROS2 客戶端範例
Hospital Medicine Management System ROS2 Client Example

示範如何使用訂單處理和藥物資訊服務
"""

import rclpy
from rclpy.node import Node
from medicine_interfaces.srv import MedicineOrder, GetMedicineInfo
import time
from datetime import datetime

class MedicineClientExample(Node):
    def __init__(self):
        super().__init__('medicine_client_example')
        
        # 創建服務客戶端
        self.order_client = self.create_client(MedicineOrder, 'medicine_order')
        self.info_client = self.create_client(GetMedicineInfo, 'get_medicine_info')
        
        self.get_logger().info('🏥 藥物管理系統客戶端範例已啟動')

    def send_medicine_order(self):
        """發送藥物訂單範例"""
        self.get_logger().info('📋 準備發送藥物訂單...')
        
        # 等待服務可用
        while not self.order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ 等待訂單處理服務...')
        
        # 創建訂單請求
        request = MedicineOrder.Request()
        request.order_id = f"ORDER_{int(time.time())}"
        request.patient_name = "張小明"
        request.patient_id = "P123456789"
        request.doctor_name = "李醫師"
        request.medicine_names = ["阿斯匹靈", "維生素C"]
        request.dosages = ["100mg", "500mg"]
        request.frequencies = ["每日三次", "每日一次"]
        request.quantities = [30, 60]
        request.notes = "ROS2訂單處理測試"
        request.timestamp = int(time.time())
        
        self.get_logger().info(f'📤 發送訂單: {request.order_id}')
        self.get_logger().info(f'👤 病患: {request.patient_name} ({request.patient_id})')
        self.get_logger().info(f'💊 藥物: {", ".join(request.medicine_names)}')
        
        # 發送請求並等待回應
        future = self.order_client.call_async(request)
        
        # 等待回應
        start_time = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 30:  # 30秒超時
                self.get_logger().error('❌ 訂單處理超時')
                return None
        
        # 處理回應
        response = future.result()
        
        if response.success:
            self.get_logger().info(f'✅ 訂單處理成功!')
            self.get_logger().info(f'📝 處理訊息: {response.message}')
            self.get_logger().info(f'🆔 處理後訂單ID: {response.processed_order_id}')
            self.get_logger().info(f'⏱️ 完成時間: {response.completion_time}')
            
            if response.processed_medicines:
                self.get_logger().info(f'💊 處理的藥物: {", ".join(response.processed_medicines)}')
        else:
            self.get_logger().error(f'❌ 訂單處理失敗: {response.message}')
            if response.error_details:
                self.get_logger().error(f'🔍 錯誤詳情: {response.error_details}')
        
        return response

    def get_medicine_info(self, medicine_name="", include_basic=True, include_detailed=True):
        """獲取藥物資訊範例"""
        if medicine_name:
            self.get_logger().info(f'🔍 查詢藥物資訊: {medicine_name}')
        else:
            self.get_logger().info('🔍 查詢所有藥物資訊')
        
        # 等待服務可用
        while not self.info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ 等待藥物資訊服務...')
        
        # 創建查詢請求
        request = GetMedicineInfo.Request()
        request.medicine_name = medicine_name
        request.include_basic = include_basic
        request.include_detailed = include_detailed
        request.check_availability = True
        
        self.get_logger().info(f'📤 發送查詢請求 (基本: {include_basic}, 詳細: {include_detailed})')
        
        # 發送請求並等待回應
        future = self.info_client.call_async(request)
        
        # 等待回應
        start_time = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 10:  # 10秒超時
                self.get_logger().error('❌ 藥物資訊查詢超時')
                return None
        
        # 處理回應
        response = future.result()
        
        if response.success:
            self.get_logger().info(f'✅ 藥物資訊查詢成功!')
            self.get_logger().info(f'📝 查詢訊息: {response.message}')
            self.get_logger().info(f'📊 總數量: {response.total_count}')
            self.get_logger().info(f'⏱️ 查詢時間: {response.query_time}')
            
            # 顯示基本藥物資訊
            if response.basic_medicines:
                self.get_logger().info(f'💊 基本藥物 ({len(response.basic_medicines)} 種):')
                for med in response.basic_medicines[:3]:  # 只顯示前3個
                    self.get_logger().info(f'  - {med.name}: 庫存 {med.amount}, 位置 {med.position}')
                    if med.prompt:
                        self.get_logger().info(f'    🤖 AI提示: {med.prompt[:50]}...')
            
            # 顯示詳細藥物資訊
            if response.detailed_medicines:
                self.get_logger().info(f'🔬 詳細藥物 ({len(response.detailed_medicines)} 種):')
                for med in response.detailed_medicines[:3]:  # 只顯示前3個
                    self.get_logger().info(f'  - {med.medicine_name}: {med.category}')
                    if med.description:
                        self.get_logger().info(f'    📄 描述: {med.description[:50]}...')
        else:
            self.get_logger().error(f'❌ 藥物資訊查詢失敗: {response.message}')
        
        return response

    def run_demo(self):
        """運行示範"""
        self.get_logger().info('🚀 開始醫院藥物管理系統ROS2示範')
        
        # 示範1: 查詢所有藥物資訊
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('🔍 示範1: 查詢所有藥物資訊')
        self.get_logger().info('='*50)
        self.get_medicine_info()
        
        time.sleep(2)
        
        # 示範2: 查詢特定藥物資訊
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('🔍 示範2: 查詢特定藥物資訊')
        self.get_logger().info('='*50)
        self.get_medicine_info("阿斯匹靈")
        
        time.sleep(2)
        
        # 示範3: 發送藥物訂單
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('📋 示範3: 發送藥物訂單')
        self.get_logger().info('='*50)
        self.send_medicine_order()
        
        time.sleep(2)
        
        # 示範4: 只查詢基本藥物資訊
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('🔍 示範4: 只查詢基本藥物資訊')
        self.get_logger().info('='*50)
        self.get_medicine_info(include_detailed=False)
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('🎉 示範完成!')
        self.get_logger().info('='*50)

def main(args=None):
    rclpy.init(args=args)
    
    client = MedicineClientExample()
    
    try:
        # 運行示範
        client.run_demo()
        
        # 持續運行以處理回調
        client.get_logger().info('⏳ 客戶端保持運行中... (按 Ctrl+C 停止)')
        rclpy.spin(client)
        
    except KeyboardInterrupt:
        client.get_logger().info('🛑 客戶端正在關閉...')
    except Exception as e:
        client.get_logger().error(f'❌ 客戶端錯誤: {str(e)}')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
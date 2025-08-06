#!/usr/bin/env python3
"""
Medicine Management Client - 自動輪詢處方並發送訂單處理
Auto-polling prescription and sending order processing

此節點會定時從FastAPI查詢新的處方籤，並將其發送到medicine_order服務處理
"""

import rclpy
from rclpy.node import Node
from rclpy.client import Client
import requests
import json
import time
from datetime import datetime
from medicine_interfaces.srv import MedicineOrder
from std_msgs.msg import String

class MedicineManagementClient(Node):
    def __init__(self):
        super().__init__('medicine_management_client')
        
        # 配置參數
        self.api_base_url = "http://localhost:8000/api"
        self.polling_interval = 10.0  # 10秒輪詢一次
        self.processed_prescriptions = set()  # 已處理的處方籤ID
        
        # 創建medicine_order服務客戶端
        self.medicine_order_client = self.create_client(
            MedicineOrder, 
            'medicine_order'
        )
        
        # 創建狀態發布者
        self.status_publisher = self.create_publisher(
            String,
            'prescription_polling_status',
            10
        )
        
        # 等待服務可用
        self.get_logger().info('🔄 等待medicine_order服務...')
        while not self.medicine_order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ medicine_order服務未就緒，等待中...')
        
        # 創建定時器進行輪詢
        self.polling_timer = self.create_timer(
            self.polling_interval,
            self.poll_prescriptions
        )
        
        self.get_logger().info(f'🚀 Medicine Management Client 已啟動')
        self.get_logger().info(f'📡 輪詢間隔: {self.polling_interval}秒')
        self.get_logger().info(f'🌐 API地址: {self.api_base_url}')
    
    def poll_prescriptions(self):
        """定時輪詢處方籤"""
        try:
            # 查詢所有處方籤
            response = requests.get(f"{self.api_base_url}/prescription/", timeout=5)
            
            if response.status_code != 200:
                self.get_logger().warning(f'❌ API請求失敗: {response.status_code}')
                return
            
            data = response.json()
            prescriptions = data.get('prescriptions', [])
            
            # 處理新的處方籤
            new_prescriptions = []
            for prescription in prescriptions:
                # 使用處方籤的唯一標識符 (可能是創建時間 + 病患名稱)
                prescription_id = self.generate_prescription_id(prescription)
                
                if prescription_id not in self.processed_prescriptions:
                    new_prescriptions.append(prescription)
                    self.processed_prescriptions.add(prescription_id)
            
            if new_prescriptions:
                self.get_logger().info(f'📋 發現 {len(new_prescriptions)} 張新處方籤')
                
                for prescription in new_prescriptions:
                    self.process_prescription(prescription)
            else:
                self.get_logger().debug('📊 無新處方籤')
            
            # 發布狀態
            status_msg = String()
            status_msg.data = f"輪詢完成 - 處方總數: {len(prescriptions)}, 新處方: {len(new_prescriptions)}"
            self.status_publisher.publish(status_msg)
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'🌐 網路請求錯誤: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'❌ 輪詢錯誤: {str(e)}')
    
    def generate_prescription_id(self, prescription):
        """生成處方籤唯一ID"""
        # 使用病患名稱、創建時間等生成唯一ID
        patient_name = prescription.get('patient_name', '')
        created_at = prescription.get('created_at', '')
        medicines_count = len(prescription.get('medicines', []))
        
        return f"{patient_name}_{created_at}_{medicines_count}"
    
    def process_prescription(self, prescription):
        """處理單個處方籤，發送到medicine_order服務"""
        try:
            self.get_logger().info(f'🏥 處理處方籤: {prescription.get("patient_name")}')
            
            # 構建medicine_order請求
            request = MedicineOrder.Request()
            
            # 填充訂單資料
            request.order_id = f"ORDER_{int(time.time())}_{prescription.get('patient_name', 'UNKNOWN')}"
            request.patient_name = prescription.get('patient_name', '')
            request.patient_id = prescription.get('patient_id', '')
            request.doctor_name = prescription.get('doctor_name', '')
            
            # 處理藥物列表
            medicines = prescription.get('medicines', [])
            request.medicine_names = []
            request.dosages = []
            request.frequencies = []
            request.quantities = []
            
            for medicine in medicines:
                if len(medicine) >= 3:  # 確保有足夠的資料
                    request.medicine_names.append(medicine[0])  # 藥物名稱
                    request.dosages.append(medicine[1])         # 個數 (現在用作劑量)
                    request.frequencies.append("依醫囑")         # 頻率 (固定值)
                    request.quantities.append(int(medicine[1]) if medicine[1].isdigit() else 1)  # 數量
            
            request.notes = f"天數: {medicine[2] if len(medicine) > 2 else '未指定'}"
            if len(medicine) > 3:
                request.notes += f", 備註: {medicine[3]}"
            
            request.timestamp = int(time.time())
            
            # 發送異步請求
            future = self.medicine_order_client.call_async(request)
            future.add_done_callback(
                lambda f, pid=prescription.get('patient_name'): self.order_response_callback(f, pid)
            )
            
            self.get_logger().info(f'📤 已發送訂單處理請求: {request.order_id}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 處理處方籤錯誤: {str(e)}')
    
    def order_response_callback(self, future, patient_name):
        """訂單處理回應回調"""
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info(f'✅ 訂單處理成功: {patient_name}')
                self.get_logger().info(f'📋 處理訊息: {response.message}')
                self.get_logger().info(f'🆔 訂單ID: {response.processed_order_id}')
            else:
                self.get_logger().error(f'❌ 訂單處理失敗: {patient_name}')
                self.get_logger().error(f'💬 錯誤訊息: {response.message}')
                if response.error_details:
                    self.get_logger().error(f'📝 錯誤詳情: {response.error_details}')
        
        except Exception as e:
            self.get_logger().error(f'❌ 訂單回應處理錯誤: {str(e)}')
    
    def get_prescription_status_summary(self):
        """獲取處方籤狀態摘要"""
        try:
            response = requests.get(f"{self.api_base_url}/prescription/", timeout=5)
            if response.status_code == 200:
                data = response.json()
                total = len(data.get('prescriptions', []))
                processed = len(self.processed_prescriptions)
                
                self.get_logger().info(f'📊 處方籤狀態摘要:')
                self.get_logger().info(f'   總處方數: {total}')
                self.get_logger().info(f'   已處理數: {processed}')
                self.get_logger().info(f'   待處理數: {total - processed}')
        
        except Exception as e:
            self.get_logger().error(f'❌ 獲取狀態摘要失敗: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    client = MedicineManagementClient()
    
    try:
        # 運行一次狀態摘要
        client.get_prescription_status_summary()
        
        # 開始輪詢
        rclpy.spin(client)
    
    except KeyboardInterrupt:
        client.get_logger().info('🛑 收到中斷信號，正在關閉...')
    
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
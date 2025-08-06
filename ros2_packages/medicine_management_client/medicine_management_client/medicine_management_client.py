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
        self.processed_prescriptions = set()  # 已處理的處方籤ID
        self.current_batch = []  # 當前批次的處方籤
        self.is_processing = False  # 是否正在處理中
        self.batch_check_interval = 5.0  # 檢查新批次的間隔
        
        # 創建medicine_order服務客戶端
        self.medicine_order_client = self.create_client(
            MedicineOrder, 
            'medicine_order'
        )
        
        # 創建狀態發布者
        self.status_publisher = self.create_publisher(
            String,
            'prescription_batch_status',
            10
        )
        
        # 等待服務可用
        self.get_logger().info('🔄 等待medicine_order服務...')
        while not self.medicine_order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ medicine_order服務未就緒，等待中...')
        
        # 創建檢查新批次的定時器
        self.batch_check_timer = self.create_timer(
            self.batch_check_interval,
            self.check_for_new_batch
        )
        
        self.get_logger().info(f'🚀 Medicine Management Client 已啟動 (串行批次處理模式)')
        self.get_logger().info(f'📡 批次檢查間隔: {self.batch_check_interval}秒')
        self.get_logger().info(f'🌐 API地址: {self.api_base_url}')
        self.get_logger().info(f'⚙️  處理模式: 串行處理 - 每次處理完整批次，一次一筆')
    
    def check_for_new_batch(self):
        """檢查是否有新的批次需要處理"""
        if self.is_processing:
            self.get_logger().debug('⏳ 正在處理中，跳過批次檢查')
            return
        
        try:
            # 查詢所有處方籤
            response = requests.get(f"{self.api_base_url}/prescription/", timeout=5)
            
            if response.status_code != 200:
                self.get_logger().warning(f'❌ API請求失敗: {response.status_code}')
                return
            
            data = response.json()
            prescriptions = data.get('prescriptions', [])
            
            # 找出新的處方籤
            new_prescriptions = []
            for prescription in prescriptions:
                prescription_id = self.generate_prescription_id(prescription)
                
                if prescription_id not in self.processed_prescriptions:
                    new_prescriptions.append(prescription)
            
            if new_prescriptions:
                self.get_logger().info(f'📦 發現新批次: {len(new_prescriptions)} 張處方籤')
                self.current_batch = new_prescriptions.copy()
                self.start_batch_processing()
            else:
                self.get_logger().debug('📊 無新處方籤，等待中...')
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'🌐 網路請求錯誤: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'❌ 批次檢查錯誤: {str(e)}')
    
    def start_batch_processing(self):
        """開始串行處理當前批次"""
        if not self.current_batch:
            self.get_logger().info('📭 無處方籤需要處理')
            return
        
        self.is_processing = True
        batch_size = len(self.current_batch)
        
        self.get_logger().info(f'🚀 開始串行處理批次 - 共 {batch_size} 張處方籤')
        self.get_logger().info(f'⚙️  處理模式: 依序處理，每次等待完成後再處理下一筆')
        
        # 發布批次開始狀態
        status_msg = String()
        status_msg.data = f"開始處理批次 - 總數: {batch_size}"
        self.status_publisher.publish(status_msg)
        
        # 開始處理第一筆
        self.process_next_prescription()
    
    def process_next_prescription(self):
        """處理下一張處方籤"""
        if not self.current_batch:
            self.finish_batch_processing()
            return
        
        # 取出第一張處方籤
        prescription = self.current_batch.pop(0)
        remaining = len(self.current_batch)
        
        self.get_logger().info(f'🏥 處理處方籤: {prescription.get("patient_name")} (剩餘: {remaining})')
        
        # 標記為已處理
        prescription_id = self.generate_prescription_id(prescription)
        self.processed_prescriptions.add(prescription_id)
        
        # 處理這張處方籤
        self.process_prescription_sync(prescription)
    
    def finish_batch_processing(self):
        """完成批次處理"""
        self.is_processing = False
        self.get_logger().info('✅ 批次處理完成，等待下一個批次')
        
        # 發布批次完成狀態
        status_msg = String()
        status_msg.data = "批次處理完成，等待新批次"
        self.status_publisher.publish(status_msg)
        
        # 顯示處理統計
        self.get_prescription_status_summary()
    
    def generate_prescription_id(self, prescription):
        """生成處方籤唯一ID"""
        # 使用病患名稱、創建時間等生成唯一ID
        patient_name = prescription.get('patient_name', '')
        created_at = prescription.get('created_at', '')
        medicines_count = len(prescription.get('medicines', []))
        
        return f"{patient_name}_{created_at}_{medicines_count}"
    
    def process_prescription_sync(self, prescription):
        """同步處理單個處方籤，等待完成後再處理下一筆"""
        try:
            patient_name = prescription.get("patient_name")
            self.get_logger().info(f'🏥 開始處理處方籤: {patient_name}')
            
            # 構建medicine_order請求
            request = MedicineOrder.Request()
            
            # 填充訂單資料
            request.order_id = f"ORDER_{int(time.time())}_{patient_name}"
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
            
            self.get_logger().info(f'📤 發送訂單處理請求: {request.order_id}')
            
            # 同步調用服務，等待完成
            future = self.medicine_order_client.call_async(request)
            
            # 等待服務回應
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f'✅ 處方籤處理成功: {patient_name}')
                    self.get_logger().info(f'📋 訊息: {response.message}')
                    self.get_logger().info(f'🆔 訂單ID: {response.processed_order_id}')
                else:
                    self.get_logger().error(f'❌ 處方籤處理失敗: {patient_name}')
                    self.get_logger().error(f'💬 錯誤: {response.message}')
            else:
                self.get_logger().error(f'⏰ 處方籤處理超時: {patient_name}')
            
            # 處理完成，繼續下一筆
            self.get_logger().info(f'➡️  準備處理下一張處方籤...')
            self.process_next_prescription()
            
        except Exception as e:
            self.get_logger().error(f'❌ 處理處方籤錯誤: {str(e)}')
            # 即使出錯也要繼續處理下一筆
            self.process_next_prescription()
    

    
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
        
        client.get_logger().info('📡 開始監控新處方籤批次...')
        client.get_logger().info('⚙️  處理模式: 串行批次處理 - 依序完成每張處方籤')
        
        # 開始批次監控和處理
        rclpy.spin(client)
    
    except KeyboardInterrupt:
        client.get_logger().info('🛑 收到中斷信號，正在關閉...')
    
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
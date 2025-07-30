#!/usr/bin/env python3
"""
ROS2處方服務節點
ROS2 Prescription Service Node

功能：
1. 調用最新的待處理處方
2. 回傳藥品處理結果到系統
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests
import time
from datetime import datetime

API_BASE = "http://localhost:8000"

class PrescriptionServiceNode(Node):
    def __init__(self):
        super().__init__('prescription_service_node')
        
        self.get_logger().info('🏥 ROS2處方服務節點啟動')
        
        # 創建發布者 - 發布待處理的處方
        self.pending_prescription_publisher = self.create_publisher(
            String, 
            'pending_prescriptions', 
            10
        )
        
        # 創建訂閱者 - 接收處理結果
        self.processing_result_subscriber = self.create_subscription(
            String,
            'prescription_processing_result',
            self.processing_result_callback,
            10
        )
        
        # 定時器 - 每30秒檢查待處理的處方
        self.check_timer = self.create_timer(30.0, self.check_pending_prescriptions)
        
        # 追蹤已發送的處方
        self.sent_prescriptions = set()
        
        self.get_logger().info('📡 ROS2處方服務節點準備就緒')
        self.get_logger().info('   - 發布主題: pending_prescriptions')
        self.get_logger().info('   - 訂閱主題: prescription_processing_result')
        self.get_logger().info('   - 檢查間隔: 30秒')

    def check_pending_prescriptions(self):
        """檢查待處理的處方"""
        try:
            # 獲取待處理的處方
            response = requests.get(f"{API_BASE}/api/prescription/status/pending", timeout=5)
            
            if response.status_code == 200:
                pending_prescriptions = response.json()
                
                if pending_prescriptions:
                    self.get_logger().info(f'📋 發現 {len(pending_prescriptions)} 個待處理處方')
                    
                    # 發布最新的待處理處方
                    for prescription in pending_prescriptions:
                        prescription_id = prescription['id']
                        
                        # 避免重複發送同一個處方
                        if prescription_id not in self.sent_prescriptions:
                            self.publish_prescription(prescription)
                            self.sent_prescriptions.add(prescription_id)
                else:
                    self.get_logger().info('✅ 目前沒有待處理的處方')
            else:
                self.get_logger().warning(f'⚠️  獲取待處理處方失敗: {response.status_code}')
                
        except Exception as e:
            self.get_logger().error(f'❌ 檢查待處理處方錯誤: {str(e)}')

    def publish_prescription(self, prescription):
        """發布處方資訊到ROS2主題"""
        try:
            # 構建要發送的處方資訊
            prescription_data = {
                'prescription_id': prescription['id'],
                'patient_name': prescription['patient_name'],
                'doctor_name': prescription['doctor_name'],
                'diagnosis': prescription.get('diagnosis', ''),
                'medicines': prescription['medicines'],
                'prescription_date': prescription.get('prescription_date', ''),
                'created_time': prescription.get('created_time', ''),
                'status': prescription.get('status', 'pending'),
                'ros2_timestamp': datetime.now().isoformat()
            }
            
            # 轉換為JSON字符串
            json_message = json.dumps(prescription_data, ensure_ascii=False, indent=2)
            
            # 創建ROS2消息
            msg = String()
            msg.data = json_message
            
            # 發布消息
            self.pending_prescription_publisher.publish(msg)
            
            self.get_logger().info(f'📤 已發布處方 #{prescription["id"]} - {prescription["patient_name"]}')
            self.get_logger().info(f'   醫師: {prescription["doctor_name"]}')
            self.get_logger().info(f'   藥物數量: {len(prescription["medicines"])}')
            
            # 將處方狀態更新為"processing"（避免重複發送）
            self.update_prescription_status(prescription['id'], 'processing', 'ROS2系統')
            
        except Exception as e:
            self.get_logger().error(f'❌ 發布處方錯誤: {str(e)}')

    def processing_result_callback(self, msg):
        """處理ROS2回傳的處理結果"""
        try:
            # 解析收到的處理結果
            result_data = json.loads(msg.data)
            
            prescription_id = result_data.get('prescription_id')
            processing_status = result_data.get('status', 'completed')
            processing_notes = result_data.get('notes', '')
            processed_by = result_data.get('processed_by', 'ROS2系統')
            
            self.get_logger().info(f'📥 收到處方 #{prescription_id} 的處理結果')
            self.get_logger().info(f'   狀態: {processing_status}')
            self.get_logger().info(f'   處理者: {processed_by}')
            
            if processing_notes:
                self.get_logger().info(f'   備註: {processing_notes}')
            
            # 更新處方狀態
            success = self.update_prescription_status(
                prescription_id, 
                processing_status, 
                processed_by, 
                processing_notes
            )
            
            if success:
                self.get_logger().info(f'✅ 處方 #{prescription_id} 狀態更新成功')
                # 從已發送清單中移除
                self.sent_prescriptions.discard(prescription_id)
            else:
                self.get_logger().error(f'❌ 處方 #{prescription_id} 狀態更新失敗')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ 解析處理結果JSON錯誤: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'❌ 處理結果回調錯誤: {str(e)}')

    def update_prescription_status(self, prescription_id, status, updated_by, notes=''):
        """更新處方狀態"""
        try:
            update_data = {
                'status': status,
                'updated_by': updated_by,
                'notes': notes or f'ROS2系統更新狀態為: {status}'
            }
            
            response = requests.put(
                f"{API_BASE}/api/prescription/{prescription_id}/status",
                json=update_data,
                headers={'Content-Type': 'application/json'},
                timeout=10
            )
            
            if response.status_code == 200:
                return True
            else:
                self.get_logger().error(f'更新狀態失敗: {response.status_code} - {response.text}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'更新處方狀態錯誤: {str(e)}')
            return False

    def get_prescription_statistics(self):
        """獲取處方統計資訊"""
        try:
            response = requests.get(f"{API_BASE}/api/prescription/", timeout=5)
            if response.status_code == 200:
                all_prescriptions = response.json()
                
                stats = {
                    'total': len(all_prescriptions),
                    'pending': len([p for p in all_prescriptions if p.get('status') == 'pending']),
                    'processing': len([p for p in all_prescriptions if p.get('status') == 'processing']),
                    'completed': len([p for p in all_prescriptions if p.get('status') == 'completed'])
                }
                
                return stats
            else:
                return None
                
        except Exception as e:
            self.get_logger().error(f'獲取統計資訊錯誤: {str(e)}')
            return None

def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    
    try:
        # 創建節點
        node = PrescriptionServiceNode()
        
        # 測試API連接
        try:
            response = requests.get(f"{API_BASE}/api/system/status", timeout=5)
            if response.status_code == 200:
                node.get_logger().info('✅ API連接正常')
                
                # 顯示初始統計
                stats = node.get_prescription_statistics()
                if stats:
                    node.get_logger().info(f'📊 處方統計: 總數={stats["total"]}, 待處理={stats["pending"]}, 處理中={stats["processing"]}, 已完成={stats["completed"]}')
            else:
                node.get_logger().warning(f'⚠️  API狀態異常: {response.status_code}')
        except Exception as e:
            node.get_logger().error(f'❌ 無法連接API: {str(e)}')
            node.get_logger().info('請確保醫院管理系統伺服器正在運行')
        
        # 執行節點
        node.get_logger().info('🚀 開始監聽處方處理服務...')
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n🛑 收到中斷信號，正在關閉節點...')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print('✅ ROS2處方服務節點已關閉')

if __name__ == '__main__':
    main()
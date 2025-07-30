#!/usr/bin/env python3
"""
ROS2處方處理測試客戶端
ROS2 Prescription Processing Test Client

模擬藥品處理系統，接收待處理處方並回傳處理結果
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random
from datetime import datetime

class PrescriptionProcessorClient(Node):
    def __init__(self):
        super().__init__('prescription_processor_client')
        
        self.get_logger().info('🤖 ROS2處方處理客戶端啟動')
        
        # 訂閱待處理的處方
        self.prescription_subscriber = self.create_subscription(
            String,
            'pending_prescriptions',
            self.prescription_callback,
            10
        )
        
        # 發布處理結果
        self.result_publisher = self.create_publisher(
            String,
            'prescription_processing_result',
            10
        )
        
        # 處理計數器
        self.processed_count = 0
        
        self.get_logger().info('🔧 處方處理客戶端準備就緒')
        self.get_logger().info('   - 訂閱主題: pending_prescriptions')
        self.get_logger().info('   - 發布主題: prescription_processing_result')

    def prescription_callback(self, msg):
        """處理收到的處方"""
        try:
            # 解析收到的處方資料
            prescription_data = json.loads(msg.data)
            prescription_id = prescription_data.get('prescription_id')
            patient_name = prescription_data.get('patient_name')
            medicines = prescription_data.get('medicines', [])
            
            self.get_logger().info(f'📥 收到處方 #{prescription_id} - {patient_name}')
            self.get_logger().info(f'   藥物數量: {len(medicines)}')
            
            # 列出藥物清單
            for i, medicine in enumerate(medicines, 1):
                self.get_logger().info(f'   {i}. {medicine.get("medicine_name", "Unknown")} - {medicine.get("dosage", "Unknown")}')
            
            # 模擬處理時間（5-15秒）
            processing_time = random.randint(5, 15)
            self.get_logger().info(f'⏳ 開始處理，預計需要 {processing_time} 秒...')
            
            # 非阻塞處理（使用定時器）
            self.create_timer(
                processing_time, 
                lambda: self.complete_processing(prescription_data)
            )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ 解析處方JSON錯誤: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'❌ 處理處方錯誤: {str(e)}')

    def complete_processing(self, prescription_data):
        """完成處方處理"""
        try:
            prescription_id = prescription_data.get('prescription_id')
            patient_name = prescription_data.get('patient_name')
            medicines = prescription_data.get('medicines', [])
            
            # 模擬處理結果
            processing_success = random.choice([True, True, True, False])  # 75%成功率
            
            if processing_success:
                status = 'completed'
                notes = self.generate_success_notes(medicines)
                self.get_logger().info(f'✅ 處方 #{prescription_id} 處理完成')
            else:
                status = 'pending'  # 回到待處理狀態
                notes = self.generate_failure_notes()
                self.get_logger().warning(f'⚠️  處方 #{prescription_id} 處理失敗，退回待處理')
            
            # 創建處理結果
            result_data = {
                'prescription_id': prescription_id,
                'patient_name': patient_name,
                'status': status,
                'processed_by': f'ROS2藥品處理系統-{self.get_name()}',
                'processing_time': datetime.now().isoformat(),
                'notes': notes,
                'medicines_processed': len(medicines),
                'success': processing_success
            }
            
            # 發布處理結果
            self.publish_result(result_data)
            
            self.processed_count += 1
            self.get_logger().info(f'📊 累計處理: {self.processed_count} 個處方')
            
        except Exception as e:
            self.get_logger().error(f'❌ 完成處理錯誤: {str(e)}')

    def publish_result(self, result_data):
        """發布處理結果"""
        try:
            # 轉換為JSON字符串
            json_message = json.dumps(result_data, ensure_ascii=False, indent=2)
            
            # 創建ROS2消息
            msg = String()
            msg.data = json_message
            
            # 發布結果
            self.result_publisher.publish(msg)
            
            prescription_id = result_data['prescription_id']
            status = result_data['status']
            
            self.get_logger().info(f'📤 已發布處方 #{prescription_id} 的處理結果: {status}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 發布處理結果錯誤: {str(e)}')

    def generate_success_notes(self, medicines):
        """生成成功處理的備註"""
        medicine_names = [med.get('medicine_name', 'Unknown') for med in medicines]
        
        success_templates = [
            f"藥品配置完成，共 {len(medicines)} 種藥物已備齊",
            f"所有藥物已檢核完畢：{', '.join(medicine_names[:2])}{'等' if len(medicine_names) > 2 else ''}",
            f"藥品包裝完成，請病患至領藥櫃檯領取",
            f"處方調配成功，藥物品質檢驗通過",
            f"配藥完成，包含 {len(medicines)} 項藥物，已做標示說明"
        ]
        
        return random.choice(success_templates)

    def generate_failure_notes(self):
        """生成處理失敗的備註"""
        failure_reasons = [
            "部分藥物庫存不足，需要補貨",
            "藥物過期，需要更換新批次",
            "處方資訊不完整，需要醫師確認",
            "藥物交互作用警示，需要藥師確認",
            "特殊藥物需要額外審核時間"
        ]
        
        return random.choice(failure_reasons)

class PrescriptionMonitorClient(Node):
    """處方監控客戶端 - 僅監聽不處理"""
    def __init__(self):
        super().__init__('prescription_monitor_client')
        
        self.get_logger().info('👁️  ROS2處方監控客戶端啟動')
        
        # 訂閱待處理的處方（僅監控）
        self.prescription_subscriber = self.create_subscription(
            String,
            'pending_prescriptions',
            self.monitor_callback,
            10
        )
        
        # 訂閱處理結果（僅監控）
        self.result_subscriber = self.create_subscription(
            String,
            'prescription_processing_result',
            self.result_monitor_callback,
            10
        )
        
        self.get_logger().info('👀 處方監控客戶端準備就緒')

    def monitor_callback(self, msg):
        """監控待處理的處方"""
        try:
            prescription_data = json.loads(msg.data)
            prescription_id = prescription_data.get('prescription_id')
            patient_name = prescription_data.get('patient_name')
            
            self.get_logger().info(f'👁️  監控到新處方: #{prescription_id} - {patient_name}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 監控處方錯誤: {str(e)}')

    def result_monitor_callback(self, msg):
        """監控處理結果"""
        try:
            result_data = json.loads(msg.data)
            prescription_id = result_data.get('prescription_id')
            status = result_data.get('status')
            success = result_data.get('success', False)
            
            status_icon = '✅' if success else '❌'
            self.get_logger().info(f'{status_icon} 監控到處理結果: #{prescription_id} -> {status}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 監控結果錯誤: {str(e)}')

def main_processor(args=None):
    """處理器客戶端主函數"""
    rclpy.init(args=args)
    
    try:
        node = PrescriptionProcessorClient()
        node.get_logger().info('🚀 開始監聽待處理處方...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n🛑 處理器客戶端停止')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

def main_monitor(args=None):
    """監控客戶端主函數"""
    rclpy.init(args=args)
    
    try:
        node = PrescriptionMonitorClient()
        node.get_logger().info('🚀 開始監控處方流程...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n🛑 監控客戶端停止')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    """主函數 - 選擇運行模式"""
    import sys
    
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if mode == 'monitor':
            print('🚀 啟動監控模式...')
            main_monitor(args)
        elif mode == 'processor':
            print('🚀 啟動處理器模式...')
            main_processor(args)
        else:
            print('❌ 未知模式，請使用: processor 或 monitor')
    else:
        print('🚀 默認啟動處理器模式...')
        main_processor(args)

if __name__ == '__main__':
    main()
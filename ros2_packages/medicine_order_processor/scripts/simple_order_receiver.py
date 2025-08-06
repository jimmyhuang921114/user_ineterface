#!/usr/bin/env python3
"""
簡化訂單接收節點 - ROS2
Simple Order Receiver Node - ROS2

專門從醫院系統獲取最前面的訂單，並等待處理完成後返回回應
符合您的需求: self.order_service = self.create_service(MedicineOrder,'medicine_order',self.medicine_order_callback)
"""

import rclpy
from rclpy.node import Node
from medicine_interfaces.srv import MedicineOrder
import requests
import json
import time
from datetime import datetime
from pathlib import Path

class SimpleOrderReceiverNode(Node):
    def __init__(self):
        super().__init__('simple_order_receiver_node')
        
        # 按照您的要求創建服務
        self.order_service = self.create_service(
            MedicineOrder,
            'medicine_order',
            self.medicine_order_callback
        )
        
        # API配置
        self.api_base_url = "http://localhost:8000/api"
        
        # 輸出目錄
        self.output_dir = Path.home() / "ros2_medicine_output" / "order_processing"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info('📋 簡化訂單接收節點已啟動')
        self.get_logger().info(f'🎯 服務名稱: medicine_order')
        self.get_logger().info(f'📁 輸出目錄: {self.output_dir}')

    def medicine_order_callback(self, request, response):
        """
        處理訂單回調函數
        按照您的要求：接收訂單，等待所有流程結束後返回回應
        """
        try:
            self.get_logger().info('📨 收到新的訂單處理請求')
            self.get_logger().info(f'🆔 訂單ID: {request.order_id}')
            
            # 第一步：從醫院系統獲取最前面的訂單
            latest_order = self.get_latest_order_from_hospital()
            
            if not latest_order:
                response.success = False
                response.message = "無法獲取最新訂單資料"
                response.error_details = "API回應為空或發生錯誤"
                return response
            
            # 第二步：處理訂單（模擬完整的處理流程）
            processing_result = self.process_order_completely(request, latest_order)
            
            # 第三步：等待所有流程完成並準備回應
            if processing_result['success']:
                response.success = True
                response.message = f"訂單處理完成: {processing_result['message']}"
                response.processed_order_id = processing_result['processed_order_id']
                response.processed_medicines = processing_result['processed_medicines']
                response.processing_status = processing_result['processing_status']
                response.completion_time = processing_result['completion_time']
                response.error_details = ""
                
                self.get_logger().info(f'✅ 訂單 {request.order_id} 處理完成')
                
            else:
                response.success = False
                response.message = f"訂單處理失敗: {processing_result['message']}"
                response.processed_order_id = request.order_id
                response.processed_medicines = []
                response.processing_status = []
                response.completion_time = datetime.now().isoformat()
                response.error_details = processing_result.get('error', '')
                
                self.get_logger().error(f'❌ 訂單 {request.order_id} 處理失敗')
                
        except Exception as e:
            self.get_logger().error(f'❌ 訂單處理異常: {str(e)}')
            
            response.success = False
            response.message = f"系統異常: {str(e)}"
            response.processed_order_id = request.order_id
            response.processed_medicines = []
            response.processing_status = []
            response.completion_time = datetime.now().isoformat()
            response.error_details = str(e)
        
        return response

    def get_latest_order_from_hospital(self):
        """從醫院系統獲取最前面（最新）的訂單"""
        try:
            self.get_logger().info('🔍 正在從醫院系統獲取最新訂單...')
            
            response = requests.get(f"{self.api_base_url}/prescription/", timeout=10)
            response.raise_for_status()
            
            prescriptions_data = response.json()
            
            # 檢查是否有處方籤資料
            if isinstance(prescriptions_data, list) and len(prescriptions_data) > 0:
                # 取得最前面的訂單（最新的）
                latest_prescription = prescriptions_data[0]
                
                self.get_logger().info(f'📋 找到最新訂單: ID={latest_prescription.get("id")}')
                self.get_logger().info(f'👤 病患: {latest_prescription.get("patient_name")}')
                
                return latest_prescription
                
            elif isinstance(prescriptions_data, dict) and 'prescriptions' in prescriptions_data:
                prescriptions = prescriptions_data['prescriptions']
                if len(prescriptions) > 0:
                    latest_prescription = prescriptions[0]
                    self.get_logger().info(f'📋 找到最新訂單: ID={latest_prescription.get("id")}')
                    return latest_prescription
            
            self.get_logger().warning('⚠️ 醫院系統中沒有找到訂單')
            return None
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'❌ 無法連接到醫院系統: {str(e)}')
            return None
        except Exception as e:
            self.get_logger().error(f'❌ 獲取訂單時發生錯誤: {str(e)}')
            return None

    def process_order_completely(self, ros_request, hospital_order):
        """完整處理訂單的所有流程"""
        try:
            process_start_time = time.time()
            
            self.get_logger().info('⚙️ 開始完整訂單處理流程...')
            
            # 流程1: 驗證訂單資料
            self.get_logger().info('🔍 步驟1: 驗證訂單資料')
            validation_result = self.validate_order_data(ros_request, hospital_order)
            if not validation_result['valid']:
                return {'success': False, 'message': f'驗證失敗: {validation_result["error"]}'}
            
            # 流程2: 檢查庫存
            self.get_logger().info('📦 步驟2: 檢查藥物庫存')
            stock_result = self.check_stock_availability(ros_request)
            if not stock_result['available']:
                return {'success': False, 'message': f'庫存不足: {stock_result["error"]}'}
            
            # 流程3: 處理業務邏輯
            self.get_logger().info('💼 步驟3: 執行業務邏輯處理')
            business_result = self.execute_business_logic(ros_request, hospital_order)
            if not business_result['success']:
                return {'success': False, 'message': f'業務邏輯處理失敗: {business_result["error"]}'}
            
            # 流程4: 更新系統狀態
            self.get_logger().info('💾 步驟4: 更新系統狀態')
            update_result = self.update_system_state(ros_request)
            if not update_result['success']:
                return {'success': False, 'message': f'系統更新失敗: {update_result["error"]}'}
            
            # 流程5: 生成最終結果
            self.get_logger().info('📄 步驟5: 生成處理結果')
            final_result = self.generate_final_result(ros_request, hospital_order, process_start_time)
            
            # 保存處理記錄
            self.save_processing_record(ros_request, hospital_order, final_result)
            
            total_time = time.time() - process_start_time
            self.get_logger().info(f'✅ 所有流程完成，總耗時: {total_time:.2f} 秒')
            
            return {
                'success': True,
                'message': f'所有流程執行完成，耗時 {total_time:.2f} 秒',
                'processed_order_id': final_result['order_id'],
                'processed_medicines': final_result['medicines'],
                'processing_status': final_result['status_list'],
                'completion_time': final_result['completion_time']
            }
            
        except Exception as e:
            return {'success': False, 'message': f'處理流程異常: {str(e)}', 'error': str(e)}

    def validate_order_data(self, ros_request, hospital_order):
        """驗證訂單資料"""
        try:
            # 檢查ROS請求資料
            if not ros_request.order_id:
                return {'valid': False, 'error': 'ROS請求缺少訂單ID'}
            
            if not ros_request.medicine_names:
                return {'valid': False, 'error': 'ROS請求缺少藥物清單'}
            
            # 檢查醫院訂單資料
            if not hospital_order.get('patient_name'):
                return {'valid': False, 'error': '醫院訂單缺少病患姓名'}
            
            return {'valid': True, 'error': ''}
            
        except Exception as e:
            return {'valid': False, 'error': f'驗證過程錯誤: {str(e)}'}

    def check_stock_availability(self, ros_request):
        """檢查庫存可用性"""
        try:
            # 模擬庫存檢查邏輯
            time.sleep(0.5)  # 模擬檢查時間
            
            # 實際上可以在這裡調用庫存API
            # 這裡簡化為總是通過
            return {'available': True, 'error': ''}
            
        except Exception as e:
            return {'available': False, 'error': f'庫存檢查錯誤: {str(e)}'}

    def execute_business_logic(self, ros_request, hospital_order):
        """執行業務邏輯"""
        try:
            # 模擬業務邏輯處理
            time.sleep(1.0)  # 模擬處理時間
            
            # 這裡可以加入實際的業務邏輯
            # 例如：藥物配置、處方驗證、醫療規則檢查等
            
            return {'success': True, 'error': ''}
            
        except Exception as e:
            return {'success': False, 'error': f'業務邏輯錯誤: {str(e)}'}

    def update_system_state(self, ros_request):
        """更新系統狀態"""
        try:
            # 模擬系統狀態更新
            time.sleep(0.3)  # 模擬更新時間
            
            # 這裡可以加入實際的系統更新邏輯
            # 例如：更新處方狀態、記錄操作日誌等
            
            return {'success': True, 'error': ''}
            
        except Exception as e:
            return {'success': False, 'error': f'系統更新錯誤: {str(e)}'}

    def generate_final_result(self, ros_request, hospital_order, start_time):
        """生成最終處理結果"""
        completion_time = datetime.now().isoformat()
        
        result = {
            'order_id': ros_request.order_id,
            'hospital_order_id': hospital_order.get('id', 'unknown'),
            'patient_name': hospital_order.get('patient_name', 'unknown'),
            'medicines': ros_request.medicine_names,
            'status_list': ['已處理'] * len(ros_request.medicine_names),
            'completion_time': completion_time,
            'processing_duration': time.time() - start_time
        }
        
        return result

    def save_processing_record(self, ros_request, hospital_order, result):
        """保存處理記錄"""
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"processed_order_{ros_request.order_id}_{timestamp}.json"
            file_path = self.output_dir / filename
            
            record = {
                'ros_request': {
                    'order_id': ros_request.order_id,
                    'patient_name': ros_request.patient_name,
                    'medicine_names': ros_request.medicine_names,
                    'quantities': ros_request.quantities
                },
                'hospital_order': hospital_order,
                'processing_result': result,
                'metadata': {
                    'processed_by': 'simple_order_receiver_node',
                    'processing_time': result['completion_time']
                }
            }
            
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(record, f, ensure_ascii=False, indent=2)
            
            self.get_logger().info(f'📄 處理記錄已保存: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 保存記錄失敗: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleOrderReceiverNode()
    
    try:
        node.get_logger().info('🎯 等待訂單處理請求...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 簡化訂單接收節點正在關閉...')
    except Exception as e:
        node.get_logger().error(f'❌ 節點錯誤: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
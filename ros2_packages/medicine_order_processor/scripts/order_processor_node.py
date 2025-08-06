#!/usr/bin/env python3
"""
藥物訂單處理節點 - ROS2
Medicine Order Processor Node - ROS2

接收訂單，處理完整流程，並返回回應
"""

import rclpy
from rclpy.node import Node
from medicine_interfaces.srv import MedicineOrder
import requests
import json
import time
from datetime import datetime
from pathlib import Path

class OrderProcessorNode(Node):
    def __init__(self):
        super().__init__('order_processor_node')
        
        # 創建服務
        self.order_service = self.create_service(
            MedicineOrder,
            'medicine_order',
            self.medicine_order_callback
        )
        
        # API配置
        self.api_base_url = "http://localhost:8000/api"
        
        # 處理記錄
        self.processing_orders = {}
        
        # 輸出目錄配置
        self.output_dir = Path.home() / "ros2_medicine_output" / "processed_orders"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info('🏥 藥物訂單處理節點已啟動')
        self.get_logger().info(f'📁 輸出目錄: {self.output_dir}')
        self.get_logger().info(f'🌐 API地址: {self.api_base_url}')

    def medicine_order_callback(self, request, response):
        """處理藥物訂單服務請求"""
        order_id = request.order_id
        start_time = time.time()
        
        try:
            self.get_logger().info(f'📋 收到訂單處理請求: {order_id}')
            self.get_logger().info(f'👤 病患: {request.patient_name} ({request.patient_id})')
            self.get_logger().info(f'👨‍⚕️ 醫生: {request.doctor_name}')
            self.get_logger().info(f'💊 藥物數量: {len(request.medicine_names)}')
            
            # 添加到處理記錄
            self.processing_orders[order_id] = {
                'start_time': datetime.now().isoformat(),
                'status': 'processing',
                'patient_name': request.patient_name,
                'patient_id': request.patient_id,
                'doctor_name': request.doctor_name
            }
            
            # 驗證訂單資料
            validation_result = self.validate_order(request)
            if not validation_result['valid']:
                response.success = False
                response.message = f"訂單驗證失敗: {validation_result['error']}"
                response.error_details = validation_result['error']
                return response
            
            # 檢查藥物庫存
            stock_check_result = self.check_medicine_stock(request)
            if not stock_check_result['available']:
                response.success = False
                response.message = f"庫存不足: {stock_check_result['error']}"
                response.error_details = stock_check_result['error']
                return response
            
            # 創建處方籤
            prescription_result = self.create_prescription(request)
            if not prescription_result['success']:
                response.success = False
                response.message = f"處方籤創建失敗: {prescription_result['error']}"
                response.error_details = prescription_result['error']
                return response
            
            # 更新庫存
            stock_update_result = self.update_medicine_stock(request)
            if not stock_update_result['success']:
                response.success = False
                response.message = f"庫存更新失敗: {stock_update_result['error']}"
                response.error_details = stock_update_result['error']
                return response
            
            # 生成處理記錄
            processing_record = self.generate_processing_record(request, prescription_result)
            
            # 保存處理記錄
            self.save_processing_record(order_id, processing_record)
            
            # 更新處理狀態
            self.processing_orders[order_id]['status'] = 'completed'
            self.processing_orders[order_id]['end_time'] = datetime.now().isoformat()
            
            # 設置成功回應
            processing_time = time.time() - start_time
            response.success = True
            response.message = f"訂單處理成功，耗時 {processing_time:.2f} 秒"
            response.processed_order_id = order_id
            response.processed_medicines = request.medicine_names
            response.processing_status = ["已處理"] * len(request.medicine_names)
            response.completion_time = datetime.now().isoformat()
            response.error_details = ""
            
            self.get_logger().info(f'✅ 訂單 {order_id} 處理完成 (耗時: {processing_time:.2f}s)')
            
            # 回報狀態到Web API
            self.report_status_to_web(order_id, "completed", response.message, request)
            
        except Exception as e:
            # 處理異常
            self.get_logger().error(f'❌ 訂單處理錯誤: {str(e)}')
            
            response.success = False
            response.message = f"訂單處理異常: {str(e)}"
            response.processed_order_id = order_id
            response.processed_medicines = []
            response.processing_status = []
            response.completion_time = datetime.now().isoformat()
            response.error_details = str(e)
            
            # 回報錯誤狀態到Web API
            self.report_status_to_web(order_id, "failed", response.message, request, str(e))
            
            # 更新錯誤狀態
            if order_id in self.processing_orders:
                self.processing_orders[order_id]['status'] = 'error'
                self.processing_orders[order_id]['error'] = str(e)
        
        return response

    def validate_order(self, request):
        """驗證訂單資料"""
        try:
            # 檢查必要欄位
            if not request.order_id:
                return {'valid': False, 'error': '訂單ID不能為空'}
            
            if not request.patient_name:
                return {'valid': False, 'error': '病患姓名不能為空'}
            
            if not request.patient_id:
                return {'valid': False, 'error': '病患編號不能為空'}
            
            if not request.medicine_names:
                return {'valid': False, 'error': '藥物清單不能為空'}
            
            # 檢查藥物清單長度一致性
            medicine_count = len(request.medicine_names)
            if (len(request.dosages) != medicine_count or
                len(request.frequencies) != medicine_count or
                len(request.quantities) != medicine_count):
                return {'valid': False, 'error': '藥物資料欄位長度不一致'}
            
            # 檢查數量是否為正數
            for i, quantity in enumerate(request.quantities):
                if quantity <= 0:
                    return {'valid': False, 'error': f'藥物 {request.medicine_names[i]} 的數量必須大於0'}
            
            return {'valid': True, 'error': ''}
            
        except Exception as e:
            return {'valid': False, 'error': f'驗證過程發生錯誤: {str(e)}'}

    def check_medicine_stock(self, request):
        """檢查藥物庫存"""
        try:
            # 獲取所有基本藥物資料
            response = requests.get(f"{self.api_base_url}/medicine/basic", timeout=10)
            response.raise_for_status()
            
            medicine_data = response.json()
            medicines = medicine_data.get('medicines', medicine_data) if isinstance(medicine_data, dict) else medicine_data
            
            # 建立藥物庫存映射
            stock_map = {}
            for med in medicines:
                stock_map[med['name']] = med['amount']
            
            # 檢查每個藥物的庫存
            insufficient_medicines = []
            for i, medicine_name in enumerate(request.medicine_names):
                required_quantity = request.quantities[i]
                available_stock = stock_map.get(medicine_name, 0)
                
                if available_stock < required_quantity:
                    insufficient_medicines.append(
                        f"{medicine_name} (需要: {required_quantity}, 庫存: {available_stock})"
                    )
            
            if insufficient_medicines:
                return {
                    'available': False,
                    'error': f"庫存不足的藥物: {', '.join(insufficient_medicines)}"
                }
            
            return {'available': True, 'error': ''}
            
        except Exception as e:
            return {'available': False, 'error': f'庫存檢查失敗: {str(e)}'}

    def create_prescription(self, request):
        """創建處方籤"""
        try:
            # 準備處方籤資料
            prescription_data = {
                'patient_name': request.patient_name,
                'patient_id': request.patient_id,
                'doctor_name': request.doctor_name or '系統醫生',
                'medicines': []
            }
            
            # 添加藥物資料
            for i in range(len(request.medicine_names)):
                medicine_info = {
                    'name': request.medicine_names[i],
                    'dosage': request.dosages[i],
                    'frequency': request.frequencies[i],
                    'quantity': request.quantities[i]
                }
                prescription_data['medicines'].append(medicine_info)
            
            if request.notes:
                prescription_data['notes'] = request.notes
            
            # 發送處方籤創建請求
            response = requests.post(
                f"{self.api_base_url}/prescription/",
                json=prescription_data,
                timeout=10
            )
            response.raise_for_status()
            
            result = response.json()
            return {'success': True, 'prescription_id': result.get('id', 'unknown'), 'error': ''}
            
        except Exception as e:
            return {'success': False, 'prescription_id': None, 'error': f'處方籤創建失敗: {str(e)}'}

    def update_medicine_stock(self, request):
        """更新藥物庫存"""
        try:
            # 為每個藥物減少庫存
            for i, medicine_name in enumerate(request.medicine_names):
                quantity_to_deduct = request.quantities[i]
                
                # 準備庫存調整資料
                stock_adjustment = {
                    'medicine_name': medicine_name,
                    'adjustment_type': 'subtract',
                    'quantity': quantity_to_deduct,
                    'reason': f'處方籤用藥 - 訂單ID: {request.order_id}'
                }
                
                # 發送庫存調整請求
                response = requests.post(
                    f"{self.api_base_url}/medicine/adjust-stock",
                    json=stock_adjustment,
                    timeout=10
                )
                response.raise_for_status()
            
            return {'success': True, 'error': ''}
            
        except Exception as e:
            return {'success': False, 'error': f'庫存更新失敗: {str(e)}'}

    def generate_processing_record(self, request, prescription_result):
        """生成處理記錄"""
        record = {
            'order_info': {
                'order_id': request.order_id,
                'patient_name': request.patient_name,
                'patient_id': request.patient_id,
                'doctor_name': request.doctor_name,
                'notes': request.notes,
                'timestamp': request.timestamp
            },
            'medicines': [],
            'processing_result': {
                'prescription_id': prescription_result.get('prescription_id'),
                'processed_at': datetime.now().isoformat(),
                'status': 'completed',
                'processor_node': 'order_processor_node'
            }
        }
        
        # 添加藥物詳情
        for i in range(len(request.medicine_names)):
            medicine_record = {
                'name': request.medicine_names[i],
                'dosage': request.dosages[i],
                'frequency': request.frequencies[i],
                'quantity': request.quantities[i],
                'status': 'processed'
            }
            record['medicines'].append(medicine_record)
        
        return record

    def save_processing_record(self, order_id, record):
        """保存處理記錄"""
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"order_{order_id}_{timestamp}.json"
            file_path = self.output_dir / filename
            
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(record, f, ensure_ascii=False, indent=2)
            
            self.get_logger().info(f'📄 處理記錄已保存: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 保存處理記錄失敗: {str(e)}')

    def report_status_to_web(self, order_id, status, message, request=None, error_details=None):
        """回報處理狀態到Web API"""
        try:
            # 構建狀態更新資料
            status_data = {
                "order_id": order_id,
                "status": status,  # "processing", "completed", "failed"
                "message": message,
                "timestamp": datetime.now().isoformat(),
                "processed_by": "ROS2_OrderProcessor"
            }
            
            # 添加處理詳情
            if request:
                status_data["patient_name"] = request.patient_name
                status_data["patient_id"] = request.patient_id
                status_data["medicine_count"] = len(request.medicine_names)
                status_data["medicines"] = []
                
                for i, medicine_name in enumerate(request.medicine_names):
                    medicine_info = {
                        "name": medicine_name,
                        "quantity": request.quantities[i] if i < len(request.quantities) else 0,
                        "dosage": request.dosages[i] if i < len(request.dosages) else "",
                        "status": status
                    }
                    status_data["medicines"].append(medicine_info)
            
            # 添加錯誤詳情
            if error_details:
                status_data["error_details"] = error_details
            
            # 發送狀態更新到Web API
            response = requests.post(
                f"{self.api_base_url}/prescription/status-update",
                json=status_data,
                timeout=10
            )
            
            if response.status_code == 200:
                self.get_logger().info(f'📊 狀態回報成功: {order_id} -> {status}')
            else:
                self.get_logger().warning(f'⚠️ 狀態回報失敗 ({response.status_code}): {response.text}')
        
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'🌐 狀態回報網路錯誤: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'❌ 狀態回報錯誤: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = OrderProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 訂單處理節點正在關閉...')
    except Exception as e:
        node.get_logger().error(f'❌ 節點錯誤: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
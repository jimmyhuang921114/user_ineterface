#!/usr/bin/env python3
"""
ROS2 醫院藥物管理客戶端
ROS2 Hospital Medicine Management Client
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
import requests
import json
from datetime import datetime
import threading
import time

class MedicineManagementClient(Node):
    def __init__(self):
        super().__init__('medicine_management_client')
        
        # 配置
        self.medicine_api = "http://localhost:8000"
        self.prescription_api = "http://localhost:8001"
        
        # 創建服務客戶端
        self.medicine_service_client = self.create_client(Empty, 'get_all_medicines')
        
        # 創建發布者
        self.medicine_publisher = self.create_publisher(String, 'medicine_data', 10)
        self.prescription_publisher = self.create_publisher(String, 'prescription_status', 10)
        
        # 創建訂閱者
        self.prescription_subscriber = self.create_subscription(
            String,
            'prescription_request',
            self.prescription_request_callback,
            10
        )
        
        # 創建定時器
        self.medicine_timer = self.create_timer(30.0, self.publish_medicine_data)  # 每30秒發布一次
        self.status_timer = self.create_timer(10.0, self.check_prescription_status)  # 每10秒檢查狀態
        
        self.get_logger().info('ROS2醫院藥物管理客戶端已啟動')
        
        # 狀態追蹤
        self.last_medicine_update = None
        self.active_prescriptions = []
        
    def call_medicine_service(self):
        """調用藥物服務"""
        if not self.medicine_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('藥物服務不可用')
            return None
            
        request = Empty.Request()
        future = self.medicine_service_client.call_async(request)
        return future
    
    def get_all_medicines_from_api(self):
        """從API獲取所有藥物資訊"""
        try:
            # 獲取基本藥物
            basic_response = requests.get(f"{self.medicine_api}/api/medicine/", timeout=5)
            basic_medicines = basic_response.json() if basic_response.status_code == 200 else []
            
            # 獲取詳細藥物
            detailed_response = requests.get(f"{self.medicine_api}/api/medicine/detailed/", timeout=5)
            detailed_medicines = detailed_response.json() if detailed_response.status_code == 200 else {}
            
            # 獲取整合資料
            integrated_response = requests.get(f"{self.medicine_api}/api/export/medicines/integrated", timeout=5)
            integrated_data = integrated_response.json() if integrated_response.status_code == 200 else {}
            
            result = {
                "timestamp": datetime.now().isoformat(),
                "basic_medicines": basic_medicines,
                "detailed_medicines": detailed_medicines,
                "integrated_data": integrated_data.get("data", {}),
                "total_basic": len(basic_medicines),
                "total_detailed": len(detailed_medicines),
                "service_status": "success"
            }
            
            self.get_logger().info(f'成功獲取藥物資訊: {len(basic_medicines)} 基本, {len(detailed_medicines)} 詳細')
            return result
            
        except Exception as e:
            self.get_logger().error(f'獲取藥物資訊失敗: {str(e)}')
            return {
                "timestamp": datetime.now().isoformat(),
                "error": str(e),
                "service_status": "error"
            }
    
    def get_medicine_by_name(self, medicine_name):
        """根據名稱獲取特定藥物資訊"""
        try:
            # 獲取整合資訊
            response = requests.get(f"{self.medicine_api}/api/medicine/integrated/{medicine_name}", timeout=5)
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().warn(f'找不到藥物: {medicine_name}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'獲取藥物資訊失敗: {str(e)}')
            return None
    
    def get_medicine_by_code(self, code):
        """根據包裝編號獲取藥物資訊"""
        try:
            response = requests.get(f"{self.medicine_api}/api/medicine/search/code/{code}", timeout=5)
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().warn(f'找不到包裝編號: {code}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'根據編號搜尋失敗: {str(e)}')
            return None
    
    def publish_medicine_data(self):
        """定期發布藥物資料"""
        medicine_data = self.get_all_medicines_from_api()
        
        if medicine_data:
            msg = String()
            msg.data = json.dumps(medicine_data, ensure_ascii=False)
            self.medicine_publisher.publish(msg)
            
            self.last_medicine_update = datetime.now()
            self.get_logger().info('已發布藥物資料到ROS2主題')
    
    def get_prescription_status(self):
        """獲取處方狀態"""
        try:
            response = requests.get(f"{self.prescription_api}/api/prescription/", timeout=5)
            if response.status_code == 200:
                prescriptions = response.json()
                
                # 統計各狀態數量
                status_count = {
                    "pending": 0,
                    "processing": 0,
                    "completed": 0,
                    "cancelled": 0
                }
                
                for prescription in prescriptions:
                    status = prescription.get("status", "unknown")
                    if status in status_count:
                        status_count[status] += 1
                
                return {
                    "timestamp": datetime.now().isoformat(),
                    "total_prescriptions": len(prescriptions),
                    "status_breakdown": status_count,
                    "prescriptions": prescriptions,
                    "service_status": "success"
                }
            else:
                return {"error": "無法獲取處方資料", "service_status": "error"}
                
        except Exception as e:
            self.get_logger().error(f'獲取處方狀態失敗: {str(e)}')
            return {"error": str(e), "service_status": "error"}
    
    def check_prescription_status(self):
        """定期檢查處方狀態"""
        prescription_status = self.get_prescription_status()
        
        if prescription_status:
            msg = String()
            msg.data = json.dumps(prescription_status, ensure_ascii=False)
            self.prescription_publisher.publish(msg)
            
            # 檢查是否有新的待處理處方
            if prescription_status.get("service_status") == "success":
                pending_count = prescription_status.get("status_breakdown", {}).get("pending", 0)
                if pending_count > 0:
                    self.get_logger().info(f'發現 {pending_count} 個待處理處方')
    
    def prescription_request_callback(self, msg):
        """處理處方請求回調"""
        try:
            request_data = json.loads(msg.data)
            request_type = request_data.get("type", "unknown")
            
            if request_type == "get_medicine_info":
                medicine_name = request_data.get("medicine_name", "")
                medicine_info = self.get_medicine_by_name(medicine_name)
                
                response_msg = String()
                response_msg.data = json.dumps({
                    "request_type": request_type,
                    "medicine_name": medicine_name,
                    "medicine_info": medicine_info,
                    "timestamp": datetime.now().isoformat()
                }, ensure_ascii=False)
                
                self.medicine_publisher.publish(response_msg)
                self.get_logger().info(f'已響應藥物資訊請求: {medicine_name}')
                
            elif request_type == "search_by_code":
                code = request_data.get("code", "")
                medicine_info = self.get_medicine_by_code(code)
                
                response_msg = String()
                response_msg.data = json.dumps({
                    "request_type": request_type,
                    "code": code,
                    "medicine_info": medicine_info,
                    "timestamp": datetime.now().isoformat()
                }, ensure_ascii=False)
                
                self.medicine_publisher.publish(response_msg)
                self.get_logger().info(f'已響應編號搜尋請求: {code}')
                
            elif request_type == "update_prescription_status":
                prescription_id = request_data.get("prescription_id")
                new_status = request_data.get("status")
                updated_by = request_data.get("updated_by", "ROS2_Client")
                
                # 更新處方狀態
                update_data = {
                    "prescription_id": prescription_id,
                    "status": new_status,
                    "updated_by": updated_by,
                    "notes": f"ROS2服務更新狀態為: {new_status}"
                }
                
                response = requests.put(
                    f"{self.prescription_api}/api/prescription/{prescription_id}/status",
                    json=update_data,
                    timeout=5
                )
                
                response_msg = String()
                response_msg.data = json.dumps({
                    "request_type": request_type,
                    "prescription_id": prescription_id,
                    "update_success": response.status_code == 200,
                    "timestamp": datetime.now().isoformat()
                }, ensure_ascii=False)
                
                self.prescription_publisher.publish(response_msg)
                self.get_logger().info(f'已更新處方狀態: {prescription_id} -> {new_status}')
                
        except Exception as e:
            self.get_logger().error(f'處理處方請求失敗: {str(e)}')
    
    def create_test_prescription(self, patient_name, doctor_name, medicines):
        """創建測試處方"""
        try:
            prescription_data = {
                "patient_name": patient_name,
                "doctor_name": doctor_name,
                "medicines": medicines,
                "diagnosis": "測試診斷",
                "instructions": "測試用藥指示",
                "priority": "normal"
            }
            
            response = requests.post(
                f"{self.prescription_api}/api/prescription/",
                json=prescription_data,
                timeout=5
            )
            
            if response.status_code == 200:
                result = response.json()
                self.get_logger().info(f'成功創建測試處方: {result.get("id")}')
                return result
            else:
                self.get_logger().error('創建測試處方失敗')
                return None
                
        except Exception as e:
            self.get_logger().error(f'創建測試處方失敗: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    # 創建節點
    client = MedicineManagementClient()
    
    try:
        # 在後台線程中運行一些測試
        def run_tests():
            time.sleep(5)  # 等待系統初始化
            
            # 測試獲取藥物資訊
            client.get_logger().info('開始測試藥物API...')
            medicine_data = client.get_all_medicines_from_api()
            if medicine_data:
                client.get_logger().info('藥物API測試成功')
            
            # 測試特定藥物查詢
            medicine_info = client.get_medicine_by_name("心律錠")
            if medicine_info:
                client.get_logger().info('特定藥物查詢測試成功')
            
            # 測試編號查詢
            code_result = client.get_medicine_by_code("202801")
            if code_result:
                client.get_logger().info('編號查詢測試成功')
            
            # 測試創建處方
            test_medicines = [
                {
                    "medicine_name": "心律錠",
                    "dosage": "10mg",
                    "frequency": "每日三次",
                    "duration": "7天",
                    "instructions": "飯後服用"
                }
            ]
            
            prescription = client.create_test_prescription(
                "測試病人",
                "測試醫生",
                test_medicines
            )
            
            if prescription:
                client.get_logger().info('處方創建測試成功')
        
        # 啟動測試線程
        test_thread = threading.Thread(target=run_tests)
        test_thread.daemon = True
        test_thread.start()
        
        # 運行ROS2節點
        rclpy.spin(client)
        
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
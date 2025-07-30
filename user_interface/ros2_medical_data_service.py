#!/usr/bin/env python3
"""
ROS2醫療數據服務節點
ROS2 Medical Data Service Node
提供病例、藥物和處方數據查詢服務
"""

import rclpy
from rclpy.node import Node
import requests
import json
from datetime import datetime

# 注意：在實際ROS2環境中，需要導入實際的服務類型
# from medical_interfaces.srv import MedicalDataService, CompletionReportService

# 模擬服務類型
class MedicalDataServiceRequest:
    def __init__(self):
        self.query_type = ""
        self.query_id = ""
        self.patient_id = ""
        self.additional_params = ""

class MedicalDataServiceResponse:
    def __init__(self):
        self.success = False
        self.message = ""
        self.data = ""
        self.error_code = ""

class CompletionReportServiceRequest:
    def __init__(self):
        self.task_id = ""
        self.task_type = ""
        self.status = ""
        self.notes = ""
        self.processed_data = ""
        self.timestamp = ""

class CompletionReportServiceResponse:
    def __init__(self):
        self.success = False
        self.message = ""
        self.report_id = ""
        self.next_action = ""

class MedicalDataServiceNode(Node):
    def __init__(self):
        super().__init__('medical_data_service_node')
        
        # API基礎URL
        self.api_base_url = "http://localhost:8000"
        
        # 創建服務
        # 在實際ROS2環境中使用以下代碼：
        # self.medical_data_service = self.create_service(
        #     MedicalDataService, 
        #     'query_medical_data', 
        #     self.handle_medical_data_query
        # )
        # 
        # self.completion_report_service = self.create_service(
        #     CompletionReportService,
        #     'report_completion',
        #     self.handle_completion_report
        # )
        
        # 完成報告記錄
        self.completion_reports = []
        self.next_report_id = 1
        
        self.get_logger().info('醫療數據服務節點已啟動')
        self.get_logger().info(f'API基礎URL: {self.api_base_url}')
        
        # 啟動測試模式
        self.test_mode_timer = self.create_timer(10.0, self.test_services)
    
    def handle_medical_data_query(self, request, response):
        """處理醫療數據查詢請求"""
        self.get_logger().info(f'收到醫療數據查詢請求: {request.query_type}, ID: {request.query_id}')
        
        try:
            if request.query_type == "medical_record":
                data = self.query_medical_record(request.query_id, request.patient_id)
            elif request.query_type == "medicine":
                data = self.query_medicine_data(request.query_id)
            elif request.query_type == "prescription":
                data = self.query_prescription_data(request.query_id)
            else:
                response.success = False
                response.message = f"不支援的查詢類型: {request.query_type}"
                response.error_code = "UNSUPPORTED_QUERY_TYPE"
                return response
            
            response.success = True
            response.message = "查詢成功"
            response.data = json.dumps(data, ensure_ascii=False)
            response.error_code = ""
            
        except Exception as e:
            self.get_logger().error(f'查詢失敗: {str(e)}')
            response.success = False
            response.message = f"查詢失敗: {str(e)}"
            response.error_code = "QUERY_FAILED"
            response.data = ""
        
        return response
    
    def handle_completion_report(self, request, response):
        """處理完成報告"""
        self.get_logger().info(f'收到完成報告: {request.task_type}, 狀態: {request.status}')
        
        try:
            # 創建報告記錄
            report = {
                "report_id": f"RPT_{self.next_report_id:06d}",
                "task_id": request.task_id,
                "task_type": request.task_type,
                "status": request.status,
                "notes": request.notes,
                "processed_data": request.processed_data,
                "timestamp": request.timestamp or datetime.now().isoformat(),
                "received_time": datetime.now().isoformat()
            }
            
            self.completion_reports.append(report)
            self.next_report_id += 1
            
            # 根據任務類型決定下一步動作
            next_action = self.determine_next_action(request.task_type, request.status)
            
            response.success = True
            response.message = "完成報告已記錄"
            response.report_id = report["report_id"]
            response.next_action = next_action
            
            self.get_logger().info(f'完成報告已記錄: {report["report_id"]}')
            
        except Exception as e:
            self.get_logger().error(f'處理完成報告失敗: {str(e)}')
            response.success = False
            response.message = f"處理失敗: {str(e)}"
            response.report_id = ""
            response.next_action = ""
        
        return response
    
    def query_medical_record(self, record_id, patient_id=None):
        """查詢病例數據"""
        if patient_id:
            # 按病患ID查詢
            url = f"{self.api_base_url}/api/medical_record/patient/{patient_id}"
        else:
            # 按病例ID查詢
            url = f"{self.api_base_url}/api/medical_record/{record_id}"
        
        response = requests.get(url)
        response.raise_for_status()
        return response.json()
    
    def query_medicine_data(self, medicine_name):
        """查詢藥物數據"""
        # 查詢整合的藥物資訊
        url = f"{self.api_base_url}/api/medicine/integrated/{medicine_name}"
        
        response = requests.get(url)
        response.raise_for_status()
        return response.json()
    
    def query_prescription_data(self, prescription_id):
        """查詢處方數據"""
        url = f"{self.api_base_url}/api/prescription/{prescription_id}"
        
        response = requests.get(url)
        response.raise_for_status()
        return response.json()
    
    def determine_next_action(self, task_type, status):
        """決定下一步動作"""
        if status == "completed":
            if task_type == "medical_record_processing":
                return "archive_record"
            elif task_type == "medicine_dispensing":
                return "update_inventory"
            else:
                return "no_action_required"
        elif status == "failed":
            return "retry_task"
        elif status == "partially_completed":
            return "complete_remaining_tasks"
        else:
            return "review_status"
    
    def test_services(self):
        """測試服務功能（模擬模式）"""
        self.get_logger().info('執行服務測試...')
        
        # 模擬醫療數據查詢
        try:
            # 測試病例查詢
            test_data = self.query_medical_record("1")
            self.get_logger().info(f'病例查詢測試成功: {len(test_data) if isinstance(test_data, list) else "單一記錄"}')
        except Exception as e:
            self.get_logger().warn(f'病例查詢測試失敗: {str(e)}')
        
        # 測試藥物查詢
        try:
            test_data = self.query_medicine_data("測試藥物")
            self.get_logger().info('藥物查詢測試完成')
        except Exception as e:
            self.get_logger().warn(f'藥物查詢測試失敗: {str(e)}')
    
    def get_all_completion_reports(self):
        """獲取所有完成報告"""
        return self.completion_reports
    
    def get_completion_report_by_id(self, report_id):
        """按ID獲取完成報告"""
        for report in self.completion_reports:
            if report["report_id"] == report_id:
                return report
        return None

# 獨立函數用於模擬服務調用
def simulate_medical_data_query(query_type, query_id, patient_id="", additional_params=""):
    """模擬醫療數據查詢服務調用"""
    print(f"\n=== 模擬醫療數據查詢 ===")
    print(f"查詢類型: {query_type}")
    print(f"查詢ID: {query_id}")
    print(f"病患ID: {patient_id}")
    
    # 模擬請求
    request = MedicalDataServiceRequest()
    request.query_type = query_type
    request.query_id = query_id
    request.patient_id = patient_id
    request.additional_params = additional_params
    
    # 模擬響應
    response = MedicalDataServiceResponse()
    
    try:
        api_base_url = "http://localhost:8000"
        
        if query_type == "medical_record":
            if patient_id:
                url = f"{api_base_url}/api/medical_record/patient/{patient_id}"
            else:
                url = f"{api_base_url}/api/medical_record/{query_id}"
        elif query_type == "medicine":
            url = f"{api_base_url}/api/medicine/integrated/{query_id}"
        elif query_type == "prescription":
            url = f"{api_base_url}/api/prescription/{query_id}"
        else:
            raise ValueError(f"不支援的查詢類型: {query_type}")
        
        api_response = requests.get(url)
        api_response.raise_for_status()
        
        response.success = True
        response.message = "查詢成功"
        response.data = json.dumps(api_response.json(), ensure_ascii=False, indent=2)
        response.error_code = ""
        
        print(f"查詢成功!")
        print(f"數據: {response.data[:200]}...")
        
    except Exception as e:
        response.success = False
        response.message = f"查詢失敗: {str(e)}"
        response.error_code = "QUERY_FAILED"
        response.data = ""
        
        print(f"查詢失敗: {str(e)}")
    
    return response

def simulate_completion_report(task_id, task_type, status, notes="", processed_data=""):
    """模擬完成報告服務調用"""
    print(f"\n=== 模擬完成報告 ===")
    print(f"任務ID: {task_id}")
    print(f"任務類型: {task_type}")
    print(f"狀態: {status}")
    
    # 模擬請求
    request = CompletionReportServiceRequest()
    request.task_id = task_id
    request.task_type = task_type
    request.status = status
    request.notes = notes
    request.processed_data = processed_data
    request.timestamp = datetime.now().isoformat()
    
    # 模擬響應
    response = CompletionReportServiceResponse()
    response.success = True
    response.message = "完成報告已記錄"
    response.report_id = f"RPT_{hash(task_id) % 100000:06d}"
    
    # 決定下一步動作
    if status == "completed":
        if task_type == "medical_record_processing":
            response.next_action = "archive_record"
        elif task_type == "medicine_dispensing":
            response.next_action = "update_inventory"
        else:
            response.next_action = "no_action_required"
    elif status == "failed":
        response.next_action = "retry_task"
    else:
        response.next_action = "review_status"
    
    print(f"報告ID: {response.report_id}")
    print(f"下一步動作: {response.next_action}")
    
    return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MedicalDataServiceNode()
        print("醫療數據服務節點已啟動")
        print("按 Ctrl+C 停止")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n正在停止醫療數據服務節點...")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
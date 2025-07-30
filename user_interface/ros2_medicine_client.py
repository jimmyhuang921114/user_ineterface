#!/usr/bin/env python3
"""
ROS2
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

        #
        self.medicine_api = "http://localhost:8000"
        self.prescription_api = "http://localhost:8001"

        #
        self.medicine_service_client = self.create_client(Empty, 'get_all_medicines')

        #
        self.medicine_publisher = self.create_publisher(String, 'medicine_data', 10)
        self.prescription_publisher = self.create_publisher(String, 'prescription_status', 10)

        #
        self.prescription_subscriber = self.create_subscription(
            String,
            'prescription_request',
            self.prescription_request_callback,
            10
        )

        #
        self.medicine_timer = self.create_timer(30.0, self.publish_medicine_data)  # 30
        self.status_timer = self.create_timer(10.0, self.check_prescription_status)  # 10

        self.get_logger().info('ROS2')

        #
        self.last_medicine_update = None
        self.active_prescriptions = []

    def call_medicine_service(self):
        """"""
        if not self.medicine_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('')
            return None

        request = Empty.Request()
        future = self.medicine_service_client.call_async(request)
        return future

    def get_all_medicines_from_api(self):
        """API"""
        try:
            #
            basic_response = requests.get(f"{self.medicine_api}/api/medicine/", timeout=5)
            basic_medicines = basic_response.json() if basic_response.status_code == 200 else []

            #
            detailed_response = requests.get(f"{self.medicine_api}/api/medicine/detailed/", timeout=5)
            detailed_medicines = detailed_response.json() if detailed_response.status_code == 200 else {}

            #
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

            self.get_logger().info(f': {len(basic_medicines)} , {len(detailed_medicines)} ')
            return result

        except Exception as e:
            self.get_logger().error(f': {str(e)}')
            return {
                "timestamp": datetime.now().isoformat(),
                "error": str(e),
                "service_status": "error"
            }

    def get_medicine_by_name(self, medicine_name):
        """"""
        try:
            #
            response = requests.get(f"{self.medicine_api}/api/medicine/integrated/{medicine_name}", timeout=5)
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().warn(f': {medicine_name}')
                return None

        except Exception as e:
            self.get_logger().error(f': {str(e)}')
            return None

    def get_medicine_by_code(self, code):
        """"""
        try:
            response = requests.get(f"{self.medicine_api}/api/medicine/search/code/{code}", timeout=5)
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().warn(f': {code}')
                return None

        except Exception as e:
            self.get_logger().error(f': {str(e)}')
            return None

    def publish_medicine_data(self):
        """"""
        medicine_data = self.get_all_medicines_from_api()

        if medicine_data:
            msg = String()
            msg.data = json.dumps(medicine_data, ensure_ascii=False)
            self.medicine_publisher.publish(msg)

            self.last_medicine_update = datetime.now()
            self.get_logger().info('ROS2')

    def get_prescription_status(self):
        """"""
        try:
            response = requests.get(f"{self.prescription_api}/api/prescription/", timeout=5)
            if response.status_code == 200:
                prescriptions = response.json()

                #
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
                return {"error": "", "service_status": "error"}

        except Exception as e:
            self.get_logger().error(f': {str(e)}')
            return {"error": str(e), "service_status": "error"}

    def check_prescription_status(self):
        """"""
        prescription_status = self.get_prescription_status()

        if prescription_status:
            msg = String()
            msg.data = json.dumps(prescription_status, ensure_ascii=False)
            self.prescription_publisher.publish(msg)

            #
            if prescription_status.get("service_status") == "success":
                pending_count = prescription_status.get("status_breakdown", {}).get("pending", 0)
                if pending_count > 0:
                    self.get_logger().info(f' {pending_count} ')

    def prescription_request_callback(self, msg):
        """"""
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
                self.get_logger().info(f': {medicine_name}')

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
                self.get_logger().info(f': {code}')

            elif request_type == "update_prescription_status":
                prescription_id = request_data.get("prescription_id")
                new_status = request_data.get("status")
                updated_by = request_data.get("updated_by", "ROS2_Client")

                #
                update_data = {
                    "prescription_id": prescription_id,
                    "status": new_status,
                    "updated_by": updated_by,
                    "notes": f"ROS2: {new_status}"
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
                self.get_logger().info(f': {prescription_id} -> {new_status}')

        except Exception as e:
            self.get_logger().error(f': {str(e)}')

    def create_test_prescription(self, patient_name, doctor_name, medicines):
        """"""
        try:
            prescription_data = {
                "patient_name": patient_name,
                "doctor_name": doctor_name,
                "medicines": medicines,
                "diagnosis": "",
                "instructions": "",
                "priority": "normal"
            }

            response = requests.post(
                f"{self.prescription_api}/api/prescription/",
                json=prescription_data,
                timeout=5
            )

            if response.status_code == 200:
                result = response.json()
                self.get_logger().info(f': {result.get("id")}')
                return result
            else:
                self.get_logger().error('')
                return None

        except Exception as e:
            self.get_logger().error(f': {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)

    #
    client = MedicineManagementClient()

    try:
        #
        def run_tests():
            time.sleep(5)  #

            #
            client.get_logger().info('API...')
            medicine_data = client.get_all_medicines_from_api()
            if medicine_data:
                client.get_logger().info('API')

            #
            medicine_info = client.get_medicine_by_name("")
            if medicine_info:
                client.get_logger().info('')

            #
            code_result = client.get_medicine_by_code("202801")
            if code_result:
                client.get_logger().info('')

            #
            test_medicines = [
                {
                    "medicine_name": "",
                    "dosage": "10mg",
                    "frequency": "",
                    "duration": "7",
                    "instructions": ""
                }
            ]

            prescription = client.create_test_prescription(
                "",
                "",
                test_medicines
            )

            if prescription:
                client.get_logger().info('')

        #
        test_thread = threading.Thread(target=run_tests)
        test_thread.daemon = True
        test_thread.start()

        # ROS2
        rclpy.spin(client)

    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
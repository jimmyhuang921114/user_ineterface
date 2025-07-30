#!/usr/bin/env python3
"""
ROS2
Hospital Medicine Management ROS2 Service Node
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import requests
import json
from datetime import datetime

class MedicineServiceNode(Node):
    def __init__(self):
        super().__init__('medicine_service_node')

        #
        self.api_base = "http://localhost:8000"

        #
        self.medicine_service = self.create_service(
            Empty,
            'get_all_medicines',
            self.get_all_medicines_callback
        )

        self.get_logger().info('ROS2')

    def get_all_medicines_callback(self, request, response):
        """"""
        try:
            #
            basic_response = requests.get(f"{self.api_base}/api/medicine/")
            basic_medicines = basic_response.json() if basic_response.status_code == 200 else []

            #
            detailed_response = requests.get(f"{self.api_base}/api/medicine/detailed/")
            detailed_medicines = detailed_response.json() if detailed_response.status_code == 200 else {}

            #
            result = {
                "timestamp": datetime.now().isoformat(),
                "basic_medicines": basic_medicines,
                "detailed_medicines": detailed_medicines,
                "total_basic": len(basic_medicines),
                "total_detailed": len(detailed_medicines)
            }

            self.get_logger().info(f': {len(basic_medicines)} , {len(detailed_medicines)} ')

            # ROS2
            with open('/tmp/medicine_data_ros2.json', 'w', encoding='utf-8') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)

            return response

        except Exception as e:
            self.get_logger().error(f': {str(e)}')
            return response

def main(args=None):
    rclpy.init(args=args)
    node = MedicineServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

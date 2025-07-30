#!/usr/bin/env python3
"""
ROS2
ROS2 Prescription Service Node


1.
2.
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

        self.get_logger().info(' ROS2')

        #  -
        self.pending_prescription_publisher = self.create_publisher(
            String,
            'pending_prescriptions',
            10
        )

        #  -
        self.processing_result_subscriber = self.create_subscription(
            String,
            'prescription_processing_result',
            self.processing_result_callback,
            10
        )

        #  - 30
        self.check_timer = self.create_timer(30.0, self.check_pending_prescriptions)

        #
        self.sent_prescriptions = set()

        self.get_logger().info(' ROS2')
        self.get_logger().info('   - : pending_prescriptions')
        self.get_logger().info('   - : prescription_processing_result')
        self.get_logger().info('   - : 30')

    def check_pending_prescriptions(self):
        """"""
        try:
            #
            response = requests.get(f"{API_BASE}/api/prescription/status/pending", timeout=5)

            if response.status_code == 200:
                pending_prescriptions = response.json()

                if pending_prescriptions:
                    self.get_logger().info(f'  {len(pending_prescriptions)} ')

                    #
                    for prescription in pending_prescriptions:
                        prescription_id = prescription['id']

                        #
                        if prescription_id not in self.sent_prescriptions:
                            self.publish_prescription(prescription)
                            self.sent_prescriptions.add(prescription_id)
                else:
                    self.get_logger().info(' ')
            else:
                self.get_logger().warning(f'  : {response.status_code}')

        except Exception as e:
            self.get_logger().error(f' : {str(e)}')

    def publish_prescription(self, prescription):
        """ROS2"""
        try:
            #
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

            # JSON
            json_message = json.dumps(prescription_data, ensure_ascii=False, indent=2)

            # ROS2
            msg = String()
            msg.data = json_message

            #
            self.pending_prescription_publisher.publish(msg)

            self.get_logger().info(f'  #{prescription["id"]} - {prescription["patient_name"]}')
            self.get_logger().info(f'   : {prescription["doctor_name"]}')
            self.get_logger().info(f'   : {len(prescription["medicines"])}')

            # "processing"
            self.update_prescription_status(prescription['id'], 'processing', 'ROS2')

        except Exception as e:
            self.get_logger().error(f' : {str(e)}')

    def processing_result_callback(self, msg):
        """ROS2"""
        try:
            #
            result_data = json.loads(msg.data)

            prescription_id = result_data.get('prescription_id')
            processing_status = result_data.get('status', 'completed')
            processing_notes = result_data.get('notes', '')
            processed_by = result_data.get('processed_by', 'ROS2')

            self.get_logger().info(f'  #{prescription_id} ')
            self.get_logger().info(f'   : {processing_status}')
            self.get_logger().info(f'   : {processed_by}')

            if processing_notes:
                self.get_logger().info(f'   : {processing_notes}')

            #
            success = self.update_prescription_status(
                prescription_id,
                processing_status,
                processed_by,
                processing_notes
            )

            if success:
                self.get_logger().info(f'  #{prescription_id} ')
                #
                self.sent_prescriptions.discard(prescription_id)
            else:
                self.get_logger().error(f'  #{prescription_id} ')

        except json.JSONDecodeError as e:
            self.get_logger().error(f' JSON: {str(e)}')
        except Exception as e:
            self.get_logger().error(f' : {str(e)}')

    def update_prescription_status(self, prescription_id, status, updated_by, notes=''):
        """"""
        try:
            update_data = {
                'status': status,
                'updated_by': updated_by,
                'notes': notes or f'ROS2: {status}'
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
                self.get_logger().error(f': {response.status_code} - {response.text}')
                return False

        except Exception as e:
            self.get_logger().error(f': {str(e)}')
            return False

    def get_prescription_statistics(self):
        """"""
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
            self.get_logger().error(f': {str(e)}')
            return None

def main(args=None):
    """"""
    rclpy.init(args=args)

    try:
        #
        node = PrescriptionServiceNode()

        # API
        try:
            response = requests.get(f"{API_BASE}/api/system/status", timeout=5)
            if response.status_code == 200:
                node.get_logger().info(' API')

                #
                stats = node.get_prescription_statistics()
                if stats:
                    node.get_logger().info(f' : ={stats["total"]}, ={stats["pending"]}, ={stats["processing"]}, ={stats["completed"]}')
            else:
                node.get_logger().warning(f'  API: {response.status_code}')
        except Exception as e:
            node.get_logger().error(f' API: {str(e)}')
            node.get_logger().info('')

        #
        node.get_logger().info(' ...')
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('\n ...')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print(' ROS2')

if __name__ == '__main__':
    main()
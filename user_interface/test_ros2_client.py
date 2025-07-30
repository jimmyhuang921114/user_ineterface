#!/usr/bin/env python3
"""
ROS2
ROS2 Prescription Processing Test Client


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

        self.get_logger().info(' ROS2')

        #
        self.prescription_subscriber = self.create_subscription(
            String,
            'pending_prescriptions',
            self.prescription_callback,
            10
        )

        #
        self.result_publisher = self.create_publisher(
            String,
            'prescription_processing_result',
            10
        )

        #
        self.processed_count = 0

        self.get_logger().info(' ')
        self.get_logger().info('   - : pending_prescriptions')
        self.get_logger().info('   - : prescription_processing_result')

    def prescription_callback(self, msg):
        """"""
        try:
            #
            prescription_data = json.loads(msg.data)
            prescription_id = prescription_data.get('prescription_id')
            patient_name = prescription_data.get('patient_name')
            medicines = prescription_data.get('medicines', [])

            self.get_logger().info(f'  #{prescription_id} - {patient_name}')
            self.get_logger().info(f'   : {len(medicines)}')

            #
            for i, medicine in enumerate(medicines, 1):
                self.get_logger().info(f'   {i}. {medicine.get("medicine_name", "Unknown")} - {medicine.get("dosage", "Unknown")}')

            # 5-15
            processing_time = random.randint(5, 15)
            self.get_logger().info(f'⏳  {processing_time} ...')

            #
            self.create_timer(
                processing_time,
                lambda: self.complete_processing(prescription_data)
            )

        except json.JSONDecodeError as e:
            self.get_logger().error(f' JSON: {str(e)}')
        except Exception as e:
            self.get_logger().error(f' : {str(e)}')

    def complete_processing(self, prescription_data):
        """"""
        try:
            prescription_id = prescription_data.get('prescription_id')
            patient_name = prescription_data.get('patient_name')
            medicines = prescription_data.get('medicines', [])

            #
            processing_success = random.choice([True, True, True, False])  # 75%

            if processing_success:
                status = 'completed'
                notes = self.generate_success_notes(medicines)
                self.get_logger().info(f'  #{prescription_id} ')
            else:
                status = 'pending'  #
                notes = self.generate_failure_notes()
                self.get_logger().warning(f'   #{prescription_id} ')

            #
            result_data = {
                'prescription_id': prescription_id,
                'patient_name': patient_name,
                'status': status,
                'processed_by': f'ROS2-{self.get_name()}',
                'processing_time': datetime.now().isoformat(),
                'notes': notes,
                'medicines_processed': len(medicines),
                'success': processing_success
            }

            #
            self.publish_result(result_data)

            self.processed_count += 1
            self.get_logger().info(f' : {self.processed_count} ')

        except Exception as e:
            self.get_logger().error(f' : {str(e)}')

    def publish_result(self, result_data):
        """"""
        try:
            # JSON
            json_message = json.dumps(result_data, ensure_ascii=False, indent=2)

            # ROS2
            msg = String()
            msg.data = json_message

            #
            self.result_publisher.publish(msg)

            prescription_id = result_data['prescription_id']
            status = result_data['status']

            self.get_logger().info(f'  #{prescription_id} : {status}')

        except Exception as e:
            self.get_logger().error(f' : {str(e)}')

    def generate_success_notes(self, medicines):
        """"""
        medicine_names = [med.get('medicine_name', 'Unknown') for med in medicines]

        success_templates = [
            f" {len(medicines)} ",
            f"{', '.join(medicine_names[:2])}{'' if len(medicine_names) > 2 else ''}",
            f"",
            f"",
            f" {len(medicines)} "
        ]

        return random.choice(success_templates)

    def generate_failure_notes(self):
        """"""
        failure_reasons = [
            "",
            "",
            "",
            "",
            ""
        ]

        return random.choice(failure_reasons)

class PrescriptionMonitorClient(Node):
    """ - """
    def __init__(self):
        super().__init__('prescription_monitor_client')

        self.get_logger().info('  ROS2')

        #
        self.prescription_subscriber = self.create_subscription(
            String,
            'pending_prescriptions',
            self.monitor_callback,
            10
        )

        #
        self.result_subscriber = self.create_subscription(
            String,
            'prescription_processing_result',
            self.result_monitor_callback,
            10
        )

        self.get_logger().info(' ')

    def monitor_callback(self, msg):
        """"""
        try:
            prescription_data = json.loads(msg.data)
            prescription_id = prescription_data.get('prescription_id')
            patient_name = prescription_data.get('patient_name')

            self.get_logger().info(f'  : #{prescription_id} - {patient_name}')

        except Exception as e:
            self.get_logger().error(f' : {str(e)}')

    def result_monitor_callback(self, msg):
        """"""
        try:
            result_data = json.loads(msg.data)
            prescription_id = result_data.get('prescription_id')
            status = result_data.get('status')
            success = result_data.get('success', False)

            status_icon = '' if success else ''
            self.get_logger().info(f'{status_icon} : #{prescription_id} -> {status}')

        except Exception as e:
            self.get_logger().error(f' : {str(e)}')

def main_processor(args=None):
    """"""
    rclpy.init(args=args)

    try:
        node = PrescriptionProcessorClient()
        node.get_logger().info(' ...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n ')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

def main_monitor(args=None):
    """"""
    rclpy.init(args=args)

    try:
        node = PrescriptionMonitorClient()
        node.get_logger().info(' ...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n ')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    """ - """
    import sys

    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if mode == 'monitor':
            print(' ...')
            main_monitor(args)
        elif mode == 'processor':
            print(' ...')
            main_processor(args)
        else:
            print(' : processor  monitor')
    else:
        print(' ...')
        main_processor(args)

if __name__ == '__main__':
    main()
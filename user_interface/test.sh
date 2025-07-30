#!/bin/bash

#
# Hospital Management System Test Script

echo "========================================"
echo " - "
echo "Hospital Management System - Test Script"
echo "========================================"

#
API_BASE="http://localhost:8000"
TIMEOUT=10

#
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

#
print_status() {
    local status=$1
    local message=$2
    if [ "$status" = "success" ]; then
        echo -e "${GREEN}${NC} $message"
    elif [ "$status" = "error" ]; then
        echo -e "${RED}${NC} $message"
    elif [ "$status" = "info" ]; then
        echo -e "${BLUE}ℹ${NC} $message"
    elif [ "$status" = "warning" ]; then
        echo -e "${YELLOW}${NC} $message"
    fi
}

# API
test_connection() {
    print_status "info" "API..."
    response=$(curl -s --connect-timeout $TIMEOUT "$API_BASE/api/test" 2>/dev/null)
    if [ $? -eq 0 ] && echo "$response" | grep -q "success"; then
        print_status "success" "API"
        return 0
    else
        print_status "error" "API"
        return 1
    fi
}

#
add_basic_medicine() {
    local name=$1
    local amount=$2
    local usage_days=$3
    local position=$4

    print_status "info" ": $name"

    response=$(curl -s -X POST "$API_BASE/api/medicine/" \
        -H "Content-Type: application/json" \
        -d '{
            "name": "'$name'",
            "amount": '$amount',
            "usage_days": '$usage_days',
            "position": "'$position'"
        }' 2>/dev/null)

    if [ $? -eq 0 ] && echo "$response" | grep -q '"id"'; then
        print_status "success" ": $name"
        echo "$response" | python3 -m json.tool 2>/dev/null || echo "$response"
        return 0
    else
        print_status "error" ": $name"
        echo "$response"
        return 1
    fi
}

#
add_detailed_medicine() {
    local medicine_name=$1
    local json_file=$2

    print_status "info" ": $medicine_name"

    if [ -f "$json_file" ]; then
        medicine_data=$(cat "$json_file")
    else
        #
        medicine_data='{
            "": {
                "": "'$medicine_name'",
                "": "10",
                "": ""
            },
            "": {
                "": "",
                "": ""
            },
            "": {
                "1": "TEST001",
                "2": "TEST002"
            },
            "": {
                "": "2025/12/31"
            },
            "": "",
            "": "",
            "": "",
            "": "",
            "": "B",
            "": ""
        }'
    fi

    response=$(curl -s -X POST "$API_BASE/api/medicine/detailed/" \
        -H "Content-Type: application/json" \
        -d '{
            "medicine_name": "'$medicine_name'",
            "medicine_data": '$medicine_data'
        }' 2>/dev/null)

    if [ $? -eq 0 ] && echo "$response" | grep -q "success"; then
        print_status "success" ": $medicine_name"
        return 0
    else
        print_status "error" ": $medicine_name"
        echo "$response"
        return 1
    fi
}

# (ROS2)
get_all_medicine_for_ros2() {
    print_status "info" " (ROS2)"

    #
    basic_medicines=$(curl -s "$API_BASE/api/medicine/" 2>/dev/null)

    #
    detailed_medicines=$(curl -s "$API_BASE/api/medicine/detailed/" 2>/dev/null)

    # ROS2
    ros2_output='{
        "timestamp": "'$(date -Iseconds)'",
        "total_medicines": '$(echo "$basic_medicines" | jq length 2>/dev/null || echo 0)',
        "basic_medicines": '$basic_medicines',
        "detailed_medicines": '$detailed_medicines',
        "integration_status": "ready_for_ros2"
    }'

    echo "$ros2_output" > medicine_data_for_ros2.json
    print_status "success" " medicine_data_for_ros2.json"

    #
    echo ""
    print_status "info" "ROS2:"
    echo "$ros2_output" | python3 -m json.tool 2>/dev/null || echo "$ros2_output"
}

#
test_prescription_system() {
    print_status "info" "..."

    #
    prescription_data='{
        "patient_name": "",
        "doctor_name": "",
        "prescription_date": "'$(date -Idate)'",
        "medicines": [
            {
                "medicine_name": "A",
                "dosage": "10mg",
                "frequency": "",
                "duration": "7"
            },
            {
                "medicine_name": "B",
                "dosage": "5mg",
                "frequency": "",
                "duration": "14"
            }
        ],
        "instructions": "",
        "status": "pending"
    }'

    #
    echo "$prescription_data" > test_prescription.json
    print_status "success" ": test_prescription.json"

    #
    echo '{"status": "completed", "completed_time": "'$(date -Iseconds)'"}' > prescription_status.json
    print_status "success" ": prescription_status.json"
}

# ROS2
create_ros2_service_config() {
    print_status "info" "ROS2..."

    # ROS2
    cat > MedicineService.srv << 'EOF'
#  (Request)
string request_type  # "get_all", "get_by_name", "get_detailed"
string medicine_name #  ()
---
#  (Response)
bool success
string message
string data_json     # JSON
int32 total_count    #
EOF

    # ROS2
    cat > medicine_ros2_node.py << 'EOF'
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
EOF

    chmod +x medicine_ros2_node.py
    print_status "success" "ROS2"
}

#
main_test() {
    echo ""
    print_status "info" "..."

    # 1.
    if ! test_connection; then
        print_status "error" "API"
        exit 1
    fi

    echo ""
    print_status "info" "=== 1 ==="

    #
    add_basic_medicine "A" 100 30 "A1-TEST"
    add_basic_medicine "B" 50 14 "B2-TEST"
    add_basic_medicine "C" 200 60 "C1-VIT"

    echo ""
    print_status "info" "=== 2 ==="

    #
    add_detailed_medicine "A"
    add_detailed_medicine "B"
    add_detailed_medicine "C"

    echo ""
    print_status "info" "=== 3ROS2 ==="

    # ROS2
    get_all_medicine_for_ros2

    echo ""
    print_status "info" "=== 4 ==="

    #
    test_prescription_system

    echo ""
    print_status "info" "=== 5ROS2 ==="

    # ROS2
    create_ros2_service_config

    echo ""
    print_status "success" "===  ==="
    print_status "info" ":"
    ls -la medicine_data_for_ros2.json test_prescription.json prescription_status.json MedicineService.srv medicine_ros2_node.py 2>/dev/null

    echo ""
    print_status "info" ":"
    echo "1.  medicine_data_for_ros2.json ROS2"
    echo "2.  python3 medicine_ros2_node.py ROS2"
    echo "3.  http://localhost:8000/Medicine.html "
    echo "4.  test_prescription.json "
}

#
main_test
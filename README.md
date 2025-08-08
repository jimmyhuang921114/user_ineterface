# ROS2 Hospital Medicine Management System

## Quick Start

### 1. Start Web System and Adapter
```bash
cd user_interface
python3 integration_with_your_node.py
```

### 2. Start Your ROS2 Node
```bash
# New terminal
source /opt/ros/humble/setup.bash
export ORDER_BASE_URL='http://127.0.0.1:8002'
python3 your_order_handler_node.py
```

### 3. (Optional) Start Medicine Query Service
```bash
# New terminal  
source /opt/ros/humble/setup.bash
python3 user_interface/medicine_detail_service_node.py
```

## YAML Order Format

Your ROS2 node will receive:
```yaml
order_id: "000001"
prescription_id: 1
patient_name: "John Doe"
medicine:
  - name: Aspirin
    amount: 10
    locate: [2, 3]
    prompt: tablet
```

## Web Interface

- Medicine Management: http://localhost:8001/integrated_medicine_management.html
- Doctor Workstation: http://localhost:8001/doctor.html  
- Prescription Management: http://localhost:8001/Prescription.html

## ROS2 Medicine Query Service

```bash
ros2 service call /hospital/get_medicine_detail tm_robot_if/srv/MedicineDetail "{name: 'Aspirin'}"
ros2 service call /hospital/get_all_medicines tm_robot_if/srv/MedicineDetail "{name: ''}"
ros2 service call /hospital/search_medicines tm_robot_if/srv/MedicineDetail "{name: 'pain'}"
```

## Essential Files

- `user_interface/database_final.py` - Database models
- `user_interface/simple_server_final.py` - Web server
- `user_interface/integration_with_your_node.py` - ROS2 adapter
- `user_interface/medicine_detail_service_node.py` - Medicine query service
- `user_interface/test_order_flow.py` - Testing utility
- `user_interface/static/` - Web interface files
- `user_interface/hospital_medicine_final.db` - Database

## Implementation in Your Node

```python
def _process_medicine(self, order_id: str, med: Dict[str, Any], idx: int, total: int):
    name = med.get('name')
    amount = med.get('amount') 
    locate = med.get('locate')  # [row, col]
    prompt = med.get('prompt')  # tablet/capsule/white_circle_box
    
    # Implement your robot logic here
    # 1. Move to locate position
    # 2. Pick medicine according to prompt and amount
    # 3. Place in dispensing area
```

## Testing

```bash
cd user_interface
python3 test_order_flow.py basic
```

## Environment Variables for Your ROS2 Node

```bash
export ORDER_BASE_URL='http://127.0.0.1:8002'
export ORDER_PULL_URL='http://127.0.0.1:8002/api/order/next'
export ORDER_PULL_INTERVAL='3'
export ORDER_PROGRESS_PATH='/api/order/progress'
export ORDER_COMPLETE_PATH='/api/order/complete'
```

## Architecture

```
Web Interface (8001) <-> Adapter API (8002) <-> Your ROS2 Node
```

The system automatically:
- Pushes YAML orders to your ROS2 node via HTTP
- Processes one order at a time
- Waits for completion before sending next order
- Updates prescription status in web interface
- Provides medicine detail query via ROS2 service

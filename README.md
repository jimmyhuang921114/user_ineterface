# Hospital Medicine Management System

A fully functional ROS2-integrated hospital medicine management system with web interface and automated order processing.

## Quick Start

### 1. Install Dependencies
```bash
pip3 install fastapi uvicorn sqlalchemy pydantic requests pyyaml
```

### 2. Start the Complete System
```bash
cd user_interface
python3 start_working_system.py
```

### 3. Access Web Interfaces
- Medicine Management: http://localhost:8001/integrated_medicine_management.html
- Doctor Interface: http://localhost:8001/doctor.html
- Prescription Management: http://localhost:8001/Prescription.html
- API Documentation: http://localhost:8001/docs

## ROS2 Integration

### Environment Variables for Your ROS2 Node
```bash
export ORDER_BASE_URL='http://127.0.0.1:8002'
export ORDER_PULL_URL='http://127.0.0.1:8002/api/order/next'
export ORDER_PULL_INTERVAL='3'
export ORDER_PROGRESS_PATH='/api/order/progress'
export ORDER_COMPLETE_PATH='/api/order/complete'
```

### YAML Order Format
Your ROS2 node will receive orders in this format:
```yaml
order_id: "000001"
prescription_id: 1
patient_name: "John Doe"
medicine:
  - name: Aspirin
    amount: 10
    locate: [1, 2]
    prompt: tablet
  - name: Vitamin C
    amount: 5
    locate: [2, 1]
    prompt: capsule
```

### ROS2 Node Implementation Example
```python
def process_medicine(self, order_id: str, med: Dict[str, Any], idx: int, total: int):
    name = med.get('name')
    amount = med.get('amount')
    locate = med.get('locate')  # [row, col]
    prompt = med.get('prompt')  # tablet/capsule/white_circle_box
    
    # Your robot logic here:
    # 1. Move to position: locate[0], locate[1]
    # 2. Pick medicine according to prompt and amount
    # 3. Place in dispensing area
    
    # Report progress
    requests.post(f"{ORDER_BASE_URL}/api/order/progress", json={
        "order_id": order_id,
        "stage": "processing",
        "message": f"Processing {name}",
        "item": name,
        "index": idx,
        "total": total
    })
    
    # After completion, report success
    requests.post(f"{ORDER_BASE_URL}/api/order/complete", json={
        "order_id": order_id,
        "status": "success",
        "details": "Order completed successfully"
    })
```

## System Architecture

```
Web Interface (8001) <-> ROS2 Adapter (8002) <-> Your ROS2 Node
```

The system automatically:
- Manages medicine inventory
- Creates prescriptions via web interface
- Converts prescriptions to YAML orders
- Provides HTTP API for ROS2 nodes to pull orders
- Processes one order at a time
- Updates status based on ROS2 node feedback
- Maintains prescription status in web interface

## Core Files

- `simple_working_system.py` - Main hospital system server
- `ros2_node_adapter.py` - ROS2 integration adapter
- `start_working_system.py` - System launcher
- `static/` - Web interface files

## Features

- Complete medicine inventory management
- Doctor prescription interface
- Automated prescription-to-order conversion
- ROS2 HTTP API integration
- Real-time status tracking
- Stock management with automatic deduction
- One-order-at-a-time processing
- YAML order format for ROS2 compatibility

## Testing

1. Start the system: `python3 start_working_system.py`
2. Create medicines via web interface
3. Create prescriptions via doctor interface
4. Your ROS2 node will automatically receive orders
5. Monitor status via web interface

## Troubleshooting

- Ensure ports 8001 and 8002 are available
- Check that all dependencies are installed
- Verify your ROS2 node can access the adapter API
- Monitor logs for connection issues

System designed for production use with stable, emoji-free codebase.

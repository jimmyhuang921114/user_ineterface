# Hospital Medicine Management System

Simple, clean system with built-in web interface and ROS2 integration.

## Quick Start

### 1. Install Dependencies
```bash
pip3 install fastapi uvicorn sqlalchemy pydantic requests pyyaml
```

### 2. Start System
```bash
cd user_interface
python3 minimal_system.py
```

### 3. Use Web Interface
- Medicine Management: http://localhost:8001/medicine
- Doctor Interface: http://localhost:8001/doctor
- Prescription Management: http://localhost:8001/prescriptions

## ROS2 Integration

### Pull Orders
```bash
curl http://localhost:8001/api/order/next
```

### Report Progress
```bash
curl -X POST http://localhost:8001/api/order/progress \
  -H "Content-Type: application/json" \
  -d '{"order_id": "000001", "stage": "processing", "message": "Working on order"}'
```

### Report Completion
```bash
curl -X POST http://localhost:8001/api/order/complete \
  -H "Content-Type: application/json" \
  -d '{"order_id": "000001", "status": "success", "details": "Order completed"}'
```

## YAML Order Format

Your ROS2 node receives:
```yaml
order_id: "000001"
prescription_id: 1
patient_name: "John Doe"
medicine:
  - name: Aspirin
    amount: 10
    locate: [1, 1]
    prompt: tablet
```

## Implementation Example

```python
import requests
import yaml
import time

BASE_URL = "http://localhost:8001"

def poll_for_orders():
    while True:
        try:
            # Get next order
            response = requests.get(f"{BASE_URL}/api/order/next")
            if response.status_code == 204:
                print("No orders available")
                time.sleep(3)
                continue
            
            data = response.json()
            order = data["order"]
            
            print(f"Processing order: {order['order_id']}")
            print(f"Patient: {order['patient_name']}")
            
            # Process each medicine
            for i, med in enumerate(order["medicine"]):
                name = med["name"]
                amount = med["amount"]
                location = med["locate"]  # [row, col]
                
                print(f"Processing {name} (amount: {amount}) at location {location}")
                
                # Report progress
                requests.post(f"{BASE_URL}/api/order/progress", json={
                    "order_id": order["order_id"],
                    "stage": "processing",
                    "message": f"Processing {name}",
                    "item": name,
                    "index": i + 1,
                    "total": len(order["medicine"])
                })
                
                # Your robot logic here
                time.sleep(2)  # Simulate work
            
            # Report completion
            requests.post(f"{BASE_URL}/api/order/complete", json={
                "order_id": order["order_id"],
                "status": "success",
                "details": "All medicines processed"
            })
            
            print(f"Order {order['order_id']} completed")
            
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(5)

if __name__ == "__main__":
    poll_for_orders()
```

## System Features

- Single file system - no complex dependencies
- Built-in web interface with embedded HTML/CSS/JS
- Automatic database creation and sample data
- Real-time prescription status updates
- Stock management with automatic deduction
- One-order-at-a-time processing
- Complete ROS2 HTTP API integration

## File Structure

```
workspace/
├── README.md
├── requirements.txt
└── user_interface/
    ├── minimal_system.py    # Complete system in one file
    └── static/             # Original web files (optional)
```

The system is completely self-contained in `minimal_system.py` with embedded web interface.

## Testing

1. Start system: `python3 minimal_system.py`
2. Open http://localhost:8001/medicine
3. Add medicines
4. Open http://localhost:8001/doctor  
5. Create prescriptions
6. Your ROS2 node will receive orders automatically

Clean, working, production-ready system.

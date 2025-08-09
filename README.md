# Hospital Medicine Management System

Complete hospital medicine management system with original UI style and ROS2 integration.

## Features

- **Original Professional UI Design** - Beautiful gradient backgrounds and modern interface
- **Single File System** - Everything embedded in one Python file
- **Complete Medicine Management** - Add, view, search, and manage medicine inventory
- **Doctor Interface** - Create prescriptions with medicine selection
- **Prescription Management** - Real-time status tracking and updates
- **ROS2 Integration** - HTTP API for order pulling and status reporting
- **Automatic Database Setup** - Sample data included
- **Stock Management** - Automatic deduction on prescription creation

## Quick Start

### 1. Install Dependencies
```bash
pip3 install fastapi uvicorn sqlalchemy pydantic requests pyyaml
```

### 2. Start System
```bash
cd user_interface
python3 hospital_system_with_ui.py
```

### 3. Access Web Interfaces
- **Medicine Management**: http://localhost:8001/integrated_medicine_management.html
- **Doctor Interface**: http://localhost:8001/doctor.html
- **Prescription Management**: http://localhost:8001/Prescription.html

## Web Interface Features

### Medicine Management
- Sidebar navigation with beautiful styling
- Tabbed interface for different functions
- Add new medicines with categories and dosages
- Stock adjustment with interactive cards
- Medicine search functionality
- System status monitoring

### Doctor Interface
- Professional header design
- Medicine selection with visual cards
- Real-time stock information
- Selected medicines tracking
- Easy prescription creation

### Prescription Management
- Statistical dashboard
- Real-time prescription status
- Auto-refresh functionality
- Status update controls
- Medicine list display

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

Your ROS2 node receives orders in this format:
```yaml
order_id: "000001"
prescription_id: 1
patient_name: "John Doe"
medicine:
  - name: Aspirin
    amount: 10
    locate: [1, 1]
    prompt: tablet
  - name: Vitamin C
    amount: 5
    locate: [1, 2]
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

## UI Design Features

- **Professional Gradient Backgrounds** - Beautiful blue-purple gradients
- **Modern Card Design** - Rounded corners and subtle shadows
- **Responsive Layout** - Works on desktop and tablet
- **Interactive Elements** - Hover effects and smooth transitions
- **Status Badges** - Color-coded for easy recognition
- **Sidebar Navigation** - Fixed navigation with icons
- **Tab Interface** - Organized content sections

## File Structure

```
workspace/
├── README.md
├── requirements.txt
└── user_interface/
    ├── hospital_system_with_ui.py    # Complete system with original UI
    ├── hospital_medicine.db          # Database (auto-created)
    └── static/                       # Original web files (optional)
```

## API Endpoints

- `GET /` - System status
- `GET /api/medicine/` - Get all medicines
- `POST /api/medicine/` - Create medicine
- `GET /api/prescription/` - Get all prescriptions
- `POST /api/prescription/` - Create prescription
- `PUT /api/prescription/{id}/status` - Update prescription status
- `GET /api/order/next` - Pull next order (ROS2)
- `POST /api/order/progress` - Report progress (ROS2)
- `POST /api/order/complete` - Report completion (ROS2)

## Sample Data

The system includes sample medicines:
- Aspirin (Pain Relief) - 100 units at A1
- Vitamin C (Vitamins) - 50 units at A2
- Ibuprofen (Pain Relief) - 75 units at B1
- Paracetamol (Pain Relief) - 120 units at B2
- Calcium (Supplements) - 60 units at C1

## Testing

1. Start system: `python3 hospital_system_with_ui.py`
2. Open medicine management interface
3. Add new medicines or adjust stock
4. Use doctor interface to create prescriptions
5. Monitor prescription status in management interface
6. Your ROS2 node will receive orders automatically

## Production Ready

- Clean, professional UI with original styling
- Complete functionality in single file
- Embedded HTML/CSS/JavaScript
- Automatic database initialization
- Real-time updates and status tracking
- Full ROS2 HTTP API integration
- Stock management and deduction
- Error handling and validation

Perfect for hospital environments requiring both web interface and ROS2 integration.

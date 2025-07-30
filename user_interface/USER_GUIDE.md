# Hospital Medicine Management System - User Guide
醫院藥物管理系統 - 使用者指南

## 📚 Table of Contents
1. [Package Overview](#package-overview)
2. [System Structure](#system-structure)
3. [Read Operations](#read-operations)
4. [Write Operations](#write-operations)
5. [API Usage Examples](#api-usage-examples)
6. [Package Operations](#package-operations)
7. [Troubleshooting](#troubleshooting)

---

## 📦 Package Overview

This system is organized into focused packages for different functionalities:

### Core Packages Structure
```
user_interface/
├── 🚀 Server Package
│   └── enhanced_server.py          # Main application server
├── 📖 API Client Package  
│   ├── api_client_examples.py      # Complete API examples
│   └── quick_api_guide.py          # Quick start guide
├── 🧪 Testing Package
│   └── test_enhanced_system.py     # System testing
├── ⚙️ Management Package
│   ├── run_system.py               # System management
│   └── start_local.py              # Local startup
├── 📋 Documentation Package
│   ├── packages_usage.md           # Package usage guide
│   ├── system_architecture.md      # System architecture
│   ├── QUICK_START.md              # Quick start
│   └── USER_GUIDE.md               # This file
└── 🔧 Extension Packages
    ├── route/                      # API routing
    ├── services/                   # Business logic
    ├── model/                      # Data models
    └── schemas/                    # Data schemas
```

---

## 🏗️ System Structure

### Data Flow Architecture
```
Frontend (web/) ←→ API Layer ←→ Business Logic ←→ Data Storage
      ↓                ↓              ↓             ↓
  User Interface → FastAPI Routes → Services → Memory Database
```

### Package Dependencies
```
enhanced_server.py (Core)
    ├── Depends on: fastapi, uvicorn, pydantic
    ├── Serves: web/ directory (static files)
    └── Provides: REST API endpoints

api_client_examples.py
    ├── Depends on: requests, json
    └── Connects to: enhanced_server.py API

test_enhanced_system.py
    ├── Depends on: requests
    └── Tests: enhanced_server.py endpoints
```

---

## 📖 Read Operations

### 1. Reading Medicine Data

#### Basic Medicine Information
```python
import requests

# Get all medicines
response = requests.get('http://localhost:8000/api/medicine/')
medicines = response.json()

for medicine in medicines:
    print(f"Name: {medicine['name']}")
    print(f"Amount: {medicine['amount']}")
    print(f"Position: {medicine['position']}")
```

#### Detailed Medicine Information
```python
# Get detailed medicine info by name
medicine_name = "心律錠"
response = requests.get(f'http://localhost:8000/api/medicine/detailed/{medicine_name}')
detailed_info = response.json()

print(f"Basic Info: {detailed_info['基本資訊']}")
print(f"Indications: {detailed_info['適應症']}")
print(f"Side Effects: {detailed_info['可能的副作用']}")
```

#### Search Medicine by Code
```python
# Search by packaging code
code = "202801"
response = requests.get(f'http://localhost:8000/api/medicine/search/code/{code}')
search_results = response.json()

for name, data in search_results.items():
    print(f"Found: {name}")
    print(f"Matched Code: {data['matched_code']}")
```

### 2. Reading Patient Data

#### Get All Patients
```python
response = requests.get('http://localhost:8000/api/patients/')
patients = response.json()

for patient in patients:
    print(f"ID: {patient['id']}, Name: {patient['name']}")
    print(f"Age: {patient['age']}, Gender: {patient['gender']}")
```

#### Get Patient Records
```python
patient_id = 1
response = requests.get(f'http://localhost:8000/api/records/patient/{patient_id}')
records = response.json()

for record in records:
    print(f"Visit Date: {record['visit_date']}")
    print(f"Diagnosis: {record['diagnosis']}")
    print(f"Medicines: {record['prescribed_medicines']}")
```

### 3. Export Data (Read All)

#### Export Complete System Data
```python
response = requests.get('http://localhost:8000/api/export/complete')
complete_data = response.json()

# Save to file
import json
with open('system_backup.json', 'w', encoding='utf-8') as f:
    json.dump(complete_data, f, ensure_ascii=False, indent=2)
```

---

## ✏️ Write Operations

### 1. Creating Medicine Data

#### Add Basic Medicine
```python
new_medicine = {
    "name": "新藥物",
    "amount": 100,
    "position": "A1-05",
    "unit": "錠",
    "expiry_date": "2024-12-31"
}

response = requests.post('http://localhost:8000/api/medicine/', json=new_medicine)
created_medicine = response.json()
print(f"Created medicine with ID: {created_medicine['id']}")
```

#### Add Detailed Medicine Information
```python
detailed_medicine = {
    "medicine_name": "新藥物",
    "medicine_data": {
        "基本資訊": {
            "名稱": "新藥物",
            "廠商": "某製藥公司",
            "劑量": "10毫克"
        },
        "外觀": {
            "顏色": "白色",
            "形狀": "圓形"
        },
        "包裝編號": {
            "編號1": "NEW001",
            "編號2": "NEW002"
        },
        "適應症": "測試用藥物",
        "可能的副作用": "無已知副作用",
        "使用說明": "按醫囑服用",
        "注意事項": "存放於陰涼乾燥處"
    }
}

response = requests.post('http://localhost:8000/api/medicine/detailed/', json=detailed_medicine)
```

### 2. Creating Patient Data

#### Add New Patient
```python
new_patient = {
    "name": "張三",
    "age": 30,
    "gender": "男",
    "phone": "0912345678",
    "address": "台北市信義區",
    "medical_history": "無特殊病史",
    "allergies": "無"
}

response = requests.post('http://localhost:8000/api/patients/', json=new_patient)
created_patient = response.json()
print(f"Created patient with ID: {created_patient['id']}")
```

### 3. Creating Medical Records

#### Add Patient Record
```python
from datetime import datetime

new_record = {
    "patient_id": 1,
    "visit_date": datetime.now().isoformat(),
    "diagnosis": "高血壓",
    "prescribed_medicines": ["心律錠 10mg"],
    "dosage_instructions": "每日一次，飯後服用",
    "doctor_notes": "定期回診追蹤"
}

response = requests.post('http://localhost:8000/api/records/', json=new_record)
created_record = response.json()
print(f"Created record with ID: {created_record['id']}")
```

### 4. Update Operations

#### Update Medicine
```python
medicine_id = 1
updated_data = {
    "name": "更新的藥名",
    "amount": 150,
    "position": "A1-06"
}

response = requests.put(f'http://localhost:8000/api/medicine/{medicine_id}', json=updated_data)
```

#### Update Patient
```python
patient_id = 1
updated_patient = {
    "name": "張三",
    "age": 31,  # Updated age
    "phone": "0987654321"  # Updated phone
}

response = requests.put(f'http://localhost:8000/api/patients/{patient_id}', json=updated_patient)
```

### 5. Delete Operations

#### Delete Medicine
```python
medicine_id = 1
response = requests.delete(f'http://localhost:8000/api/medicine/{medicine_id}')
if response.status_code == 200:
    print("Medicine deleted successfully")
```

#### Delete Patient Record
```python
record_id = 1
response = requests.delete(f'http://localhost:8000/api/records/{record_id}')
if response.status_code == 200:
    print("Record deleted successfully")
```

---

## 🔧 API Usage Examples

### Using the API Client Package

#### Complete API Client Example
```python
from api_client_examples import HospitalSystemAPI

# Initialize API client
api = HospitalSystemAPI()

# Medicine operations
medicines = api.get_all_medicines_detailed()
heart_medicine = api.get_medicine_by_name("心律錠")

# Patient operations
patients = api.get_all_patients()
patient_records = api.get_patient_records(1)

# Search operations
code_results = api.search_medicine_by_code("202801")

# Export operations
medicines_data = api.export_integrated_medicines("backup.json")
```

#### Quick API Guide Usage
```python
from quick_api_guide import get_medicine_info, export_data

# Quick medicine lookup
medicine_info = get_medicine_info("心律錠")
print(medicine_info)

# Quick data export
all_data = export_data('complete')
```

---

## 📋 Package Operations

### 1. Server Package Operations

#### Start the Server
```bash
# Method 1: Full system management
python3 run_system.py

# Method 2: Local startup
python3 start_local.py

# Method 3: Direct server start
python3 enhanced_server.py
```

#### Server Configuration
```python
# In enhanced_server.py
app = FastAPI(
    title="Hospital Medicine Management System",
    description="Complete hospital management solution",
    version="2.0.0"
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### 2. Testing Package Operations

#### Run System Tests
```bash
python3 test_enhanced_system.py
```

#### Custom Test Creation
```python
import requests

def test_custom_feature():
    # Test medicine creation
    new_medicine = {"name": "Test Medicine", "amount": 10}
    response = requests.post('http://localhost:8000/api/medicine/', json=new_medicine)
    assert response.status_code == 200
    
    # Test medicine retrieval
    response = requests.get('http://localhost:8000/api/medicine/')
    medicines = response.json()
    assert len(medicines) > 0
```

### 3. Extension Package Operations

#### Adding New Routes (route/ package)
```python
# In route/custom_routes.py
from fastapi import APIRouter

router = APIRouter()

@router.get("/api/custom/endpoint")
async def custom_endpoint():
    return {"message": "Custom endpoint"}

# Add to main server
from route.custom_routes import router as custom_router
app.include_router(custom_router)
```

#### Adding Business Logic (services/ package)
```python
# In services/custom_service.py
def process_custom_data(data):
    # Custom business logic
    processed_data = data.upper()
    return processed_data
```

#### Adding Data Models (model/ package)
```python
# In model/custom_models.py
from pydantic import BaseModel

class CustomModel(BaseModel):
    id: int
    name: str
    description: str
```

---

## 🔍 Troubleshooting

### Common Read Operation Issues

#### 1. Connection Refused
```python
try:
    response = requests.get('http://localhost:8000/api/medicine/')
except requests.exceptions.ConnectionError:
    print("Server not running. Start with: python3 enhanced_server.py")
```

#### 2. Empty Response
```python
response = requests.get('http://localhost:8000/api/medicine/')
if response.status_code == 200:
    data = response.json()
    if not data:
        print("No data available. Add some medicines first.")
```

#### 3. Invalid Medicine Name
```python
medicine_name = "NonExistent"
response = requests.get(f'http://localhost:8000/api/medicine/detailed/{medicine_name}')
if response.status_code == 404:
    print(f"Medicine '{medicine_name}' not found")
```

### Common Write Operation Issues

#### 1. Validation Errors
```python
invalid_medicine = {"name": "", "amount": -1}  # Invalid data
response = requests.post('http://localhost:8000/api/medicine/', json=invalid_medicine)
if response.status_code == 422:
    print("Validation error:", response.json())
```

#### 2. Duplicate Entries
```python
# Check if medicine exists before creating
existing = requests.get('http://localhost:8000/api/medicine/')
existing_names = [m['name'] for m in existing.json()]
if new_medicine_name not in existing_names:
    # Safe to create
    requests.post('http://localhost:8000/api/medicine/', json=new_medicine)
```

### Server Package Issues

#### 1. Port Already in Use
```bash
# Check what's using port 8000
lsof -i :8000

# Kill the process
pkill -f uvicorn
```

#### 2. Missing Dependencies
```bash
pip3 install fastapi uvicorn pydantic requests
```

#### 3. Database Connection Issues
```python
# The system uses in-memory database
# Data is reset on server restart
# For persistent data, use the export/import features
```

---

## 📖 Reading Package Code

### Understanding the Server Package
```python
# enhanced_server.py structure
"""
1. Imports and setup
2. Data models (Pydantic)
3. In-memory databases
4. API routes
5. Static file serving
6. Server startup
"""
```

### Understanding the API Client Package
```python
# api_client_examples.py structure
"""
1. HospitalSystemAPI class
2. Medicine management methods
3. Patient management methods
4. Record management methods
5. Export methods
6. Usage examples
"""
```

---

## ✏️ Writing Package Extensions

### Adding New Features

#### 1. Add New Data Model
```python
# In enhanced_server.py, add new Pydantic model
class NewFeature(BaseModel):
    id: Optional[int] = None
    name: str
    description: str
    created_date: datetime
```

#### 2. Add New API Endpoint
```python
# Add new route
@app.post("/api/newfeature/")
async def create_new_feature(feature: NewFeature):
    # Implementation
    return {"id": 1, "message": "Created successfully"}
```

#### 3. Add Client Method
```python
# In api_client_examples.py, add new method
def create_new_feature(self, feature_data):
    response = self.session.post(f"{self.base_url}/api/newfeature/", json=feature_data)
    return response.json() if response.status_code == 200 else None
```

---

## 🎯 Best Practices

### For Reading Operations
1. Always check response status codes
2. Handle exceptions gracefully
3. Use appropriate timeouts
4. Cache frequently accessed data

### For Writing Operations
1. Validate data before sending
2. Check for duplicates
3. Handle errors appropriately
4. Confirm successful operations

### For Package Management
1. Keep core functionality in enhanced_server.py
2. Use separate files for extensions
3. Document new features
4. Test thoroughly before deployment

---

This user guide provides comprehensive information about reading and writing operations, package structure, and system usage. Use it as a reference for all your interactions with the Hospital Medicine Management System.
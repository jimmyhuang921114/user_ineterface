#  -
Hospital Medicine Management System - System Architecture Documentation

## 1.

FastAPI

## 2.

```

  (Frontend Layer)
    web/html/
       Medicine.html          #
       Patients.html          #
       Records.html           #
       doctor.html            #
       Prescription.html      #
    web/css/
       medicine.css           #
       doctor_style.css       #
       Prescription.css       #
    web/js/
        medicine.js            #
        doctor.js              #
        Prescription.js        #

 API (API Layer)
    enhanced_server.py         # API
    hospital_server.py         # API
    API
        API
        API
        API
        API

  (Business Logic Layer)
    user_interface/route/
       routes_medicine.py     #
    user_interface/services/
       crud_medicine.py       # CRUD
    user_interface/models.py   #

  (Data Access Layer)
     (In-Memory Database)
       medicines_db          #
       detailed_medicines_db #
       patients_db           #
       patient_records_db    #
    JSON

  (Utility Layer)
     api_client_examples.py     # API
     quick_api_guide.py         #
     test_enhanced_system.py    #
     run_system.py              #
```

## 3.

```
[] ←→ [HTTP/JSON] ←→ [FastAPI] ←→ [] ←→ []
     ↓                                                            ↓
[]                                                [JSON]
```

## 4.

### 4.1

#### HTML
- **Medicine.html**:
  - :
  - : Handsontable

- **Patients.html**:
  - : CRUD
  - :

- **Records.html**:
  - :
  - :

- **doctor.html**:
  - :

- **Prescription.html**:
  - :

#### JavaScript
- **medicine.js**:
  - MedicineManager
  - : API
  - : loadMedicines, addMedicine, deleteMedicine, exportToJSON

#### CSS
- **medicine.css**:
- **doctor_style.css**:
- **Prescription.css**:

### 4.2 API

####
- **enhanced_server.py**:
  - : FastAPI
  - : APICORS
  - :

#### API

##### API
```
POST   /api/medicine/                    #
GET    /api/medicine/                    #
PUT    /api/medicine/{id}                #
DELETE /api/medicine/{id}                #
POST   /api/medicine/detailed/           #
GET    /api/medicine/detailed/{name}     #
GET    /api/medicine/detailed/           #
GET    /api/medicine/search/detailed/{query}  #
GET    /api/medicine/search/code/{code}  #
GET    /api/medicine/integrated/{name}   #
```

##### API
```
GET    /api/patients/                    #
POST   /api/patients/                    #
GET    /api/patients/{id}                #
PUT    /api/patients/{id}                #
DELETE /api/patients/{id}                #
```

##### API
```
GET    /api/records/                     #
POST   /api/records/                     #
GET    /api/records/patient/{id}         #
DELETE /api/records/{id}                 #
```

##### API
```
GET    /api/export/medicines/basic       #
GET    /api/export/medicines/detailed    #
GET    /api/export/medicines/integrated  #
GET    /api/export/patients              #
GET    /api/export/records               #
GET    /api/export/complete              #
```

### 4.3

####
- **routes_medicine.py**:
  - : API
  - : CRUD

####
- **crud_medicine.py**:
  - : CRUD
  - : create_medicine, get_medicine_by_id, update_medicine, delete_medicine

####
- **models.py**:
  - :
  - : SQLAlchemy

### 4.4

####
- **medicines_db**:
  - : Python List
  - :
  - : {id, name, amount, position, unit, expiry_date}

- **detailed_medicines_db**:
  - : Python Dictionary
  - :
  - : JSON

- **patients_db**:
  - : Python List
  - :
  - : {id, name, age, gender, phone, address, medical_history, allergies}

- **patient_records_db**:
  - : Python List
  - :
  - : {id, patient_id, visit_date, diagnosis, prescribed_medicines, dosage_instructions, doctor_notes}

### 4.5

#### API
- **api_client_examples.py**:
  - : API
  - : HospitalSystemAPI
  - : API

- **quick_api_guide.py**:
  - : API
  - :

####
- **test_enhanced_system.py**:
  - :
  - : API

####
- **run_system.py**:
  - :
  - :

## 5.

###
- **FastAPI**: Web
- **Uvicorn**: ASGI
- **Pydantic**:
- **SQLAlchemy**: ORM ()
- **Jinja2**:

###
- **HTML5**:
- **CSS3**:
- **JavaScript (ES6)**:
- **Handsontable**:
- **Fetch API**: HTTP

###
- **Python 3.x**:
- **Requests**: HTTP
- **JSON**:

## 6.

### 6.1
-
-
-

### 6.2
-
- API
-

### 6.3
-
-
-

### 6.4
- Web
- API
-

## 7.

### 7.1
- PostgreSQLMySQL
- SQLAlchemy

### 7.2
-
- APIRESTful

### 7.3
-
- Redis
-

## 8.

### 8.1 API
- CORS
-
- HTTP

### 8.2
-
-

## 9.

### 9.1
```bash
python3 enhanced_server.py
```

### 9.2
```bash
uvicorn enhanced_server:app --host 0.0.0.0 --port 8000 --workers 4
```

### 9.3
- Docker
- docker-compose
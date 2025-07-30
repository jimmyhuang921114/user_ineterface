#  -
Hospital Medicine Management System - Package Usage Documentation

##

### 1.  (Backend Server Package)

#### 1.1
```
user_interface/
 enhanced_server.py     #
 hospital_server.py     #
 final_server.py        #
 working_server.py      #
 main.py               #
```

****:

|  |  |  |  |
|---------|----------|----------|----------|
| **enhanced_server.py** |  |  |  |
| **hospital_server.py** |  | JSON |  |
| **final_server.py** |  |  |  |
| **working_server.py** |  |  |  |
| **main.py** |  | SQLAlchemy ORM |  |

#### 1.2
```
user_interface/
 models.py             #
 route/
    routes_medicine.py    #
 services/
     crud_medicine.py      # CRUD
```

****:
- **models.py**: SQLAlchemy ORM
- **routes_medicine.py**: API
- **crud_medicine.py**:

### 2. API (API Client Package)

```
user_interface/
 api_client_examples.py   # API
 quick_api_guide.py       #
```

****:

#### 2.1 api_client_examples.py
- ****: API
- ****: `HospitalSystemAPI`
- ****:
- ****:
  -
  -
  -
  -
  -

#### 2.2 quick_api_guide.py
- ****: API
- ****:
- ****:
  -
  -
  -

### 3.  (Testing Package)

```
user_interface/
 test_enhanced_system.py  #
 quick_test.py            #
 test_api.py              # API
```

****:

#### 3.1 test_enhanced_system.py
- ****:
- ****:
  - API
  -
  -
  -
- ****:

#### 3.2 quick_test.py
- ****:
- ****:
  -
  - CRUD
  -
- ****:

#### 3.3 test_api.py
- ****: API
- ****: API

### 4.  (System Management Package)

```
user_interface/
 run_system.py           #
 start_server.py         #
```

****:

#### 4.1 run_system.py
- ****:
- ****:
  -
  -
  -
  -
- ****:

#### 4.2 start_server.py
- ****:
- ****:

### 5.  (Frontend Package)

```
web/
 html/
    Medicine.html        #
    Patients.html        #
    Records.html         #
    doctor.html          #
    Prescription.html    #
 css/
    medicine.css         #
    doctor_style.css     #
    Prescription.css     #
 js/
     medicine.js          #
     doctor.js            #
     Prescription.js      #
```

****:

#### 5.1 HTML
|  |  |  | API |
|------|----------|----------|---------|
| **Medicine.html** |  | Handsontable | /api/medicine/ |
| **Patients.html** |  |  | /api/patients/ |
| **Records.html** |  |  | /api/records/ |
| **doctor.html** |  |  | API |
| **Prescription.html** |  |  | API |

#### 5.2 CSS
- **medicine.css**:
- **doctor_style.css**:
- **Prescription.css**:

#### 5.3 JavaScript
- **medicine.js**:
  - `MedicineManager`
  - API
  -
  -
- **doctor.js**:
- **Prescription.js**:

### 6.  (Documentation Package)

```
user_interface/
 system_architecture.md   #
 system_functions.md      #
 project_structure.md     #
 packages_usage.md        #
 README.md               #
```

****:
- **system_architecture.md**:
- **system_functions.md**: API
- **project_structure.md**:
- **packages_usage.md**:
- **README.md**:

##

###

####
1. **quick_api_guide.py** - API
2. **final_server.py** -
3. **Medicine.html** -

####
1. **enhanced_server.py** -
2. **api_client_examples.py** -
3. **test_enhanced_system.py** -

####
1. **run_system.py** -
2. **enhanced_server.py** -
3. **web/**  -

###

####
1.  **enhanced_server.py** API
2.  **api_client_examples.py**
3.  **test_enhanced_system.py**
4.  **web/**

####
1.  **models.py**
2.  **enhanced_server.py**
3.  **services/**

###

```
: run_system.py
    ↓
: enhanced_server.py
    ↓
: web/
    ↓
API: api_client_examples.py
    ↓
: test_enhanced_system.py
```

###

####
```
:
 enhanced_server.py      # API
 web/html/Medicine.html  #
 web/css/medicine.css    #
 web/js/medicine.js      #
```

####
```
:
 enhanced_server.py          #
 api_client_examples.py      # API
 test_enhanced_system.py     #
 run_system.py              #
 web/                    #
  .md                #
```

API
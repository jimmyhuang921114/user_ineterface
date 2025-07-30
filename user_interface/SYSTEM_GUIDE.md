#  -

##

```

  (fixed_server.py)

    Medicine.html     #
    doctor.html       #
    Prescription.html #
 ROS2
    ros2_medicine_client.py
    medicine_ros2_node.py

     test.sh
     examples_package.py
```

##

### 1.

#### `fixed_server.py` -
****
- API
-
-
-

****
```python
# API
GET  /api/medicine/                    #
POST /api/medicine/                    #
PUT  /api/medicine/{id}                #
POST /api/medicine/detailed/           #

# API
POST /api/prescription/                #
GET  /api/prescription/                #
PUT  /api/prescription/{id}/status     #

# API
GET  /api/test                         #
```

### 2.

#### `Medicine.html` -
****
- Handsontable
-
-

****
-
-
- /
- JSON

#### `doctor.html` -
****
-
- Medicine
-

****
-
-
- Medicine

#### `Prescription.html` -
****
-
-
-

****
-
-
-

### 3. ROS2

#### `ros2_medicine_client.py` - ROS2
****
- APIROS2
- ROS2
- ROS2

****
```python
# ROS2
/medicine_data          #
/prescription_status    #

# ROS2
/get_all_medicines     #
```

#### `medicine_ros2_node.py` - ROS2
****
- ROS2
-
- ROS2

### 4.

#### `test.sh` -
****
-
-
- API

****
```bash
# 1
add_basic_medicine "A" 100 30 "A1-TEST"

# 2
add_detailed_medicine "A"

# 3ROS2
get_all_medicine_for_ros2

# 4
test_prescription_system

# 5ROS2
create_ros2_service_config
```

#### `examples_package.py` -
****
- API
-
-

##

### 1
```bash
cd /workspace/user_interface

#
python3 fixed_server.py
```

### 2
```bash
#
curl http://localhost:8000/api/test
```

### 3
```bash
#
./test.sh
```

### 4
1. ****`http://localhost:8000/doctor.html`
   -
   - Medicine

2. ****`http://localhost:8000/Medicine.html`
   -
   -

3. ****`http://localhost:8000/Prescription.html`
   -

### 5ROS2
```bash
# ROS2
python3 ros2_medicine_client.py

# ROS2
python3 medicine_ros2_node.py
```

##

### 1.

####
```bash
#
curl -X POST http://localhost:8000/api/medicine/ \
  -H "Content-Type: application/json" \
  -d '{
    "name": "",
    "amount": 100,
    "usage_days": 30,
    "position": "A1-01"
  }'

#
curl http://localhost:8000/api/medicine/
```

####
```bash
#
curl -X POST http://localhost:8000/api/medicine/detailed/ \
  -H "Content-Type: application/json" \
  -d '{
    "medicine_name": "",
    "medicine_data": {
      "": {
        "": "",
        "": "10"
      },
      "": {
        "": "",
        "": ""
      }
    }
  }'
```

####
```bash
#
curl -X POST http://localhost:8000/api/prescription/ \
  -H "Content-Type: application/json" \
  -d '{
    "patient_name": "",
    "doctor_name": "",
    "medicines": [{
      "medicine_name": "",
      "dosage": "10mg",
      "frequency": "",
      "duration": "7"
    }]
  }'
```

### 2.

####
```bash
# examples_package.py
python3 examples_package.py
```

#### ROS2
```bash
# ROS2
cat medicine_data_for_ros2.json

# ROS2
curl http://localhost:8000/api/ros2/services/status
```

##

###
- [ ]
- [ ] API
- [ ] CRUD
- [ ]
- [ ]

###
- [ ] Medicine.html
- [ ] doctor.html
- [ ] Prescription.html
- [ ]
- [ ]

###  ROS2
- [ ] ROS2
- [ ]
- [ ]
- [ ]

###
- [ ]
- [ ]
- [ ] JSON
- [ ]

##

###

1. ****
   ```bash
   #
   lsof -i :8000

   #
   pkill -f fixed_server
   ```

2. **API**
   ```bash
   #
   curl http://localhost:8000/api/test

   #
   tail -f server.log
   ```

3. ****
   ```bash
   #
   ls -la static/html/

   #
   chmod 644 static/html/*.html
   ```

4. **ROS2**
   ```bash
   # ROS2
   echo $ROS_DOMAIN_ID

   #
   ros2 topic list
   ```

##

###
```bash
#
curl http://localhost:8000/ | jq .statistics

#
ps aux | grep fixed_server

# API
time curl http://localhost:8000/api/medicine/
```

##

1. ****API
2. ****
3. ****
4. ****
5. ****


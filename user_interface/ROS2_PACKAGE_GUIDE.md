# ROS2 Package 完整解釋

## 📦 什麼是 ROS2 Package？

ROS2 Package 是 ROS2 系統中的基本組織單位，類似於其他程式語言中的模組或函式庫。每個 package 包含相關的程式碼、配置檔案、依賴關係和元數據。

## 🏗️ ROS2 Package 結構

### 標準 ROS2 Package 目錄結構
```
my_robot_package/
├── package.xml          # Package 元數據和依賴關係
├── setup.py            # Python package 安裝配置
├── setup.cfg           # 建置配置
├── my_robot_package/   # Python 模組目錄
│   ├── __init__.py
│   ├── robot_node.py   # ROS2 節點
│   └── utils.py        # 工具函數
├── launch/             # Launch 檔案目錄
│   └── robot_launch.py
├── config/             # 配置檔案
│   └── params.yaml
├── msg/                # 自定義訊息類型
│   └── CustomMsg.msg
├── srv/                # 自定義服務類型
│   └── CustomSrv.srv
└── resource/           # 資源檔案
    └── my_robot_package
```

### package.xml 範例
```xml
<?xml version="1.0"?>
<package format="3">
  <name>hospital_medicine_ros2</name>
  <version>1.0.0</version>
  <description>Hospital Medicine Management ROS2 Package</description>
  
  <maintainer email="admin@hospital.com">Hospital Admin</maintainer>
  <license>MIT</license>
  
  <!-- 建置工具依賴 -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>
  
  <!-- 執行時依賴 -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  
  <!-- 測試依賴 -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## 🤖 本系統中的 ROS2 整合

### 1. 模擬 ROS2 Package (`ros2_mock_clean.py`)

我們的醫院系統包含一個模擬的 ROS2 package，提供以下功能：

```python
# 模擬的 ROS2 節點類別
class MockHospitalROS2Node:
    """模擬醫院 ROS2 節點"""
    
    def __init__(self):
        # 模擬 ROS2 節點初始化
        self.node_name = "hospital_medicine_node"
        self.order_queue = Queue()
        self.publishers = {}
        self.subscribers = {}
        self.services = {}
```

### 2. 真實 ROS2 Package 結構

如果要建立真實的 ROS2 package，目錄結構如下：

```
hospital_medicine_ros2/
├── package.xml
├── setup.py
├── hospital_medicine_ros2/
│   ├── __init__.py
│   ├── medicine_dispenser_node.py    # 藥物配發節點
│   ├── robot_controller_node.py      # 機器人控制節點
│   ├── inventory_manager_node.py     # 庫存管理節點
│   └── prescription_processor_node.py # 處方籤處理節點
├── launch/
│   ├── hospital_system_launch.py     # 系統啟動檔
│   └── robot_bringup_launch.py       # 機器人啟動檔
├── config/
│   ├── robot_params.yaml             # 機器人參數
│   └── medicine_locations.yaml       # 藥物位置配置
├── msg/
│   ├── MedicineOrder.msg             # 藥物訂單訊息
│   ├── RobotStatus.msg               # 機器人狀態訊息
│   └── InventoryUpdate.msg           # 庫存更新訊息
└── srv/
    ├── ProcessPrescription.srv        # 處理處方籤服務
    └── CheckInventory.srv             # 檢查庫存服務
```

## 📨 ROS2 訊息和服務定義

### 自定義訊息類型 (msg/)

#### MedicineOrder.msg
```
# 藥物訂單訊息
std_msgs/Header header
string order_id
string prescription_id
string patient_name
string patient_id
MedicineItem[] medicines
uint8 priority
float32 estimated_duration

---
# MedicineItem 子訊息
uint32 medicine_id
string medicine_name
string position
uint32 quantity
string dosage
string instructions
```

#### RobotStatus.msg
```
# 機器人狀態訊息
std_msgs/Header header
string robot_id
uint8 status          # 0=idle, 1=moving, 2=picking, 3=error
geometry_msgs/Pose current_pose
float32 battery_level
string current_task
string[] task_queue
```

#### InventoryUpdate.msg
```
# 庫存更新訊息
std_msgs/Header header
uint32 medicine_id
string medicine_name
int32 quantity_change
uint32 current_stock
string reason
string updated_by
```

### 自定義服務類型 (srv/)

#### ProcessPrescription.srv
```
# 請求
uint32 prescription_id
string patient_name
MedicineItem[] medicines
uint8 priority

---
# 回應
bool success
string message
string order_id
float32 estimated_completion_time
```

#### CheckInventory.srv
```
# 請求
uint32[] medicine_ids

---
# 回應
bool all_available
InventoryStatus[] status

---
# InventoryStatus 結構
uint32 medicine_id
string medicine_name
uint32 available_quantity
bool sufficient
```

## 🚀 ROS2 節點實現範例

### 1. 藥物配發節點

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hospital_medicine_ros2.msg import MedicineOrder, RobotStatus
from hospital_medicine_ros2.srv import ProcessPrescription

class MedicineDispenserNode(Node):
    def __init__(self):
        super().__init__('medicine_dispenser_node')
        
        # 建立發布者
        self.robot_cmd_pub = self.create_publisher(
            RobotCommand, 
            '/robot/command', 
            10
        )
        
        # 建立訂閱者
        self.order_sub = self.create_subscription(
            MedicineOrder,
            '/medicine/orders',
            self.handle_medicine_order,
            10
        )
        
        # 建立服務
        self.process_srv = self.create_service(
            ProcessPrescription,
            '/medicine/process_prescription',
            self.process_prescription_callback
        )
        
        self.get_logger().info('Medicine Dispenser Node 已啟動')
    
    def handle_medicine_order(self, msg):
        """處理藥物訂單"""
        self.get_logger().info(f'收到藥物訂單: {msg.order_id}')
        
        # 處理每個藥物項目
        for medicine in msg.medicines:
            self.dispatch_robot_to_medicine(medicine)
    
    def process_prescription_callback(self, request, response):
        """處理處方籤服務回調"""
        try:
            # 驗證處方籤
            if self.validate_prescription(request):
                # 創建訂單
                order_id = self.create_medicine_order(request)
                
                response.success = True
                response.order_id = order_id
                response.estimated_completion_time = self.estimate_completion_time(request.medicines)
                response.message = "處方籤已接受處理"
            else:
                response.success = False
                response.message = "處方籤驗證失敗"
                
        except Exception as e:
            response.success = False
            response.message = f"處理錯誤: {str(e)}"
            
        return response
```

### 2. 機器人控制節點

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from hospital_medicine_ros2.msg import RobotStatus, MedicineOrder

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        
        # 機器人控制發布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        
        # 狀態發布者
        self.status_pub = self.create_publisher(RobotStatus, '/robot/status', 10)
        
        # 任務訂閱者
        self.task_sub = self.create_subscription(
            MedicineOrder,
            '/robot/tasks',
            self.execute_task,
            10
        )
        
        # 定時器發布狀態
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # 機器人狀態
        self.current_pose = Pose()
        self.battery_level = 100.0
        self.robot_status = 0  # idle
        self.current_task = ""
        
    def execute_task(self, order_msg):
        """執行藥物配發任務"""
        self.get_logger().info(f'開始執行任務: {order_msg.order_id}')
        self.current_task = order_msg.order_id
        self.robot_status = 1  # moving
        
        for medicine in order_msg.medicines:
            # 移動到藥物位置
            self.move_to_position(medicine.position)
            
            # 抓取藥物
            self.pick_medicine(medicine)
            
            # 更新狀態
            self.robot_status = 2  # picking
        
        # 任務完成
        self.robot_status = 0  # idle
        self.current_task = ""
    
    def publish_status(self):
        """發布機器人狀態"""
        status_msg = RobotStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.robot_id = "hospital_robot_01"
        status_msg.status = self.robot_status
        status_msg.current_pose = self.current_pose
        status_msg.battery_level = self.battery_level
        status_msg.current_task = self.current_task
        
        self.status_pub.publish(status_msg)
```

## 🔧 Launch 檔案

### hospital_system_launch.py
```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 啟動藥物配發節點
        Node(
            package='hospital_medicine_ros2',
            executable='medicine_dispenser_node',
            name='medicine_dispenser',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        # 啟動機器人控制節點
        Node(
            package='hospital_medicine_ros2',
            executable='robot_controller_node',
            name='robot_controller',
            output='screen',
            parameters=[{'robot_id': 'hospital_robot_01'}]
        ),
        
        # 啟動庫存管理節點
        Node(
            package='hospital_medicine_ros2',
            executable='inventory_manager_node',
            name='inventory_manager',
            output='screen'
        ),
        
        # 啟動 FastAPI 伺服器
        ExecuteProcess(
            cmd=['python3', 'simple_server_clean.py'],
            cwd='/workspace/user_interface',
            output='screen'
        )
    ])
```

## 📊 系統整合架構

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   FastAPI       │    │   ROS2 Bridge   │    │   ROS2 Nodes    │
│   醫院管理系統    │◄──►│                 │◄──►│                 │
│                 │    │                 │    │                 │
│ • 處方籤管理     │    │ • HTTP to ROS2  │    │ • 機器人控制     │
│ • 庫存管理       │    │ • 訊息轉換       │    │ • 藥物配發       │
│ • 用戶界面       │    │ • 狀態同步       │    │ • 庫存同步       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   SQLite DB     │    │   ROS2 Topics   │    │   實體機器人     │
│                 │    │                 │    │                 │
│ • 處方籤資料     │    │ • /medicine/*   │    │ • 移動平台       │
│ • 藥物庫存       │    │ • /robot/*      │    │ • 機械手臂       │
│ • 用戶資料       │    │ • /inventory/*  │    │ • 感測器         │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 🛠️ 建置和安裝

### 1. 創建 ROS2 Workspace
```bash
# 創建工作空間
mkdir -p ~/hospital_ws/src
cd ~/hospital_ws/src

# 創建 package
ros2 pkg create --build-type ament_python hospital_medicine_ros2 \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

### 2. 建置 Package
```bash
cd ~/hospital_ws
colcon build --packages-select hospital_medicine_ros2
source install/setup.bash
```

### 3. 運行節點
```bash
# 啟動整個系統
ros2 launch hospital_medicine_ros2 hospital_system_launch.py

# 或分別啟動節點
ros2 run hospital_medicine_ros2 medicine_dispenser_node
ros2 run hospital_medicine_ros2 robot_controller_node
ros2 run hospital_medicine_ros2 inventory_manager_node
```

## 🔍 調試和監控

### ROS2 工具命令
```bash
# 查看可用節點
ros2 node list

# 查看話題
ros2 topic list

# 監聽話題
ros2 topic echo /medicine/orders

# 查看服務
ros2 service list

# 調用服務
ros2 service call /medicine/process_prescription hospital_medicine_ros2/srv/ProcessPrescription

# 查看節點資訊
ros2 node info /medicine_dispenser_node

# 視覺化節點圖
ros2 run rqt_graph rqt_graph
```

## 🎯 與現有系統的整合

### 當前模擬系統轉換為真實 ROS2

1. **保留現有 FastAPI 介面**
2. **創建 ROS2 Bridge**
3. **實現真實 ROS2 節點**
4. **配置硬體接口**

### 轉換步驟

```python
# 在 simple_server_clean.py 中添加 ROS2 Bridge
class ROS2Bridge:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('hospital_bridge')
        
        # 創建發布者
        self.order_pub = self.node.create_publisher(
            MedicineOrder, 
            '/medicine/orders', 
            10
        )
        
    def send_prescription_to_ros2(self, prescription_data):
        """發送處方籤到 ROS2 系統"""
        order_msg = MedicineOrder()
        order_msg.order_id = f"ORDER_{prescription_data['id']}"
        order_msg.prescription_id = str(prescription_data['id'])
        # ... 填充其他欄位
        
        self.order_pub.publish(order_msg)
```

這樣就能將現有的醫院管理系統與真實的 ROS2 機器人系統無縫整合！
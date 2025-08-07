# ROS2 服務接口使用指南

## 🎯 概述

這份指南將教您如何使用 ROS2 服務與醫院藥物管理系統進行通信，包括：
- **獲取訂單** - 從系統獲取新的藥物訂單
- **查詢藥物詳細資訊** - 獲取藥物的詳細描述
- **完成訂單** - 通知系統訂單已完成

## 📋 服務架構

```
您的 ROS2 系統 ↔️ 醫院服務節點 ↔️ Web 系統
                 (ros2_services_interface.py)
```

## 🚀 快速開始

### 1. 啟動 Web 系統
```bash
cd /workspace/user_interface
python3 start_complete_system.py
```

### 2. 啟動 ROS2 服務節點
```bash
# 新開一個終端
cd /workspace/user_interface
python3 ros2_services_interface.py
```

### 3. 啟動您的 ROS2 客戶端
```bash
# 新開一個終端
cd /workspace/user_interface
python3 ros2_client_example.py
```

## 📡 可用服務和 Topic

### 🔧 服務 (Services)
| 服務名稱 | 類型 | 功能 |
|---------|------|------|
| `/hospital/get_order` | `std_srvs/srv/Empty` | 獲取新訂單 |
| `/hospital/complete_order` | `std_srvs/srv/Empty` | 完成當前訂單 |

### 📢 Topic
| Topic 名稱 | 類型 | 方向 | 功能 |
|-----------|------|------|------|
| `/hospital/order_data` | `std_msgs/msg/String` | 接收 | 訂單數據 |
| `/hospital/medicine_data` | `std_msgs/msg/String` | 接收 | 藥物詳細資訊 |
| `/hospital/medicine_request` | `std_msgs/msg/String` | 發送 | 藥物查詢請求 |
| `/hospital/status` | `std_msgs/msg/String` | 接收 | 系統狀態 |

## 📄 數據格式

### 訂單數據格式
```json
{
  "order_id": "000001",
  "prescription_id": 1,
  "patient_name": "張三",
  "medicines": [
    {
      "name": "阿斯匹靈",
      "amount": 10,
      "locate": [2, 3],
      "prompt": "tablet"
    },
    {
      "name": "維他命C",
      "amount": 5,
      "locate": [1, 5],
      "prompt": "capsule"
    }
  ]
}
```

### 藥物詳細資訊格式
```json
{
  "name": "阿斯匹靈",
  "description": "解熱鎮痛藥，用於緩解頭痛、發燒等症狀",
  "found": true
}
```

## 🤖 您的 ROS2 實作

### 基本結構
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
import json

class YourHospitalRobot(Node):
    def __init__(self):
        super().__init__('your_robot')
        
        # 創建服務客戶端
        self.get_order_client = self.create_client(Empty, 'hospital/get_order')
        self.complete_order_client = self.create_client(Empty, 'hospital/complete_order')
        
        # 創建 Topic 發布者和訂閱者
        self.medicine_request_pub = self.create_publisher(
            String, 'hospital/medicine_request', 10)
        
        self.order_sub = self.create_subscription(
            String, 'hospital/order_data', self.order_callback, 10)
        self.medicine_sub = self.create_subscription(
            String, 'hospital/medicine_data', self.medicine_callback, 10)
        
        self.current_order = None
```

### 1. 獲取新訂單
```python
def get_new_order(self):
    """獲取新訂單"""
    request = Empty.Request()
    future = self.get_order_client.call_async(request)
    
    # 等待服務回應
    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
    
    if future.result() is not None:
        # 訂單數據會通過 order_callback 接收
        self.get_logger().info("✅ 訂單請求已發送")
        return True
    else:
        self.get_logger().error("❌ 獲取訂單失敗")
        return False

def order_callback(self, msg):
    """處理接收到的訂單"""
    try:
        order_data = json.loads(msg.data)
        self.current_order = order_data
        
        # 開始處理訂單
        self.process_order(order_data)
        
    except Exception as e:
        self.get_logger().error(f"處理訂單錯誤: {e}")
```

### 2. 查詢藥物詳細資訊
```python
def query_medicine_detail(self, medicine_name):
    """查詢藥物詳細資訊"""
    msg = String()
    msg.data = medicine_name
    self.medicine_request_pub.publish(msg)

def medicine_callback(self, msg):
    """處理藥物詳細資訊回應"""
    try:
        medicine_data = json.loads(msg.data)
        
        if medicine_data.get('found'):
            name = medicine_data['name']
            description = medicine_data['description']
            self.get_logger().info(f"藥物 {name}: {description}")
        else:
            self.get_logger().warn(f"未找到藥物: {medicine_data.get('name')}")
            
    except Exception as e:
        self.get_logger().error(f"處理藥物資訊錯誤: {e}")
```

### 3. 完成訂單
```python
def complete_order(self):
    """完成當前訂單"""
    if not self.current_order:
        self.get_logger().warn("沒有正在處理的訂單")
        return False
        
    request = Empty.Request()
    future = self.complete_order_client.call_async(request)
    
    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
    
    if future.result() is not None:
        self.get_logger().info("✅ 訂單完成通知已發送")
        self.current_order = None
        return True
    else:
        self.get_logger().error("❌ 完成訂單失敗")
        return False
```

### 4. 完整的訂單處理流程
```python
def process_order(self, order_data):
    """處理訂單的主要邏輯"""
    order_id = order_data['order_id']
    medicines = order_data['medicines']
    
    self.get_logger().info(f"開始處理訂單: {order_id}")
    
    for medicine in medicines:
        name = medicine['name']
        amount = medicine['amount']
        locate = medicine['locate']  # [row, col]
        prompt = medicine['prompt']  # 藥物類型
        
        # 1. 查詢藥物詳細資訊（如果需要）
        self.query_medicine_detail(name)
        
        # 2. 導航到藥物位置
        self.navigate_to_location(locate)
        
        # 3. 根據類型抓取藥物
        self.pick_medicine_by_type(name, amount, prompt)
        
        # 4. 運送藥物
        self.deliver_medicine()
    
    # 5. 完成訂單
    self.complete_order()

def navigate_to_location(self, locate):
    """導航到指定位置"""
    row, col = locate
    self.get_logger().info(f"導航到位置 [{row}, {col}]")
    # 您的導航邏輯

def pick_medicine_by_type(self, name, amount, prompt):
    """根據類型抓取藥物"""
    self.get_logger().info(f"抓取 {amount} 個 {name} ({prompt})")
    
    if prompt == 'tablet':
        # 片劑抓取邏輯
        pass
    elif prompt == 'capsule':
        # 膠囊抓取邏輯  
        pass
    elif prompt == 'white_circle_box':
        # 盒裝藥物抓取邏輯
        pass
    
    # 您的抓取邏輯

def deliver_medicine(self):
    """運送藥物到分配點"""
    self.get_logger().info("運送藥物中...")
    # 您的運送邏輯
```

## 🔄 完整工作流程

```
1. 您的系統調用 get_order 服務
   ↓
2. 系統通過 order_data topic 發送訂單
   ↓
3. 您的系統處理訂單中的每個藥物:
   - 查詢藥物詳細資訊 (可選)
   - 導航到位置
   - 抓取藥物
   - 運送藥物
   ↓
4. 處理完成後調用 complete_order 服務
   ↓
5. 系統更新訂單狀態，準備處理下一個
```

## 📋 命令行測試

### 測試服務
```bash
# 獲取新訂單
ros2 service call /hospital/get_order std_srvs/srv/Empty

# 完成訂單
ros2 service call /hospital/complete_order std_srvs/srv/Empty
```

### 測試 Topic
```bash
# 查詢藥物詳細資訊
ros2 topic pub /hospital/medicine_request std_msgs/msg/String 'data: "阿斯匹靈"'

# 監聽訂單數據
ros2 topic echo /hospital/order_data

# 監聽藥物詳細資訊
ros2 topic echo /hospital/medicine_data

# 監聽系統狀態
ros2 topic echo /hospital/status
```

## 🛠️ 自定義和擴展

### 修改藥物位置算法
編輯 `ros2_services_interface.py` 中的 `_get_medicine_location` 方法

### 修改藥物類型識別
編輯 `ros2_services_interface.py` 中的 `_get_medicine_prompt` 方法

### 添加更多狀態訊息
在服務節點中添加更多的狀態發布

## ❗ 重要注意事項

1. **一次一個訂單**: 系統確保一次只處理一個訂單
2. **必須完成**: 處理完訂單後**必須**調用 `complete_order`
3. **錯誤處理**: 如果發生錯誤，也要調用 `complete_order` 來重置狀態
4. **藥物描述**: 藥物詳細資訊只包含「藥物描述」欄位
5. **服務等待**: 確保服務可用後再開始調用

## 🎯 總結

使用這個 ROS2 服務接口，您可以：
- ✅ **自動獲取訂單** - 無需輪詢，主動推送
- ✅ **查詢藥物資訊** - 獲取詳細的藥物描述
- ✅ **狀態同步** - 自動與 Web 系統同步
- ✅ **簡單整合** - 使用標準 ROS2 服務和 Topic
- ✅ **一對一處理** - 確保訂單順序處理

**現在您可以開始將這個接口整合到您的 ROS2 系統中！** 🚀
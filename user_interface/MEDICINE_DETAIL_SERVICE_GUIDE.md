# 藥物詳細資料 ROS2 服務使用指南

## 🎯 概述

這是一個專用的 ROS2 服務，用於查詢藥物的詳細資訊。您只需要輸入藥物名稱，就可以獲得完整的 YAML 格式詳細資料。

## 🚀 快速開始

### 1. 啟動 Web 系統
```bash
cd /workspace/user_interface
python3 start_system_modes.py
# 選擇任一模式（建議選擇 3 - 僅 Web 系統）
```

### 2. 啟動藥物詳細資料服務
```bash
# 新開一個終端
cd /workspace/user_interface
python3 ros2_medicine_detail_service.py
```

### 3. 啟動客戶端（可選）
```bash
# 新開一個終端
cd /workspace/user_interface
python3 medicine_client_example.py
```

## 📡 服務接口

### Topic 接口
| Topic 名稱 | 類型 | 方向 | 功能 |
|-----------|------|------|------|
| `/medicine/detail_request` | `std_msgs/msg/String` | 發送 | 藥物名稱查詢請求 |
| `/medicine/detail_response` | `std_msgs/msg/String` | 接收 | YAML 格式詳細資料 |

### Service 接口
| 服務名稱 | 類型 | 功能 |
|---------|------|------|
| `/medicine/get_detail` | `std_srvs/srv/Empty` | 同步查詢（需要先設置名稱） |

## 📄 使用方式

### 方法 1: 命令行使用

#### 查詢藥物詳細資料
```bash
# 發送查詢請求
ros2 topic pub /medicine/detail_request std_msgs/msg/String 'data: "阿斯匹靈"'

# 接收詳細資料
ros2 topic echo /medicine/detail_response
```

#### 使用服務方式
```bash
# 先設置要查詢的藥物
ros2 topic pub /medicine/detail_request std_msgs/msg/String 'data: "維他命C"'

# 調用服務
ros2 service call /medicine/get_detail std_srvs/srv/Empty
```

### 方法 2: Python 程式使用

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

class MedicineQueryClient(Node):
    def __init__(self):
        super().__init__('medicine_query_client')
        
        # 創建發布者和訂閱者
        self.publisher = self.create_publisher(String, 'medicine/detail_request', 10)
        self.subscriber = self.create_subscription(
            String, 'medicine/detail_response', self.response_callback, 10)
        
    def query_medicine(self, medicine_name):
        """查詢藥物詳細資料"""
        msg = String()
        msg.data = medicine_name
        self.publisher.publish(msg)
        print(f"🔍 查詢藥物: {medicine_name}")
        
    def response_callback(self, msg):
        """處理查詢回應"""
        try:
            # 解析 YAML 格式
            data = yaml.safe_load(msg.data)
            
            if data.get('found'):
                print(f"✅ 找到藥物: {data['name']}")
                print(f"📝 描述: {data.get('description', 'N/A')}")
                print(f"📦 庫存: {data.get('stock_quantity', 0)}")
            else:
                print(f"❌ 未找到藥物: {data.get('name', 'Unknown')}")
                
        except Exception as e:
            print(f"❌ 處理回應錯誤: {e}")

def main():
    rclpy.init()
    client = MedicineQueryClient()
    
    # 查詢範例
    client.query_medicine("阿斯匹靈")
    
    # 讓程式運行一段時間以接收回應
    rclpy.spin_once(client, timeout_sec=5.0)
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 方法 3: 互動式客戶端

```bash
python3 medicine_client_example.py
```

選擇模式：
- **互動模式**: 手動輸入藥物名稱查詢
- **測試模式**: 自動測試常用藥物

## 📄 回應格式

### 成功查詢的 YAML 格式
```yaml
name: 阿斯匹靈
found: true
description: "解熱鎮痛藥，用於緩解頭痛、發燒等症狀"
category: "解熱鎮痛藥"
unit_dose: "500mg"
stock_quantity: 100
query_name: "阿斯匹靈"
```

### 查詢失敗的 YAML 格式
```yaml
name: 不存在的藥物
found: false
error: "藥物未找到"
description: ""
```

## 🔍 查詢功能

### 精確匹配
```bash
ros2 topic pub /medicine/detail_request std_msgs/msg/String 'data: "阿斯匹靈"'
```

### 模糊匹配
支援部分匹配，例如：
```bash
# 查詢包含 "維他命" 的藥物
ros2 topic pub /medicine/detail_request std_msgs/msg/String 'data: "維他命"'

# 查詢包含 "感冒" 的藥物
ros2 topic pub /medicine/detail_request std_msgs/msg/String 'data: "感冒"'
```

## 📋 常用範例

### 1. 批量查詢
```python
medicines = ["阿斯匹靈", "維他命C", "感冒藥", "止痛藥"]

for medicine in medicines:
    client.query_medicine(medicine)
    time.sleep(1)  # 等待回應
```

### 2. 命令行快速查詢
```bash
python3 medicine_client_example.py "阿斯匹靈"
```

### 3. 檢查藥物庫存
```python
def check_stock(self, msg):
    data = yaml.safe_load(msg.data)
    if data.get('found'):
        stock = data.get('stock_quantity', 0)
        if stock > 10:
            print(f"✅ {data['name']} 庫存充足: {stock}")
        else:
            print(f"⚠️ {data['name']} 庫存不足: {stock}")
```

## 🔧 高級功能

### 1. 自動補全建議
服務會自動列出所有可用的藥物：

```bash
# 啟動服務時會顯示可用藥物列表
python3 ros2_medicine_detail_service.py
```

### 2. 錯誤處理
- 自動處理網路連接錯誤
- 支援超時重試
- 提供詳細的錯誤訊息

### 3. 效能優化
- 支援併發查詢
- 快取機制（最後一次查詢結果）
- 智慧匹配算法

## 🛠️ 故障排除

### 問題 1: 服務無回應
```bash
# 檢查服務是否運行
ros2 node list | grep medicine_detail_service

# 檢查 topic
ros2 topic list | grep medicine
```

### 問題 2: Web API 連接失敗
```bash
# 確認 Web 服務器運行中
curl http://localhost:8001/api/system/status
```

### 問題 3: 找不到藥物
- 檢查藥物名稱拼寫
- 使用模糊匹配
- 查看可用藥物列表

## 🎯 整合建議

### 與您的 ROS2 系統整合
```python
class YourRobotNode(Node):
    def __init__(self):
        super().__init__('your_robot')
        
        # 創建藥物查詢客戶端
        self.medicine_publisher = self.create_publisher(
            String, 'medicine/detail_request', 10)
        self.medicine_subscriber = self.create_subscription(
            String, 'medicine/detail_response', self.medicine_info_callback, 10)
        
    def before_picking_medicine(self, medicine_name):
        """抓取藥物前先查詢詳細資訊"""
        # 查詢藥物詳細資訊
        msg = String()
        msg.data = medicine_name
        self.medicine_publisher.publish(msg)
        
        # 等待回應以決定抓取策略
        
    def medicine_info_callback(self, msg):
        """根據藥物詳細資訊調整抓取策略"""
        data = yaml.safe_load(msg.data)
        
        if data.get('found'):
            # 根據藥物類型調整抓取參數
            category = data.get('category', '')
            if '液體' in category:
                self.use_careful_grip()
            elif '易碎' in category:
                self.use_gentle_grip()
```

## 🚀 總結

這個藥物詳細資料服務提供了：

- ✅ **簡單易用** - 只需輸入藥物名稱
- ✅ **YAML 格式** - 標準化輸出格式
- ✅ **模糊匹配** - 智慧查詢功能
- ✅ **多種接口** - Topic 和 Service 支援
- ✅ **錯誤處理** - 完善的異常處理
- ✅ **即插即用** - 易於整合到現有系統

**現在您可以輕鬆查詢任何藥物的詳細資訊！** 🎊
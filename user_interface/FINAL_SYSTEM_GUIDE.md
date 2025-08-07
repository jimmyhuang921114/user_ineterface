# 醫院藥物管理系統 - 最終版使用說明

## 🎯 系統概述

這是完全乾淨的最終版本，具備以下特點：

- ✅ **完全乾淨** - 無任何測試資料
- ✅ **不自動模擬** - ROS2 接口模式，等待您的整合
- ✅ **保留所有接口** - 供您的 ROS2 系統使用
- ✅ **功能完整** - 所有 API 和網頁界面正常工作

## 🚀 快速啟動

### 方法 1: 使用啟動腳本
```bash
cd /workspace/user_interface
python3 start_final_server.py
```

### 方法 2: 手動啟動
```bash
cd /workspace/user_interface

# 初始化數據庫
python3 database_final.py

# 啟動服務器
python3 simple_server_final.py
```

## 🌐 網頁界面

| 功能 | 網址 | 描述 |
|------|------|------|
| 整合管理 | http://localhost:8001/integrated_medicine_management.html | 藥物管理主界面 |
| 醫生工作台 | http://localhost:8001/doctor.html | 醫生開立處方籤 |
| 處方籤管理 | http://localhost:8001/Prescription.html | 查看和管理處方籤 |
| ROS2 客戶端 | http://localhost:8001/ros2_client.html | ROS2 藥物查詢和訂單 |
| API 文檔 | http://localhost:8001/docs | FastAPI 自動文檔 |

## 🔌 ROS2 接口

### Python 模組使用

```python
from ros2_interface_final import (
    ros2_query_medicine,
    ros2_process_order,
    ros2_complete_order,
    ros2_get_current_order,
    ros2_get_status
)

# 1. 查詢藥物詳細資訊（您指定的 YAML 格式）
medicine_detail = ros2_query_medicine("Antipsychotics")
print(medicine_detail)

# 2. 處理訂單（一次處理一個）
order_result = ros2_process_order("000001", [
    {"name": "Antipsychotics", "quantity": 87},
    {"name": "測試藥物B", "quantity": 212}
])
print(order_result)

# 3. 獲取當前待處理訂單（供您的 ROS2 系統查詢）
current_order = ros2_get_current_order()
if current_order:
    print(f"需要處理的訂單: {current_order['order_id']}")

# 4. 完成訂單（您的 ROS2 系統完成後調用）
completion_result = ros2_complete_order("000001")
print(completion_result)

# 5. 獲取接口狀態
status = ros2_get_status()
print(status)
```

### API 接口使用

```python
import requests

# 查詢藥物詳細資訊
response = requests.post('http://localhost:8001/api/ros2/query-medicine-detail', 
                        json={"medicine_name": "Antipsychotics"})
if response.status_code == 200:
    result = response.json()
    print(result['detail'])  # YAML 格式

# 發送訂單
order_data = {
    "order_id": "000001",
    "medicines": [
        {"name": "Antipsychotics", "quantity": 87}
    ]
}
response = requests.post('http://localhost:8001/api/ros2/process-order', 
                        json=order_data)

# 檢查服務狀態
response = requests.get('http://localhost:8001/api/ros2/service-status')
status = response.json()
print(f"服務運行: {status['service_running']}")
```

## 📋 數據格式

### 藥物詳細查詢 - 輸出格式

**輸入**: 藥物名稱（如 "Antipsychotics"）

**輸出**: 您指定的 YAML 格式
```yaml
name: Antipsychotics
constant:
  名稱: "Antipsychotics"
  成分: "Nirmatrelvir"
  分類: "3CL proteinase inhibitors"
  劑量: "1 special pill"
  服用方式: "口服 (Oral use)"
  有效日期: "2027/11/09"
  適應症: "適用於12歲以上、體重至少40公斤，於5天內確診輕度至中度COVID-19，且具嚴重疾病風險因子的成人與兒童"
  可能副作用: "味覺異常、腹瀉、噁心、嘔吐、頭痛"
  條碼編號: "TEST-367842394"
  外觀:
    顏色: "藍色條紋 白色外觀"
    形狀: "圓扁形"
```

### 訂單處理 - 輸出格式

**輸入**: 
```json
{
  "order_id": "000001",
  "medicines": [
    {"name": "Antipsychotics", "quantity": 87}
  ]
}
```

**輸出**: 您指定的 YAML 格式
```yaml
order_id: "000001"
medicine:
  - name: Antipsychotics
    amount: 87
    locate: [9, 6]
    prompt: white_circle_box
```

## 🔧 整合您的 ROS2 系統

### 基本整合流程

1. **監聽訂單**: 定期查詢 `ros2_get_current_order()` 檢查是否有新訂單
2. **處理訂單**: 根據訂單資訊進行藥物準備
3. **完成訂單**: 處理完成後調用 `ros2_complete_order(order_id)`

### 範例整合代碼

```python
import time
from ros2_interface_final import ros2_get_current_order, ros2_complete_order

def ros2_integration_loop():
    """ROS2 整合主循環"""
    print("🤖 ROS2 整合程序啟動")
    
    while True:
        # 檢查是否有待處理訂單
        current_order = ros2_get_current_order()
        
        if current_order:
            order_id = current_order['order_id']
            medicines = current_order['medicines']
            
            print(f"📦 處理訂單: {order_id}")
            
            # 這裡整合您的 ROS2 邏輯
            for medicine in medicines:
                name = medicine['name']
                amount = medicine['amount']
                locate = medicine['locate']
                prompt = medicine['prompt']
                
                print(f"   處理藥物: {name} x{amount} 位置:{locate} 類型:{prompt}")
                
                # TODO: 在這裡調用您的 ROS2 節點
                # 例如: ros_node.move_to_location(locate)
                # 例如: ros_node.pick_medicine(name, amount, prompt)
            
            # 模擬處理時間
            time.sleep(10)
            
            # 完成訂單
            result = ros2_complete_order(order_id)
            print(f"   ✅ 訂單完成: {result['message']}")
        
        else:
            print("💤 等待新訂單...")
            time.sleep(5)

if __name__ == "__main__":
    ros2_integration_loop()
```

## 🛠️ 自定義配置

### 修改藥物位置算法

編輯 `ros2_interface_final.py`:

```python
def _get_medicine_location(self, medicine_name: str) -> List[int]:
    """自定義藥物位置算法"""
    # 連接到您的庫存管理系統
    # 例如: return warehouse_system.get_location(medicine_name)
    
    # 目前使用簡單哈希算法
    hash_value = hash(medicine_name) % 100
    row = (hash_value // 10) + 1
    col = (hash_value % 10) + 1
    return [row, col]
```

### 修改藥物類型識別

```python
def _get_medicine_prompt(self, medicine_name: str) -> str:
    """自定義藥物類型識別"""
    # 連接到您的藥物類型數據庫
    # 例如: return medicine_db.get_type(medicine_name)
    
    name_lower = medicine_name.lower()
    if 'tablet' in name_lower:
        return 'tablet'
    elif 'capsule' in name_lower:
        return 'capsule'
    # 添加更多類型...
    else:
        return 'white_circle_box'
```

## 📊 API 端點總覽

### 系統 API
| 端點 | 方法 | 功能 |
|------|------|------|
| `/api/system/status` | GET | 獲取系統狀態 |

### 藥物 API
| 端點 | 方法 | 功能 |
|------|------|------|
| `/api/medicine/basic` | GET | 獲取基本藥物列表 |
| `/api/medicine/detailed` | GET | 獲取詳細藥物列表 |
| `/api/medicine/unified` | POST | 創建統一藥物 |

### 處方籤 API
| 端點 | 方法 | 功能 |
|------|------|------|
| `/api/prescription/` | GET | 獲取處方籤列表 |
| `/api/prescription/` | POST | 創建新處方籤 |

### ROS2 API
| 端點 | 方法 | 功能 |
|------|------|------|
| `/api/ros2/status` | GET | ROS2 狀態 |
| `/api/ros2/service/basic-medicine` | POST | 基本藥物服務 |
| `/api/ros2/service/detailed-medicine` | POST | 詳細藥物服務 |
| `/api/ros2/query-medicine-detail` | POST | 查詢藥物詳細資訊 |
| `/api/ros2/process-order` | POST | 處理訂單 |
| `/api/ros2/service-status` | GET | 服務狀態 |
| `/api/ros2/complete-order` | POST | 完成訂單 |

## 🎯 與其他版本的差異

| 版本 | 測試資料 | ROS2 模擬 | 目的 |
|------|----------|-----------|------|
| `simple_server.py` | ✅ 有 | ✅ 自動模擬 | 開發測試 |
| `simple_server_clean.py` | ❌ 無 | ✅ 自動模擬 | 清潔測試 |
| `simple_server_production.py` | ❌ 無 | ✅ 自動模擬 | 生產環境（帶模擬） |
| `simple_server_final.py` | ❌ 無 | ❌ 僅接口 | **最終版本（供整合）** |

## 🔒 安全注意事項

1. **生產環境部署**: 建議使用 HTTPS 和身份驗證
2. **數據庫安全**: 考慮使用 PostgreSQL 並配置適當權限
3. **API 限流**: 在生產環境中添加 API 限流機制
4. **日誌管理**: 設定適當的日誌輪轉和監控

## 📞 技術支援

如果您在整合過程中遇到問題，請檢查：

1. **系統狀態**: `GET /api/system/status`
2. **ROS2 接口狀態**: `GET /api/ros2/service-status`
3. **數據庫連接**: 確保 `hospital_medicine_final.db` 可訪問
4. **端口占用**: 確保 8001 端口未被其他程序使用

## 🎉 總結

這個最終版本為您提供了：

- ✅ **完全乾淨的數據庫** - 無測試資料干擾
- ✅ **完整的 ROS2 接口** - 等待您的整合
- ✅ **所有必要的 API** - 功能完整
- ✅ **友好的網頁界面** - 易於操作
- ✅ **詳細的文檔** - 快速上手

**系統已準備就緒，等待您整合真實的 ROS2 功能！**
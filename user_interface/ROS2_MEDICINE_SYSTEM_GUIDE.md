# ROS2 藥物查詢與訂單系統使用說明

## 🎯 系統概述

這是一個完整的 ROS2 藥物查詢與訂單處理系統，提供：

1. **詳細藥物查詢** - 根據藥物名稱返回詳細資訊（您指定的 YAML 格式）
2. **訂單處理** - 一次處理一個訂單，包含位置和提示符資訊
3. **Web 客戶端** - 友好的網頁界面進行查詢和訂單管理
4. **狀態監控** - 即時追蹤系統和訂單狀態

## 🚀 快速開始

### 1. 啟動系統

```bash
# 確保在正確目錄
cd /workspace/user_interface

# 啟動正式版服務器
python3 simple_server_production.py
```

### 2. 訪問網頁界面

打開瀏覽器訪問：http://localhost:8001/ros2_client.html

## 💊 藥物詳細查詢功能

### 輸入藥物名稱，獲取詳細資訊

**輸入範例：**
```
Antipsychotics
```

**輸出格式（您指定的 YAML 格式）：**
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

### API 使用方式

```python
import requests

# 查詢藥物詳細資訊
response = requests.post('http://localhost:8001/api/ros2/query-medicine-detail', 
                        json={"medicine_name": "Antipsychotics"})

if response.status_code == 200:
    result = response.json()
    print(result['detail'])  # YAML 格式的詳細資訊
```

## 📦 訂單處理功能

### 一次處理一個訂單

**輸入範例：**
```json
{
  "order_id": "000001",
  "medicines": [
    {
      "name": "Antipsychotics",
      "quantity": 87
    },
    {
      "name": "測試藥物B",
      "quantity": 212
    }
  ]
}
```

**輸出格式（您指定的格式）：**
```yaml
order_id: "000001"
medicine:
  - name: Antipsychotics
    amount: 87
    locate: [1, 1]
    prompt: white_circle_box

  - name: 測試藥物B
    amount: 212
    locate: [1, 3]
    prompt: tablet
```

### API 使用方式

```python
import requests

# 發送訂單
order_data = {
    "order_id": "000001",
    "medicines": [
        {"name": "Antipsychotics", "quantity": 87},
        {"name": "測試藥物B", "quantity": 212}
    ]
}

response = requests.post('http://localhost:8001/api/ros2/process-order', 
                        json=order_data)

if response.status_code == 200:
    result = response.json()
    print(result['message'])  # 包含 YAML 格式的訂單詳情
```

## 🔧 ROS2 服務接口

### ROS2MedicineQueryService 類

```python
from ros2_medicine_query_service import get_medicine_query_service

# 獲取服務實例
service = get_medicine_query_service()

# 查詢藥物詳細資訊
detail = service.query_medicine_detail("Antipsychotics")
print(detail)

# 處理訂單
order_result = service.process_order({
    "order_id": "TEST_001",
    "medicines": [{"name": "Antipsychotics", "quantity": 87}]
})
print(order_result)

# 獲取服務狀態
status = service.get_status()
print(status)
```

## 🌐 Web 客戶端功能

### 藥物查詢區域
- 輸入藥物名稱
- 點擊「查詢詳細資訊」
- 查看 YAML 格式結果

### 訂單管理區域
- 設定訂單編號
- 新增藥物項目（名稱 + 數量）
- 發送訂單給 ROS2 主控制器
- 查看處理結果

### 系統狀態監控
- 即時服務狀態
- 當前訂單處理狀態
- 自動更新功能
- 詳細日誌記錄

## ⚙️ 系統特性

### 1. 一次處理一個訂單
```python
# 如果正在處理其他訂單，新訂單會被拒絕
if self.current_order is not None:
    return "錯誤: 目前正在處理其他訂單，請稍後再試"
```

### 2. 智能位置分配
```python
def _get_medicine_location(self, medicine_name: str) -> List[int]:
    # 使用哈希算法分配位置
    hash_value = hash(medicine_name) % 100
    row = (hash_value // 10) + 1
    col = (hash_value % 10) + 1
    return [row, col]
```

### 3. 藥物類型識別
```python
def _get_medicine_prompt(self, medicine_name: str) -> str:
    name_lower = medicine_name.lower()
    if 'tablet' in name_lower or '錠' in medicine_name:
        return 'tablet'
    elif 'capsule' in name_lower or '膠囊' in medicine_name:
        return 'capsule'
    # ... 更多類型
```

### 4. 自動狀態更新
- 訂單處理完成後自動更新處方籤狀態
- 通知 FastAPI 後端更新數據庫
- 網頁界面即時顯示狀態變化

## 📋 完整使用流程

### 1. 系統準備
```bash
# 1. 啟動服務器
python3 simple_server_production.py

# 2. 打開網頁客戶端
# 瀏覽器訪問: http://localhost:8001/ros2_client.html
```

### 2. 查詢藥物
```
1. 在「藥物詳細查詢」區域輸入藥物名稱
2. 點擊「查詢詳細資訊」
3. 查看 YAML 格式的詳細資訊
```

### 3. 發送訂單
```
1. 在「訂單管理」區域設定訂單編號
2. 新增所需藥物及數量
3. 點擊「發送訂單」
4. 系統會返回處理後的訂單（包含位置和提示符）
5. 等待處理完成（約15秒）
```

### 4. 監控狀態
```
1. 查看「系統狀態監控」區域
2. 啟用自動更新追蹤處理進度
3. 查看詳細日誌了解系統運行情況
```

## 🔌 API 端點總覽

| 端點 | 方法 | 功能 |
|------|------|------|
| `/api/ros2/query-medicine-detail` | POST | 查詢藥物詳細資訊 |
| `/api/ros2/process-order` | POST | 處理訂單請求 |
| `/api/ros2/service-status` | GET | 獲取服務狀態 |
| `/ros2_client.html` | GET | Web 客戶端界面 |

## 🛠️ 自定義配置

### 修改藥物位置算法
```python
def _get_medicine_location(self, medicine_name: str) -> List[int]:
    # 您可以在這裡實現自己的位置分配邏輯
    # 例如：連接到實際的庫存管理系統
    pass
```

### 修改處理時間
```python
def _process_order_async(self, order_id: str):
    processing_time = 15  # 修改處理時間（秒）
```

### 添加新的藥物類型
```python
def _get_medicine_prompt(self, medicine_name: str) -> str:
    # 添加新的藥物類型識別邏輯
    if 'your_new_type' in name_lower:
        return 'your_prompt'
```

## 🎯 實際部署建議

1. **生產環境部署**
   - 使用 HTTPS
   - 配置防火牆
   - 設定日誌輪轉

2. **ROS2 整合**
   - 替換模擬服務為真實 ROS2 節點
   - 配置 ROS2 網路設定
   - 實現真實的硬體控制

3. **數據庫優化**
   - 使用 PostgreSQL 或 MySQL
   - 添加索引優化查詢
   - 實現數據備份

4. **監控與維護**
   - 添加系統監控
   - 實現錯誤告警
   - 定期維護更新

## 🎉 總結

這個系統提供了完整的 ROS2 藥物查詢與訂單處理功能，包括：

✅ **藥物詳細查詢** - 您指定的 YAML 格式輸出  
✅ **訂單處理** - 一次一個，包含位置和提示符  
✅ **Web 界面** - 友好的用戶界面  
✅ **狀態監控** - 即時追蹤和日誌  
✅ **API 接口** - 完整的程式化訪問  

系統已準備就緒，可以立即使用或進一步客製化以滿足您的特定需求！
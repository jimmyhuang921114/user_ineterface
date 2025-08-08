# 完整醫院管理系統 - 使用指南

## 🏥 系統概述

這是一個完整的醫院藥物管理系統，包含基本和詳細藥物信息管理、處方籤狀態追蹤、以及完整的 ROS2 整合功能。

## 🎯 核心功能

### 📊 **藥物管理**
- **基本資訊**: 位置、名稱、提示詞、信心值
- **詳細資訊**: 完整藥物內容描述
- **庫存管理**: 實時庫存追蹤

### 👨‍⚕️ **醫師功能**
- **拉取現有藥物**: 從資料庫選擇藥物
- **配合數量**: 設定處方籤藥物數量
- **建立處方籤**: 自動加入處理佇列

### 📋 **處方籤管理**
- **狀態顯示**: pending, processing, completed, failed
- **接收和回傳狀態**: 完整的狀態管理
- **自動更新**: 即時狀態同步

### 🤖 **ROS2 整合**
- **一次處理一個訂單**: 單一訂單處理機制
- **自動傳最舊訂單**: 按時間順序處理
- **完成後回傳**: 自動狀態更新

## 🚀 快速啟動

### 1. 系統啟動
```bash
cd user_interface
python3 complete_hospital_system.py
```

### 2. 訪問界面
- **藥物管理**: http://localhost:8001/medicine.html
- **醫生工作台**: http://localhost:8001/doctor.html
- **處方籤管理**: http://localhost:8001/prescription.html

## 📋 資料結構

### 🧬 **藥物基本資訊**
```python
{
    "id": 1,
    "name": "Aspirin",           # 藥物名稱
    "position": "A1",            # 位置
    "prompt": "pain_relief",     # 提示詞
    "confidence": 0.95,          # 信心值 (0-1)
    "amount": 100                # 庫存數量
}
```

### 📝 **藥物詳細資訊**
```python
{
    "medicine_id": 1,
    "content": "阿斯匹靈 - 非類固醇消炎藥，用於緩解疼痛、發燒和炎症..."  # 詳細內容
}
```

### 🗂️ **處方籤結構**
```python
{
    "id": 1,
    "patient_name": "張三",
    "status": "pending",         # pending/processing/completed/failed
    "created_at": "2025-01-01T10:00:00",
    "updated_at": "2025-01-01T10:05:00",
    "medicines": [
        {
            "id": 1,
            "name": "Aspirin",
            "amount": 10,
            "position": "A1",
            "prompt": "pain_relief"
        }
    ]
}
```

## 🤖 ROS2 API 整合

### 📡 **拉取訂單**
```bash
# 獲取下一個待處理訂單
curl http://localhost:8001/api/ros2/order/next

# 回應格式 (YAML)
{
    "order": {
        "order_id": "000001",
        "prescription_id": 1,
        "patient_name": "張三",
        "medicine": [
            {
                "name": "Aspirin",
                "amount": 10,
                "position": "A1",
                "prompt": "pain_relief_tablet",
                "confidence": 0.95
            }
        ]
    },
    "yaml": "order_id: '000001'\n..."
}
```

### 📬 **回報完成**
```bash
curl -X POST http://localhost:8001/api/ros2/order/complete \
  -H "Content-Type: application/json" \
  -d '{
    "order_id": "000001",
    "status": "success",
    "details": "訂單處理完成"
  }'
```

### 📊 **進度更新**
```bash
curl -X POST http://localhost:8001/api/ros2/order/progress \
  -H "Content-Type: application/json" \
  -d '{
    "order_id": "000001",
    "stage": "processing",
    "message": "正在處理第2個藥物"
  }'
```

### 💊 **藥物資訊查詢**

#### 基本資訊
```bash
curl http://localhost:8001/api/ros2/medicine/basic/Aspirin
```

#### 詳細資訊
```bash
curl http://localhost:8001/api/ros2/medicine/detailed/Aspirin
```

## 🔄 工作流程

### 📋 **完整處理流程**
```
1. 醫師開立處方籤 (doctor.html)
   ↓
2. 處方籤進入待處理佇列 (status: pending)
   ↓
3. ROS2 拉取最舊的待處理訂單 (/api/ros2/order/next)
   ↓
4. 處方籤狀態更新為處理中 (status: processing)
   ↓
5. ROS2 處理訂單並回報進度 (/api/ros2/order/progress)
   ↓
6. ROS2 完成處理並回報結果 (/api/ros2/order/complete)
   ↓
7. 處方籤狀態更新為完成 (status: completed)
   ↓
8. 系統準備處理下一個訂單
```

### 🔄 **訂單佇列機制**
- **一次一個**: 系統確保同時只處理一個訂單
- **等待完成**: 必須等當前訂單完成才會發送下一個
- **按時間排序**: 最舊的待處理處方籤優先處理
- **自動追蹤**: 系統自動維護處理佇列

## 📊 API 端點總覽

### 🔧 **藥物管理 API**
- `GET /api/medicine/basic` - 獲取基本藥物資訊
- `GET /api/medicine/detailed` - 獲取詳細藥物資訊
- `GET /api/medicine/detailed/{id}` - 獲取特定藥物詳細資訊
- `POST /api/medicine/` - 新增藥物
- `POST /api/medicine/{id}/detail` - 新增/更新藥物詳細資訊

### 📋 **處方籤管理 API**
- `GET /api/prescription/` - 獲取所有處方籤
- `POST /api/prescription/` - 建立新處方籤
- `PUT /api/prescription/{id}/status` - 更新處方籤狀態

### 🤖 **ROS2 整合 API**
- `GET /api/ros2/order/next` - 拉取下一個訂單
- `POST /api/ros2/order/complete` - 回報訂單完成
- `POST /api/ros2/order/progress` - 回報處理進度
- `GET /api/ros2/medicine/basic/{name}` - 基本藥物資訊查詢
- `GET /api/ros2/medicine/detailed/{name}` - 詳細藥物資訊查詢

### ⚙️ **系統狀態 API**
- `GET /api/system/status` - 系統狀態檢查

## 🎮 使用方式

### 💊 **新增藥物**
1. 進入藥物管理頁面
2. 填寫基本資訊 (名稱、位置、提示詞、信心值、庫存)
3. 點擊「新增藥物」
4. 選擇藥物並填寫詳細內容
5. 點擊「新增詳細資訊」

### 👨‍⚕️ **開立處方籤**
1. 進入醫生工作台
2. 輸入病患姓名
3. 點擊「載入可用藥物」
4. 選擇藥物並設定數量
5. 點擊「開立處方籤」
6. 處方籤自動加入處理佇列

### 📋 **管理處方籤**
1. 進入處方籤管理頁面
2. 查看統計數據 (待處理、處理中、已完成)
3. 監控處方籤狀態
4. 手動更新狀態 (如需要)

### 🤖 **ROS2 整合**
1. 系統自動維護訂單佇列
2. ROS2 呼叫 `/api/ros2/order/next` 拉取訂單
3. 處理過程中回報進度
4. 完成後呼叫 `/api/ros2/order/complete`
5. 系統自動更新處方籤狀態

## 🛠️ 技術特色

### 🔒 **資料庫設計**
- **Medicine**: 基本藥物資訊
- **MedicineDetail**: 詳細藥物內容
- **Prescription**: 處方籤主表
- **PrescriptionMedicine**: 處方籤藥物關聯表

### 🔄 **佇列管理**
- **全域變數追蹤**: `current_processing_order`, `order_queue`
- **自動佇列更新**: 當佇列為空時自動重新載入
- **狀態同步**: 處方籤狀態與訂單處理狀態同步

### 📡 **ROS2 整合**
- **HTTP 拉取模式**: ROS2 主動拉取訂單
- **YAML 格式輸出**: 支援 ROS2 慣用格式
- **按名稱查詢**: 支援藥物名稱模糊搜尋

### 🎨 **界面設計**
- **響應式設計**: 支援不同螢幕尺寸
- **即時更新**: 自動刷新處方籤狀態
- **視覺回饋**: 清楚的狀態指示和操作反饋

## 🎯 核心優勢

### ✅ **完整功能**
- **基本/詳細藥物管理** - 分離的資訊架構
- **處方籤全生命週期** - 從建立到完成的完整追蹤
- **ROS2 完整整合** - 支援訂單處理和藥物查詢

### ⚡ **高效處理**
- **一次一個訂單** - 避免資源衝突
- **自動佇列管理** - 無需手動干預
- **即時狀態同步** - 確保資料一致性

### 🔧 **易於維護**
- **單檔案系統** - 所有功能集中管理
- **清晰的 API 結構** - 標準 REST API 設計
- **完整的錯誤處理** - 穩定的系統運行

**🚀 這個完整系統滿足了您所有的需求，包含基本/詳細藥物管理、處方籤狀態追蹤、ROS2 整合、以及一次一個訂單的處理機制！**
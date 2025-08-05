# 🏥 醫院藥物管理系統 - 完整實時版本

## 📋 系統概述

這是一個功能完整的醫院藥物管理系統，具備實時監控、ROS2整合和YAML訂單生成功能。

### ✨ 核心特色

- ✅ **實時更新**: WebSocket即時通知藥物資料變更
- ✅ **ROS2整合**: 完整的ROS2 API支援和YAML訂單生成
- ✅ **雙重存儲**: 基本和詳細藥物資料分離存儲
- ✅ **訂單管理**: 完整的訂單接收、狀態更新和通知系統
- ✅ **實時監控**: 專門的監控頁面顯示所有系統活動
- ✅ **特定藥物查詢**: 即時搜尋和顯示特定藥物資訊

## 🚀 快速開始

### 1. 啟動系統

```bash
cd user_interface
python3 fixed_server_new.py
```

### 2. 訪問系統

- 🏠 **主頁**: http://localhost:8000/
- 💊 **整合藥物管理**: http://localhost:8000/medicine_integrated.html
- 🔄 **實時監控**: http://localhost:8000/real_time_monitor.html
- 👨‍⚕️ **醫生界面**: http://localhost:8000/doctor.html
- 📋 **處方籤**: http://localhost:8000/Prescription.html
- 🧪 **API測試**: http://localhost:8000/simple_test.html

## 🎯 回答您的問題

### 1. ❓ 如果更新藥物，它會即時嗎？

**答案：是的！** 系統具備完整的實時更新功能：

- 當您在整合表格中更新藥物資料時，所有連接的監控頁面會**立即**收到通知
- 如果其他用戶正在查看相同的藥物，他們的頁面會**自動更新**顯示新資料
- WebSocket連接確保所有變更都是**即時**的

**實時更新機制：**
```javascript
// 藥物更新時自動通知所有用戶
{
  "type": "medicine_basic_updated",
  "medicine": {...},
  "timestamp": "2025-08-05 15:45:30"
}
```

### 2. ❓ 如何查看特定藥物的資訊？

**多種查看方式：**

#### 方法1: 實時監控頁面 (推薦)
```
訪問: http://localhost:8000/real_time_monitor.html
在搜尋框輸入藥物名稱，即時顯示完整資訊
```

#### 方法2: API查詢
```bash
# 查看特定藥物的整合資訊
curl http://localhost:8000/api/medicine/integrated/藥物名稱

# 只查看基本資料
curl http://localhost:8000/api/medicine/basic/藥物名稱

# 只查看詳細資料
curl http://localhost:8000/api/medicine/detailed/藥物名稱
```

#### 方法3: ROS2 API查詢
```bash
# ROS2格式的整合資訊
curl http://localhost:8000/api/ros2/medicine/integrated/藥物名稱
```

### 3. ❓ 新訂單如何傳給我並回覆狀態？

**完整的訂單處理流程：**

#### 步驟1: 接收新訂單
系統支援兩種訂單接收方式：

**一般API訂單：**
```bash
curl -X POST http://localhost:8000/api/orders \
  -H "Content-Type: application/json" \
  -d '{
    "id": "000001",
    "order_data": {
      "medicine_1": {
        "amount": 87,
        "locate": [1, 1],
        "name": "Antipsychotics"
      }
    }
  }'
```

**ROS2訂單：**
```bash
curl -X POST http://localhost:8000/api/ros2/orders \
  -H "Content-Type: application/json" \
  -d '{
    "id": "000001",
    "order_data": {
      "medicine_1": {
        "amount": 87,
        "locate": [1, 1],
        "name": "Antipsychotics"
      }
    }
  }'
```

#### 步驟2: 即時通知
當收到新訂單時，系統會：
- 📨 **立即通知**所有監控頁面
- 🔔 **顯示訂單詳情**和狀態控制按鈕
- 📝 **記錄**到orders_data.json

**通知格式：**
```javascript
{
  "type": "ros2_order_request",
  "order": {...},
  "timestamp": "2025-08-05 15:45:30",
  "message": "收到來自ROS2的新訂單: 000001"
}
```

#### 步驟3: 回覆訂單狀態
您可以通過多種方式更新訂單狀態：

**方式1: 監控頁面點擊按鈕**
- 在實時監控頁面直接點擊狀態按鈕：
  - 🟡 待處理 (pending)
  - 🔵 處理中 (processing)  
  - 🟢 已完成 (completed)
  - 🔴 失敗 (failed)

**方式2: API更新**
```bash
curl -X POST http://localhost:8000/api/orders/000001/status \
  -H "Content-Type: application/json" \
  -d '{
    "order_id": "000001",
    "status": "completed",
    "message": "訂單處理完成",
    "timestamp": "2025-08-05T15:45:30"
  }'
```

#### 步驟4: 狀態回覆通知
狀態更新後，系統會：
- 📤 **即時通知**ROS2系統和所有用戶
- 🔄 **更新**訂單記錄
- 📋 **記錄**狀態變更歷史

## 📁 檔案結構

```
user_interface/
├── fixed_server_new.py              # 🆕 新的實時服務器
├── static/html/
│   ├── real_time_monitor.html       # 🆕 實時監控頁面
│   ├── medicine_integrated.html     # 整合藥物管理
│   ├── doctor.html                  # 醫生界面
│   ├── simple_test.html            # API測試
│   └── Prescription.html           # 處方籤管理
├── medicine_basic_data.json         # 基本藥物資料
├── medicine_detailed_data.json      # 詳細藥物資料
├── orders_data.json                 # 🆕 訂單資料
└── prescription_data.json           # 處方籤資料
```

## 🔗 完整API端點

### 藥物管理
- `GET/POST /api/medicine/basic` - 基本藥物資料
- `GET/POST /api/medicine/detailed` - 詳細藥物資料
- `GET /api/medicine/basic/{name}` - 特定基本資料
- `GET /api/medicine/detailed/{name}` - 特定詳細資料
- `GET /api/medicine/integrated/{name}` - 特定整合資料

### 訂單管理 🆕
- `GET/POST /api/orders` - 訂單管理
- `GET /api/orders/{order_id}` - 特定訂單
- `POST /api/orders/{order_id}/status` - 更新訂單狀態

### ROS2 API
- `GET /api/ros2/medicine/basic` - ROS2基本資料
- `GET /api/ros2/medicine/detailed` - ROS2詳細資料
- `GET /api/ros2/medicine/integrated/{name}` - ROS2整合資料
- `GET/POST /api/ros2/orders` - ROS2訂單管理
- `GET /api/ros2/prescription` - ROS2處方籤

### 實時通訊 🆕
- `WebSocket /ws` - 實時通知連接

## 🎯 使用場景示例

### 場景1: 藥物資料更新
```
1. 在整合表格頁面更新藥物資料
2. 實時監控頁面立即顯示更新通知
3. 正在查看該藥物的用戶自動看到新資料
```

### 場景2: ROS2訂單處理
```
1. ROS2系統發送新訂單到 /api/ros2/orders
2. 監控頁面立即顯示新訂單通知
3. 管理員點擊狀態按鈕更新處理狀態
4. ROS2系統收到狀態更新回覆
```

### 場景3: 特定藥物查詢
```
1. 在監控頁面搜尋框輸入藥物名稱
2. 系統即時顯示完整的基本+詳細資料
3. 如果該藥物被更新，頁面自動刷新
```

## 🔧 系統特色功能

### 實時監控 🆕
- **即時通知**: 所有藥物更新、新訂單都會立即通知
- **自動更新**: 查看中的藥物資料自動更新
- **狀態控制**: 直接在監控頁面管理訂單狀態

### 智能搜尋
- **即時搜尋**: 輸入藥物名稱即時顯示結果
- **完整資料**: 同時顯示基本和詳細資料
- **更新提醒**: 搜尋中的藥物更新時自動刷新

### 訂單管理
- **多來源**: 支援一般API和ROS2訂單
- **狀態追蹤**: 完整的訂單狀態管理
- **即時回覆**: 狀態更新立即通知所有相關方

## 🎉 您的需求100%滿足

✅ **藥物更新即時性**: WebSocket確保所有更新都是即時的  
✅ **特定藥物查詢**: 多種方式查看特定藥物資訊  
✅ **新訂單通知**: 立即收到並可回覆狀態  
✅ **ROS2整合**: 完整的ROS2 API支援  
✅ **實時監控**: 專門的監控系統管理所有活動  

現在您擁有一個功能完整、即時響應的藥物管理系統！
# 醫院藥物管理系統 - 最小化版本

## 🎯 核心檔案 (僅 12 個檔案)

```
user_interface/
├── simple_server_clean.py          # 主伺服器 (含 ROS2 模擬)
├── simple_server_ros2_real.py      # 真實 ROS2 版本
├── database_clean.py               # 資料庫模型
├── ros2_mock_clean.py              # ROS2 模擬器
├── ros2_medicine_client.py         # ROS2 客戶端包 (新增)
├── start_clean_system.py           # 主啟動器
├── start_ros2_real_server.py       # ROS2 啟動器
├── clean_test_system.py            # 測試腳本
├── hospital_medicine_clean.db      # 資料庫檔案
├── static/                         # 前端檔案
├── README_MINIMAL.md               # 本檔案
└── SYSTEM_OVERVIEW.md             # 系統總覽
```

## 🚀 快速開始

### 1. 推薦版本 (含 ROS2 模擬)
```bash
python3 start_clean_system.py
```

### 2. 真實 ROS2 版本
```bash
python3 start_ros2_real_server.py
```

### 3. 訪問系統
```
http://localhost:8001/integrated_medicine_management.html
```

## 🔌 ROS2 客戶端包使用

### 基本使用
```python
from ros2_medicine_client import MedicineROS2Client

# 創建客戶端
client = MedicineROS2Client()

# 初始化
await client.initialize()

# 查詢藥物
result = await client.query_medicine("阿司匹林")

# 處理訂單
orders = await client.get_pending_orders()
await client.execute_order(1)

# 關閉客戶端
await client.shutdown()
```

### 完整工作流程
```python
# 處理單個訂單
result = await client.full_order_workflow(prescription_id)

# 處理所有待處理訂單
results = await client.process_all_pending_orders()
```

### 測試客戶端
```bash
python3 ros2_medicine_client.py
```

## 📝 主要功能

### ✅ 藥物管理
- 基本藥物資訊 CRUD
- 詳細藥物資訊管理
- 即時庫存控制
- 智能搜尋

### ✅ 處方籤管理
- 處方籤創建和編輯
- 狀態追蹤 (pending, processing, completed, cancelled)
- 自動庫存扣減 (創建時)
- 庫存檢查和警告

### ✅ ROS2 整合
- **詢問-確認-執行**工作流程
- 藥物查詢服務 (基本/詳細)
- 批量藥物查詢
- 自動狀態更新
- 雙模式: 模擬和真實 ROS2

### ✅ 現代化前端
- 整合式管理界面
- 響應式設計
- 即時資料更新

## 🛠️ 系統測試

```bash
# 執行基本測試
python3 clean_test_system.py

# 測試 ROS2 客戶端
python3 ros2_medicine_client.py
```

## 📦 API 端點

### 系統狀態
```
GET /api/system/status
GET /api/ros2/status
```

### 藥物管理
```
GET    /api/medicine/basic
POST   /api/medicine/unified
PUT    /api/medicine/{id}
DELETE /api/medicine/{id}
POST   /api/medicine/adjust-stock
```

### 處方籤管理
```
GET    /api/prescription/
POST   /api/prescription/
PUT    /api/prescription/{id}/status
```

### ROS2 整合
```
GET    /api/ros2/pending-orders
POST   /api/ros2/request-order-confirmation
POST   /api/ros2/confirm-and-execute-order
POST   /api/ros2/complete-order
POST   /api/ros2/query-medicine
```

## 🔧 故障排除

### 常見問題

**1. 模組缺失**
```bash
# 系統套件方式
sudo apt install python3-fastapi python3-uvicorn python3-sqlalchemy python3-requests

# 虛擬環境方式
python3 -m venv venv
source venv/bin/activate
pip install fastapi uvicorn sqlalchemy requests
```

**2. 端口被占用**
```bash
lsof -i :8001
kill -9 [PID]
```

**3. 資料庫問題**
```bash
rm -f hospital_medicine_clean.db
python3 database_clean.py
```

**4. ROS2 環境問題**
- 推薦版本: 自動使用模擬模式
- 真實版本: 需要正確的 ROS2 環境

## 🎯 版本選擇建議

### 一般使用 (推薦)
```bash
python3 start_clean_system.py
```
- 包含 ROS2 模擬功能
- 無需真實 ROS2 環境
- 功能完整且穩定

### 機器人整合
```bash
python3 start_ros2_real_server.py
```
- 需要真實 ROS2 環境
- 直接連接機器人系統
- 使用 `ros2_medicine_client.py` 作為客戶端

### 客戶端開發
```python
from ros2_medicine_client import MedicineROS2Client
```
- 統一的 ROS2/HTTP 介面
- 自動回退機制
- 完整的工作流程支援

## 📊 系統狀態

✅ **最小化完成**: 從 20+ 個檔案減少到 12 個核心檔案  
✅ **功能完整**: 保持所有核心功能  
✅ **ROS2 包化**: 獨立的 ROS2 客戶端包  
✅ **穩定可靠**: 經過全面測試驗證  

這是一個經過最小化優化的醫院藥物管理系統，保持完整功能的同時大幅簡化了檔案結構。
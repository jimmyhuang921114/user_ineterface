# 醫院藥物管理系統 - 完整說明文件

這是一個集成 ROS2 的醫院藥物管理系統，提供完整的藥物管理、處方籤管理和自動化訂單處理功能。

## 📋 目錄

- [系統架構](#系統架構)
- [版本說明](#版本說明)
- [核心組件](#核心組件)
- [API 端點](#api-端點)
- [前端頁面](#前端頁面)
- [啟動方式](#啟動方式)
- [功能特色](#功能特色)
- [資料庫結構](#資料庫結構)
- [ROS2 整合](#ros2-整合)

## 🏗️ 系統架構

```
醫院藥物管理系統
├── 後端 API (FastAPI)
│   ├── 藥物管理模組
│   ├── 處方籤管理模組
│   └── ROS2 整合模組
├── 前端界面 (HTML/CSS/JS)
│   ├── 醫生工作站
│   ├── 藥物管理介面
│   └── 處方籤管理介面
├── 資料庫 (SQLite)
│   ├── 藥物基本資料表
│   ├── 藥物詳細資料表
│   ├── 處方籤表
│   └── 處方籤藥物關聯表
└── ROS2 模擬系統
    ├── 訂單處理佇列
    ├── 機器人控制模擬
    └── 主控制器通訊
```

## 📦 版本說明

### 測試版本（包含樣本資料）
- `simple_server.py` - 主伺服器（測試版）
- `database.py` - 資料庫模型（包含樣本資料）
- `ros2_mock.py` - ROS2 模擬模組（豐富日誌）

### 乾淨版本（生產環境）
- `simple_server_clean.py` - 主伺服器（乾淨版）
- `database_clean.py` - 資料庫模型（空資料庫）
- `ros2_mock_clean.py` - ROS2 模擬模組（簡潔日誌）
- `start_clean_server.py` - 乾淨版啟動腳本

## 🧩 核心組件

### 1. 主伺服器 (`simple_server.py` / `simple_server_clean.py`)

**功能**：
- FastAPI 網頁應用框架
- RESTful API 端點定義
- 靜態檔案服務
- 資料庫連接管理
- ROS2 系統整合
- 日誌記錄系統

**主要特色**：
- 自動 API 文檔生成 (`/docs`)
- 支援 CORS 跨域請求
- 完整的錯誤處理
- 資料驗證和序列化
- 背景任務處理

**文件大小**：約 25KB (乾淨版 23KB)

### 2. 資料庫模組 (`database.py` / `database_clean.py`)

**功能**：
- SQLAlchemy ORM 模型定義
- 資料庫連接配置
- 表格關聯定義
- 資料庫初始化

**資料表結構**：
```sql
-- 基本藥物表
medicine_basic (id, name, amount, position, manufacturer, dosage, is_active, created_at, updated_at)

-- 詳細藥物表  
medicine_detailed (id, medicine_id, description, ingredient, category, usage_method, ...)

-- 處方籤表
prescriptions (id, patient_name, patient_id, doctor_name, diagnosis, status, created_at, updated_at)

-- 處方籤藥物關聯表
prescription_medicines (id, prescription_id, medicine_id, dosage, frequency, duration, quantity)
```

**差異**：
- 測試版：自動添加 16 種樣本藥物
- 乾淨版：空資料庫，獨立資料庫檔案

**文件大小**：測試版 7KB，乾淨版 4KB

### 3. ROS2 整合模組 (`ros2_mock.py` / `ros2_mock_clean.py`)

**功能**：
- 模擬 ROS2 節點操作
- 訂單佇列管理
- 背景處理執行緒
- 機器人狀態模擬
- 主控制器通訊

**核心類別**：
```python
class MockHospitalROS2Node:
    - add_order()           # 添加訂單到處理佇列
    - get_queue_status()    # 獲取佇列狀態
    - publish_medicine_data() # 發布藥物資料更新
    - emergency_stop()      # 緊急停止
    - reset_system()        # 重置系統
```

**處理流程**：
1. 驗證處方籤 (2秒)
2. 定位藥物 (3秒)
3. 移動機器人 (4秒)
4. 分配藥物 (5秒)
5. 包裝藥物 (3秒)
6. 品質檢查 (2秒)

**文件大小**：測試版 8.5KB，乾淨版 8KB

### 4. 前端靜態檔案

#### `static/doctor.html` + `static/js/doctor.js`
**功能**：醫生工作站介面
- 病患資訊輸入
- 診斷記錄
- 藥物選擇和配置
- 處方籤創建
- 即時藥物庫存查詢

#### `static/Prescription.html`
**功能**：處方籤管理介面
- 處方籤列表檢視
- 處方詳細資訊展示
- 狀態篩選
- 處方籤編輯

#### `static/integrated_medicine_management.html` + `static/js/integrated_medicine_management.js`
**功能**：綜合藥物管理介面
- 藥物庫存管理
- 基本與詳細資料檢視
- 藥物 CRUD 操作
- 批量操作功能

#### `static/test_all_functions.html` + `static/js/test_functions.js`
**功能**：系統測試介面
- API 端點測試
- 即時日誌顯示
- 功能驗證
- 效能監控

## 🔌 API 端點

### 藥物管理 API

```http
# 基本藥物操作
GET    /api/medicine/basic              # 獲取基本藥物列表
GET    /api/medicine/detailed           # 獲取詳細藥物列表  
GET    /api/medicine/{id}               # 獲取單一藥物詳情
POST   /api/medicine/                   # 創建基本藥物
POST   /api/medicine/detailed           # 創建詳細藥物
POST   /api/medicine/unified            # 創建統一藥物（推薦）
PUT    /api/medicine/{id}               # 更新藥物資訊
DELETE /api/medicine/{id_or_name}       # 刪除藥物（按ID或名稱）
```

### 處方籤管理 API

```http
# 處方籤操作
GET    /api/prescription/               # 獲取處方籤列表
POST   /api/prescription/               # 創建新處方籤
GET    /api/prescription/{id}           # 獲取處方籤詳情
PUT    /api/prescription/{id}/status    # 更新處方籤狀態
GET    /api/prescription/pending/next   # 獲取下一個待處理處方籤
```

### ROS2 整合 API

```http
# ROS2 系統操作
GET    /api/ros2/status                 # 獲取ROS2系統狀態
POST   /api/ros2/order                  # 添加ROS2訂單
POST   /api/ros2/request-next-order     # 請求下一個訂單（主控制器使用）
```

### 系統 API

```http
# 系統狀態
GET    /api/health                      # 系統健康檢查
GET    /docs                            # API 文檔
```

## 🌐 前端頁面

### 主要頁面路由

```http
GET    /                                # 系統首頁
GET    /doctor.html                     # 醫生工作站
GET    /Prescription.html               # 處方籤管理
GET    /integrated_medicine_management.html  # 藥物管理
GET    /test_all_functions.html         # 系統測試
```

### 頁面功能對照表

| 頁面 | 主要功能 | 目標用戶 | 特色 |
|------|----------|----------|------|
| 醫生工作站 | 開立處方籤 | 醫生 | 藥物搜尋、劑量配置 |
| 處方籤管理 | 處方追蹤 | 藥師/管理員 | 狀態管理、詳細檢視 |
| 藥物管理 | 庫存管理 | 藥師/管理員 | CRUD操作、批量處理 |
| 系統測試 | 功能驗證 | 開發者 | API測試、日誌監控 |

## 🚀 啟動方式

### 測試版本啟動

```bash
# 方法1：直接啟動主伺服器
python3 simple_server.py

# 方法2：使用主入口
python3 main.py

# 方法3：使用啟動腳本
./start_system_simple.sh
```

### 乾淨版本啟動

```bash
# 啟動乾淨版本（推薦生產環境）
python3 start_clean_server.py

# 或直接啟動
python3 simple_server_clean.py
```

### 系統需求

```bash
# 安裝依賴套件
pip3 install fastapi uvicorn sqlalchemy

# 對於完整ROS2支援（可選）
# 需要安裝 ROS2 和 rclpy
```

### 啟動後訪問

```
主系統：http://localhost:8001/
API文檔：http://localhost:8001/docs
醫生界面：http://localhost:8001/doctor.html
處方籤管理：http://localhost:8001/Prescription.html
藥物管理：http://localhost:8001/integrated_medicine_management.html
```

## ✨ 功能特色

### 1. 智能藥物管理
- **自動庫存追蹤**：即時更新藥物數量
- **位置管理**：支援藥物位置編碼（如：A1-01, B2-05）
- **批次操作**：支援批量添加、更新、刪除
- **安全刪除**：被處方籤使用的藥物會標記為無效而非刪除

### 2. 完整處方籤工作流
- **醫生開立**：直觀的處方籤創建界面
- **狀態追蹤**：pending → processing → completed
- **自動化處理**：與ROS2系統整合的自動配藥
- **歷史記錄**：完整的處方歷史追蹤

### 3. ROS2 整合特色
- **佇列管理**：優先處理最舊的待處理訂單
- **背景處理**：模擬真實的藥物分配流程
- **主控制器通訊**：支援外部ROS2系統請求訂單
- **即時狀態**：提供即時的系統狀態監控

### 4. 開發友好功能
- **自動API文檔**：Swagger/OpenAPI 規格
- **詳細日誌**：分級日誌記錄系統
- **測試界面**：內建的功能測試工具
- **熱重載**：開發時支援程式碼熱重載

## 🗄️ 資料庫結構

### 核心資料表

#### medicine_basic (藥物基本資料)
```sql
CREATE TABLE medicine_basic (
    id INTEGER PRIMARY KEY,
    name VARCHAR NOT NULL,           -- 藥物名稱
    amount INTEGER DEFAULT 0,        -- 庫存數量
    position VARCHAR,                -- 儲存位置
    manufacturer VARCHAR,            -- 製造商
    dosage VARCHAR,                  -- 劑量規格
    is_active BOOLEAN DEFAULT TRUE,  -- 是否啟用
    created_at DATETIME,             -- 創建時間
    updated_at DATETIME              -- 更新時間
);
```

#### medicine_detailed (藥物詳細資料)
```sql
CREATE TABLE medicine_detailed (
    id INTEGER PRIMARY KEY,
    medicine_id INTEGER REFERENCES medicine_basic(id),
    description TEXT,                -- 藥物說明
    ingredient VARCHAR,              -- 主要成分
    category VARCHAR,                -- 藥物分類
    usage_method VARCHAR,            -- 使用方法
    unit_dose VARCHAR,               -- 單位劑量
    side_effects TEXT,               -- 副作用
    storage_conditions VARCHAR,      -- 儲存條件
    expiry_date DATETIME,            -- 有效期限
    barcode VARCHAR UNIQUE,          -- 條碼
    appearance_type VARCHAR,         -- 外觀形式
    notes TEXT                       -- 備註
);
```

#### prescriptions (處方籤)
```sql
CREATE TABLE prescriptions (
    id INTEGER PRIMARY KEY,
    patient_name VARCHAR NOT NULL,   -- 病患姓名
    patient_id VARCHAR NOT NULL,     -- 病患ID
    doctor_name VARCHAR NOT NULL,    -- 醫生姓名
    diagnosis TEXT,                  -- 診斷
    status VARCHAR DEFAULT 'active', -- 狀態
    created_at DATETIME,             -- 創建時間
    updated_at DATETIME              -- 更新時間
);
```

#### prescription_medicines (處方籤藥物關聯)
```sql
CREATE TABLE prescription_medicines (
    id INTEGER PRIMARY KEY,
    prescription_id INTEGER REFERENCES prescriptions(id),
    medicine_id INTEGER REFERENCES medicine_basic(id),
    dosage VARCHAR,                  -- 劑量
    frequency VARCHAR,               -- 頻率
    duration VARCHAR,                -- 療程
    instructions TEXT,               -- 特殊說明
    quantity INTEGER DEFAULT 1       -- 數量
);
```

### 資料關聯圖

```
medicine_basic ─┬─ medicine_detailed (1:1)
                └─ prescription_medicines (1:N)
                   └─ prescriptions (N:1)
```

## 🤖 ROS2 整合

### 系統架構

```
醫院管理系統 ←→ ROS2模擬模組 ←→ 實際ROS2系統
     ↓              ↓              ↓
  API呼叫        訊息佇列        機器人控制
```

### ROS2 功能模組

#### 1. 訂單管理
- **佇列系統**：FIFO 訂單處理佇列
- **優先級處理**：支援高優先級訂單
- **狀態追蹤**：pending → queued → processing → completed

#### 2. 機器人模擬
- **移動控制**：模擬機器人移動到藥物位置
- **抓取操作**：模擬藥物抓取和放置
- **路徑規劃**：基於藥物位置的路徑最佳化

#### 3. 主控制器通訊
```python
# 主控制器請求訂單
POST /api/ros2/request-next-order
{
    "requester_id": "ros2_master_01",
    "priority": "any"
}

# 回應格式
{
    "success": true,
    "order": {
        "order_id": "ORDER_0001",
        "prescription_id": 1,
        "patient_info": {...},
        "medicines": [...],
        "estimated_duration": 180
    }
}
```

#### 4. 即時監控
- **佇列狀態**：即時顯示處理佇列
- **機器人狀態**：電池、位置、任務狀態
- **效能指標**：處理時間、成功率統計

### ROS2 訊息格式

#### 藥物資料更新訊息
```json
{
    "message_type": "medicine_update",
    "medicine_id": 1,
    "name": "普拿疼",
    "position": "A1-01",
    "amount": 95,
    "action": "updated"
}
```

#### 新處方籤訂單訊息
```json
{
    "message_type": "new_prescription_order",
    "order_id": "ORDER_0001",
    "prescription_id": 1,
    "patient_info": {
        "name": "張三",
        "id": "A123456789"
    },
    "medicines": [
        {
            "medicine_id": 1,
            "medicine_name": "普拿疼",
            "position": "A1-01",
            "quantity": 30,
            "dosage": "500mg"
        }
    ],
    "priority": "normal",
    "estimated_completion_time": 1691234567
}
```

## 📊 系統監控

### 日誌系統
- **分級記錄**：DEBUG, INFO, WARNING, ERROR
- **文件輸出**：自動記錄到 `.log` 檔案
- **即時監控**：測試頁面提供即時日誌檢視

### 效能指標
- **API 回應時間**：平均 < 100ms
- **資料庫查詢**：優化索引，快速檢索
- **併發處理**：支援多用戶同時操作
- **記憶體使用**：輕量級設計，低資源占用

### 健康檢查
```bash
# 系統健康檢查
curl http://localhost:8001/api/health

# 回應範例
{
    "status": "healthy",
    "message": "系統運行正常",
    "ros2_status": "mock",
    "ros2_mode": "mock"
}
```

## 🔧 開發資訊

### 技術棧
- **後端**：Python 3.8+, FastAPI, SQLAlchemy, Uvicorn
- **前端**：HTML5, CSS3, Vanilla JavaScript
- **資料庫**：SQLite 3
- **ROS2**：模擬模組 (可替換為真實 ROS2)

### 程式碼結構
```
user_interface/
├── simple_server.py           # 主伺服器（測試版）
├── simple_server_clean.py     # 主伺服器（乾淨版）
├── database.py                # 資料庫模型（測試版）
├── database_clean.py          # 資料庫模型（乾淨版）
├── ros2_mock.py               # ROS2模擬（測試版）
├── ros2_mock_clean.py         # ROS2模擬（乾淨版）
├── start_clean_server.py      # 乾淨版啟動腳本
├── main.py                    # 主入口點
├── start_system_simple.sh     # 系統啟動腳本
├── static/                    # 前端靜態檔案
│   ├── doctor.html
│   ├── Prescription.html
│   ├── integrated_medicine_management.html
│   ├── test_all_functions.html
│   ├── css/
│   │   └── unified_style.css
│   └── js/
│       ├── doctor.js
│       ├── integrated_medicine_management.js
│       └── test_functions.js
└── README.md                  # 本文件
```

### 部署建議
- **開發環境**：使用測試版本，包含樣本資料
- **測試環境**：使用乾淨版本，手動添加測試資料
- **生產環境**：使用乾淨版本，配置外部資料庫

---

## 📞 支援與貢獻

如有問題或建議，請查看：
- API 文檔：http://localhost:8001/docs
- 系統測試：http://localhost:8001/test_all_functions.html
- 健康檢查：http://localhost:8001/api/health

**版本資訊**：v1.0.0
**最後更新**：2025-08-07
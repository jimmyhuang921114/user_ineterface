# 醫院藥物管理系統 - 系統架構文檔
Hospital Medicine Management System - System Architecture Documentation

## 1. 系統概覽

本系統是一個基於FastAPI的醫院藥物管理系統，採用前後端分離架構，支援藥物庫存管理、病人資料管理、病例記錄管理等功能。

## 2. 系統架構圖

```
醫院藥物管理系統架構
├── 前端層 (Frontend Layer)
│   ├── web/html/
│   │   ├── Medicine.html          # 藥物管理頁面
│   │   ├── Patients.html          # 病人管理頁面
│   │   ├── Records.html           # 病例記錄頁面
│   │   ├── doctor.html            # 醫生管理頁面
│   │   └── Prescription.html      # 處方管理頁面
│   ├── web/css/
│   │   ├── medicine.css           # 藥物頁面樣式
│   │   ├── doctor_style.css       # 醫生頁面樣式
│   │   └── Prescription.css       # 處方頁面樣式
│   └── web/js/
│       ├── medicine.js            # 藥物管理邏輯
│       ├── doctor.js              # 醫生管理邏輯
│       └── Prescription.js        # 處方管理邏輯
│
├── API層 (API Layer)
│   ├── enhanced_server.py         # 主要API伺服器
│   ├── hospital_server.py         # 備用API伺服器
│   └── API端點群組
│       ├── 藥物管理API
│       ├── 病人管理API
│       ├── 病例記錄API
│       └── 資料導出API
│
├── 業務邏輯層 (Business Logic Layer)
│   ├── user_interface/route/
│   │   └── routes_medicine.py     # 藥物路由定義
│   ├── user_interface/services/
│   │   └── crud_medicine.py       # 藥物CRUD操作
│   └── user_interface/models.py   # 資料模型定義
│
├── 資料存取層 (Data Access Layer)
│   ├── 記憶體資料庫 (In-Memory Database)
│   │   ├── medicines_db          # 基本藥物庫存
│   │   ├── detailed_medicines_db # 詳細藥物資訊
│   │   ├── patients_db           # 病人資料
│   │   └── patient_records_db    # 病例記錄
│   └── JSON匯出功能
│
└── 工具層 (Utility Layer)
    ├── api_client_examples.py     # API使用範例
    ├── quick_api_guide.py         # 快速使用指南
    ├── test_enhanced_system.py    # 系統測試腳本
    └── run_system.py              # 系統啟動腳本
```

## 3. 資料流架構

```
[前端界面] ←→ [HTTP/JSON] ←→ [FastAPI路由] ←→ [業務邏輯] ←→ [記憶體資料庫]
     ↓                                                            ↓
[使用者操作]                                                [JSON匯出]
```

## 4. 各模組功能詳述

### 4.1 前端模組

#### HTML頁面模組
- **Medicine.html**: 藥物庫存管理界面
  - 功能: 新增、修改、刪除藥物資訊
  - 特色: 使用Handsontable提供表格編輯功能
  
- **Patients.html**: 病人資料管理界面
  - 功能: 病人基本資料CRUD操作
  - 特色: 支援病人資訊查詢和編輯

- **Records.html**: 病例記錄管理界面
  - 功能: 病例記錄的新增、查詢、管理
  - 特色: 關聯病人資料顯示

- **doctor.html**: 醫生管理界面
  - 功能: 醫生資訊管理
  
- **Prescription.html**: 處方管理界面
  - 功能: 處方開立和管理

#### JavaScript模組
- **medicine.js**: 
  - MedicineManager類別
  - 功能: API呼叫、表格操作、資料驗證
  - 方法: loadMedicines, addMedicine, deleteMedicine, exportToJSON

#### CSS模組
- **medicine.css**: 藥物頁面樣式定義
- **doctor_style.css**: 醫生頁面樣式定義
- **Prescription.css**: 處方頁面樣式定義

### 4.2 API層模組

#### 主要伺服器
- **enhanced_server.py**: 
  - 功能: 完整的FastAPI應用程式
  - 特色: 整合所有API端點、CORS支援、靜態檔案服務
  - 資料庫: 使用記憶體資料庫

#### API端點群組

##### 藥物管理API
```
POST   /api/medicine/                    # 新增基本藥物
GET    /api/medicine/                    # 獲取所有基本藥物
PUT    /api/medicine/{id}                # 更新藥物
DELETE /api/medicine/{id}                # 刪除藥物
POST   /api/medicine/detailed/           # 新增詳細藥物資訊
GET    /api/medicine/detailed/{name}     # 獲取詳細藥物資訊
GET    /api/medicine/detailed/           # 獲取所有詳細藥物
GET    /api/medicine/search/detailed/{query}  # 搜尋詳細藥物
GET    /api/medicine/search/code/{code}  # 根據包裝編號搜尋
GET    /api/medicine/integrated/{name}   # 獲取整合藥物資訊
```

##### 病人管理API
```
GET    /api/patients/                    # 獲取所有病人
POST   /api/patients/                    # 新增病人
GET    /api/patients/{id}                # 獲取特定病人
PUT    /api/patients/{id}                # 更新病人資訊
DELETE /api/patients/{id}                # 刪除病人
```

##### 病例記錄API
```
GET    /api/records/                     # 獲取所有病例記錄
POST   /api/records/                     # 新增病例記錄
GET    /api/records/patient/{id}         # 獲取病人的所有記錄
DELETE /api/records/{id}                 # 刪除病例記錄
```

##### 資料導出API
```
GET    /api/export/medicines/basic       # 導出基本藥物資訊
GET    /api/export/medicines/detailed    # 導出詳細藥物資訊
GET    /api/export/medicines/integrated  # 導出整合藥物資訊
GET    /api/export/patients              # 導出病人資料
GET    /api/export/records               # 導出病例記錄
GET    /api/export/complete              # 導出完整系統資料
```

### 4.3 業務邏輯層模組

#### 路由模組
- **routes_medicine.py**:
  - 功能: 定義藥物相關API路由
  - 包含: CRUD操作、搜尋功能、批次操作

#### 服務模組
- **crud_medicine.py**:
  - 功能: 藥物資料的CRUD操作邏輯
  - 方法: create_medicine, get_medicine_by_id, update_medicine, delete_medicine

#### 模型模組
- **models.py**:
  - 功能: 定義資料模型結構
  - 包含: SQLAlchemy模型定義

### 4.4 資料存取層模組

#### 記憶體資料庫
- **medicines_db**: 
  - 類型: Python List
  - 內容: 基本藥物庫存資訊
  - 結構: {id, name, amount, position, unit, expiry_date}

- **detailed_medicines_db**:
  - 類型: Python Dictionary
  - 內容: 詳細藥物資訊
  - 結構: 完整的藥物JSON資料（如心律錠範例）

- **patients_db**:
  - 類型: Python List
  - 內容: 病人基本資料
  - 結構: {id, name, age, gender, phone, address, medical_history, allergies}

- **patient_records_db**:
  - 類型: Python List
  - 內容: 病例記錄
  - 結構: {id, patient_id, visit_date, diagnosis, prescribed_medicines, dosage_instructions, doctor_notes}

### 4.5 工具層模組

#### API客戶端工具
- **api_client_examples.py**:
  - 功能: 完整的API使用範例
  - 類別: HospitalSystemAPI
  - 包含: 所有API操作的示例代碼

- **quick_api_guide.py**:
  - 功能: 簡化的API使用指南
  - 包含: 常用功能的快速調用方法

#### 測試工具
- **test_enhanced_system.py**:
  - 功能: 系統功能完整性測試
  - 包含: 所有API端點的自動化測試

#### 系統管理工具
- **run_system.py**:
  - 功能: 系統啟動和依賴檢查
  - 特色: 自動安裝依賴、啟動伺服器、執行測試

## 5. 技術堆疊

### 後端技術
- **FastAPI**: Web框架
- **Uvicorn**: ASGI伺服器
- **Pydantic**: 資料驗證
- **SQLAlchemy**: ORM (原始設計，現改用記憶體資料庫)
- **Jinja2**: 模板引擎

### 前端技術
- **HTML5**: 頁面結構
- **CSS3**: 樣式設計
- **JavaScript (ES6)**: 前端邏輯
- **Handsontable**: 表格元件
- **Fetch API**: HTTP請求

### 開發工具
- **Python 3.x**: 程式語言
- **Requests**: HTTP客戶端庫
- **JSON**: 資料交換格式

## 6. 系統特色

### 6.1 模組化設計
- 清晰的分層架構
- 獨立的功能模組
- 易於擴展和維護

### 6.2 資料整合
- 基本藥物資訊與詳細資訊分離存儲
- 提供整合查詢API
- 支援多種資料導出格式

### 6.3 搜尋功能
- 支援藥物名稱搜尋
- 支援包裝編號搜尋
- 支援部分匹配和精確匹配

### 6.4 使用者友善
- 直觀的Web界面
- 完整的API文檔和範例
- 詳細的錯誤處理和回饋

## 7. 擴展性考量

### 7.1 資料庫升級
- 目前使用記憶體資料庫，可輕易升級至PostgreSQL、MySQL等
- 保留SQLAlchemy模型定義，便於資料庫遷移

### 7.2 功能擴展
- 模組化設計支援新功能添加
- API設計遵循RESTful原則，易於擴展

### 7.3 效能優化
- 支援非同步處理
- 可添加Redis快取層
- 支援資料庫連接池

## 8. 安全考量

### 8.1 API安全
- CORS設定
- 輸入資料驗證
- HTTP狀態碼規範使用

### 8.2 資料安全
- 資料驗證和清理
- 錯誤處理和日誌記錄

## 9. 部署建議

### 9.1 開發環境
```bash
python3 enhanced_server.py
```

### 9.2 生產環境
```bash
uvicorn enhanced_server:app --host 0.0.0.0 --port 8000 --workers 4
```

### 9.3 容器化部署
- 支援Docker容器化
- 可使用docker-compose進行服務編排
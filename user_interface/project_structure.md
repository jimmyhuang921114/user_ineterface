# 醫院藥物管理系統 - 專案檔案結構
Hospital Medicine Management System - Project Structure

## 專案目錄結構

```
workspace/
├── user_interface/                    # 後端主要目錄
│   ├── enhanced_server.py            # 主要API伺服器 (推薦使用)
│   ├── hospital_server.py            # 備用伺服器
│   ├── final_server.py               # 簡化版伺服器
│   ├── working_server.py             # 測試版伺服器
│   ├── main.py                       # 原始伺服器檔案
│   │
│   ├── models.py                     # 資料模型定義
│   │
│   ├── route/                        # 路由定義目錄
│   │   └── routes_medicine.py        # 藥物路由
│   │
│   ├── services/                     # 業務邏輯目錄
│   │   └── crud_medicine.py          # 藥物CRUD操作
│   │
│   ├── api_client_examples.py        # 完整API使用範例
│   ├── quick_api_guide.py            # 快速API指南
│   │
│   ├── test_enhanced_system.py       # 系統測試腳本
│   ├── quick_test.py                 # 快速測試腳本
│   ├── test_api.py                   # API測試腳本
│   │
│   ├── run_system.py                 # 系統啟動腳本
│   ├── start_server.py               # 伺服器啟動腳本
│   │
│   ├── system_architecture.md        # 系統架構文檔
│   ├── system_functions.md           # 功能說明文檔
│   ├── project_structure.md          # 本檔案
│   └── README.md                     # 系統說明文檔
│
└── web/                              # 前端檔案目錄
    ├── html/                         # HTML頁面
    │   ├── Medicine.html             # 藥物管理頁面
    │   ├── Patients.html             # 病人管理頁面
    │   ├── Records.html              # 病例記錄頁面
    │   ├── doctor.html               # 醫生管理頁面
    │   └── Prescription.html         # 處方管理頁面
    │
    ├── css/                          # 樣式檔案
    │   ├── medicine.css              # 藥物頁面樣式
    │   ├── doctor_style.css          # 醫生頁面樣式
    │   └── Prescription.css          # 處方頁面樣式
    │
    └── js/                           # JavaScript檔案
        ├── medicine.js               # 藥物管理邏輯
        ├── doctor.js                 # 醫生管理邏輯
        └── Prescription.js           # 處方管理邏輯
```

## 核心檔案說明

### 伺服器檔案
| 檔案名稱 | 功能描述 | 使用建議 |
|---------|----------|----------|
| `enhanced_server.py` | 完整功能伺服器，包含所有模組 | **主要使用** |
| `hospital_server.py` | 穩定版伺服器，基本功能完整 | 備用選擇 |
| `final_server.py` | 簡化版伺服器，適合學習 | 學習參考 |
| `working_server.py` | 測試版伺服器 | 開發測試 |
| `main.py` | 原始版本，使用SQLAlchemy | 參考用 |

### API客戶端工具
| 檔案名稱 | 功能描述 | 使用場景 |
|---------|----------|----------|
| `api_client_examples.py` | 完整的API使用範例和類別 | 完整程式開發 |
| `quick_api_guide.py` | 簡化的API使用指南 | 快速上手 |

### 測試工具
| 檔案名稱 | 功能描述 | 使用時機 |
|---------|----------|----------|
| `test_enhanced_system.py` | 完整系統功能測試 | 系統驗證 |
| `quick_test.py` | 快速功能測試 | 基本檢查 |
| `test_api.py` | API端點測試 | API驗證 |

### 啟動工具
| 檔案名稱 | 功能描述 | 適用情況 |
|---------|----------|----------|
| `run_system.py` | 完整的系統啟動和檢查 | **推薦使用** |
| `start_server.py` | 簡單的伺服器啟動 | 快速啟動 |

### 文檔檔案
| 檔案名稱 | 內容描述 |
|---------|----------|
| `system_architecture.md` | 系統架構和技術棧說明 |
| `system_functions.md` | 詳細功能說明和API文檔 |
| `project_structure.md` | 本檔案，專案結構說明 |
| `README.md` | 快速入門和安裝指南 |

## 前端檔案結構

### HTML頁面
- **Medicine.html**: 藥物庫存管理主頁面
- **Patients.html**: 病人資料管理頁面
- **Records.html**: 病例記錄管理頁面
- **doctor.html**: 醫生資訊管理頁面
- **Prescription.html**: 處方開立管理頁面

### CSS樣式
- **medicine.css**: 藥物頁面專用樣式，包含表格和按鈕樣式
- **doctor_style.css**: 醫生頁面樣式
- **Prescription.css**: 處方頁面樣式

### JavaScript邏輯
- **medicine.js**: 包含MedicineManager類別，處理藥物CRUD操作
- **doctor.js**: 醫生管理相關邏輯
- **Prescription.js**: 處方管理相關邏輯

## 資料存儲結構

### 記憶體資料庫
```
enhanced_server.py 內建資料庫:
├── medicines_db (List)           # 基本藥物庫存
├── detailed_medicines_db (Dict)  # 詳細藥物資訊
├── patients_db (List)            # 病人基本資料
└── patient_records_db (List)     # 病例記錄
```

### 資料關聯
```
病人 (patients_db)
├── id: 唯一識別碼
└── 關聯: patient_records_db (透過 patient_id)

藥物資料
├── medicines_db: 基本庫存資訊
└── detailed_medicines_db: 詳細醫療資訊
    └── 關聯: 透過藥物名稱
```

## 開發流程建議

### 1. 快速開始
```bash
# 啟動系統
python3 user_interface/run_system.py

# 或直接啟動伺服器
python3 user_interface/enhanced_server.py
```

### 2. 測試驗證
```bash
# 執行完整測試
python3 user_interface/test_enhanced_system.py

# 快速測試
python3 user_interface/quick_test.py
```

### 3. API開發
```bash
# 參考API使用範例
python3 user_interface/api_client_examples.py

# 快速API測試
python3 user_interface/quick_api_guide.py
```

## 部署檔案優先級

### 生產環境
1. `enhanced_server.py` - 主要伺服器
2. `web/` 目錄 - 前端檔案
3. `system_*.md` - 系統文檔

### 開發環境
1. 所有伺服器檔案 - 用於測試不同版本
2. 所有測試工具 - 用於開發驗證
3. API範例檔案 - 用於學習參考

### 最小部署
```
必要檔案:
├── enhanced_server.py
├── web/html/Medicine.html
├── web/css/medicine.css
├── web/js/medicine.js
└── README.md
```

## 檔案依賴關係

### 後端依賴
```
enhanced_server.py
├── 依賴: fastapi, uvicorn, pydantic
├── 靜態檔案: web/ 目錄
└── 可選: models.py, route/, services/
```

### 前端依賴
```
Medicine.html
├── 依賴: medicine.css, medicine.js
├── 外部庫: Handsontable
└── API: enhanced_server.py
```

### 工具依賴
```
api_client_examples.py
├── 依賴: requests, json
└── 目標: enhanced_server.py API

test_enhanced_system.py
├── 依賴: requests
└── 目標: enhanced_server.py
```

這個檔案結構提供了清晰的專案組織，讓開發者能夠快速理解各個檔案的功能和相互關係。
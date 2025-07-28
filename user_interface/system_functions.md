# 醫院藥物管理系統 - 功能說明文檔
Hospital Medicine Management System - Function Documentation

## 1. 系統功能總覽

### 1.1 核心功能模組
- **藥物管理模組**: 基本庫存管理 + 詳細藥物資訊
- **病人管理模組**: 病人基本資料管理
- **病例記錄模組**: 就診記錄和處方管理
- **資料導出模組**: 多格式JSON資料導出
- **搜尋查詢模組**: 多條件搜尋功能

### 1.2 技術特色
- **前後端分離**: 獨立的API服務和Web界面
- **即時同步**: 前端操作即時反映到後端
- **多重搜尋**: 支援名稱、編號、模糊搜尋
- **資料整合**: 庫存與詳細資訊整合查詢
- **中文支援**: 完整的繁體中文界面

## 2. 各模組詳細功能

### 2.1 藥物管理模組

#### 2.1.1 基本藥物管理
**檔案位置**: `/api/medicine/`

**功能描述**:
- 管理藥物基本庫存資訊
- 包含藥名、數量、位置、單位、有效期

**資料結構**:
```json
{
  "id": 1,
  "name": "心律錠",
  "amount": 100,
  "position": "A1-01",
  "unit": "錠",
  "expiry_date": "2024-12-31"
}
```

**API端點**:
- `GET /api/medicine/` - 獲取所有基本藥物
- `POST /api/medicine/` - 新增藥物
- `PUT /api/medicine/{id}` - 更新藥物資訊
- `DELETE /api/medicine/{id}` - 刪除藥物

#### 2.1.2 詳細藥物資訊管理
**檔案位置**: `/api/medicine/detailed/`

**功能描述**:
- 管理藥物詳細醫療資訊
- 包含基本資訊、外觀、包裝編號、適應症等

**資料結構**:
```json
{
  "基本資訊": {
    "名稱": "心律錠(Propranolol)",
    "廠商": "生達",
    "劑量": "10毫克"
  },
  "外觀": {
    "顏色": "明紫紅",
    "形狀": "圓扁形"
  },
  "包裝編號": {
    "編號1": "202801",
    "編號2": "TP071014",
    "編號3": "衛署藥製字第009102號"
  },
  "適應症": "狹心症、不整律...",
  "可能的副作用": "常見-心智混亂...",
  "使用說明": "用法用量請遵照醫囑...",
  "注意事項": "1. 本藥會掩飾低血糖症狀..."
}
```

**API端點**:
- `GET /api/medicine/detailed/` - 獲取所有詳細藥物
- `POST /api/medicine/detailed/` - 新增詳細藥物資訊
- `GET /api/medicine/detailed/{name}` - 獲取特定藥物詳細資訊
- `GET /api/medicine/search/detailed/{query}` - 搜尋詳細藥物

#### 2.1.3 整合藥物查詢
**檔案位置**: `/api/medicine/integrated/`

**功能描述**:
- 整合基本庫存和詳細資訊
- 提供完整的藥物資料視圖

**API端點**:
- `GET /api/medicine/integrated/{name}` - 獲取整合藥物資訊
- `GET /api/medicine/integrated/` - 獲取所有整合資訊

#### 2.1.4 包裝編號搜尋
**檔案位置**: `/api/medicine/search/code/`

**功能描述**:
- 根據包裝編號搜尋藥物
- 支援部分匹配和精確匹配

**API端點**:
- `GET /api/medicine/search/code/{code}` - 根據編號搜尋（部分匹配）
- `GET /api/medicine/search/exact-code/{code}` - 精確編號搜尋
- `GET /api/medicine/codes/` - 列出所有藥物編號

### 2.2 病人管理模組

#### 2.2.1 病人資料管理
**檔案位置**: `/api/patients/`

**功能描述**:
- 管理病人基本個人資料
- 包含姓名、年齡、性別、聯絡方式、病史

**資料結構**:
```json
{
  "id": 1,
  "name": "王小明",
  "age": 45,
  "gender": "男",
  "phone": "0912345678",
  "address": "台北市信義區...",
  "medical_history": "高血壓、糖尿病",
  "allergies": "盤尼西林"
}
```

**API端點**:
- `GET /api/patients/` - 獲取所有病人
- `POST /api/patients/` - 新增病人
- `GET /api/patients/{id}` - 獲取特定病人
- `PUT /api/patients/{id}` - 更新病人資訊
- `DELETE /api/patients/{id}` - 刪除病人

### 2.3 病例記錄模組

#### 2.3.1 就診記錄管理
**檔案位置**: `/api/records/`

**功能描述**:
- 管理病人就診記錄
- 包含診斷、處方、醫囑等資訊

**資料結構**:
```json
{
  "id": 1,
  "patient_id": 1,
  "visit_date": "2024-01-15T09:30:00",
  "diagnosis": "高血壓",
  "prescribed_medicines": ["心律錠 10mg", "降血壓藥 5mg"],
  "dosage_instructions": "每日一次，飯後服用",
  "doctor_notes": "建議定期回診"
}
```

**API端點**:
- `GET /api/records/` - 獲取所有病例記錄
- `POST /api/records/` - 新增病例記錄
- `GET /api/records/patient/{id}` - 獲取病人的所有記錄
- `DELETE /api/records/{id}` - 刪除病例記錄

### 2.4 資料導出模組

#### 2.4.1 基本資料導出
**檔案位置**: `/api/export/`

**功能描述**:
- 提供多種格式的資料導出
- 支援完整系統資料導出

**API端點**:
- `GET /api/export/medicines/basic` - 導出基本藥物資訊
- `GET /api/export/medicines/detailed` - 導出詳細藥物資訊
- `GET /api/export/medicines/integrated` - 導出整合藥物資訊
- `GET /api/export/patients` - 導出病人資料
- `GET /api/export/records` - 導出病例記錄
- `GET /api/export/complete` - 導出完整系統資料

#### 2.4.2 整合導出格式
**回傳格式**:
```json
{
  "export_time": "2024-01-15T10:30:00",
  "total_count": 50,
  "data": [...],
  "metadata": {
    "version": "2.0.0",
    "export_type": "integrated"
  }
}
```

## 3. 前端界面功能

### 3.1 藥物管理頁面 (Medicine.html)
**功能特色**:
- 表格式藥物庫存管理
- 即時新增、編輯、刪除
- 搜尋和篩選功能
- JSON資料導出
- 與API即時同步

**主要組件**:
- Handsontable表格元件
- 搜尋輸入框
- 控制按鈕群組
- 訊息提示系統

### 3.2 病人管理頁面 (Patients.html)
**功能特色**:
- 病人資料CRUD操作
- 病人資訊查詢
- 關聯病例記錄檢視

### 3.3 病例記錄頁面 (Records.html)
**功能特色**:
- 病例記錄管理
- 處方藥物關聯
- 病人資料關聯顯示

### 3.4 醫生管理頁面 (doctor.html)
**功能特色**:
- 醫生資訊管理
- 排程和診間分配

### 3.5 處方管理頁面 (Prescription.html)
**功能特色**:
- 處方開立和管理
- 藥物劑量設定
- 用藥指示記錄

## 4. 搜尋查詢功能

### 4.1 藥物搜尋
**支援搜尋方式**:
- 藥物名稱搜尋（模糊匹配）
- 包裝編號搜尋（精確/模糊）
- 廠商名稱搜尋
- 適應症關鍵字搜尋

**搜尋API**:
```
GET /api/medicine/search/detailed/{query}    # 名稱搜尋
GET /api/medicine/search/code/{code}         # 編號搜尋
```

### 4.2 病人搜尋
**支援搜尋方式**:
- 姓名搜尋
- 病歷號搜尋
- 聯絡電話搜尋

### 4.3 病例搜尋
**支援搜尋方式**:
- 診斷關鍵字搜尋
- 處方藥物搜尋
- 就診日期範圍搜尋

## 5. 資料整合功能

### 5.1 藥物資料整合
**整合內容**:
- 基本庫存資訊 + 詳細醫療資訊
- 統一的查詢界面
- 完整的藥物檔案

**整合API示例**:
```json
{
  "medicine_name": "心律錠",
  "basic_info": {
    "amount": 100,
    "position": "A1-01",
    "unit": "錠"
  },
  "detailed_info": {
    "基本資訊": {...},
    "適應症": "...",
    "副作用": "..."
  },
  "status": "available"
}
```

### 5.2 病人病例整合
**整合內容**:
- 病人基本資料 + 所有就診記錄
- 用藥歷史追蹤
- 過敏史和禁忌提醒

## 6. 系統安全與驗證

### 6.1 資料驗證
**輸入驗證**:
- Pydantic模型驗證
- 資料類型檢查
- 必填欄位驗證
- 格式規範檢查

### 6.2 錯誤處理
**錯誤回應**:
- HTTP狀態碼規範
- 詳細錯誤訊息
- 中文錯誤提示
- 日誌記錄

### 6.3 CORS設定
**跨域支援**:
- 允許前端跨域請求
- 安全的跨域政策
- 開發環境配置

## 7. 效能優化功能

### 7.1 記憶體資料庫
**特色**:
- 快速資料存取
- 即時查詢回應
- 適合中小型部署
- 易於開發測試

### 7.2 API快取
**快取策略**:
- 靜態資料快取
- 查詢結果快取
- 減少重複計算

## 8. 開發工具功能

### 8.1 API測試工具
**檔案**: `test_enhanced_system.py`, `quick_test.py`
**功能**:
- 自動化API測試
- 功能完整性驗證
- 效能基準測試

### 8.2 API使用範例
**檔案**: `api_client_examples.py`, `quick_api_guide.py`
**功能**:
- 完整的使用範例
- 快速入門指南
- 程式化調用示例

### 8.3 系統啟動工具
**檔案**: `run_system.py`, `start_server.py`
**功能**:
- 依賴檢查和安裝
- 自動伺服器啟動
- 系統健康檢查

## 9. 資料格式標準

### 9.1 日期時間格式
**標準**: ISO 8601格式
**範例**: `2024-01-15T09:30:00`

### 9.2 API回應格式
**成功回應**:
```json
{
  "status": "success",
  "data": {...},
  "message": "操作成功"
}
```

**錯誤回應**:
```json
{
  "status": "error",
  "detail": "錯誤描述",
  "error_code": "E001"
}
```

### 9.3 匯出資料格式
**標準結構**:
```json
{
  "export_time": "2024-01-15T10:30:00",
  "version": "2.0.0",
  "total_count": 100,
  "data_type": "medicines",
  "data": [...]
}
```

## 10. 擴展性設計

### 10.1 模組化架構
- 獨立的功能模組
- 清晰的介面定義
- 易於新增功能

### 10.2 資料庫抽象
- 可切換資料庫後端
- ORM模型支援
- 遷移腳本準備

### 10.3 API版本控制
- RESTful設計原則
- 向後相容考量
- 版本升級路徑

這個系統提供了完整的醫院藥物管理解決方案，從基本的庫存管理到詳細的醫療資訊，再到病人管理和病例記錄，形成了一個完整的醫療資訊管理生態系統。
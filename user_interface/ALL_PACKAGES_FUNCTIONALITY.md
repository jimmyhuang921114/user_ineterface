# 醫院管理系統 - 所有包功能說明
Hospital Management System - All Packages Functionality Documentation

## 目錄結構總覽
```
user_interface/
├── api/                              # API模組包
├── data/                             # 資料儲存目錄
├── model/                            # 資料模型包
├── route/                            # 路由處理包
├── schemas/                          # 資料驗證架構包
├── services/                         # 業務邏輯服務包
├── static/                           # 靜態資源包
│   ├── css/                          # 樣式文件
│   ├── html/                         # 網頁文件
│   └── js/                           # JavaScript文件
└── [各種功能文件]                    # 主要功能文件
```

---

## 1. API模組包 (api/)

### api/__init__.py
- **功能**: Python包初始化文件
- **內容**: 空文件，標識此目錄為Python包
- **用途**: 允許從api包導入模組

### api/medicine_api.py
- **功能**: 藥物管理API路由模組
- **主要功能**:
  - 基本藥物CRUD操作 (創建、讀取、更新、刪除)
  - 詳細藥物資訊管理
  - 藥物搜索功能 (按名稱、編號)
  - 整合查詢功能 (合併基本和詳細資訊)
- **API端點**:
  - `GET /` - 獲取所有基本藥物
  - `POST /` - 新增基本藥物
  - `PUT /{medicine_id}` - 更新基本藥物
  - `DELETE /{medicine_id}` - 刪除基本藥物
  - `POST /detailed/` - 新增詳細藥物資訊
  - `GET /detailed/{medicine_name}` - 獲取特定藥物詳細資訊
  - `GET /detailed/` - 獲取所有詳細藥物資訊
  - `GET /search/code/{code}` - 按編號搜索藥物
  - `GET /integrated/{medicine_name}` - 獲取整合藥物資訊
- **資料模型**: MedicineBasic, MedicineDetailed
- **儲存**: 記憶體內資料庫 (medicines_db, detailed_medicines_db)

### api/prescription_api.py
- **功能**: 處方管理API路由模組
- **主要功能**:
  - 處方CRUD操作
  - 處方狀態管理
  - 按醫師、狀態篩選處方
  - 處方狀態更新追蹤
- **API端點**:
  - `POST /` - 創建新處方
  - `GET /` - 獲取所有處方
  - `GET /{prescription_id}` - 獲取特定處方
  - `GET /status/{status}` - 按狀態篩選處方
  - `GET /doctor/{doctor_name}` - 按醫師篩選處方
  - `PUT /{prescription_id}/status` - 更新處方狀態
  - `DELETE /{prescription_id}` - 刪除處方
- **資料模型**: PrescriptionCreate, PrescriptionMedicine, PrescriptionStatus
- **儲存**: 記憶體內資料庫 (prescriptions_db, prescription_status_db)

---

## 2. 資料儲存目錄 (data/)

### data/medicines.json
- **功能**: 基本藥物資料持久化儲存
- **格式**: JSON
- **內容**: 
  - medicines: 基本藥物清單
  - next_id: 下一個藥物ID
  - last_updated: 最後更新時間

### data/detailed_medicines.json
- **功能**: 詳細藥物資料持久化儲存
- **格式**: JSON
- **內容**: 詳細藥物資訊字典 (key: 藥物名稱)

### data/prescriptions.json
- **功能**: 處方資料持久化儲存
- **格式**: JSON
- **內容**:
  - prescriptions: 處方清單
  - next_id: 下一個處方ID
  - last_updated: 最後更新時間

### data/prescription_status.json
- **功能**: 處方狀態歷史記錄
- **格式**: JSON
- **內容**: 處方狀態變更歷史記錄

### data/backups/
- **功能**: 資料備份目錄
- **內容**: 自動和手動創建的資料備份
- **格式**: 每個備份為一個子目錄，包含對應時間點的所有JSON文件

---

## 3. 資料模型包 (model/)

### model/medicine_models.py
- **功能**: 藥物相關資料模型定義
- **使用框架**: SQLModel/Pydantic
- **模型定義**: Medicine基礎模型類

### model/prescription_models.py
- **功能**: 處方相關資料模型定義
- **使用框架**: SQLModel/Pydantic
- **模型定義**: Prescription, Patient相關模型類

---

## 4. 路由處理包 (route/)

### route/routes_medicine.py
- **功能**: 藥物相關路由處理邏輯
- **責任**: HTTP請求路由到對應的服務函數

### route/routes_prescription.py
- **功能**: 處方相關路由處理邏輯
- **責任**: HTTP請求路由到對應的服務函數

---

## 5. 資料驗證架構包 (schemas/)

### schemas/medicine_schema.py
- **功能**: 藥物資料驗證架構
- **責任**: 定義藥物資料的輸入/輸出格式驗證規則

### schemas/prescription_schemas.py
- **功能**: 處方資料驗證架構
- **責任**: 定義處方資料的輸入/輸出格式驗證規則

---

## 6. 業務邏輯服務包 (services/)

### services/crud_medicine.py
- **功能**: 藥物CRUD業務邏輯
- **責任**: 實現藥物資料的創建、讀取、更新、刪除邏輯

### services/crud_prescription.py
- **功能**: 處方CRUD業務邏輯
- **責任**: 實現處方資料的創建、讀取、更新、刪除邏輯

---

## 7. 靜態資源包 (static/)

### static/css/
#### unified_style.css
- **功能**: 統一樣式表
- **責任**: 為所有網頁提供一致的視覺風格
- **特色**: 繁體中文支援、現代化設計、響應式佈局

#### medicine.css
- **功能**: 藥物管理頁面專用樣式
- **責任**: 藥物庫存管理介面的特殊樣式定義

#### doctor_style.css
- **功能**: 醫生工作台頁面專用樣式
- **責任**: 醫生介面的特殊樣式定義

#### Prescription.css
- **功能**: 處方管理頁面專用樣式
- **責任**: 處方管理介面的特殊樣式定義

### static/html/
#### doctor.html
- **功能**: 醫生工作台主頁面
- **主要功能**:
  - 基本藥物資訊輸入
  - 詳細藥物資訊輸入 (靈活格式)
  - 處方開立功能
  - 三個分頁介面 (基本藥物、詳細藥物、處方管理)
- **使用技術**: HTML5, Handsontable, Fetch API

#### Medicine.html
- **功能**: 藥物庫存管理頁面
- **主要功能**:
  - 藥物統計顯示
  - 基本藥物清單管理
  - 詳細藥物資訊查看
  - 藥物搜索功能 (按名稱/編號)
  - 資料匯出功能 (CSV/JSON)
- **使用技術**: HTML5, JavaScript, 模態框

#### Prescription.html
- **功能**: 處方管理系統頁面
- **主要功能**:
  - 處方清單顯示
  - 處方狀態管理
  - 處方詳情查看
  - 處方統計
  - 處方狀態更新
- **使用技術**: HTML5, JavaScript, 動態表格

#### background.html
- **功能**: 背景頁面模板
- **責任**: 提供網頁背景佈局基礎

### static/js/
#### doctor.js
- **功能**: 醫生工作台JavaScript邏輯
- **主要功能**:
  - 表單驗證和提交
  - Handsontable初始化和管理
  - 分頁切換邏輯
  - API請求處理
  - 動態內容更新

#### medicine.js
- **功能**: 藥物管理頁面JavaScript邏輯
- **主要功能**:
  - 藥物資料載入和顯示
  - 搜索功能實現
  - 模態框管理
  - 資料匯出功能
  - 統計資訊更新

#### Prescription.js
- **功能**: 處方管理頁面JavaScript邏輯
- **主要功能**:
  - 處方資料載入和渲染
  - 狀態更新邏輯
  - 篩選功能
  - 詳情顯示
  - 統計計算

---

## 8. 主要功能文件

### modular_server.py
- **功能**: 主要FastAPI伺服器應用程式
- **主要功能**:
  - FastAPI應用初始化
  - API路由整合 (medicine_api, prescription_api)
  - 靜態文件服務
  - CORS中間件配置
  - 資料持久化整合
  - 自動儲存機制
  - 系統狀態API
- **啟動方式**: `python3 modular_server.py`
- **端口**: 8000

### data_persistence.py
- **功能**: 資料持久化管理模組
- **主要功能**:
  - JSON文件讀寫
  - 自動資料儲存
  - 備份創建和還原
  - 資料載入和儲存
  - 資料文件資訊查詢
- **特色**: 避免重啟後資料遺失

### start_modular.py
- **功能**: 模組化伺服器啟動腳本
- **主要功能**:
  - 依賴檢查
  - 舊進程清理
  - 伺服器啟動
  - API連接測試
  - 啟動資訊顯示

### test_data_persistence.py
- **功能**: 資料持久化測試腳本
- **主要功能**:
  - 資料持久化功能測試
  - 備份功能測試
  - 系統狀態檢查
  - 完整性驗證

### test_patient_creation.py
- **功能**: 病患記錄創建測試腳本
- **主要功能**:
  - 處方創建測試
  - 病患資訊驗證
  - 處方狀態流程測試
  - 系統報告生成

### ros2_prescription_service.py
- **功能**: ROS2處方服務節點
- **主要功能**:
  - 定期查詢待處理處方
  - 發布處方到ROS2話題
  - 接收處理結果
  - 更新處方狀態
- **ROS2整合**: 與機器人系統通信

### test_ros2_client.py
- **功能**: ROS2處方處理客戶端
- **主要功能**:
  - 監聽處方話題
  - 模擬處方處理
  - 發送處理結果
  - 監控模式
- **模式**: processor (處理器) / monitor (監控器)

### ros2_medicine_client.py
- **功能**: ROS2藥物資料客戶端
- **主要功能**:
  - 藥物資料查詢服務
  - ROS2服務整合
  - 藥物資訊提供

### medicine_ros2_node.py
- **功能**: ROS2藥物節點
- **主要功能**:
  - ROS2藥物服務實現
  - 與API後端通信

---

## 9. 測試和工具文件

### test.sh
- **功能**: 綜合測試腳本
- **主要功能**:
  - 添加詳細藥物資訊
  - 處方系統測試
  - ROS2整合測試
  - 系統功能驗證

### quick_test_system.py
- **功能**: 快速系統測試腳本
- **主要功能**:
  - 基本功能測試
  - API端點測試
  - 快速健康檢查

### remove_all_emojis.py
- **功能**: Emoji移除工具
- **主要功能**:
  - 遞歸掃描所有文件
  - 移除Emoji字符
  - 保持代碼格式
  - 處理統計報告

---

## 10. 文檔文件

### SYSTEM_GUIDE.md
- **功能**: 系統指南
- **內容**: 系統架構、使用方法、部署指南

### USER_GUIDE.md
- **功能**: 使用者指南
- **內容**: 詳細使用說明、API文檔、範例代碼

### packages_usage.md
- **功能**: 包使用說明
- **內容**: 各包的用途和使用順序說明

### system_architecture.md
- **功能**: 系統架構文檔
- **內容**: 架構設計、模組關係、技術棧說明

### QUICK_START.md
- **功能**: 快速開始指南
- **內容**: 快速部署和使用說明

---

## 11. 配置和資料文件

### MedicineService.srv
- **功能**: ROS2服務定義文件
- **內容**: 藥物服務的輸入輸出格式定義

### medicine_data_for_ros2.json
- **功能**: ROS2系統用藥物資料
- **內容**: 格式化的藥物資料，供ROS2系統使用

### test_prescription.json
- **功能**: 測試處方資料
- **內容**: 用於測試的範例處方數據

---

## 12. 其他伺服器文件 (備用)

### fixed_server.py
- **功能**: 修復版本伺服器 (備用)
- **用途**: 整合版本的FastAPI伺服器

### enhanced_server.py
- **功能**: 增強版本伺服器 (備用)
- **用途**: 功能擴展的FastAPI伺服器

### simple_prescription_server.py
- **功能**: 簡單處方伺服器 (備用)
- **用途**: 獨立的處方管理伺服器

### prescription_server.py
- **功能**: 處方伺服器 (備用)
- **用途**: 專門的處方管理伺服器

---

## 使用順序建議

1. **啟動系統**: `python3 start_modular.py`
2. **訪問介面**:
   - 醫生工作台: http://localhost:8000/doctor.html
   - 藥物管理: http://localhost:8000/Medicine.html
   - 處方管理: http://localhost:8000/Prescription.html
3. **API文檔**: http://localhost:8000/docs
4. **ROS2整合**: 運行ROS2節點進行機器人系統整合

## 系統特色

- **模組化設計**: API、前端、資料分離
- **資料持久化**: JSON文件儲存，避免重啟資料遺失
- **自動備份**: 定期自動儲存和備份機制
- **ROS2整合**: 支援機器人系統通信
- **統一風格**: 繁體中文介面，一致的視覺設計
- **完整測試**: 全面的測試腳本和驗證工具
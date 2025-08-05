# 🏥 醫院藥物管理系統
## Hospital Medicine Management System

一個現代化的醫院藥物管理系統，提供完整的藥物庫存管理、處方籤管理和醫生操作介面。

## 📋 目錄
- [功能特色](#功能特色)
- [系統架構](#系統架構)
- [安裝指南](#安裝指南)
- [使用說明](#使用說明)
- [API文檔](#api文檔)
- [專案結構](#專案結構)
- [開發指南](#開發指南)
- [故障排除](#故障排除)

## ✨ 功能特色

### 🔧 核心功能
- **統一藥物管理**: 在單一表單中填寫基本和詳細藥物資料，無需換頁
- **智能處方籤系統**: 自動生成病患編號、處方時間，美化藥物選擇介面
- **庫存管理**: 完整的增刪改查功能，即時庫存調整
- **身份證號驗證**: 自動驗證並生成唯一病患編號
- **下拉式藥物選擇**: 美觀的藥物選擇介面，顯示庫存狀態
- **YAML數據匯出**: 自動將數據匯出為ROS2兼容的YAML格式
- **即時更新**: WebSocket支援的即時數據同步
- **響應式設計**: 支援桌面和移動設備

### 🤖 ROS2整合
- **訂單服務**: ROS2服務獲取處方籤訂單並生成YAML輸出
- **基本藥物提供者**: 自動發布基本藥物庫存資訊
- **詳細藥物提供者**: 提供完整藥物詳細資訊和安全等級
- **實時數據同步**: 定時更新和即時服務響應

### 🛡️ 技術特色
- **FastAPI後端**: 高效能的Python Web框架
- **現代化前端**: HTML5、CSS3、JavaScript ES6+
- **RESTful API**: 標準化的API設計
- **雙重數據存儲**: JSON + YAML格式並行存儲
- **ROS2節點架構**: 完整的機器人系統整合
- **CORS支援**: 跨域資源共享

## 🏗️ 系統架構

```
醫院藥物管理系統
├── 前端界面 (Frontend)
│   ├── 藥物管理頁面 (Medicine.html)
│   ├── 處方籤管理 (Prescription.html)
│   └── 醫生介面 (doctor.html)
├── 後端API (Backend)
│   ├── FastAPI應用 (fixed_server.py)
│   ├── 數據模型 (Pydantic Models)
│   └── WebSocket支援
└── 數據層 (Data Layer)
    ├── 藥物基本資料 (medicines.json)
    ├── 藥物詳細資料 (detailed_medicines.json)
    ├── 處方籤資料 (prescriptions.json)
    └── 處方籤狀態 (prescription_status.json)
```

## 🚀 安裝指南

### 前置需求
- Python 3.8+
- pip (Python套件管理器)

### 快速開始

1. **克隆專案**
```bash
git clone <repository-url>
cd <project-directory>
```

2. **建立虛擬環境** (建議)
```bash
python -m venv venv
source venv/bin/activate  # Linux/Mac
# 或
venv\Scripts\activate     # Windows
```

3. **安裝依賴**
```bash
pip install -r requirements.txt
```

4. **啟動系統**
```bash
cd user_interface
python main.py
```

5. **啟動整合系統** (建議)
```bash
python start_ros2_medicine_system.py
```

**或單獨啟動Web服務**
```bash
cd user_interface
python3 main.py
```

6. **訪問系統**
- 主頁面: http://localhost:8000
- **醫生工作台**: http://localhost:8000/doctor.html ⭐ 全新設計
- **整合藥物管理**: http://localhost:8000/integrated_medicine_management.html ⭐ 四合一介面
- 統一藥物管理: http://localhost:8000/unified_medicine.html
- 藥物管理: http://localhost:8000/Medicine.html
- 處方籤管理: http://localhost:8000/Prescription.html
- API文檔: http://localhost:8000/docs

## 📖 使用說明

### 🏥 醫生工作台 (主要介面) ⭐ 推薦使用
**訪問地址**: `http://localhost:8000/doctor.html`

#### 💊 統一藥物管理標籤
- **同頁表單**: 基礎資料 + 詳細資料在同一頁面，無需換頁
- **必填欄位**: 藥物名稱、初始數量、儲存位置
- **選填欄位**: 詳細資料包含成分、分類、描述、副作用等
- **自動功能**: 保存後自動同步YAML和更新ROS2節點

#### 📋 智能處方籤標籤
- **病患資訊**:
  - 病患姓名 (必填)
  - 身份證號 (必填，10位數，自動驗證)
  - 病患編號 (自動生成，格式: P+身份證後4位+時間戳)
  - 處方時間 (自動生成並每分鐘更新)

- **處方用藥**:
  - 下拉式藥物選擇 (顯示庫存狀態)
  - 庫存警告 (少於10時紅色標示)
  - 自動填入建議劑量和天數
  - 支援多種藥物 (可動態新增/移除)
  - 必填: 藥物名稱、劑量、頻率、天數

- **已移除功能**:
  - ❌ 診斷結果欄位
  - ❌ 醫生名稱輸入 (自動設為"系統醫生")

### 🔧 整合藥物管理
**訪問地址**: `http://localhost:8000/integrated_medicine_management.html`

#### 四合一管理介面:
1. **➕ 新增藥物**: 完整的藥物資料填寫
2. **📦 庫存管理**: 增加/減少藥物數量，搜尋篩選
3. **📋 藥物清單**: 查看、編輯、刪除藥物
4. **🩺 病例管理**: 處方籤查看、編輯、刪除

### 📊 傳統介面 (保留相容性)
- **藥物管理**: `Medicine.html` - 原始藥物管理介面
- **處方籤管理**: `Prescription.html` - 處方籤列表和狀態管理
- **統一表單**: `unified_medicine.html` - 簡化版藥物表單

## 🔗 API文檔

### 主要端點

#### 💊 藥物管理 API
- `GET /api/medicine/` - 獲取所有藥物 (基本+詳細)
- `GET /api/medicine/basic` - 獲取基本藥物資料
- `GET /api/medicine/detailed` - 獲取詳細藥物資料
- `POST /api/medicine/unified` - **統一新增藥物** (基本+詳細)
- `PUT /api/medicine/{name}` - 更新藥物資料
- `DELETE /api/medicine/{name}` - 刪除藥物
- `POST /api/medicine/adjust-stock` - **庫存調整** (增加/減少)

#### 📋 處方籤管理 API
- `GET /api/prescription/` - 獲取所有處方籤
- `POST /api/prescription/` - **創建新處方籤** (含自動編號)
- `GET /api/prescription/{id}` - 獲取特定處方籤
- `DELETE /api/prescription/{id}` - 刪除處方籤

#### 📄 YAML匯出 API
- `GET /api/export/yaml/sync` - 同步JSON到YAML並匯出ROS2格式
- `GET /api/medicine/yaml/basic` - 獲取基本藥物YAML格式
- `GET /api/medicine/yaml/detailed` - 獲取詳細藥物YAML格式

#### 🤖 ROS2整合 API
- `GET /api/ros2/prescription` - ROS2格式的處方籤資料

#### 🔄 WebSocket
- `ws://localhost:8000/ws` - 即時數據更新
  - 藥物新增/更新/刪除通知
  - 庫存調整通知
  - 處方籤狀態變更通知

完整的API文檔請訪問: http://localhost:8000/docs

## 📁 專案結構

```
醫院藥物管理系統/
├── user_interface/           # 主要應用程式目錄
│   ├── main.py              # 應用程式入口點
│   ├── fixed_server.py      # FastAPI後端伺服器
│   ├── data/                # 數據檔案目錄
│   │   ├── medicines.json           # 藥物基本資料
│   │   ├── detailed_medicines.json  # 藥物詳細資料
│   │   ├── prescriptions.json       # 處方籤資料
│   │   └── prescription_status.json # 處方籤狀態
│   └── static/              # 靜態資源目錄
│       ├── html/            # HTML頁面
│       │   ├── Medicine.html        # 藥物管理頁面
│       │   ├── Prescription.html    # 處方籤管理頁面
│       │   └── doctor.html          # 醫生介面
│       ├── css/             # 樣式檔案
│       │   └── unified_style.css    # 統一樣式表
│       └── js/              # JavaScript檔案
│           ├── medicine.js          # 藥物管理腳本
│           ├── doctor.js            # 醫生介面腳本
│           └── Prescription.js      # 處方籤管理腳本
├── requirements.txt         # Python依賴清單
└── README.md               # 專案說明文檔
```

## 🛠️ 開發指南

### 技術棧
- **後端**: FastAPI、Python 3.8+、Uvicorn
- **前端**: HTML5、CSS3、JavaScript ES6+
- **數據**: JSON檔案儲存
- **通訊**: RESTful API、WebSocket

### 開發環境設置
1. 遵循安裝指南設置基本環境
2. 啟用開發模式 (自動重載):
```bash
uvicorn fixed_server:app --reload --host 0.0.0.0 --port 8000
```

### 代碼規範
- 使用Python PEP 8編碼規範
- JavaScript使用ES6+語法
- CSS使用模組化設計
- 所有API端點都有適當的錯誤處理

### 新增功能
1. 在 `fixed_server.py` 中新增API端點
2. 更新前端頁面和JavaScript
3. 測試所有功能
4. 更新API文檔

## 🐛 故障排除

### 常見問題

**Q: 啟動時顯示端口被佔用**
A: 請檢查8000端口是否被其他應用使用，或修改 `main.py` 中的端口設定

**Q: 前端頁面顯示404錯誤**
A: 確保靜態檔案路徑正確，檢查 `static` 目錄結構

**Q: API請求失敗**
A: 檢查CORS設定，確保前端和後端在同一域名或已正確配置跨域

**Q: 數據丟失**
A: 檢查 `data/` 目錄中的JSON檔案是否存在且格式正確

### 日誌和除錯
- 啟動時會顯示詳細的系統資訊
- API錯誤會記錄在控制台
- 檢查瀏覽器開發者工具的網路和控制台標籤

### 獲取幫助
如果遇到問題，請：
1. 檢查控制台輸出
2. 查看API文檔 (http://localhost:8000/docs)
3. 確認所有依賴已正確安裝
4. 檢查網路連接和防火牆設定

## 📝 更新記錄

### v2.0.0 (目前版本) ⭐ 重大更新
- **全新醫生工作台**: 統一藥物表單 + 智能處方籤系統
- **自動化功能**: 病患編號生成、處方時間、庫存警告
- **美化介面**: 下拉式藥物選擇、響應式設計
- **整合管理**: 四合一藥物管理介面 (新增、庫存、清單、病例)
- **CRUD完整性**: 藥物和處方籤的完整增刪改查
- **ROS2深度整合**: 三個專用ROS2節點 + YAML自動匯出
- **身份證號驗證**: 台灣身份證號格式驗證和編號生成

### v1.0.0 (舊版本)
- 基礎藥物管理系統
- 簡單處方籤功能
- 基本醫生介面
- WebSocket即時更新
- RESTful API

---

## 🤝 貢獻

歡迎提交Issue和Pull Request來改善這個專案！

## 📄 授權

請參考專案授權條款。

---

**開發團隊** | **最後更新**: 2024年12月
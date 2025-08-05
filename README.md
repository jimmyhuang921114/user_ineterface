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
- **藥物管理**: 完整的藥物庫存追蹤和管理
- **處方籤系統**: 數位化處方籤創建和管理
- **醫生介面**: 專門為醫生設計的操作介面
- **即時更新**: WebSocket支援的即時數據同步
- **響應式設計**: 支援桌面和移動設備

### 🛡️ 技術特色
- **FastAPI後端**: 高效能的Python Web框架
- **現代化前端**: HTML5、CSS3、JavaScript ES6+
- **RESTful API**: 標準化的API設計
- **JSON數據存儲**: 輕量級數據持久化
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

5. **訪問系統**
- 主頁面: http://localhost:8000
- 藥物管理: http://localhost:8000/Medicine.html
- 處方籤管理: http://localhost:8000/Prescription.html
- 醫生介面: http://localhost:8000/doctor.html
- API文檔: http://localhost:8000/docs

## 📖 使用說明

### 藥物管理
1. 訪問 `Medicine.html` 頁面
2. 可以新增、查看、編輯和刪除藥物資料
3. 支援基本資料和詳細資料管理
4. 即時庫存狀態顯示

### 處方籤管理
1. 訪問 `Prescription.html` 頁面
2. 創建新的處方籤
3. 查看和管理現有處方籤
4. 追蹤處方籤狀態

### 醫生介面
1. 訪問 `doctor.html` 頁面
2. 專門為醫生設計的簡化介面
3. 快速處方籤創建
4. 病人資料管理

## 🔗 API文檔

### 主要端點

#### 藥物管理 API
- `GET /api/medicine/` - 獲取所有藥物
- `POST /api/medicine/` - 新增藥物
- `PUT /api/medicine/{name}` - 更新藥物資料
- `DELETE /api/medicine/{name}` - 刪除藥物

#### 處方籤管理 API
- `GET /api/prescription/` - 獲取所有處方籤
- `POST /api/prescription/` - 創建新處方籤
- `GET /api/prescription/{id}` - 獲取特定處方籤
- `PUT /api/prescription/{id}` - 更新處方籤

#### WebSocket
- `ws://localhost:8000/ws` - 即時數據更新

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

### v1.0.0 (目前版本)
- 完整的藥物管理系統
- 處方籤管理功能
- 醫生專用介面
- WebSocket即時更新
- 響應式設計
- RESTful API

---

## 🤝 貢獻

歡迎提交Issue和Pull Request來改善這個專案！

## 📄 授權

請參考專案授權條款。

---

**開發團隊** | **最後更新**: 2024年12月
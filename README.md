# 簡化醫院藥物管理系統 (Simple Hospital Medicine Management System)

## 系統概述 (System Overview)

這是一個簡化的醫院藥物管理系統，專注於核心功能：藥物管理和處方籤管理。系統使用 FastAPI 和 SQLite 資料庫，提供穩定、簡潔的解決方案。

This is a simplified hospital medicine management system focusing on core functions: medicine management and prescription management. The system uses FastAPI and SQLite database to provide a stable, clean solution.

## 主要功能 (Main Features)

- ✅ **藥物基本資料管理** - 新增、查詢、修改、刪除藥物資料
- ✅ **處方籤管理** - 創建和查詢處方籤
- ✅ **Web API** - RESTful API 接口
- ✅ **網頁界面** - 直觀的使用者介面
- ✅ **資料庫整合** - SQLite 資料庫存儲

## 快速開始 (Quick Start)

### 1. 安裝依賴 (Install Dependencies)

```bash
pip install --break-system-packages fastapi uvicorn sqlalchemy pydantic pyyaml
```

或使用 requirements 文件：
```bash
pip install --break-system-packages -r requirements_simple.txt
```

### 2. 啟動系統 (Start System)

**方式一 - 使用簡單啟動腳本：**
```bash
python3 start_simple.py
```

**方式二 - 直接啟動：**
```bash
cd user_interface
python3 simple_server.py
```

**方式三 - 使用 main.py：**
```bash
cd user_interface
python3 main.py
```

### 3. 訪問系統 (Access System)

- 🌐 **藥物管理頁面**: http://localhost:8000/Medicine.html
- 📋 **處方籤管理頁面**: http://localhost:8000/Prescription.html
- 👨‍⚕️ **醫生界面**: http://localhost:8000/doctor.html
- 📖 **API 文檔**: http://localhost:8000/docs
- ⚡ **健康檢查**: http://localhost:8000/api/health

## API 接口 (API Endpoints)

### 健康檢查 (Health Check)
- `GET /api/health` - 系統健康狀態

### 藥物管理 (Medicine Management)
- `GET /api/medicine/basic` - 獲取所有藥物列表
- `POST /api/medicine/basic` - 新增藥物
- `GET /api/medicine/basic/{id}` - 獲取特定藥物
- `PUT /api/medicine/basic/{id}` - 更新藥物資料
- `DELETE /api/medicine/basic/{id}` - 刪除藥物

### 處方籤管理 (Prescription Management)
- `GET /api/prescription/` - 獲取所有處方籤
- `POST /api/prescription/` - 新增處方籤
- `GET /api/prescription/{id}` - 獲取特定處方籤

## 系統結構 (System Structure)

```
├── start_simple.py              # 簡單啟動腳本
├── requirements_simple.txt      # 簡化依賴列表
├── README.md                   # 系統說明文件
└── user_interface/
    ├── simple_server.py        # 簡化伺服器主檔案
    ├── main.py                 # 主要入口點
    ├── database.py             # 資料庫模型和配置
    ├── data/
    │   └── hospital_management.db  # SQLite 資料庫
    └── static/                 # 網頁前端檔案
        ├── Medicine.html
        ├── Prescription.html
        └── doctor.html
```

## 資料庫結構 (Database Structure)

系統使用 SQLite 資料庫，包含以下主要資料表：

- **medicine_basic** - 基本藥物資料表
- **medicine_detailed** - 詳細藥物資料表
- **prescriptions** - 處方籤主表
- **prescription_medicines** - 處方籤藥物明細表
- **stock_logs** - 庫存異動記錄
- **system_logs** - 系統操作記錄

## 技術規格 (Technical Specifications)

- **後端框架**: FastAPI 0.116.1
- **資料庫**: SQLite (透過 SQLAlchemy ORM)
- **API 文檔**: 自動生成的 OpenAPI/Swagger 文檔
- **前端**: HTML + JavaScript
- **Python 版本**: 3.7+

## 系統特色 (System Features)

✅ **簡潔穩定** - 移除複雜功能，專注核心需求  
✅ **易於部署** - 單一資料庫文件，無外部依賴  
✅ **API 優先** - 完整的 RESTful API 接口  
✅ **自動文檔** - 內建 API 文檔生成  
✅ **錯誤處理** - 完善的錯誤處理機制  

## 故障排除 (Troubleshooting)

### 常見問題 (Common Issues)

1. **資料庫連接錯誤**
   ```bash
   cd user_interface
   python3 database.py  # 重新初始化資料庫
   ```

2. **端口被占用**
   - 更改 `simple_server.py` 中的端口號
   - 或停止占用 8000 端口的其他程序

3. **依賴安裝失敗**
   ```bash
   pip install --break-system-packages -r requirements_simple.txt
   ```

## 版本資訊 (Version Info)

- **系統版本**: 1.0.0 (Simplified)
- **最後更新**: 2025-08-06
- **相容性**: Linux, macOS, Windows
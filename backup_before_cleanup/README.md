# 醫院藥物管理系統

一個簡化的醫院藥物和處方籤管理系統。

## 功能特色

- 📦 藥物庫存管理
- 📋 處方籤管理
- 👨‍⚕️ 醫生工作站
- 🔍 藥物搜尋
- 📊 庫存統計

## 快速開始

### 1. 安裝依賴

```bash
pip install -r requirements_simple.txt
```

### 2. 啟動系統

```bash
cd user_interface
python3 main.py
```

### 3. 訪問系統

- 🌐 藥物管理: http://localhost:8000/Medicine.html
- 📋 處方籤管理: http://localhost:8000/Prescription.html
- 👨‍⚕️ 醫生工作站: http://localhost:8000/doctor.html
- 📖 API文檔: http://localhost:8000/docs

## 系統架構

- **後端**: FastAPI + SQLAlchemy
- **資料庫**: SQLite
- **前端**: HTML + JavaScript

## 資料庫初始化

系統會自動創建資料庫和示例資料。如需重新初始化：

```bash
cd user_interface
python3 database.py
```

## 停止系統

按 `Ctrl+C` 停止伺服器。
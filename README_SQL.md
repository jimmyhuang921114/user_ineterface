# 🏥 醫院藥物管理系統 - SQL資料庫版本

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.100+-green.svg)](https://fastapi.tiangolo.com/)
[![SQLAlchemy](https://img.shields.io/badge/SQLAlchemy-2.0+-red.svg)](https://www.sqlalchemy.org/)
[![SQLite](https://img.shields.io/badge/SQLite-3.0+-lightgrey.svg)](https://sqlite.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

專為醫院和診所設計的現代化藥物管理系統，採用 **SQL資料庫** 架構，提供完整的CRUD操作、資料關聯性、交易安全性和效能優化。

---

## ✨ 新功能特色 (SQL版本)

### 🗄️ 強大的資料庫功能
- **關聯式資料庫**: SQLite/PostgreSQL/MySQL支援
- **資料完整性**: Foreign Key約束和事務安全
- **查詢效能**: 索引優化和複雜查詢支援
- **資料備份**: 完整的資料庫備份和還原
- **歷史記錄**: 完整的操作日誌和審計追蹤
- **軟刪除**: 資料標記刪除，可恢復

### 💡 智能功能
- **🤖 AI提示詞**: 每種藥物可設定AI智能推薦提示
- **📊 統計分析**: 庫存趨勢、使用統計、處方分析
- **⚠️ 智能警告**: 庫存不足、過期提醒、重複用藥
- **🔍 高級搜尋**: 多條件組合查詢、模糊搜尋
- **📈 報表生成**: 自動生成各種管理報表

### 🔒 企業級安全
- **操作記錄**: 每個操作都有完整的系統日誌
- **IP追蹤**: 記錄操作來源IP和瀏覽器資訊
- **資料驗證**: 嚴格的資料驗證和約束
- **備份機制**: 自動資料庫備份功能

---

## 🏗️ 系統架構

### 📊 資料庫設計
```sql
🗄️ 資料庫結構
├── medicine_basic          # 基本藥物資料表
│   ├── id (PK)            # 主鍵
│   ├── name (UNIQUE)      # 藥物名稱
│   ├── amount             # 庫存數量
│   ├── position           # 儲存位置
│   ├── prompt            # AI提示詞 ⭐ 新功能
│   └── is_active         # 軟刪除標記
│
├── medicine_detailed       # 詳細藥物資料表
│   ├── id (PK)
│   ├── medicine_id (FK)   # 關聯基本藥物
│   ├── description        # 詳細描述
│   ├── side_effects      # 副作用
│   └── storage_conditions # 儲存條件
│
├── prescriptions          # 處方籤主表
│   ├── id (PK)
│   ├── patient_name       # 病患姓名
│   ├── patient_id        # 病患編號
│   ├── doctor_name       # 醫生姓名
│   └── status            # 處方狀態
│
├── prescription_medicines  # 處方藥物明細表
│   ├── prescription_id (FK)
│   ├── medicine_id (FK)
│   ├── dosage            # 劑量
│   ├── frequency         # 頻率
│   └── duration          # 療程
│
├── stock_logs             # 庫存異動記錄表
│   ├── medicine_id (FK)
│   ├── action            # 操作類型
│   ├── quantity_before   # 異動前數量
│   ├── quantity_change   # 異動數量
│   └── quantity_after    # 異動後數量
│
└── system_logs            # 系統操作記錄表
    ├── action            # 操作類型
    ├── table_name        # 資料表名稱
    ├── ip_address        # 操作IP
    └── created_at        # 操作時間
```

### 🔄 技術架構
```
┌─────────────────────────────────────────┐
│              前端界面                      │
│  HTML + CSS + JavaScript + WebSocket    │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│              FastAPI                    │
│        RESTful API + WebSocket          │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│            SQLAlchemy ORM               │
│        資料庫抽象層 + 查詢優化            │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│         SQLite Database                 │
│      關聯式資料庫 + 事務管理             │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│            YAML Export                  │
│        ROS2整合 + 資料匯出              │
└─────────────────────────────────────────┘
```

---

## 🚀 快速開始

### 📋 系統需求
- **Python**: 3.8 或更高版本
- **作業系統**: Windows, macOS, Linux
- **記憶體**: 最少 1GB RAM
- **硬碟**: 最少 500MB 可用空間

### ⚡ 安裝步驟

1. **下載專案**
```bash
git clone https://github.com/jimmyhuang921114/user_ineterface.git
cd user_ineterface
git checkout feature/sql-database
```

2. **安裝SQL版本依賴**
```bash
pip install -r requirements_sql.txt
```

3. **初始化資料庫**
```bash
cd user_interface
python database.py
```

4. **啟動SQL版本系統**
```bash
python sql_server.py
```

5. **訪問系統**
- 主頁面: http://localhost:8000
- **醫生工作台**: http://localhost:8000/doctor.html ⭐ 推薦
- **整合管理**: http://localhost:8000/integrated_medicine_management.html
- API文檔: http://localhost:8000/docs
- 資料庫檔案: `./data/hospital_management.db`

---

## 📖 功能說明

### 🏥 醫生工作台 (全新設計)
**訪問地址**: `http://localhost:8000/doctor.html`

#### 💊 統一藥物管理
- **同頁表單**: 基礎+詳細資料在同一頁面
- **AI提示詞**: 🤖 為每種藥物設定智能推薦提示
- **自動關聯**: 基本和詳細資料自動建立關聯
- **即時驗證**: 表單資料即時驗證和提示

#### 📋 智能處方籤系統
- **自動編號**: 根據身份證號自動生成病患編號
- **即時庫存**: 顯示藥物即時庫存和AI提示
- **智能建議**: 根據AI提示詞提供用藥建議
- **歷史追蹤**: 完整的處方開立歷史記錄

### 🔧 整合管理系統
**訪問地址**: `http://localhost:8000/integrated_medicine_management.html`

#### 四合一管理介面:
1. **➕ 新增藥物**: 完整的統一表單
2. **📦 庫存管理**: 增加/減少數量，操作記錄
3. **📋 藥物清單**: 查看、編輯、軟刪除
4. **🩺 處方管理**: 查看、編輯、取消處方

### 📊 進階功能

#### 🔍 高級搜尋
- **多條件搜尋**: 名稱、分類、製造商等
- **模糊查詢**: 支援部分關鍵字搜尋
- **範圍搜尋**: 數量、日期範圍搜尋
- **組合查詢**: 多個條件組合搜尋

#### 📈 統計分析
- **庫存統計**: 即時庫存統計和趨勢
- **使用分析**: 藥物使用頻率分析
- **處方統計**: 醫生處方統計
- **警告提醒**: 庫存不足、過期提醒

---

## 🔗 API文檔 (SQL版本)

### 💊 藥物管理 API
```http
GET    /api/medicine/basic          # 獲取所有基本藥物
POST   /api/medicine/basic          # 創建基本藥物
GET    /api/medicine/basic/{id}     # 獲取特定藥物
PUT    /api/medicine/basic/{id}     # 更新藥物資料
DELETE /api/medicine/{name}         # 軟刪除藥物

GET    /api/medicine/detailed       # 獲取所有詳細藥物
POST   /api/medicine/detailed       # 創建詳細資料
POST   /api/medicine/unified        # 統一創建(基本+詳細)
POST   /api/medicine/adjust-stock   # 庫存調整
```

### 📋 處方籤管理 API
```http
GET    /api/prescription/           # 獲取所有處方籤
POST   /api/prescription/           # 創建新處方籤
DELETE /api/prescription/{id}       # 取消處方籤
```

### 🤖 ROS2整合 API
```http
GET    /api/ros2/prescription       # ROS2格式處方資料
POST   /api/export/yaml/sync        # 手動YAML同步
```

### 📊 系統管理 API
```http
GET    /api/logs/system            # 系統操作記錄
GET    /api/logs/stock             # 庫存異動記錄
GET    /api/stats/medicines        # 藥物統計
GET    /api/stats/prescriptions    # 處方統計
```

---

## 🗂️ 專案結構 (SQL版本)

```
醫院藥物管理系統-SQL版本/
├── user_interface/
│   ├── sql_server.py              # SQL版本主伺服器 ⭐
│   ├── database.py                # 資料庫模型定義 ⭐
│   ├── yaml_storage.py            # YAML匯出模組
│   ├── data/
│   │   ├── hospital_management.db # SQLite資料庫 ⭐
│   │   ├── basic_medicines.yaml   # YAML匯出檔案
│   │   └── detailed_medicines.yaml
│   └── static/
│       ├── html/                  # 前端頁面
│       ├── css/                   # 樣式表
│       └── js/                    # JavaScript邏輯
├── ros2_packages/                 # ROS2整合套件
├── requirements_sql.txt           # SQL版本依賴 ⭐
├── README_SQL.md                  # SQL版本說明 ⭐
└── alembic.ini                    # 資料庫遷移設定
```

---

## 🔄 資料庫遷移

### 📦 Alembic設定
```bash
# 初始化遷移環境
alembic init alembic

# 生成遷移腳本
alembic revision --autogenerate -m "Add prompt field to medicines"

# 執行遷移
alembic upgrade head

# 回退遷移
alembic downgrade -1
```

### 💾 資料備份與還原
```bash
# 備份資料庫
cp data/hospital_management.db data/backup_$(date +%Y%m%d_%H%M%S).db

# 還原資料庫
cp data/backup_20241212_143000.db data/hospital_management.db
```

---

## 🆚 版本比較

| 功能特色 | JSON版本 | SQL版本 ⭐ |
|---------|----------|-----------|
| **資料儲存** | JSON檔案 | SQLite資料庫 |
| **資料關聯** | 手動維護 | 自動Foreign Key |
| **查詢效能** | 線性搜尋 | 索引優化查詢 |
| **交易安全** | ❌ | ✅ ACID特性 |
| **歷史記錄** | 基本 | 完整操作日誌 |
| **備份還原** | 檔案複製 | 專業資料庫工具 |
| **擴展性** | 有限 | 高度可擴展 |
| **複雜查詢** | ❌ | ✅ SQL強大查詢 |
| **資料完整性** | 手動檢查 | 約束自動檢查 |
| **並發處理** | 有限 | 完整並發支援 |

---

## 🔧 開發指南

### 🧪 測試
```bash
# 運行所有測試
pytest

# 運行特定測試
pytest tests/test_database.py

# 測試覆蓋率
pytest --cov=user_interface
```

### 🐛 除錯
```bash
# 開啟SQL查詢日誌
export SQLALCHEMY_ECHO=True

# 檢查資料庫狀態
python -c "from database import init_database; init_database()"
```

### 📈 效能優化
```python
# 查詢優化範例
from sqlalchemy import text

# 使用索引
medicines = session.query(MedicineBasic).filter(
    MedicineBasic.name.like('%aspirin%')
).all()

# 預載入關聯資料
medicines = session.query(MedicineBasic).options(
    joinedload(MedicineBasic.detailed_info)
).all()
```

---

## 📝 更新記錄

### v2.1.0 (SQL版本) ⭐ 當前版本
- **🗄️ 完整SQL資料庫**: SQLite/PostgreSQL/MySQL支援
- **🤖 AI提示詞功能**: 每種藥物可設定AI推薦提示
- **📊 歷史記錄追蹤**: 完整的操作和庫存異動記錄
- **🔒 企業級安全**: IP追蹤、操作日誌、資料驗證
- **⚡ 效能優化**: 索引優化、查詢快取、並發處理
- **🔍 高級搜尋**: 多條件組合查詢、模糊搜尋
- **📈 統計分析**: 庫存趨勢、使用統計、報表生成
- **💾 專業備份**: 資料庫備份還原、遷移工具

### v2.0.0 (JSON版本)
- 基礎JSON檔案儲存
- 統一藥物表單
- ROS2整合
- WebSocket即時更新

---

## 🤝 貢獻指南

我們歡迎各種形式的貢獻！

### 🐛 報告問題
- 使用GitHub Issues報告bug
- 提供詳細的錯誤訊息和重現步驟

### 💡 功能建議
- 在Issues中提出功能請求
- 說明使用場景和預期效果

### 🔧 程式碼貢獻
1. Fork此專案
2. 創建功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交變更 (`git commit -m 'Add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 開啟Pull Request

---

## 📞 支援與聯絡

- **GitHub**: [專案頁面](https://github.com/jimmyhuang921114/user_ineterface)
- **Issues**: [問題回報](https://github.com/jimmyhuang921114/user_ineterface/issues)
- **分支**: `feature/sql-database` (SQL版本)
- **主分支**: `cursor/git-afed` (JSON版本)

---

## 📄 授權條款

本專案採用 MIT 授權條款 - 詳見 [LICENSE](LICENSE) 檔案

---

**🏥 醫院藥物管理系統 - SQL版本，為現代醫療提供專業的數位化解決方案！**
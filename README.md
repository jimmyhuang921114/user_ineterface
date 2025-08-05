# 🏥 醫院藥物管理系統

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.100+-green.svg)](https://fastapi.tiangolo.com/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble+-orange.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

專為醫院和診所設計的現代化藥物管理系統，支援**JSON**和**SQL**雙版本資料庫，提供完整的藥物庫存、處方籤管理、ROS2整合和即時數據監控功能。

---

## ✨ 主要功能

### 🎯 核心系統
- **👨‍⚕️ 醫生工作台**: 專業的處方籤開立系統
- **🔧 整合管理**: 全功能的藥物庫存和處方管理
- **📋 處方籤管理**: 完整的處方記錄查看和管理
- **🔄 統一表單**: 快速一次性藥物資料新增

### 💡 智能功能
- **🤖 AI提示詞**: 每種藥物可設定智能推薦提示
- **📊 自動編號**: 根據身份證號自動生成病患編號
- **⏰ 即時更新**: 自動生成處方時間和數據同步
- **🔍 智能選擇**: 下拉式藥物選擇，顯示庫存和AI提示

### 🚀 技術特色
- **📡 ROS2整合**: 完整的服務和主題支援
- **💾 雙資料庫**: JSON輕量級 + SQL企業級選擇
- **🌐 即時通訊**: WebSocket支援的即時數據同步
- **📱 響應式設計**: 支援桌面和移動設備

---

## 🌐 系統訪問

### 📍 主要頁面
| 頁面 | 網址 | 功能描述 |
|------|------|----------|
| **👨‍⚕️ 醫生工作台** | http://localhost:8000/doctor.html | 專業處方籤開立系統 |
| **🔧 整合管理** | http://localhost:8000/integrated_medicine_management.html | 全功能管理介面 |
| **📋 處方籤管理** | http://localhost:8000/Prescription.html | 處方記錄查看管理 |
| **🔄 統一表單** | http://localhost:8000/unified_medicine.html | 快速新增藥物資料 |

### 🔗 API和文檔
- **📖 API文檔**: http://localhost:8000/docs
- **💾 JSON版本**: 預設資料夾系統
- **🗄️ SQL版本**: `feature/sql-database` 分支

---

## 🚀 快速開始

### 📋 系統需求
- **Python**: 3.8 或更高版本
- **作業系統**: Windows, macOS, Linux
- **記憶體**: 最少 1GB RAM
- **硬碟**: 最少 500MB 可用空間

### ⚡ 安裝步驟

#### 1. 下載專案
```bash
git clone https://github.com/jimmyhuang921114/user_ineterface.git
cd user_ineterface
```

#### 2. 選擇版本

**📁 JSON版本 (輕量級)**
```bash
git checkout cursor/git-afed  # 主要分支
pip install -r requirements.txt
```

**🗄️ SQL版本 (企業級)**
```bash
git checkout feature/sql-database
pip install -r requirements_sql.txt
```

#### 3. 啟動系統

**JSON版本啟動**
```bash
cd user_interface
python3 main.py
```

**SQL版本啟動**
```bash
cd user_interface
python3 database.py    # 初始化資料庫
python3 sql_server.py  # 啟動SQL伺服器
```

#### 4. 訪問系統
開啟瀏覽器訪問: http://localhost:8000

---

## 🎯 功能導覽

### 👨‍⚕️ 醫生工作台
**專為醫師設計的處方籤開立系統**

✅ **核心功能**
- 自動病患編號生成 (基於身份證號)
- 自動處方時間生成
- 智能藥物選擇下拉選單
- 庫存狀態即時顯示
- AI提示詞智能推薦

✅ **使用流程**
1. 輸入病患基本資訊
2. 選擇藥物並設定劑量
3. 系統自動驗證庫存
4. 生成完整處方籤

### 🔧 整合管理
**全功能的系統管理介面**

✅ **四大管理模組**
1. **➕ 新增藥物**: 統一的藥物資料新增
2. **📦 庫存管理**: 增加/減少數量，操作記錄
3. **📋 藥物清單**: 查看、編輯、刪除藥物
4. **🩺 處方管理**: 查看、編輯、取消處方

✅ **進階功能**
- 即時搜尋和篩選
- 批量操作支援
- 詳細統計分析
- 完整的操作記錄

### 📋 處方籤管理
**專業的處方記錄管理**

✅ **管理功能**
- 處方籤列表查看
- 詳細資訊檢視
- 處方狀態更新
- 歷史記錄追蹤

### 🔄 統一表單
**快速藥物資料新增**

✅ **特色設計**
- 一次性填寫基本和詳細資料
- 智能表單驗證
- 自動資料關聯
- 快速導航面板

✅ **新增側邊導航**
- 🚀 快速導航到其他頁面
- 📝 表單功能說明
- ✅ 操作指引提示

---

## 🤖 ROS2整合

### 📡 ROS2服務
```bash
# 傳統服務 (向後兼容)
/get_basic_medicines      # 基本藥物資料
/get_detailed_medicines   # 詳細藥物資料  
/get_medicine_orders      # 藥物訂單資料

# 增強服務 (新功能)
/get_single_order         # 單筆訂單查詢
/get_all_basic_medicines  # 所有基本藥物
/get_medicine_info        # 程式用結構化資料
```

### 📤 ROS2主題
```bash
# 傳統主題
/basic_medicines_output    # 基本藥物輸出
/detailed_medicines_output # 詳細藥物輸出
/medicine_order_output     # 訂單輸出

# 增強主題
/single_order_output           # 單筆訂單
/enhanced_basic_medicines_output # 增強基本藥物
/structured_medicine_data      # 程式結構化資料
```

### 🎛️ 一鍵啟動ROS2
```bash
# 啟動完整ROS2服務系統
python3 start_enhanced_ros2_system.py

# 啟動傳統ROS2服務
python3 start_ros2_medicine_system.py
```

### 📁 輸出檔案位置
```bash
~/ros2_medicine_output/
├── single_order_20241212_143000.yaml
├── all_basic_medicines_20241212_143100.yaml  
├── structured_medicines_20241212_143200.json
└── structured_medicines_20241212_143200.yaml
```

---

## 🗄️ 資料庫架構

### 📁 JSON版本 (輕量級)
```
user_interface/data/
├── basic_medicines.json     # 基本藥物資料
├── detailed_medicines.json  # 詳細藥物資料
├── prescriptions.json       # 處方籤資料
├── basic_medicines.yaml     # YAML匯出 (ROS2)
└── detailed_medicines.yaml  # YAML匯出 (ROS2)
```

### 🗄️ SQL版本 (企業級)
```sql
-- 資料庫表格結構
medicine_basic              # 基本藥物資料表
├── id (PK)                # 主鍵
├── name (UNIQUE)          # 藥物名稱
├── amount                 # 庫存數量
├── position               # 儲存位置
├── prompt                 # AI提示詞 🤖
└── is_active             # 軟刪除標記

medicine_detailed           # 詳細藥物資料表
├── id (PK)
├── medicine_id (FK)       # 關聯基本藥物
├── description            # 詳細描述
└── side_effects          # 副作用

prescriptions              # 處方籤主表
prescription_medicines     # 處方藥物明細表  
stock_logs                # 庫存異動記錄表
system_logs               # 系統操作記錄表
```

---

## 🔗 API文檔

### 💊 藥物管理API
```http
GET    /api/medicine/basic          # 獲取所有基本藥物
POST   /api/medicine/basic          # 創建基本藥物
POST   /api/medicine/detailed       # 創建詳細資料
POST   /api/medicine/unified        # 統一創建(基本+詳細)
POST   /api/medicine/adjust-stock   # 庫存調整
DELETE /api/medicine/{name}         # 刪除藥物
```

### 📋 處方籤管理API
```http
GET    /api/prescription/           # 獲取所有處方籤
POST   /api/prescription/           # 創建新處方籤
DELETE /api/prescription/{id}       # 取消處方籤
```

### 🤖 ROS2整合API
```http
GET    /api/ros2/prescription       # ROS2格式處方資料
POST   /api/export/yaml/sync        # 手動YAML同步
```

---

## 🗂️ 專案結構

```
醫院藥物管理系統/
├── user_interface/                 # 主要應用程式
│   ├── main.py                    # JSON版本伺服器
│   ├── sql_server.py              # SQL版本伺服器 ⭐
│   ├── database.py                # SQL資料庫模型 ⭐
│   ├── fixed_server.py            # 固定版伺服器
│   ├── yaml_storage.py            # YAML處理模組
│   ├── data/                      # 資料儲存目錄
│   │   ├── *.json                # JSON資料檔案
│   │   ├── *.yaml                # YAML匯出檔案
│   │   └── hospital_management.db # SQL資料庫 ⭐
│   └── static/                    # 靜態檔案
│       ├── html/                  # HTML頁面
│       │   ├── doctor.html        # 醫生工作台
│       │   ├── integrated_medicine_management.html # 整合管理
│       │   ├── Prescription.html  # 處方籤管理
│       │   └── unified_medicine.html # 統一表單 ⭐
│       ├── css/                   # 樣式表
│       └── js/                    # JavaScript邏輯
├── ros2_packages/                 # ROS2整合套件
│   ├── medicine_order_service/    # 訂單服務
│   ├── medicine_basic_provider/   # 基本藥物服務
│   └── medicine_detailed_provider/ # 詳細藥物服務
├── start_enhanced_ros2_system.py  # 增強版ROS2啟動器 ⭐
├── start_ros2_medicine_system.py  # 傳統ROS2啟動器
├── requirements.txt               # JSON版本依賴
├── requirements_sql.txt           # SQL版本依賴 ⭐
└── README.md                      # 專案說明
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
# JSON版本測試
cd user_interface
python3 main.py

# SQL版本測試
cd user_interface  
python3 sql_server.py

# ROS2服務測試
python3 start_enhanced_ros2_system.py
```

### 🐛 除錯
```bash
# 檢查依賴
pip install -r requirements.txt      # JSON版本
pip install -r requirements_sql.txt  # SQL版本

# 檢查連接埠
netstat -tulpn | grep :8000

# 檢查ROS2環境
source /opt/ros/humble/setup.bash  # 根據您的ROS2版本調整
```

### 📝 自定義設定
- **連接埠**: 預設8000，可在伺服器檔案中修改
- **資料庫**: SQL版本使用SQLite，可切換到PostgreSQL/MySQL
- **ROS2**: 可自定義服務名稱和主題名稱

---

## 🎊 更新記錄

### v3.0.0 (目前版本) ⭐
- **🎯 頁面功能優化**: 統一導航系統，移除重複頁面
- **🔄 統一表單增強**: 新增快速導航面板和功能說明
- **👨‍⚕️ 醫生工作台精簡**: 專注於處方籤開立功能
- **🚀 ROS2服務增強**: 新增單筆訂單和結構化資料服務
- **📋 完整README**: 全新的使用者指南和技術文檔

### v2.1.0 (SQL版本)
- **🗄️ 完整SQL資料庫**: SQLite/PostgreSQL/MySQL支援
- **🤖 AI提示詞功能**: 每種藥物可設定AI推薦提示
- **📊 歷史記錄追蹤**: 完整的操作和庫存異動記錄
- **🔒 企業級安全**: IP追蹤、操作日誌、資料驗證

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
- **JSON版本分支**: `cursor/git-afed` (主分支)
- **SQL版本分支**: `feature/sql-database`

---

## 📄 授權條款

本專案採用 MIT 授權條款 - 詳見 [LICENSE](LICENSE) 檔案

---

**🏥 醫院藥物管理系統 - 為現代醫療提供專業的數位化解決方案！**

## 🎯 系統特色總結

✨ **四個核心頁面，完整功能覆蓋**  
✨ **雙資料庫支援，彈性部署選擇**  
✨ **ROS2深度整合，支援機器人應用**  
✨ **AI智能提示，提升醫療決策品質**  
✨ **現代化介面，優秀的使用者體驗**
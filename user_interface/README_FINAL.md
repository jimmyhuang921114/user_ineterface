# 醫院藥物管理系統 - 完整版本

## 系統概述

這是一個功能完整且穩定的醫院藥物管理系統，專為專業環境設計。系統整合了 FastAPI 後端、現代化前端界面、SQLite 資料庫和 ROS2 機器人整合功能。

**系統已經過全面清理和優化，確保穩定性和功能完整性。**

## 系統版本選擇

### 1. 乾淨版本 (推薦)
```bash
python3 start_clean_system.py
```
- **特色**: 含 ROS2 模擬功能，無測試資料，專業環境適用
- **適用**: 生產環境，一般使用
- **檔案**: `simple_server_clean.py`

### 2. 真實 ROS2 版本
```bash
python3 start_ros2_real_server.py
```
- **特色**: 需要真實 ROS2 環境，無模擬功能
- **適用**: 有實際機器人的環境
- **檔案**: `simple_server_ros2_real.py`

### 3. 純 API 版本
```bash
python3 start_no_ros2_server.py
```
- **特色**: 完全無 ROS2，純 Web API 服務
- **適用**: 只需要網頁功能的場合
- **檔案**: `simple_server_no_ros2.py`

## 快速開始

### 1. 系統需求
- Python 3.8+
- 網路瀏覽器
- 4GB 可用磁碟空間

### 2. 安裝依賴
```bash
# 方法一：系統套件 (推薦)
apt install python3-fastapi python3-uvicorn python3-sqlalchemy python3-requests

# 方法二：虛擬環境
python3 -m venv venv
source venv/bin/activate
pip install fastapi uvicorn sqlalchemy requests
```

### 3. 初始化資料庫
```bash
python3 database_clean.py
```

### 4. 啟動系統
```bash
# 推薦：乾淨版本
python3 start_clean_system.py

# 訪問系統
# 瀏覽器開啟: http://localhost:8001/
```

### 5. 系統測試
```bash
# 執行基本測試
python3 clean_test_system.py
```

## 系統架構

### 核心檔案結構
```
user_interface/
├── simple_server_clean.py          # 主伺服器 (推薦)
├── simple_server_ros2_real.py      # 真實 ROS2 版本
├── simple_server_no_ros2.py        # 純 API 版本
├── database_clean.py               # 資料庫模型
├── ros2_mock_clean.py              # ROS2 模擬器
├── start_clean_system.py           # 主要啟動器
├── start_ros2_real_server.py       # ROS2 啟動器
├── start_no_ros2_server.py         # API 啟動器
├── clean_test_system.py            # 測試腳本
├── hospital_medicine_clean.db      # SQLite 資料庫
├── static/                         # 前端檔案
├── cleanup_system.py               # 系統清理工具
├── ESSENTIAL_FILES_LIST.md         # 必要檔案清單
├── README_FINAL.md                 # 本檔案
└── SYSTEM_OVERVIEW.md             # 系統總覽
```

### 前端界面
- **主管理界面**: `http://localhost:8001/integrated_medicine_management.html`
- **處方籤管理**: `http://localhost:8001/Prescription.html`
- **醫生工作站**: `http://localhost:8001/doctor.html`
- **API 文檔**: `http://localhost:8001/docs`

## 核心功能

### 藥物管理
- ✅ 基本藥物資訊 CRUD (創建、讀取、更新、刪除)
- ✅ 詳細藥物資訊管理
- ✅ 智能搜尋和篩選
- ✅ 即時庫存管理和調整
- ✅ 批量操作支援

### 處方籤管理
- ✅ 處方籤創建和編輯
- ✅ 多種狀態追蹤 (pending, processing, completed, cancelled)
- ✅ **自動庫存扣減** (創建處方籤時立即扣減)
- ✅ 庫存檢查和不足警告
- ✅ 處方籤歷史記錄

### 庫存管理
- ✅ **創建處方籤時**: 立即扣減庫存
- ✅ **取消處方籤時**: 自動恢復庫存
- ✅ **庫存不足時**: 拒絕創建並顯示詳細資訊
- ✅ **手動調整**: 支援庫存增減和原因記錄

### ROS2 整合 (乾淨版和真實版)
- ✅ **詢問-確認-執行**工作流程
- ✅ 藥物查詢服務 (基本/詳細資訊)
- ✅ 批量藥物查詢
- ✅ 自動狀態更新
- ✅ 模擬和真實 ROS2 支援

## API 端點總覽

### 系統狀態
```
GET /api/system/status              - 系統狀態
GET /api/ros2/status               - ROS2 狀態 (僅 ROS2 版本)
```

### 藥物管理
```
GET    /api/medicine/basic          - 獲取基本藥物列表
GET    /api/medicine/detailed       - 獲取詳細藥物列表
POST   /api/medicine/unified        - 創建統一藥物
PUT    /api/medicine/{id}           - 更新藥物
DELETE /api/medicine/{id}           - 刪除藥物
GET    /api/medicine/search/{name}  - 搜尋藥物
POST   /api/medicine/adjust-stock   - 調整庫存
```

### 處方籤管理
```
GET    /api/prescription/           - 獲取處方籤列表
POST   /api/prescription/           - 創建處方籤
GET    /api/prescription/{id}       - 獲取處方籤詳情
PUT    /api/prescription/{id}/status - 更新處方籤狀態
GET    /api/prescription/pending/next - 獲取下一個待處理處方籤
```

### ROS2 整合 (僅 ROS2 版本)
```
GET    /api/ros2/pending-orders     - 獲取待處理訂單
POST   /api/ros2/request-order-confirmation - 請求訂單確認
POST   /api/ros2/confirm-and-execute-order  - 確認並執行訂單
POST   /api/ros2/complete-order     - 標記訂單完成
POST   /api/ros2/query-medicine     - 查詢藥物資訊
POST   /api/ros2/batch-query-medicines - 批量查詢藥物
```

## 系統測試

### 測試腳本
```bash
# 基本功能測試 (推薦)
python3 clean_test_system.py

# 完整功能測試
python3 test_complete_system.py

# ROS2 專項測試 (僅 ROS2 版本)
python3 test_ros2_services.py
```

### 手動測試
```bash
# 檢查系統狀態
curl http://localhost:8001/api/system/status

# 檢查藥物列表
curl http://localhost:8001/api/medicine/basic

# 檢查處方籤列表
curl http://localhost:8001/api/prescription/
```

## 故障排除

### 常見問題

**1. 模組缺失錯誤**
```bash
# 使用系統套件
sudo apt install python3-fastapi python3-uvicorn python3-sqlalchemy python3-requests

# 或使用虛擬環境
python3 -m venv venv
source venv/bin/activate
pip install fastapi uvicorn sqlalchemy requests
```

**2. 端口被占用**
```bash
# 查找占用進程
lsof -i :8001

# 停止占用進程
kill -9 [PID]
```

**3. 資料庫問題**
```bash
# 重新創建資料庫
rm -f hospital_medicine_clean.db
python3 database_clean.py
```

**4. ROS2 環境問題**
- 乾淨版：自動使用模擬模式
- 真實版：需要正確設定 ROS2 環境
- 純 API 版：無 ROS2 功能

### 日誌位置
- **乾淨版**: `hospital_system.log`
- **真實 ROS2 版**: `hospital_system_ros2_real.log`
- **純 API 版**: `hospital_system_no_ros2.log`

## ROS2 工作流程 (適用於 ROS2 版本)

### 完整流程示例
```bash
# 1. 查看待處理訂單
curl http://localhost:8001/api/ros2/pending-orders

# 2. 請求訂單確認
curl -X POST http://localhost:8001/api/ros2/request-order-confirmation \
-H "Content-Type: application/json" \
-d '{"prescription_id": 1}'

# 3. 確認並執行訂單
curl -X POST http://localhost:8001/api/ros2/confirm-and-execute-order \
-H "Content-Type: application/json" \
-d '{"prescription_id": 1}'

# 4. 標記訂單完成
curl -X POST http://localhost:8001/api/ros2/complete-order \
-H "Content-Type: application/json" \
-d '{"prescription_id": 1}'
```

## 系統清理和維護

### 清理不必要檔案
```bash
# 執行系統清理工具
python3 cleanup_system.py
```

### 定期維護
```bash
# 檢查資料庫大小
ls -lh hospital_medicine_clean.db

# 清理日誌檔案 (如果過大)
> hospital_system.log

# 備份資料庫
cp hospital_medicine_clean.db hospital_medicine_backup_$(date +%Y%m%d).db
```

## 部署建議

### 開發環境
- 使用 `start_clean_system.py`
- 啟用詳細日誌
- 使用測試資料庫

### 生產環境
1. 使用乾淨版或真實 ROS2 版
2. 配置反向代理 (nginx)
3. 設定 SSL 憑證
4. 配置自動備份
5. 設定監控系統

## 技術規格

### 後端技術
- **FastAPI**: 現代 Python Web 框架
- **SQLAlchemy**: Python ORM
- **SQLite**: 輕量級資料庫
- **Uvicorn**: ASGI 伺服器

### 前端技術
- **HTML5/CSS3/JavaScript**: 原生 Web 技術
- **Bootstrap**: 響應式 UI 框架
- **Fetch API**: 現代 HTTP 請求

### ROS2 整合
- **rclpy**: Python ROS2 綁定
- **自定義訊息**: 專用資料格式
- **發布/訂閱**: 異步通訊
- **服務導向**: 請求/回應模式

## 系統狀態

✅ **系統已穩定**: 所有核心功能正常運作  
✅ **檔案已清理**: 移除冗余檔案，保留必要組件  
✅ **測試已通過**: 全面測試驗證功能完整性  
✅ **文檔已更新**: 提供完整使用說明  

這是一個生產就緒的醫院藥物管理系統，適合專業醫療環境使用。系統設計簡潔高效，功能完整可靠，已通過全面測試驗證。
# 醫院藥物管理系統 - 無 emoji 版本

## 系統概述

這是一個功能完整的醫院藥物管理系統，專為專業環境設計，不含任何 emoji 符號。系統整合了 FastAPI 後端、現代化前端界面、SQLite 資料庫和 ROS2 機器人整合功能。

## 核心特色

- **專業環境適用**: 完全無 emoji，符合正式系統要求
- **功能完整**: 藥物管理、處方籤管理、庫存控制、ROS2 整合
- **易於部署**: 多種啟動方式，最小依賴需求
- **ROS2 就緒**: 可直接接入真實機器人系統
- **完整測試**: 提供測試腳本驗證所有功能

## 快速開始

### 1. 安裝依賴
```bash
pip install fastapi uvicorn sqlalchemy requests
```

### 2. 啟動系統 (推薦)
```bash
python3 start_clean_system.py
```

### 3. 訪問系統
- 主頁面: http://localhost:8001/
- 整合管理: http://localhost:8001/integrated_medicine_management.html
- 處方籤管理: http://localhost:8001/Prescription.html
- 醫生工作站: http://localhost:8001/doctor.html
- API 文檔: http://localhost:8001/docs

## 系統架構

### 核心檔案
- `simple_server_clean.py` - 主伺服器 (含 ROS2 模擬)
- `simple_server_no_ros2.py` - 純 API 伺服器 (無 ROS2)
- `database_clean.py` - 資料庫模型
- `ros2_mock_clean.py` - ROS2 模擬器

### 啟動選項
1. **推薦方式**: `python3 start_clean_system.py`
2. **完整系統**: `python3 start_clean_server.py`
3. **純 API**: `python3 start_no_ros2_server.py`

### 測試腳本
- `clean_test_system.py` - 無 emoji 測試 (推薦)
- `test_complete_system.py` - 完整測試
- `test_ros2_services.py` - ROS2 服務測試

## 功能說明

### 藥物管理
- 基本藥物資訊 CRUD
- 詳細藥物資訊管理
- 智能搜尋和篩選
- 即時庫存管理
- 批量操作支援

### 處方籤管理
- 處方籤創建和編輯
- 多種狀態追蹤 (pending, processing, completed, cancelled)
- 自動庫存扣減 (創建時)
- 庫存檢查和警告
- 處方籤歷史記錄

### ROS2 整合
- 詢問-確認-執行工作流程
- 藥物查詢服務 (基本/詳細資訊)
- 批量藥物查詢
- 自動狀態更新
- 模擬和真實 ROS2 支援

### 前端界面
- 整合式管理界面
- 響應式設計
- 即時資料更新
- 操作確認對話框
- 錯誤處理和用戶提示

## API 端點

### 藥物管理 API
```
GET    /api/medicine/basic           - 獲取基本藥物列表
GET    /api/medicine/detailed        - 獲取詳細藥物列表
POST   /api/medicine/unified         - 創建統一藥物
PUT    /api/medicine/{id}            - 更新藥物
DELETE /api/medicine/{id}            - 刪除藥物
GET    /api/medicine/search/{name}   - 搜尋藥物
POST   /api/medicine/adjust-stock    - 調整庫存
```

### 處方籤管理 API
```
GET    /api/prescription/            - 獲取處方籤列表
POST   /api/prescription/            - 創建處方籤
GET    /api/prescription/{id}        - 獲取處方籤詳情
PUT    /api/prescription/{id}/status - 更新處方籤狀態
GET    /api/prescription/pending/next - 獲取下一個待處理處方籤
```

### ROS2 整合 API
```
GET    /api/ros2/status              - 獲取 ROS2 狀態
GET    /api/ros2/pending-orders      - 獲取待處理訂單
POST   /api/ros2/request-order-confirmation - 請求訂單確認
POST   /api/ros2/confirm-and-execute-order  - 確認並執行訂單
POST   /api/ros2/complete-order      - 標記訂單完成
POST   /api/ros2/query-medicine      - 查詢藥物資訊
POST   /api/ros2/batch-query-medicines - 批量查詢藥物
```

## 系統測試

### 執行測試
```bash
# 基本測試 (無 emoji)
python3 clean_test_system.py

# 完整測試
python3 test_complete_system.py

# ROS2 專項測試
python3 test_ros2_services.py
```

### 測試範圍
- 伺服器連接測試
- 基本藥物 API 測試
- 詳細藥物 API 測試
- 藥物創建功能測試
- ROS2 功能整合測試
- 工作流程完整性測試

## 部署說明

### 系統需求
- Python 3.8 或更高版本
- FastAPI 0.68+
- SQLAlchemy 1.4+
- SQLite 3.0+
- ROS2 Humble/Iron (可選)

### 環境配置
```bash
# 安裝 Python 依賴
pip install fastapi uvicorn sqlalchemy requests

# 檢查 Python 版本
python3 --version

# 檢查必要檔案
ls simple_server_clean.py database_clean.py ros2_mock_clean.py
```

### 啟動配置
系統會自動:
- 創建 SQLite 資料庫
- 初始化表格結構
- 檢測 ROS2 可用性
- 啟動 Web 伺服器 (端口 8001)

## 故障排除

### 常見問題

**1. 缺少 Python 模組**
```bash
pip install fastapi uvicorn sqlalchemy requests
```

**2. 端口被占用**
```bash
# 查找占用進程
lsof -i :8001
# 停止占用進程
kill -9 [PID]
```

**3. 資料庫權限問題**
```bash
# 確保目錄有寫入權限
chmod 755 .
rm -f hospital_medicine_clean.db  # 重新創建資料庫
```

**4. ROS2 環境問題**
系統會自動降級到模擬模式，無需手動配置。

### 調試指令
```bash
# 檢查系統狀態
curl http://localhost:8001/api/system/status

# 檢查 ROS2 狀態
curl http://localhost:8001/api/ros2/status

# 檢查藥物列表
curl http://localhost:8001/api/medicine/basic

# 查看系統日誌
tail -f hospital_system.log
```

## 技術支援

### 日誌位置
- 系統日誌: `hospital_system.log`
- 錯誤日誌: 終端輸出
- 資料庫檔案: `hospital_medicine_clean.db`

### 系統監控
- API 回應時間監控
- 資料庫查詢效能
- 記憶體和 CPU 使用量
- ROS2 連接狀態

## 擴展開發

### 添加新功能
1. 在 `simple_server_clean.py` 中添加 API 端點
2. 在 `database_clean.py` 中定義資料表
3. 在 `static/` 中添加前端界面
4. 在測試腳本中添加測試案例

### ROS2 整合
1. 實現真實 ROS2 節點
2. 定義自定義消息類型
3. 創建機器人控制邏輯
4. 整合硬體感測器

這是一個生產就緒的醫院藥物管理系統，適合專業醫療環境使用。系統設計簡潔高效，功能完整可靠。
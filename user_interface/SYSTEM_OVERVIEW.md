# 醫院藥物管理系統 - 系統總覽

## 核心檔案結構

```
user_interface/
├── simple_server_clean.py          # 主伺服器 (乾淨版本，含 ROS2 模擬)
├── simple_server_ros2_real.py      # 真實 ROS2 版本 (需要 ROS2 環境)
├── simple_server_no_ros2.py        # 純 API 版本 (無 ROS2)
├── database_clean.py               # 資料庫模型定義
├── ros2_mock_clean.py              # ROS2 模擬器
├── start_clean_system.py           # 乾淨版啟動腳本 (推薦)
├── start_ros2_real_server.py       # 真實 ROS2 啟動腳本
├── start_no_ros2_server.py         # 純 API 啟動腳本
├── clean_test_system.py            # 乾淨版測試腳本
├── test_complete_system.py         # 完整系統測試
├── test_ros2_services.py           # ROS2 服務測試
├── hospital_medicine_clean.db      # SQLite 資料庫
├── static/                         # 前端檔案
├── ros2_packages/                  # ROS2 套件定義
├── ROS2_NODE_SUMMARY.md           # ROS2 節點總結
├── ROS2_PACKAGE_GUIDE.md          # ROS2 套件指南
├── README_NO_EMOJI.md             # 無 emoji 使用說明
└── SYSTEM_OVERVIEW.md             # 系統總覽 (本檔案)
```

## 版本選擇

### 1. 乾淨版本 (推薦)
```bash
python3 start_clean_system.py
```
- 含 ROS2 模擬功能
- 無測試資料，無 emoji
- 適合生產環境

### 2. 真實 ROS2 版本
```bash
python3 start_ros2_real_server.py
```
- 需要真實 ROS2 環境
- 無模擬，直接連接 ROS2
- 適合有實際機器人的場合

### 3. 純 API 版本
```bash
python3 start_no_ros2_server.py
```
- 完全無 ROS2 功能
- 純 Web API 服務
- 適合只需要網頁功能的場合

## 快速啟動

```bash
# 1. 安裝依賴
pip install fastapi uvicorn sqlalchemy requests

# 2. 啟動系統 (推薦使用乾淨版)
python3 start_clean_system.py

# 3. 訪問系統
# 前端界面: http://localhost:8001/integrated_medicine_management.html
# API 文檔: http://localhost:8001/docs
# 處方管理: http://localhost:8001/Prescription.html
# 醫生工作站: http://localhost:8001/doctor.html

# 4. 測試系統
python3 clean_test_system.py
```

## ROS2 工作流程總結

### 完整詢問-確認-執行循環

1. **詢問階段**
   ```bash
   curl http://localhost:8001/api/ros2/pending-orders
   ```
   - 獲取所有待處理訂單
   - 顯示病患資訊和藥物清單

2. **確認階段**
   ```bash
   curl -X POST http://localhost:8001/api/ros2/request-order-confirmation \
   -H "Content-Type: application/json" \
   -d '{"prescription_id": 1}'
   ```
   - 請求執行特定訂單
   - 系統回傳訂單詳情

3. **執行階段**
   ```bash
   curl -X POST http://localhost:8001/api/ros2/confirm-and-execute-order \
   -H "Content-Type: application/json" \
   -d '{"prescription_id": 1}'
   ```
   - 確認並開始執行訂單
   - 處方籤狀態變為 "processing"

4. **完成階段**
   ```bash
   curl -X POST http://localhost:8001/api/ros2/complete-order \
   -H "Content-Type: application/json" \
   -d '{"prescription_id": 1}'
   ```
   - 標記訂單完成
   - 處方籤狀態變為 "completed"

### ROS2 藥物查詢

#### 單一藥物查詢
```bash
# 按名稱查詢
curl -X POST http://localhost:8001/api/ros2/query-medicine \
-H "Content-Type: application/json" \
-d '{"medicine_name": "阿司匹林", "include_detailed": true}'

# 按 ID 查詢
curl -X POST http://localhost:8001/api/ros2/query-medicine \
-H "Content-Type: application/json" \
-d '{"medicine_id": 1, "include_detailed": false}'
```

#### 批量藥物查詢
```bash
curl -X POST http://localhost:8001/api/ros2/batch-query-medicines \
-H "Content-Type: application/json" \
-d '{
  "medicine_names": ["阿司匹林", "維他命C"],
  "include_detailed": true
}'
```

## 核心功能

### 藥物管理
- 基本藥物資訊管理 (CRUD)
- 詳細藥物資訊管理
- 庫存管理和調整
- 智能搜尋功能

### 處方籤管理
- 處方籤建立和編輯
- 狀態追蹤 (pending, processing, completed, cancelled)
- 自動庫存扣減 (建立時)
- 庫存檢查和警告

### ROS2 整合 (限 ROS2 版本)
- 詢問-確認-執行工作流程
- 藥物查詢服務 (基本/詳細資訊)
- 批量藥物查詢
- 自動狀態更新

## API 端點總覽

### 系統狀態
- `GET /api/system/status` - 系統狀態
- `GET /api/ros2/status` - ROS2 狀態 (僅 ROS2 版本)

### 藥物管理
- `GET /api/medicine/basic` - 基本藥物列表
- `GET /api/medicine/detailed` - 詳細藥物列表
- `POST /api/medicine/unified` - 建立統一藥物
- `PUT /api/medicine/{id}` - 更新藥物
- `DELETE /api/medicine/{id}` - 刪除藥物
- `GET /api/medicine/search/{name}` - 搜尋藥物
- `POST /api/medicine/adjust-stock` - 調整庫存

### 處方籤管理
- `GET /api/prescription/` - 處方籤列表
- `POST /api/prescription/` - 建立處方籤
- `GET /api/prescription/{id}` - 處方籤詳情
- `PUT /api/prescription/{id}/status` - 更新狀態
- `GET /api/prescription/pending/next` - 下一個待處理

### ROS2 整合 (僅 ROS2 版本)
- `GET /api/ros2/pending-orders` - 待處理訂單
- `POST /api/ros2/request-order-confirmation` - 請求確認
- `POST /api/ros2/confirm-and-execute-order` - 確認執行
- `POST /api/ros2/complete-order` - 標記完成
- `POST /api/ros2/query-medicine` - 查詢藥物
- `POST /api/ros2/batch-query-medicines` - 批量查詢

## 庫存管理機制

### 庫存扣減時機
1. **建立處方籤時**: 立即扣減庫存
2. **取消處方籤時**: 恢復庫存
3. **重新啟用取消的處方籤**: 重新扣減庫存

### 庫存檢查
- 建立處方籤前檢查庫存是否足夠
- 庫存不足時拒絕建立並回傳詳細資訊
- 支援手動庫存調整

## 測試工具

### 基本測試
```bash
# 乾淨版測試 (推薦)
python3 clean_test_system.py

# 完整功能測試
python3 test_complete_system.py

# ROS2 專項測試
python3 test_ros2_services.py
```

### 手動測試
```bash
# 檢查系統狀態
curl http://localhost:8001/api/system/status

# 檢查 ROS2 狀態 (僅 ROS2 版本)
curl http://localhost:8001/api/ros2/status

# 獲取藥物列表
curl http://localhost:8001/api/medicine/basic

# 獲取處方籤列表
curl http://localhost:8001/api/prescription/
```

## 部署建議

### 生產環境
1. 使用 `simple_server_clean.py` 或 `simple_server_ros2_real.py`
2. 配置反向代理 (nginx)
3. 設定 SSL 憑證
4. 配置日誌輪轉
5. 設定資料庫備份

### 開發環境
1. 使用 `start_clean_system.py` 快速開發
2. 啟用詳細日誌記錄
3. 使用測試資料庫

## 故障排除

### 常見問題
1. **端口被佔用**: `lsof -i :8001` 查找進程
2. **ROS2 環境問題**: 確保正確設定 ROS2 環境變數
3. **資料庫權限**: 確保目錄有寫入權限
4. **模組缺失**: `pip install fastapi uvicorn sqlalchemy requests`

### 日誌檔案
- 乾淨版: `hospital_system.log`
- 真實 ROS2 版: `hospital_system_ros2_real.log`
- 純 API 版: `hospital_system_no_ros2.log`

## 技術規格

### 後端技術
- FastAPI: Web 框架
- SQLAlchemy: ORM
- SQLite: 資料庫
- Uvicorn: ASGI 伺服器

### 前端技術
- HTML5/CSS3/JavaScript
- Bootstrap: UI 框架
- Fetch API: HTTP 請求

### ROS2 整合
- rclpy: Python 綁定
- 自定義訊息和服務
- 發布/訂閱模式
- 服務導向架構

這個系統提供了完整的醫院藥物管理解決方案，支援多種部署方式以滿足不同環境的需求。
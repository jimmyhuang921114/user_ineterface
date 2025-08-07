# 醫院藥物管理系統 - 完整系統指南

## 系統概覽

本系統是一個完整的醫院藥物管理解決方案，整合了 FastAPI 後端、前端界面、SQLite 資料庫和 ROS2 機器人系統。

## 核心檔案結構

```
user_interface/
├── simple_server_clean.py          # 主伺服器 (含 ROS2 模擬)
├── simple_server_no_ros2.py        # 純 API 伺服器 (無 ROS2)
├── database_clean.py               # 資料庫模型定義
├── ros2_mock_clean.py              # ROS2 模擬器
├── start_clean_server.py           # 啟動含 ROS2 版本
├── start_no_ros2_server.py         # 啟動純 API 版本
├── test_complete_system.py         # 完整系統測試
├── test_ros2_services.py           # ROS2 服務測試
├── run_complete_system.py          # 系統運行器
├── static/                         # 前端檔案
│   ├── integrated_medicine_management.html
│   ├── Prescription.html
│   ├── doctor.html
│   ├── css/
│   └── js/
├── ros2_services/                  # ROS2 服務定義
│   ├── msg/
│   ├── srv/
│   ├── medicine_service_node.py
│   └── medicine_client_example.py
└── README_FINAL.md
```

## 系統功能

### 1. 核心功能
- 藥物管理 (CRUD)
- 處方籤管理
- 庫存管理
- 智能搜尋
- 狀態追蹤

### 2. ROS2 整合功能
- 詢問-確認-執行循環
- 藥物查詢服務
- 批量處理
- 自動狀態更新

### 3. 安全機制
- 庫存保護
- 狀態驗證
- 完整日誌
- 錯誤處理

## 快速啟動

### 方式一：使用系統運行器
```bash
python3 run_complete_system.py
```

### 方式二：直接啟動
```bash
# 含 ROS2 模擬版本
python3 start_clean_server.py

# 純 API 版本
python3 start_no_ros2_server.py
```

### 方式三：手動啟動
```bash
# 含 ROS2 模擬
python3 simple_server_clean.py

# 純 API
python3 simple_server_no_ros2.py
```

## 系統測試

### 完整系統測試
```bash
python3 test_complete_system.py
```

### ROS2 服務測試
```bash
python3 test_ros2_services.py
```

## API 端點

### 藥物管理 API
- `GET /api/medicine/basic` - 獲取基本藥物列表
- `GET /api/medicine/detailed` - 獲取詳細藥物列表
- `POST /api/medicine/unified` - 創建統一藥物
- `DELETE /api/medicine/{id}` - 刪除藥物
- `PUT /api/medicine/{id}` - 更新藥物
- `GET /api/medicine/search/{name}` - 搜尋藥物

### 處方籤管理 API
- `GET /api/prescription/` - 獲取處方籤列表
- `POST /api/prescription/` - 創建處方籤
- `GET /api/prescription/{id}` - 獲取處方籤詳情
- `PUT /api/prescription/{id}/status` - 更新處方籤狀態

### 庫存管理 API
- `POST /api/medicine/adjust-stock` - 調整庫存
- `GET /api/prescription/{id}/stock-check` - 檢查庫存

### ROS2 API
- `GET /api/ros2/status` - 獲取 ROS2 狀態
- `GET /api/ros2/pending-orders` - 獲取待處理訂單
- `POST /api/ros2/request-order-confirmation` - 請求訂單確認
- `POST /api/ros2/confirm-and-execute-order` - 確認並執行訂單
- `POST /api/ros2/complete-order` - 標記訂單完成
- `POST /api/ros2/query-medicine` - 查詢藥物資訊
- `POST /api/ros2/batch-query-medicines` - 批量查詢藥物

## 前端界面

### 網頁訪問
- 主頁面: http://localhost:8001/
- 整合管理: http://localhost:8001/integrated_medicine_management.html
- 處方籤管理: http://localhost:8001/Prescription.html
- 醫生工作站: http://localhost:8001/doctor.html

### API 文檔
- Swagger UI: http://localhost:8001/docs
- ReDoc: http://localhost:8001/redoc

## ROS2 整合

### ROS2 工作流程
1. 詢問待處理訂單
2. 請求執行確認
3. 確認並執行訂單
4. 標記訂單完成
5. 重複循環

### ROS2 服務定義
- `GetBasicMedicine.srv` - 獲取基本藥物服務
- `GetDetailedMedicine.srv` - 獲取詳細藥物服務
- `MedicineBasicInfo.msg` - 基本藥物資訊消息
- `MedicineDetailedInfo.msg` - 詳細藥物資訊消息

### ROS2 節點
- `medicine_service_node.py` - 藥物服務節點
- `medicine_client_example.py` - 客戶端範例

## 資料庫架構

### 核心表格
- `medicine_basic` - 基本藥物資訊
- `medicine_detailed` - 詳細藥物資訊
- `prescriptions` - 處方籤
- `prescription_medicines` - 處方籤藥物關聯

### 資料關聯
- 一個基本藥物可以有一個詳細資訊
- 一張處方籤可以包含多種藥物
- 藥物與處方籤之間是多對多關係

## 部署說明

### 系統需求
- Python 3.8+
- FastAPI 0.68+
- SQLAlchemy 1.4+
- SQLite 3.0+
- ROS2 Humble/Iron (可選)

### 安裝相依性
```bash
pip install fastapi uvicorn sqlalchemy requests
```

### 環境配置
```bash
# 設置環境變數 (可選)
export HOSPITAL_DB_URL="sqlite:///./hospital_medicine_clean.db"
export HOSPITAL_LOG_LEVEL="INFO"
export HOSPITAL_PORT="8001"
```

## 系統配置

### 伺服器配置
- 監聽地址: 0.0.0.0
- 監聽端口: 8001
- 超時設定: 300 秒
- 日誌級別: INFO

### 資料庫配置
- 資料庫類型: SQLite
- 檔案位置: ./hospital_medicine_clean.db
- 連接池大小: 20
- 自動創建表格: 是

### ROS2 配置
- 模擬模式: 是 (當 ROS2 不可用時)
- 節點名稱: medicine_service_node
- 服務超時: 10 秒
- 佇列大小: 100

## 監控與維護

### 日誌檔案
- 系統日誌: hospital_system.log
- 錯誤日誌: stderr 輸出
- 訪問日誌: stdout 輸出

### 效能監控
- API 回應時間
- 資料庫查詢效率
- 記憶體使用量
- CPU 使用率

### 備份策略
- 資料庫定期備份
- 配置檔案備份
- 日誌檔案輪替
- 系統映像備份

## 故障排除

### 常見問題
1. 端口被占用
   - 解決方案: 更改端口或停止占用進程

2. 資料庫鎖定
   - 解決方案: 重啟服務或清理死鎖

3. ROS2 連接失敗
   - 解決方案: 檢查 ROS2 環境或使用模擬模式

4. 前端載入失敗
   - 解決方案: 檢查靜態檔案路徑

### 調試指令
```bash
# 檢查服務狀態
curl http://localhost:8001/api/system/status

# 檢查 ROS2 狀態
curl http://localhost:8001/api/ros2/status

# 檢查資料庫連接
python3 -c "from database_clean import engine; print(engine.connect())"

# 查看即時日誌
tail -f hospital_system.log
```

## 擴展功能

### 自定義 API
- 在 simple_server_clean.py 中添加新端點
- 在 database_clean.py 中定義新表格
- 在前端中添加新頁面

### ROS2 整合
- 實現真實 ROS2 節點
- 定義自定義消息類型
- 創建機器人控制邏輯

### 第三方整合
- 醫院資訊系統 (HIS)
- 電子病歷系統 (EMR)
- 庫存管理系統 (WMS)

## 安全考量

### 資料安全
- 輸入驗證
- SQL 注入防護
- XSS 防護
- CSRF 防護

### 訪問控制
- API 金鑰驗證
- 角色權限管理
- 操作日誌記錄
- 敏感資料加密

### 網路安全
- HTTPS 加密
- 防火牆配置
- VPN 訪問
- 入侵檢測

## 效能優化

### 資料庫優化
- 索引建立
- 查詢優化
- 連接池配置
- 快取策略

### API 優化
- 回應壓縮
- 非同步處理
- 批次操作
- 分頁查詢

### 前端優化
- 資源壓縮
- 快取策略
- CDN 配置
- 延遲載入

這是一個功能完善、生產就緒的醫院藥物管理系統，具備完整的 ROS2 整合能力和豐富的擴展性。
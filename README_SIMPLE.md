# 簡化醫院藥物管理系統

一個簡化且穩定的醫院藥物和處方籤管理系統，支援ROS2整合。

## 🎯 核心功能

### 📦 訂單順序處理
- **一次處理一個訂單**：系統會等待前一個訂單完成後再處理下一個
- **佇列管理**：自動管理訂單佇列，確保處理順序
- **狀態追蹤**：即時追蹤訂單處理狀態

### 💊 藥物資料管理
- **基本藥物**：名稱、數量、位置、製造商、劑量等
- **詳細藥物**：成分、使用方法、副作用、儲存條件等
- **ROS2整合**：所有藥物資料都會透過ROS2發布

### 📋 處方籤管理
- **自動轉換**：處方籤自動轉換為ROS2訂單
- **完整資料**：包含病患資訊、藥物明細、診斷結果

## 🚀 快速開始

### 1. 啟動系統
```bash
./start_system_simple.sh
```

### 2. 訪問系統
- 🌐 藥物管理: http://localhost:8001/Medicine.html
- 📋 處方籤管理: http://localhost:8001/Prescription.html
- 👨‍⚕️ 醫生工作站: http://localhost:8001/doctor.html
- 📖 API文檔: http://localhost:8001/docs

## 🤖 ROS2整合

### 訂單處理流程
```
1. 創建處方籤 → 2. 自動加入ROS2佇列 → 3. 順序處理 → 4. 狀態回傳
```

### 藥物資料傳輸
```
藥物查詢/創建 → ROS2發布 → 其他節點接收完整資料
```

### ROS2 API端點
- `GET /api/ros2/status` - 獲取ROS2狀態
- `GET /api/ros2/queue` - 獲取佇列狀態
- `POST /api/ros2/order` - 添加訂單到佇列

## 📊 系統架構

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   前端介面      │───▶│   FastAPI APIs   │───▶│  SQLite資料庫   │
│                │    │                  │    │                │
│ • 藥物管理      │    │ • 藥物API        │    │ • 基本藥物表    │
│ • 處方籤管理    │    │ • 處方籤API      │    │ • 詳細藥物表    │
│ • 醫生工作站    │    │ • ROS2整合API    │    │ • 處方籤表      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │
                                ▼
                       ┌─────────────────┐
                       │   ROS2整合      │
                       │                │
                       │ • 訂單佇列     │
                       │ • 藥物發布     │
                       │ • 狀態回傳     │
                       └─────────────────┘
```

## 🧪 測試功能

### 1. 基本功能測試
```bash
cd user_interface
python3 test_ros2_integration.py
```

### 2. ROS2節點測試（需要安裝rclpy）
```bash
# 終端1：啟動ROS2處理節點
python3 ros2_test_client.py

# 終端2：啟動主系統
python3 main.py
```

## 📁 檔案結構

```
user_interface/
├── main.py                    # 主入口
├── simple_server.py           # 簡化伺服器
├── database.py                # 資料庫模型
├── ros2_integration.py        # ROS2整合模組
├── ros2_test_client.py        # ROS2測試客戶端
├── test_ros2_integration.py   # 整合測試
├── static/                    # 靜態檔案
│   ├── Medicine.html          # 藥物管理頁面
│   ├── Prescription.html      # 處方籤管理頁面
│   ├── doctor.html            # 醫生工作站
│   └── integrated_medicine_management.html
└── data/                      # 資料庫檔案
    └── hospital_management.db
```

## 🔧 配置選項

### 資料庫配置
```python
# database.py
DATABASE_URL = "sqlite:///./data/hospital_management.db"
```

### ROS2配置
```python
# ros2_integration.py
# 訂單處理間隔（秒）
self.order_timer = self.create_timer(2.0, self.process_order_queue)
```

## 📈 監控與日誌

### 系統狀態檢查
```bash
curl http://localhost:8001/api/health
```

### ROS2狀態檢查
```bash
curl http://localhost:8001/api/ros2/status
```

### 佇列狀態檢查
```bash
curl http://localhost:8001/api/ros2/queue
```

## 🚨 故障排除

### 常見問題

1. **伺服器無法啟動**
   ```bash
   # 檢查依賴
   pip3 install --break-system-packages -r requirements_simple.txt
   ```

2. **資料庫錯誤**
   ```bash
   # 重新初始化資料庫
   cd user_interface
   rm -rf data/
   python3 database.py
   ```

3. **ROS2模組不可用**
   - 系統會自動使用模擬模式
   - 如需真實ROS2，請安裝rclpy

## 📝 版本資訊

- **版本**: 1.0.0
- **Python**: 3.8+
- **資料庫**: SQLite
- **框架**: FastAPI
- **ROS2**: 可選（模擬模式）

## 🎯 特色總結

✅ **簡化穩定**：移除複雜功能，專注核心需求  
✅ **訂單順序**：確保一次處理一個訂單  
✅ **完整資料**：藥物和處方籤的完整資料傳輸  
✅ **ROS2整合**：支援真實和模擬ROS2環境  
✅ **易於部署**：一鍵啟動，最小依賴
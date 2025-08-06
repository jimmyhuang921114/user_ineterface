# 醫院藥物管理系統 - 乾淨版本

這是醫院藥物管理系統的乾淨版本，已移除所有測試資料和 emoji 符號。

## 乾淨版本文件列表

### 主要文件
- `simple_server_clean.py` - 主伺服器（乾淨版本）
- `database_clean.py` - 資料庫模型（乾淨版本）
- `ros2_mock_clean.py` - ROS2 模擬模組（乾淨版本）
- `start_clean_server.py` - 啟動腳本（乾淨版本）

### 測試版本文件（保留）
- `simple_server.py` - 主伺服器（包含測試資料）
- `database.py` - 資料庫模型（包含測試資料）
- `ros2_mock.py` - ROS2 模擬模組（包含 emoji）

## 啟動方式

### 使用乾淨版本
```bash
python3 start_clean_server.py
```

### 使用測試版本
```bash
python3 simple_server.py
```

## 主要差異

### 乾淨版本特點
1. **無測試資料** - 資料庫啟動時不會自動添加樣本藥物
2. **無 emoji** - 所有日誌和輸出訊息都移除了 emoji 符號
3. **精簡日誌** - 日誌級別設為 INFO，減少 DEBUG 輸出
4. **乾淨輸出** - 啟動訊息和 API 回應都是純文字

### 測試版本特點
1. **包含測試資料** - 自動添加 16 種樣本藥物和詳細資料
2. **豐富的 emoji** - 日誌和輸出包含視覺化 emoji 符號
3. **詳細日誌** - DEBUG 級別日誌，包含完整追蹤信息
4. **測試友好** - 適合開發和測試使用

## API 端點

兩個版本的 API 端點完全相同：

### 藥物管理
- `GET /api/medicine/basic` - 獲取基本藥物列表
- `GET /api/medicine/detailed` - 獲取詳細藥物列表
- `POST /api/medicine/` - 創建新藥物

### 處方籤管理
- `GET /api/prescription/` - 獲取處方籤列表
- `POST /api/prescription/` - 創建新處方籤
- `PUT /api/prescription/{id}/status` - 更新處方籤狀態

### ROS2 整合
- `POST /api/ros2/request-next-order` - 請求下一個待處理訂單
- `GET /api/ros2/status` - 獲取 ROS2 狀態

## 使用建議

- **開發和測試** - 使用測試版本 (`simple_server.py`)
- **生產環境** - 使用乾淨版本 (`simple_server_clean.py`)
- **ROS2 整合** - 兩個版本都完全支援 ROS2 功能

## 資料庫

乾淨版本會創建獨立的資料庫文件 `hospital_medicine.db`，與測試版本分離。

## 注意事項

1. 乾淨版本啟動時資料庫是空的，需要手動添加藥物資料
2. 可以透過網頁界面或 API 添加藥物和處方籤
3. ROS2 模擬功能在兩個版本中都正常運作
4. 所有 API 功能保持一致，只是輸出格式更簡潔
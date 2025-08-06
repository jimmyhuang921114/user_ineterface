# 🏥 醫院藥物管理系統 v4.0.0

## 📖 系統簡介

這是一個基於 **FastAPI** 和 **ROS2** 的現代化醫院藥物管理系統，提供完整的藥物管理、處方籤開立、庫存控制和自動化訂單處理功能。

### ✨ 核心特色
- 🎯 **處方籤管理**: 完整的處方籤開立、儲存和查詢
- 🤖 **ROS2 整合**: 自動化訂單處理和狀態更新
- 💊 **藥物管理**: 基本和詳細藥物資訊管理
- 📦 **庫存控制**: 即時庫存調整和追蹤
- 🔍 **AI 智能**: 智能藥物推薦和交互提示
- 🌐 **Web 介面**: 現代化響應式使用者介面

## 🚀 快速開始

### 前置需求
- Python 3.8+
- pip 套件管理器

### 安裝步驟

1. **安裝 Python 依賴**
```bash
pip install fastapi uvicorn pydantic pathlib typing
```

2. **啟動服務器**
```bash
cd user_interface
python fixed_server.py
```

3. **訪問系統**
開啟瀏覽器，前往以下地址：
- 主頁: http://localhost:8000/
- 醫生工作站: http://localhost:8000/doctor.html
- 藥物管理: http://localhost:8000/Medicine.html
- 處方籤管理: http://localhost:8000/Prescription.html
- 整合管理: http://localhost:8000/integrated_medicine_management.html
- 功能測試: http://localhost:8000/test_all_functions.html

## 🔧 系統功能

### 💊 藥物管理
- **基本藥物資料**: 藥名、劑量、用法
- **詳細藥物資訊**: 成分、副作用、注意事項
- **庫存管理**: 即時庫存查詢和調整
- **智能搜尋**: 模糊搜尋和過濾功能

### 📋 處方籤系統
- **病患資訊**: 姓名、身份證號、病患編號
- **藥物處方**: 多種藥物組合處方
- **用量控制**: 數量、天數、使用說明
- **處方驗證**: 自動檢查和驗證機制

### 🤖 ROS2 整合
- **訂單轉換**: 處方籤自動轉為 ROS2 訂單
- **狀態追蹤**: 即時訂單狀態更新
- **自動化流程**: 無縫的訂單處理流程

### 🧠 AI 功能
- **智能推薦**: 基於症狀的藥物推薦
- **交互助手**: 自然語言藥物查詢
- **決策支援**: 處方決策輔助工具

## 📡 API 文檔

### 基本 API

#### 健康檢查
```http
GET /api/health
```
回應系統健康狀態和基本資訊。

#### 基本藥物管理
```http
GET /api/medicine/basic          # 獲取所有基本藥物
POST /api/medicine/basic         # 新增基本藥物
PUT /api/medicine/basic/{id}     # 更新基本藥物
DELETE /api/medicine/basic/{id}  # 刪除基本藥物
```

#### 詳細藥物管理
```http
GET /api/medicine/detailed       # 獲取所有詳細藥物
POST /api/medicine/detailed      # 新增詳細藥物
PUT /api/medicine/detailed/{id}  # 更新詳細藥物
DELETE /api/medicine/detailed/{id} # 刪除詳細藥物
```

#### 庫存管理
```http
POST /api/medicine/adjust-stock  # 調整藥物庫存
```

請求格式：
```json
{
  "medicine_id": "M001",
  "adjustment": 10,
  "reason": "進貨"
}
```

#### 處方籤管理
```http
GET /api/prescription/           # 獲取所有處方籤
POST /api/prescription/          # 創建新處方籤
PUT /api/prescription/{id}       # 更新處方籤
DELETE /api/prescription/{id}    # 刪除處方籤
```

處方籤格式：
```json
{
  "patient_name": "王小明",
  "patient_id": "P123456",
  "doctor_name": "李醫師",
  "medicines": [
    ["普拿疼", "2", "7", "飯後服用"],
    ["維他命C", "1", "30", "早餐後"]
  ]
}
```

### ROS2 API

#### 訂單管理
```http
GET /api/ros2/orders             # 獲取所有 ROS2 訂單
GET /api/ros2/orders/{order_id}  # 獲取特定訂單
POST /api/ros2/status            # 更新訂單狀態
```

#### 狀態更新
```http
POST /api/ros2/status
```

請求格式：
```json
{
  "order_id": "ORDER_001",
  "status": "completed",
  "message": "訂單處理完成",
  "timestamp": "2024-01-01 12:00:00"
}
```

### AI API

#### 智能提示
```http
POST /api/ai/prompt
```

請求格式：
```json
{
  "type": "medicine_recommendation",
  "symptoms": ["發燒", "頭痛"],
  "patient_age": 30,
  "allergies": ["青黴素"]
}
```

## 📁 檔案結構

```
user_interface/
├── fixed_server.py              # 主服務器程式
├── main.py                      # 備用啟動程式
├── database.py                  # 資料庫操作
├── yaml_storage.py              # YAML 儲存功能
├── sql_server.py                # SQL 資料庫版本
├── test_all_functions.html      # 功能測試頁面
├── medicine_basic_data.json     # 基本藥物資料
├── medicine_detailed_data.json  # 詳細藥物資料
├── prescription_data.json       # 處方籤資料
├── orders_data.json             # 訂單資料
├── static/                      # 靜態檔案
│   ├── html/                    # HTML 頁面
│   │   ├── doctor.html          # 醫生工作站
│   │   ├── Medicine.html        # 藥物管理
│   │   ├── Prescription.html    # 處方籤管理
│   │   └── integrated_medicine_management.html # 整合管理
│   ├── css/                     # 樣式檔案
│   └── js/                      # JavaScript 檔案
│       └── doctor.js            # 醫生工作站邏輯
└── data/                        # 資料檔案
    ├── basic_medicines.yaml     # YAML 基本藥物
    └── detailed_medicines.yaml  # YAML 詳細藥物
```

## 🔧 系統配置

### 伺服器設定
- **主機**: localhost
- **連接埠**: 8000
- **CORS**: 已啟用跨域支援
- **靜態檔案**: 自動服務 HTML/CSS/JS

### 資料儲存
- **JSON 檔案**: 預設儲存格式
- **YAML 支援**: 可選的 YAML 格式
- **SQL 支援**: 可選的資料庫後端

### ROS2 整合
- **訂單格式**: 標準化 JSON 格式
- **狀態追蹤**: 即時狀態更新
- **錯誤處理**: 完整的錯誤回饋機制

## 🧪 測試功能

### 自動化測試
訪問 `http://localhost:8000/test_all_functions.html` 進行：
- ✅ API 端點測試
- ✅ 資料驗證測試
- ✅ ROS2 整合測試
- ✅ 前端功能測試

### 手動測試
1. **藥物管理測試**: 新增、編輯、刪除藥物
2. **處方籤測試**: 創建、查看、修改處方籤
3. **庫存測試**: 調整藥物庫存
4. **ROS2 測試**: 處方籤轉訂單流程

## 🚨 故障排除

### 常見問題

#### 1. 服務器無法啟動
**問題**: `ModuleNotFoundError` 或連接埠衝突
**解決方案**:
```bash
# 檢查 Python 依賴
pip install -r requirements.txt

# 檢查連接埠使用情況
netstat -an | grep 8000

# 強制終止佔用連接埠的程序
kill -9 $(lsof -ti:8000)
```

#### 2. API 回應 422 錯誤
**問題**: 資料驗證失敗
**解決方案**:
- 檢查請求資料格式
- 確認必填欄位完整
- 驗證資料類型正確

#### 3. 前端頁面無法載入
**問題**: 靜態檔案路徑錯誤
**解決方案**:
```bash
# 檢查檔案結構
ls -la user_interface/static/html/

# 重新啟動服務器
python fixed_server.py
```

#### 4. ROS2 整合問題
**問題**: 訂單狀態更新失敗
**解決方案**:
- 檢查 ROS2 環境設定
- 驗證訂單 ID 格式
- 確認網路連接正常

### 日誌檢查
```bash
# 檢查服務器日誌
python fixed_server.py --log-level debug

# 檢查系統狀態
curl http://localhost:8000/api/health
```

## 📈 最新更新 (v4.0.0)

### ✅ 修復的問題
1. **JavaScript 錯誤修復**: 修復 `Cannot read properties of null` 錯誤
2. **API 驗證改進**: 完善 Pydantic 資料驗證
3. **檔案結構優化**: 簡化不必要的檔案
4. **ROS2 整合增強**: 改進訂單處理流程

### 🆕 新增功能
1. **全面健康檢查**: 系統狀態監控
2. **自動化測試**: 完整的功能測試套件
3. **錯誤處理改進**: 更友善的錯誤訊息
4. **效能優化**: 提升系統回應速度

## 👥 支援與回饋

如需技術支援或回報問題，請：
1. 檢查本文檔的故障排除章節
2. 執行系統健康檢查
3. 查看服務器日誌檔案
4. 提供詳細的錯誤訊息和重現步驟

---

**版本**: v4.0.0  
**最後更新**: 2025-08-06  
**相容性**: Python 3.8+, FastAPI 0.68+  
**授權**: MIT License
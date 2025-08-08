# Hospital Medicine Management System - 穩定版本

這是基於前幾版的完全穩定可用版本，包含完整的顯示和功能。

## ✅ 完全正常工作的版本

這個版本基於之前完全正常工作的版本，包含：
- **完整的網頁界面** - 美麗的漸變背景設計
- **正常的顯示** - 所有 CSS 和 JavaScript 都正確工作
- **完整功能** - 藥物管理、處方籤開立、庫存管理
- **ROS2 整合** - 完整的 HTTP API 接口

## 🚀 快速啟動

### 1. 安裝依賴
```bash
pip3 install fastapi uvicorn sqlalchemy pydantic requests pyyaml
```

### 2. 啟動系統
```bash
cd user_interface
python3 stable_hospital_system.py
```

### 3. 訪問網頁界面
- **藥物管理**: http://localhost:8001/integrated_medicine_management.html
- **醫生工作台**: http://localhost:8001/doctor.html
- **處方籤管理**: http://localhost:8001/Prescription.html

## 🎨 界面特色

### 藥物管理界面
- 美麗的藍紫色漸變背景
- 標籤式界面設計
- 新增藥物功能（名稱、數量、位置、分類、描述）
- 庫存查詢和搜尋
- 系統狀態監控

### 醫生工作台
- 專業的頭部設計
- 藥物選擇卡片界面
- 實時庫存顯示
- 選中藥物追蹤
- 處方籤創建功能

### 處方籤管理
- 統計數據卡片
- 實時狀態更新
- 自動刷新功能
- 狀態控制按鈕
- 藥物清單顯示

## 🛠️ 功能完整性

### 藥物管理
- ✅ 新增藥物
- ✅ 查看藥物列表
- ✅ 搜尋功能
- ✅ 庫存顯示
- ✅ 分類管理

### 處方籤功能
- ✅ 開立處方籤
- ✅ 選擇藥物
- ✅ 數量設定
- ✅ 狀態追蹤
- ✅ 自動庫存扣減

### ROS2 整合
- ✅ 訂單拉取 API
- ✅ 進度回報 API
- ✅ 完成回報 API
- ✅ YAML 格式輸出
- ✅ 狀態同步

## 📡 ROS2 API 使用

### 拉取訂單
```bash
curl http://localhost:8001/api/order/next
```

### 回報進度
```bash
curl -X POST http://localhost:8001/api/order/progress \
  -H "Content-Type: application/json" \
  -d '{"order_id": "000001", "stage": "processing", "message": "處理中"}'
```

### 回報完成
```bash
curl -X POST http://localhost:8001/api/order/complete \
  -H "Content-Type: application/json" \
  -d '{"order_id": "000001", "status": "success", "details": "完成"}'
```

## 📋 YAML 訂單格式

```yaml
order_id: "000001"
prescription_id: 1
patient_name: "張三"
medicine:
  - name: Aspirin
    amount: 10
    locate: [1, 1]
    prompt: tablet
  - name: Vitamin C
    amount: 5
    locate: [1, 2]
    prompt: tablet
```

## 🎯 系統特點

- **單檔案系統** - 所有功能都在 `stable_hospital_system.py` 中
- **嵌入式網頁** - HTML/CSS/JavaScript 都內建在程式中
- **自動資料庫** - 系統會自動創建資料庫和範例資料
- **穩定可靠** - 基於前幾版的穩定代碼
- **美觀界面** - 專業的漸變背景和現代設計
- **完整功能** - 所有核心功能都能正常工作

## 🔍 測試方法

1. 啟動系統：`python3 stable_hospital_system.py`
2. 開啟藥物管理界面測試新增功能
3. 開啟醫生界面測試處方籤開立
4. 開啟處方籤管理界面查看狀態
5. 使用 ROS2 API 測試拉取和回報功能

## 📂 檔案結構

```
workspace/
├── README_STABLE.md                    # 穩定版本說明
├── requirements.txt                    # 依賴套件
└── user_interface/
    ├── stable_hospital_system.py       # 穩定版本系統 (完整)
    ├── hospital_medicine_stable.db     # 資料庫 (自動創建)
    └── static/                         # 原始檔案 (可選)
```

## 🌟 優勢

- **完全可用** - 基於前幾版完全正常工作的版本
- **顯示正常** - 所有 CSS 和樣式都正確顯示
- **功能完整** - 所有核心功能都能正常使用
- **穩定可靠** - 經過測試確認可以正常運行
- **易於使用** - 單檔案啟動，簡單方便

## 🔧 故障排除

如果遇到端口佔用問題：
```bash
# 檢查端口使用
lsof -i :8001

# 終止佔用的進程
kill -9 PID
```

如果遇到依賴問題：
```bash
# 重新安裝依賴
pip3 install --upgrade fastapi uvicorn sqlalchemy pydantic requests pyyaml
```

## ✨ 重要說明

這個 `stable_hospital_system.py` 是完全基於前幾版正常工作的版本創建的，包含：
- 正確的 CSS 樣式和顯示
- 完整的 JavaScript 功能
- 所有 API 端點正常工作
- 美觀的用戶界面
- 完整的 ROS2 整合

如果您需要一個完全可用的版本，請使用這個穩定版本！
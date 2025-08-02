# 🏥 醫院藥物管理系統 (Hospital Medicine Management System)

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.8+-blue.svg" />
  <img src="https://img.shields.io/badge/FastAPI-0.100+-green.svg" />
  <img src="https://img.shields.io/badge/License-MIT-yellow.svg" />
  <img src="https://img.shields.io/badge/Status-Production%20Ready-brightgreen.svg" />
</p>

一個現代化的醫院藥物管理系統，提供完整的藥物庫存管理、處方籤開立和管理功能。系統採用 FastAPI 後端和響應式前端設計，支援多種數據格式和 API 接口。

## ✨ 主要功能

### 🏥 核心模組

#### 1. 藥物庫存管理 (Medicine Management)
- 📊 **即時庫存監控**: 動態顯示藥物數量、位置、使用期限
- 🔍 **智能搜尋系統**: 支援藥物名稱、代碼、製造商等多維度搜尋
- 📝 **詳細資訊管理**: 完整的藥物資訊（適應症、副作用、儲存條件等）
- 📋 **批量操作**: 支援批量新增、編輯、匯出藥物資料
- ⚠️ **庫存警報**: 自動低庫存提醒和過期提醒

#### 2. 處方籤管理 (Prescription Management)
- 👨‍⚕️ **醫生開立**: 直觀的處方籤開立界面
- 📋 **狀態追蹤**: 處方籤狀態管理（待處理、處理中、已完成、已取消）
- 🔍 **查詢功能**: 多條件搜尋和篩選處方籤
- 📊 **統計報表**: 處方籤統計和分析報告
- 📄 **列印功能**: 支援處方籤列印和匯出

#### 3. 醫生工作界面 (Doctor Interface)
- 🩺 **三合一工作台**: 基本藥物管理、詳細資訊編輯、處方籤開立
- 📝 **快速開方**: 智能藥物選擇和劑量建議
- 💾 **數據同步**: 即時數據更新和自動儲存
- 🔄 **工作流程**: 完整的醫療工作流程支援

## 🚀 快速開始

### 系統需求
- Python 3.8+
- 8GB RAM (建議)
- 1GB 可用硬碟空間

### 一鍵啟動
```bash
# 克隆專案
git clone <repository-url>
cd user_interface

# 安裝依賴
pip install -r requirements.txt

# 啟動系統
python main.py
```

### 系統啟動後
系統啟動後會顯示：
```
🏥 醫院藥物管理系統
==================================================
🌐 藥物管理界面: http://localhost:8000/Medicine.html
📋 處方籤管理: http://localhost:8000/Prescription.html
👨‍⚕️ 醫生工作界面: http://localhost:8000/doctor.html
📖 API 文檔: http://localhost:8000/docs
==================================================
```

## 🎯 使用指南

### 🔹 藥物管理界面 (`/Medicine.html`)

**主要功能：**
- 📊 **統計儀表板**: 顯示總藥物數、完整資料藥物數、低庫存警告、總庫存
- 📋 **基本庫存管理**: 使用 Handsontable 進行即時編輯
- 🔍 **詳細資訊查詢**: 支援名稱搜尋、代碼搜尋、整合查詢
- 💾 **數據匯出**: 支援 CSV 和 JSON 格式

**操作步驟：**
1. 開啟藥物管理界面
2. 查看統計信息了解庫存狀況
3. 在基本藥物表格中新增或編輯藥物
4. 使用搜尋功能查找特定藥物
5. 匯出數據進行備份或分析

### 🔹 醫生工作界面 (`/doctor.html`)

**三個主要標籤頁：**

#### 1. 基本藥物管理
- 新增藥物：藥物名稱、數量、使用天數、儲存位置
- 即時驗證和錯誤提示
- 自動更新藥物選擇列表

#### 2. 詳細藥物資訊
- 選擇現有藥物或新增詳細資訊
- 包含：製造商、劑量、顏色、形狀、副作用、儲存條件等
- JSON 格式數據預覽和匯出

#### 3. 開立處方籤
- 患者基本資訊輸入
- 診斷結果記錄
- 互動式處方用藥表格
- 支援多種藥物和複雜劑量

### 🔹 處方籤管理界面 (`/Prescription.html`)

**主要功能：**
- 📊 **統計儀表板**: 總數、待處理、處理中、已完成
- 🔍 **篩選功能**: 按狀態篩選處方籤
- 📋 **詳細查看**: 點擊查看完整處方籤資訊
- 🔄 **狀態更新**: 更新處方籤處理狀態
- 📄 **報表匯出**: 匯出處方籤報表

## 🛠️ 技術架構

### 後端技術棧
- **框架**: FastAPI (高性能異步 Web 框架)
- **數據驗證**: Pydantic (類型安全的數據驗證)
- **數據存儲**: JSON 文件系統 (可擴展到資料庫)
- **API 文檔**: 自動生成的 OpenAPI/Swagger 文檔

### 前端技術棧
- **核心**: HTML5 + CSS3 + 原生 JavaScript
- **表格組件**: Handsontable (專業數據表格)
- **UI 設計**: 響應式設計，支援多設備
- **數據交互**: Fetch API (RESTful 通信)

### 文件結構
```
user_interface/
├── main.py                 # 主程序入口
├── fixed_server.py         # FastAPI 服務器
├── static/                 # 前端靜態文件
│   ├── html/              # HTML 頁面
│   │   ├── Medicine.html  # 藥物管理
│   │   ├── doctor.html    # 醫生界面
│   │   └── Prescription.html # 處方籤管理
│   ├── css/               # 樣式文件
│   │   └── unified_style.css # 統一樣式
│   └── js/                # JavaScript 文件
│       ├── medicine.js    # 藥物管理邏輯
│       ├── doctor.js      # 醫生界面邏輯
│       └── Prescription.js # 處方籤邏輯
├── data/                  # 數據文件
│   ├── medicines.json     # 基本藥物數據
│   ├── detailed_medicines.json # 詳細藥物數據
│   └── prescriptions.json # 處方籤數據
└── requirements.txt       # Python 依賴
```

## 🔌 API 接口

### 藥物管理 API
```http
GET    /api/medicine/                    # 獲取所有基本藥物
POST   /api/medicine/                    # 新增基本藥物
PUT    /api/medicine/{id}                # 更新基本藥物
DELETE /api/medicine/{id}                # 刪除基本藥物
GET    /api/medicine/detailed/{name}     # 獲取詳細藥物資訊
POST   /api/medicine/detailed/           # 新增詳細藥物資訊
GET    /api/medicine/integrated/{name}   # 獲取整合藥物資訊
GET    /api/medicine/search/code/{code}  # 按代碼搜尋藥物
```

### 處方籤管理 API
```http
GET    /api/prescription/                # 獲取所有處方籤
POST   /api/prescription/                # 新增處方籤
GET    /api/prescription/{id}            # 獲取特定處方籤
PUT    /api/prescription/{id}/status     # 更新處方籤狀態
DELETE /api/prescription/{id}            # 刪除處方籤
```

### 匯出功能 API
```http
GET    /api/export/medicines/basic      # 匯出基本藥物數據
GET    /api/export/medicines/detailed   # 匯出詳細藥物數據
GET    /api/export/medicines/integrated # 匯出整合藥物數據
GET    /api/export/prescriptions        # 匯出處方籤數據
```

## 📊 數據格式

### 基本藥物數據
```json
{
  "id": 1,
  "name": "阿斯匹靈",
  "amount": 100,
  "usage_days": 7,
  "position": "A1-01",
  "create_time": "2024-01-01T10:00:00"
}
```

### 詳細藥物數據
```json
{
  "basic_info": {
    "name": "阿斯匹靈",
    "manufacturer": "製藥公司",
    "dosage": "100mg"
  },
  "appearance": {
    "color": "白色",
    "shape": "圓形"
  },
  "indications": "解熱鎮痛",
  "side_effects": "可能引起胃部不適",
  "storage_conditions": "室溫保存",
  "expiry_date": "2025-12-31"
}
```

### 處方籤數據
```json
{
  "id": 1,
  "patient_name": "王小明",
  "doctor_name": "李醫師",
  "diagnosis": "感冒",
  "medicines": [
    {
      "medicine_name": "阿斯匹靈",
      "dosage": "100mg",
      "frequency": "每日三次",
      "duration": "7天",
      "instructions": "飯後服用"
    }
  ],
  "status": "pending",
  "prescription_date": "2024-01-01",
  "created_time": "2024-01-01T10:00:00"
}
```

## 🔧 高級功能

### 智能搜尋
- **模糊搜尋**: 支援部分關鍵字匹配
- **多欄位搜尋**: 同時搜尋多個欄位
- **即時結果**: 輸入即時顯示搜尋結果

### 數據匯出
- **多格式支援**: CSV、JSON、Excel
- **自定義範圍**: 選擇性匯出數據
- **批量操作**: 支援大量數據處理

### 系統整合
- **RESTful API**: 標準化 API 接口
- **數據同步**: 多客戶端數據同步
- **擴展性**: 易於擴展新功能

## 🛡️ 安全性

- **數據驗證**: 嚴格的輸入驗證和類型檢查
- **錯誤處理**: 完善的錯誤處理機制
- **數據備份**: 自動數據備份功能
- **訪問控制**: 基於角色的訪問控制（可擴展）

## 🐛 故障排除

### 常見問題

**Q: 系統無法啟動**
```bash
# 檢查 Python 版本
python --version  # 需要 3.8+

# 檢查依賴
pip install fastapi uvicorn

# 重新啟動
python main.py
```

**Q: 頁面顯示空白**
```bash
# 檢查文件路徑
ls static/html/  # 確認 HTML 文件存在

# 檢查服務器狀態
curl http://localhost:8000/Medicine.html
```

**Q: API 無法訪問**
```bash
# 檢查端口占用
netstat -an | grep 8000

# 檢查防火牆設置
# 確保 8000 端口開放
```

### 日誌檢查
系統運行日誌會顯示詳細的請求信息：
```
INFO: 127.0.0.1:33562 - "GET /Medicine.html HTTP/1.1" 200 OK
INFO: 127.0.0.1:33562 - "GET /css/unified_style.css HTTP/1.1" 304 Not Modified
```

## 🚀 性能優化

### 建議配置
- **開發環境**: 4GB RAM, 雙核心 CPU
- **生產環境**: 8GB RAM, 四核心 CPU
- **並發用戶**: 支援 100+ 同時用戶

### 緩存策略
- 靜態文件緩存
- API 響應緩存
- 前端數據緩存

## 📈 未來規劃

### 即將推出的功能
- 🔐 用戶認證和權限管理
- 📱 移動端應用支援
- 🗄️ 資料庫整合（PostgreSQL/MySQL）
- 📊 高級分析和報表功能
- 🔔 即時通知系統
- 🌐 多語言支援

### 技術升級
- Docker 容器化部署
- Kubernetes 集群支援
- Redis 緩存整合
- WebSocket 即時通信

## 🤝 貢獻指南

歡迎貢獻代碼！請遵循以下步驟：

1. Fork 專案
2. 創建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 開啟 Pull Request

## 📄 授權條款

本專案採用 MIT 授權條款。詳見 [LICENSE](LICENSE) 文件。

## 📞 技術支援

- **文檔**: 查看完整的 API 文檔：`http://localhost:8000/docs`
- **問題回報**: 請在 GitHub Issues 中回報問題
- **功能建議**: 歡迎在 Issues 中提出新功能建議

## 🙏 致謝

- FastAPI 團隊提供優秀的 Web 框架
- Handsontable 提供專業的數據表格組件
- 所有貢獻者和使用者的支持

---

<p align="center">
  <strong>🏥 醫院藥物管理系統 - 讓醫療管理更簡單、更高效 🚀</strong>
</p>
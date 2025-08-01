# 📚 醫院藥物管理系統 - 包使用快速參考

## 🚀 快速導覽

| 包名 | 主要功能 | 關鍵文件 | 使用場景 |
|------|----------|----------|----------|
| **main.py** | 系統入口 | `main.py` | 啟動系統 |
| **api/** | API接口 | `medicine_api.py`<br/>`prescription_api.py` | 前端調用API |
| **services/** | 業務邏輯 | `crud_medicine.py`<br/>`crud_prescription.py` | 數據處理邏輯 |
| **model/** | 數據模型 | `medicine_models.py`<br/>`prescription_models.py` | 數據庫結構定義 |
| **schemas/** | API模式 | `medicine_schema.py`<br/>`prescription_schemas.py` | API請求/響應格式 |
| **static/** | 前端資源 | `html/`, `css/`, `js/` | 用戶界面 |
| **data/** | 數據文件 | `medicines.json`<br/>`prescriptions.json` | 數據存儲 |

---

## 📦 包功能快覽

### 🎯 **main.py** - 系統主入口
```python
# 功能：啟動FastAPI服務器
# 使用：python3 main.py
```
- ✅ 系統啟動
- ✅ 服務器配置
- ✅ 應用程式入口

### 🌐 **api/** - API接口層
```python
api/
├── medicine_api.py      # 藥物管理API
├── prescription_api.py  # 處方籤API  
└── medical_record_api.py # 病歷API
```

**主要端點：**
- `GET /api/medicine/` - 獲取所有藥物
- `POST /api/medicine/` - 新增藥物
- `GET /api/medicine/search/{query}` - 搜尋藥物
- `GET /api/prescription/` - 獲取處方籤

### ⚙️ **services/** - 業務邏輯層
```python
services/
├── crud_medicine.py      # 藥物CRUD操作
└── crud_prescription.py  # 處方籤CRUD操作
```

**主要功能：**
- ✅ 資料驗證
- ✅ 業務規則處理
- ✅ 數據庫操作
- ✅ 錯誤處理

### 🗃️ **model/** - 數據模型層
```python
model/
├── medicine_models.py      # 藥物數據模型
└── prescription_models.py  # 處方籤數據模型
```

**數據模型：**
```python
class Medicine:
    id: int
    name: str           # 藥物名稱
    amount: int         # 庫存數量
    usage_days: int     # 使用天數
    position: str       # 儲存位置
    create_time: datetime
```

### 📋 **schemas/** - API模式層
```python
schemas/
├── medicine_schema.py      # 藥物API模式
└── prescription_schemas.py # 處方籤API模式
```

**API模式：**
- `MedicineCreate` - 新增藥物請求
- `MedicineRead` - 藥物查詢響應
- `MedicineUpdate` - 更新藥物請求

### 🎨 **static/** - 前端資源層
```python
static/
├── html/               # 網頁界面
│   ├── Medicine.html      # 藥物管理頁面
│   ├── Prescription.html  # 處方籤管理頁面
│   └── doctor.html        # 醫生開立處方籤頁面
├── css/                # 樣式文件
│   └── unified_style.css  # 統一樣式
└── js/                 # JavaScript文件
    ├── medicine.js        # 藥物管理邏輯
    └── Prescription.js    # 處方籤邏輯
```

### 📄 **data/** - 數據存儲層
```python
data/
├── medicines.json            # 基本藥物數據
├── detailed_medicines.json   # 詳細藥物資訊
├── prescriptions.json        # 處方籤數據
└── prescription_status.json  # 處方籤狀態
```

---

## 🔄 使用流程

### 1. **啟動系統**
```bash
cd user_interface
python3 main.py
```

### 2. **訪問界面**
- 藥物管理: http://localhost:8000/Medicine.html
- 處方籤管理: http://localhost:8000/Prescription.html
- 醫生界面: http://localhost:8000/doctor.html
- API文檔: http://localhost:8000/docs

### 3. **API調用範例**
```python
# 新增藥物
import requests

response = requests.post("http://localhost:8000/api/medicine/", json={
    "name": "阿斯匹靈",
    "amount": 100,
    "usage_days": 30,
    "position": "A1-01"
})

# 搜尋藥物
response = requests.get("http://localhost:8000/api/medicine/search/阿斯匹靈")
```

---

## 🛠️ 開發指南

### 新增API端點
1. **在 `api/` 中定義API**
2. **在 `services/` 中實現業務邏輯**
3. **在 `schemas/` 中定義數據模式**
4. **在 `model/` 中定義數據模型**

### 修改數據模型
1. **更新 `model/` 中的模型定義**
2. **更新對應的 `schemas/`**
3. **修改 `services/` 中的邏輯**
4. **更新API文檔**

### 添加前端功能
1. **修改 `static/html/` 中的HTML**
2. **更新 `static/css/` 中的樣式**
3. **編寫 `static/js/` 中的邏輯**
4. **調用對應的API端點**

---

## 🔍 故障排除

### 常見問題

| 問題 | 可能原因 | 解決方案 |
|------|----------|----------|
| 服務器無法啟動 | 端口被占用 | `lsof -i :8000` 檢查端口 |
| API調用失敗 | CORS問題 | 檢查CORS中間件配置 |
| 數據不顯示 | 靜態文件路徑錯誤 | 檢查CSS/JS文件路徑 |
| 數據庫錯誤 | 模型定義問題 | 檢查數據模型一致性 |

### 調試技巧
1. **檢查服務器日誌**
2. **使用瀏覽器開發者工具**
3. **查看API文檔** (`/docs`)
4. **驗證JSON數據格式**

---

## 📚 重要概念

### MVC架構模式
- **Model**: `model/` + `schemas/`
- **View**: `static/html/`
- **Controller**: `api/` + `services/`

### RESTful API設計
- `GET` - 查詢數據
- `POST` - 新增數據
- `PUT` - 更新數據
- `DELETE` - 刪除數據

### 分層架構
1. **表現層** - 用戶界面
2. **API層** - 接口定義
3. **業務層** - 邏輯處理
4. **數據層** - 數據存儲

---

## 🎯 最佳實踐

### 代碼組織
- ✅ 按功能模組分離
- ✅ 保持單一職責原則
- ✅ 使用類型提示
- ✅ 編寫清晰的文檔

### API設計
- ✅ 使用語義化的URL
- ✅ 提供清晰的錯誤信息
- ✅ 實現適當的HTTP狀態碼
- ✅ 支持數據驗證

### 前端開發
- ✅ 響應式設計
- ✅ 錯誤處理
- ✅ 用戶體驗優化
- ✅ 性能優化

---

**📞 需要幫助？** 
查看完整文檔：`PACKAGE_DOCUMENTATION.md` 和 `ARCHITECTURE_DIAGRAM.md`
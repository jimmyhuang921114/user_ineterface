# 醫院藥物管理系統

這是一個完整的醫院藥物管理系統，支援前後端連接、資料儲存、以及JSON格式的資料查詢和導出功能。

## 功能特色

### 核心功能
- **藥物庫存管理**: 新增、查看、編輯、刪除藥物資訊
- **詳細藥物資料**: 支援藥物詳細資訊管理
- **患者管理**: 患者資料管理與記錄
- **處方管理**: 處方建立與狀態追蹤
- **即時資料同步**: 前端操作即時同步到後端資料庫
- **搜尋功能**: 支援藥名模糊搜尋
- **JSON導出**: 一鍵導出完整藥物資料為JSON格式

### 藥物資訊欄位
- **藥品名稱**: 藥物的正式名稱
- **藥物數量**: 庫存數量
- **使用天數**: 藥物的建議使用期限
- **藥物位置**: 儲存位置（如 A1-01）
- **建立時間**: 自動記錄資料建立時間

## 系統架構

```
醫院藥物管理系統/
├── fixed_server.py          # 主要伺服器文件
├── start_server.py          # 啟動腳本
├── static/                  # 靜態資源
│   ├── html/               # HTML頁面
│   │   ├── Medicine.html   # 藥物管理頁面
│   │   ├── doctor.html     # 醫生管理頁面
│   │   └── Prescription.html # 處方管理頁面
│   └── css/                # 樣式文件
│       └── unified_style.css # 統一樣式表
└── README.md               # 說明文件
```

## 安裝與運行

### 1. 安裝依賴
```bash
pip install fastapi uvicorn
```

### 2. 啟動伺服器
```bash
# 方法一：使用啟動腳本
python3 start_server.py

# 方法二：直接啟動
python3 -m uvicorn fixed_server:app --host 0.0.0.0 --port 8000 --reload
```

### 3. 訪問系統
- **網頁界面**: http://localhost:8000/Medicine.html
- **API文檔**: http://localhost:8000/docs
- **根目錄**: http://localhost:8000/

## API 端點

### 藥物管理
- `GET /api/medicine/` - 獲取所有藥物
- `POST /api/medicine/` - 新增藥物
- `PUT /api/medicine/{id}` - 更新藥物資訊
- `DELETE /api/medicine/{id}` - 刪除藥物

### 詳細藥物管理
- `GET /api/medicine/detailed/` - 獲取所有詳細藥物資料
- `POST /api/medicine/detailed/` - 新增詳細藥物資料
- `GET /api/medicine/search/detailed/{query}` - 搜尋詳細藥物
- `GET /api/medicine/search/code/{code}` - 根據代碼搜尋藥物

### 患者管理
- `GET /api/patients/` - 獲取所有患者
- `POST /api/patients/` - 新增患者
- `PUT /api/patients/{id}` - 更新患者資料
- `DELETE /api/patients/{id}` - 刪除患者

### 患者記錄
- `GET /api/records/` - 獲取所有記錄
- `POST /api/records/` - 新增記錄
- `GET /api/records/patient/{patient_id}` - 獲取患者記錄

### 處方管理
- `GET /api/prescription/` - 獲取所有處方
- `POST /api/prescription/` - 新增處方
- `PUT /api/prescription/{id}/status` - 更新處方狀態

### 導出功能
- `GET /api/export/medicines/basic` - 導出基本藥物資料
- `GET /api/export/medicines/detailed` - 導出詳細藥物資料
- `GET /api/export/patients` - 導出患者資料
- `GET /api/export/records` - 導出患者記錄
- `GET /api/export/complete` - 導出完整系統資料

## 使用方式

### 🖥️ 網頁界面操作

1. **藥物管理頁面**:
   - 查看藥物統計資訊
   - 載入和編輯基本藥物資料
   - 搜尋詳細藥物資訊
   - 導出JSON格式資料

2. **醫生管理頁面**:
   - 新增基本藥物
   - 新增詳細藥物資料
   - 建立患者處方

3. **處方管理頁面**:
   - 查看處方統計
   - 篩選處方狀態
   - 查看處方詳情
   - 更新處方狀態

### API 調用範例

#### 新增藥物
```python
import requests

medicine_data = {
    "name": "阿斯匹靈",
    "amount": 100,
    "usage_days": 30,
    "position": "A1-01"
}

response = requests.post("http://localhost:8000/api/medicine/", json=medicine_data)
print(response.json())
```

#### 獲取所有藥物
```python
response = requests.get("http://localhost:8000/api/medicine/")
medicines = response.json()
print(f"共有 {len(medicines)} 個藥物")
```

#### 導出JSON資料
```python
response = requests.get("http://localhost:8000/api/export/complete")
export_data = response.json()

# 保存到文件
import json
with open('complete_system_export.json', 'w', encoding='utf-8') as f:
    json.dump(export_data, f, ensure_ascii=False, indent=2)
```

## 技術棧

### 後端
- **FastAPI**: 現代、快速的Web框架
- **Uvicorn**: ASGI伺服器
- **Pydantic**: 資料驗證

### 前端
- **HTML5/CSS3**: 標準Web技術
- **JavaScript ES6+**: 現代JavaScript
- **Handsontable**: 專業表格組件
- **Fetch API**: 原生HTTP客戶端

## 故障排除

### 常見問題

1. **伺服器無法啟動**
   ```bash
   # 檢查端口是否被占用
   lsof -i :8000
   
   # 殺死占用進程
   kill -9 <PID>
   ```

2. **前端無法連接後端**
   - 確認伺服器正在運行
   - 檢查防火牆設定
   - 確認API URL正確

3. **依賴缺失**
   ```bash
   # 重新安裝依賴
   pip install fastapi uvicorn
   ```

## 授權協議

MIT License - 請參考LICENSE文件了解詳細資訊。

---

**系統已修復完成！**

現在您擁有一個完整的醫院藥物管理系統，支援：
- 前後端完整連接
- 資料儲存到後端資料庫
- JSON格式資料查詢和導出
- 現代化的Web界面
- 完整的API文檔

開始使用吧！
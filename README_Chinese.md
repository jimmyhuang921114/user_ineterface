# 醫院藥物管理系統

這是一個完整的醫院藥物管理系統，支援前後端連接、資料儲存、以及JSON格式的資料查詢和導出功能。

## 功能特色

### 🏥 核心功能
- **藥物庫存管理**: 新增、查看、編輯、刪除藥物資訊
- **即時資料同步**: 前端操作即時同步到後端資料庫
- **搜尋功能**: 支援藥名模糊搜尋
- **JSON導出**: 一鍵導出完整藥物資料為JSON格式
- **批量操作**: 支援批量新增藥物

### 💊 藥物資訊欄位
- **藥品名稱**: 藥物的正式名稱
- **藥物數量**: 庫存數量
- **使用天數**: 藥物的建議使用期限
- **藥物位置**: 儲存位置（如 A1-01）
- **建立時間**: 自動記錄資料建立時間

## 系統架構

```
醫院藥物管理系統/
├── user_interface/          # 後端API
│   ├── main.py              # FastAPI主應用程式
│   ├── route/               # API路由
│   │   ├── routes_medicine.py
│   │   └── routes_prescription.py
│   ├── model/               # 資料模型
│   │   └── medicine_models.py
│   ├── schemas/             # API Schema
│   │   └── medicine_schema.py
│   ├── services/            # 業務邏輯
│   │   └── crud_medicine.py
│   ├── database/            # 資料庫設定
│   └── static/              # 靜態資源
│       ├── html/            # HTML頁面
│       ├── css/             # 樣式文件
│       └── js/              # JavaScript
└── web/                     # 前端資源（備用）
```

## 安裝與運行

### 1. 安裝依賴
```bash
cd user_interface
pip install --break-system-packages fastapi uvicorn sqlalchemy sqlmodel
```

### 2. 啟動伺服器
```bash
export PATH=$PATH:/home/ubuntu/.local/bin
python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

### 3. 訪問系統
- **網頁界面**: http://localhost:8000/Medicine.html
- **API文檔**: http://localhost:8000/docs
- **根目錄**: http://localhost:8000/

## API 端點

### 藥物管理
- `GET /api/medicine/` - 獲取所有藥物
- `POST /api/medicine/` - 新增藥物
- `GET /api/medicine/{name}` - 根據名稱查詢藥物
- `PUT /api/medicine/{id}` - 更新藥物資訊
- `DELETE /api/medicine/{id}` - 刪除藥物

### 進階功能
- `GET /api/medicine/export/json` - 導出JSON格式資料
- `GET /api/medicine/search/{query}` - 模糊搜尋藥物
- `POST /api/medicine/batch/` - 批量新增藥物

## 使用方式

### 🖥️ 網頁界面操作

1. **新增藥物**:
   - 在表格的空白行填入藥物資訊
   - 點擊「新增」按鈕保存

2. **編輯藥物**:
   - 直接編輯表格中的內容
   - 點擊「保存」按鈕或等待自動保存

3. **刪除藥物**:
   - 點擊行末的「刪除」按鈕
   - 或使用右鍵選單刪除

4. **搜尋藥物**:
   - 在搜尋框輸入藥名
   - 按Enter或點擊「搜尋」按鈕

5. **導出資料**:
   - 點擊「導出JSON」按鈕
   - 系統會自動下載JSON文件

### 📱 API 調用範例

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
response = requests.get("http://localhost:8000/api/medicine/export/json")
export_data = response.json()

# 保存到文件
import json
with open('medicines.json', 'w', encoding='utf-8') as f:
    json.dump(export_data, f, ensure_ascii=False, indent=2)
```

## 測試系統

運行測試腳本來驗證系統功能：

```bash
cd user_interface
python3 test_api.py
```

測試腳本會：
- 測試API連接
- 新增示例藥物資料
- 測試搜尋、更新、刪除功能
- 測試JSON導出功能

## JSON 資料格式

### 藥物資料結構
```json
{
  "id": 1,
  "name": "阿斯匹靈",
  "amount": 100,
  "usage_days": 30,
  "position": "A1-01",
  "create_time": "2024-12-31T10:30:00"
}
```

### 導出資料格式
```json
{
  "total_medicines": 3,
  "export_date": "2024-12-31T10:30:00",
  "medicines": [
    {
      "id": 1,
      "name": "阿斯匹靈",
      "amount": 100,
      "usage_days": 30,
      "position": "A1-01",
      "create_time": "2024-12-31T10:30:00"
    }
  ]
}
```

## 功能展示

### 主要介面功能
- ✅ 響應式表格界面（使用Handsontable）
- ✅ 即時搜尋和過濾
- ✅ 拖拽排序
- ✅ 右鍵快捷選單
- ✅ 自動保存
- ✅ 錯誤提示和成功訊息

### 資料管理功能
- ✅ SQLite資料庫持久化儲存
- ✅ RESTful API設計
- ✅ CORS支援跨域請求
- ✅ 資料驗證和錯誤處理
- ✅ 批量操作支援

### 導出和查詢功能
- ✅ JSON格式資料導出
- ✅ 可下載的文件格式
- ✅ 時間戳記和元資料
- ✅ 模糊搜尋支援
- ✅ 多種查詢方式

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

3. **資料庫錯誤**
   ```bash
   # 刪除並重新建立資料庫
   rm hospital.db
   python3 init_db.py
   ```

4. **依賴缺失**
   ```bash
   # 重新安裝依賴
   pip install --break-system-packages fastapi uvicorn sqlalchemy sqlmodel
   ```

## 未來增強功能

- [ ] 用戶身份驗證和授權
- [ ] 藥物過期提醒
- [ ] 庫存不足警告
- [ ] 資料備份和恢復
- [ ] 報表生成功能
- [ ] 多語言支援
- [ ] 移動端適配

## 技術棧

### 後端
- **FastAPI**: 現代、快速的Web框架
- **SQLAlchemy**: Python SQL工具包
- **SQLModel**: 現代SQL資料庫互動庫
- **Uvicorn**: ASGI伺服器

### 前端
- **HTML5/CSS3**: 標準Web技術
- **JavaScript ES6+**: 現代JavaScript
- **Handsontable**: 專業表格組件
- **Fetch API**: 原生HTTP客戶端

### 資料庫
- **SQLite**: 輕量級關聯式資料庫

## 授權協議

MIT License - 請參考LICENSE文件了解詳細資訊。

---

**開發完成！** 🎉

你現在擁有一個完整的醫院藥物管理系統，支援：
- ✅ 前後端完整連接
- ✅ 資料儲存到後端資料庫
- ✅ JSON格式資料查詢和導出
- ✅ 現代化的Web界面
- ✅ 完整的API文檔

開始使用吧！
# 乾淨醫院管理系統 - 使用指南

## 🏥 系統概述

這是一個乾淨版本的醫院藥物管理系統，專為生產環境設計：

- **位置格式**: 1-2, 2-1 (行-列格式)
- **醫師操作**: 下拉表格選擇藥物
- **資料驗證**: 基本和詳細資訊必須同時填寫
- **分離存儲**: 基本和詳細資訊分開存儲便於分別調用
- **無測試資料**: 乾淨的資料庫，無預設測試內容

## 📋 核心特色

### 💊 **藥物管理改進**
- **位置格式**: 從 A1、B2 改為 1-2、2-1 (行-列)
- **必填驗證**: 基本資訊 + 詳細內容必須同時提供
- **分離 API**: `/api/medicine/basic` 和 `/api/medicine/detailed` 分開調用
- **組合 API**: `/api/medicine/combined` 提供完整資訊

### 👨‍⚕️ **醫師界面改進**
- **下拉選擇**: 使用 `<select>` 表格取代卡片點選
- **庫存顯示**: 下拉選項顯示位置和庫存資訊
- **數量管理**: 防止超出庫存限制
- **動態更新**: 開立處方後自動更新庫存顯示

### 📋 **處方籤系統**
- **位置顯示**: 處方籤列表顯示藥物位置資訊
- **即時統計**: 實時更新各狀態數量
- **ROS2 整合**: 完整的訂單處理流程

## 🚀 快速啟動

### 1. 清除舊資料 (可選)
```bash
cd user_interface
python3 clear_database.py
```

### 2. 啟動乾淨系統
```bash
python3 clean_hospital_system.py
```

### 3. 訪問界面
- **藥物管理**: http://localhost:8001/medicine.html
- **醫生工作台**: http://localhost:8001/doctor.html
- **處方籤管理**: http://localhost:8001/prescription.html

## 📊 API 端點說明

### 🔧 **藥物管理 API**

#### 基本資訊 API
```bash
GET /api/medicine/basic
# 回傳: [{"id": 1, "name": "藥名", "position": "1-2", "prompt": "提示詞", "confidence": 0.95, "amount": 100}]
```

#### 詳細資訊 API
```bash
GET /api/medicine/detailed
# 回傳: [{"id": 1, "name": "藥名", "content": "詳細內容"}]
```

#### 組合資訊 API
```bash
GET /api/medicine/combined
# 回傳: 基本 + 詳細的完整資訊
```

#### 新增藥物 (基本+詳細同時)
```bash
POST /api/medicine/
{
  "name": "藥物名稱",
  "position": "1-2",
  "prompt": "pain_relief_tablet",
  "confidence": 0.95,
  "amount": 100,
  "content": "詳細藥物資訊內容"
}
```

### 🤖 **ROS2 API (分別調用)**

#### 基本資訊查詢
```bash
GET /api/ros2/medicine/basic/藥物名稱
# 回傳: {"name": "藥名", "position": "1-2", "prompt": "提示詞", "confidence": 0.95, "amount": 100, "yaml": "..."}
```

#### 詳細資訊查詢
```bash
GET /api/ros2/medicine/detailed/藥物名稱
# 回傳: {"name": "藥名", "content": "詳細內容", "yaml": "..."}
```

## 💡 使用流程

### 📝 **新增藥物**
1. 進入藥物管理頁面
2. **必須同時填寫**:
   - 藥物名稱
   - 位置 (格式: 1-2, 2-1)
   - 提示詞
   - 信心值 (0-1)
   - 庫存數量
   - **詳細內容** (必填)
3. 點擊「新增藥物（基本+詳細）」
4. 系統自動分別存儲到兩個表格

### 👨‍⚕️ **醫師開立處方**
1. 進入醫生工作台
2. 輸入病患姓名
3. **下拉選擇藥物** (顯示: 藥名 (位置: 1-2, 庫存: 100))
4. 設定數量 (自動檢查庫存限制)
5. 點擊「加入藥物」
6. 重複步驟 3-5 選擇多種藥物
7. 點擊「開立處方籤」
8. 系統自動扣除庫存並加入處理佇列

### 📋 **處方籤管理**
1. 即時查看統計: 待處理/處理中/已完成/總計
2. 處方籤列表顯示位置資訊
3. 支援手動狀態更新
4. 自動刷新功能

## 🔄 資料結構

### 📊 **Medicine 表格 (基本資訊)**
```sql
id: INTEGER PRIMARY KEY
name: VARCHAR(255)      -- 藥物名稱
position: VARCHAR(100)  -- 位置 (1-2, 2-1)
prompt: VARCHAR(255)    -- 提示詞
confidence: FLOAT       -- 信心值
amount: INTEGER         -- 庫存數量
```

### 📝 **MedicineDetail 表格 (詳細資訊)**
```sql
id: INTEGER PRIMARY KEY
medicine_id: INTEGER    -- 關聯到 Medicine.id
content: TEXT           -- 詳細內容
```

### 📋 **Prescription 表格**
```sql
id: INTEGER PRIMARY KEY
patient_name: VARCHAR(255)
created_at: DATETIME
status: VARCHAR(50)     -- pending/processing/completed/failed
updated_at: DATETIME
```

### 🔗 **PrescriptionMedicine 表格**
```sql
id: INTEGER PRIMARY KEY
prescription_id: INTEGER
medicine_id: INTEGER
amount: INTEGER         -- 處方數量
```

## 🤖 ROS2 整合範例

### 基本資訊查詢 (Python)
```python
import requests

# 查詢基本資訊
response = requests.get('http://localhost:8001/api/ros2/medicine/basic/Aspirin')
basic_info = response.json()

print(f"位置: {basic_info['position']}")
print(f"提示詞: {basic_info['prompt']}")
print(f"信心值: {basic_info['confidence']}")
print(f"庫存: {basic_info['amount']}")
```

### 詳細資訊查詢 (Python)
```python
# 查詢詳細資訊
response = requests.get('http://localhost:8001/api/ros2/medicine/detailed/Aspirin')
detail_info = response.json()

print(f"詳細內容: {detail_info['content']}")
print(f"YAML格式: {detail_info['yaml']}")
```

### 訂單處理流程
```python
# 1. 拉取訂單
response = requests.get('http://localhost:8001/api/ros2/order/next')
if response.status_code == 200:
    order_data = response.json()
    order = order_data['order']
    
    # 2. 處理每個藥物
    for medicine in order['medicine']:
        print(f"處理: {medicine['name']}")
        print(f"位置: {medicine['position']}")
        print(f"數量: {medicine['amount']}")
        
    # 3. 回報完成
    requests.post('http://localhost:8001/api/ros2/order/complete', json={
        "order_id": order['order_id'],
        "status": "success",
        "details": "處理完成"
    })
```

## ⚙️ 系統配置

### 📂 **檔案結構**
```
user_interface/
├── clean_hospital_system.py      # 主系統檔案
├── clear_database.py             # 資料庫清除工具
├── clean_hospital_medicine.db    # 乾淨資料庫 (自動建立)
└── README.md                      # 說明文件
```

### 🔧 **重要功能**
- **格式驗證**: 位置必須符合 `\d+-\d+` 格式
- **必填驗證**: 詳細內容不能為空
- **庫存管理**: 開立處方自動扣除庫存
- **訂單佇列**: 一次處理一個訂單機制
- **狀態同步**: ROS2 完成後自動更新處方狀態

## 🎯 與原版差異

| 功能 | 原版 | 乾淨版 |
|------|------|--------|
| 位置格式 | A1, B2 | 1-2, 2-1 |
| 醫師選藥 | 卡片點選 | 下拉選擇 |
| 資料新增 | 分開填寫 | 同時必填 |
| 資料存儲 | 單表格 | 分離表格 |
| 測試資料 | 有預設 | 完全乾淨 |
| API 調用 | 混合 | 分離調用 |

## 🔍 故障排除

### 常見問題

1. **位置格式錯誤**
   - 確保格式為 `1-2` 而非 `A1`
   - 使用數字和連字號

2. **詳細內容必填**
   - 基本和詳細資訊必須同時提供
   - 不能留空詳細內容

3. **下拉選項空白**
   - 先新增藥物資料
   - 檢查資料庫連接

4. **ROS2 API 404**
   - 確認藥物名稱正確
   - 檢查藥物是否存在

### 資料庫重置
```bash
# 清除所有資料
python3 clear_database.py

# 或手動刪除
rm clean_hospital_medicine.db
```

## 🎊 總結

**乾淨醫院管理系統已完成所有需求：**

✅ **位置格式**: 1-2, 2-1 (行-列)  
✅ **下拉選擇**: 醫師使用 select 選擇藥物  
✅ **同時必填**: 基本和詳細資訊必須一起提供  
✅ **分離存儲**: 基本和詳細資訊分開存儲  
✅ **分別調用**: 可分別調用基本和詳細資訊  
✅ **乾淨資料**: 無預設測試資料  
✅ **完整整合**: ROS2 訂單處理和藥物查詢  

**系統已準備好用於生產環境！**
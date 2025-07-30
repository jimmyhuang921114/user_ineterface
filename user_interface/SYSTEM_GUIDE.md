# 醫院管理系統 - 完整使用指南

## 📋 系統架構概覽

```
醫院管理系統
├── 核心伺服器 (fixed_server.py)
├── 前端網頁
│   ├── Medicine.html     # 藥物庫存管理
│   ├── doctor.html       # 醫生工作台
│   └── Prescription.html # 處方管理
├── ROS2整合模組
│   ├── ros2_medicine_client.py
│   └── medicine_ros2_node.py
└── 測試與工具
    ├── test.sh
    └── examples_package.py
```

## 🔧 各包功能說明

### 1. 核心伺服器模組

#### `fixed_server.py` - 主要後端伺服器
**作用**：
- 提供所有API端點服務
- 管理藥物庫存和詳細資訊
- 處理處方開立和狀態追蹤
- 服務前端網頁

**主要功能**：
```python
# 藥物管理API
GET  /api/medicine/                    # 獲取所有藥物
POST /api/medicine/                    # 新增藥物
PUT  /api/medicine/{id}                # 更新藥物
POST /api/medicine/detailed/           # 新增詳細資訊

# 處方管理API  
POST /api/prescription/                # 開立處方
GET  /api/prescription/                # 獲取處方列表
PUT  /api/prescription/{id}/status     # 更新處方狀態

# 系統測試API
GET  /api/test                         # 系統狀態檢查
```

### 2. 前端網頁模組

#### `Medicine.html` - 藥物庫存管理頁面
**作用**：
- 顯示藥物庫存表格（Handsontable）
- 管理詳細藥物資訊
- 提供搜尋和匯出功能

**主要功能**：
- 基本庫存管理（增刪改查）
- 詳細藥物資訊輸入
- 按名稱/編號搜尋
- JSON格式匯出

#### `doctor.html` - 醫生工作台
**作用**：
- 醫生填寫藥物資訊
- 連接到Medicine系統儲存資料
- 提供簡潔的藥物輸入介面

**主要功能**：
- 藥物基本資訊輸入
- 藥物詳細資訊輸入
- 直接儲存到Medicine資料庫

#### `Prescription.html` - 處方管理頁面
**作用**：
- 醫生開立處方
- 追蹤處方狀態
- 管理病人用藥

**主要功能**：
- 處方開立表單
- 處方狀態追蹤
- 病人藥物管理

### 3. ROS2整合模組

#### `ros2_medicine_client.py` - ROS2客戶端
**作用**：
- 連接醫院API與ROS2系統
- 定期發布藥物資料到ROS2主題
- 處理ROS2服務請求

**主要功能**：
```python
# ROS2主題發布
/medicine_data          # 藥物資料發布
/prescription_status    # 處方狀態更新

# ROS2服務
/get_all_medicines     # 獲取所有藥物資訊
```

#### `medicine_ros2_node.py` - ROS2服務節點
**作用**：
- 提供ROS2服務定義
- 處理藥物資訊請求
- 儲存資料供其他ROS2節點使用

### 4. 測試與工具模組

#### `test.sh` - 綜合測試腳本
**作用**：
- 自動化測試所有功能
- 產生測試資料
- 驗證API連接

**測試內容**：
```bash
# 第1階段：添加基本藥物
add_basic_medicine "測試藥物A" 100 30 "A1-TEST"

# 第2階段：添加詳細藥物資訊
add_detailed_medicine "測試藥物A"

# 第3階段：ROS2整合測試
get_all_medicine_for_ros2

# 第4階段：處方系統測試
test_prescription_system

# 第5階段：創建ROS2服務配置
create_ros2_service_config
```

#### `examples_package.py` - 實用範例工具
**作用**：
- 提供API使用範例
- 演示完整工作流程
- 互動式測試介面

## 🚀 使用順序指南

### 步驟1：啟動系統
```bash
cd /workspace/user_interface

# 啟動主伺服器（唯一需要的伺服器）
python3 fixed_server.py
```

### 步驟2：測試系統連接
```bash
# 在另一個終端執行
curl http://localhost:8000/api/test
```

### 步驟3：執行完整測試
```bash
# 執行自動化測試
./test.sh
```

### 步驟4：使用網頁介面
1. **醫生工作台**：`http://localhost:8000/doctor.html`
   - 用於填寫藥物資訊
   - 連接Medicine系統儲存

2. **藥物管理**：`http://localhost:8000/Medicine.html`
   - 查看和管理藥物庫存
   - 詳細資訊管理

3. **處方管理**：`http://localhost:8000/Prescription.html`
   - 開立和追蹤處方

### 步驟5：ROS2整合（可選）
```bash
# 啟動ROS2客戶端
python3 ros2_medicine_client.py

# 在另一個終端啟動ROS2節點
python3 medicine_ros2_node.py
```

## 🧪 詳細測試內容

### 1. 基本功能測試

#### 藥物管理測試
```bash
# 測試新增藥物
curl -X POST http://localhost:8000/api/medicine/ \
  -H "Content-Type: application/json" \
  -d '{
    "name": "測試藥物",
    "amount": 100,
    "usage_days": 30,
    "position": "A1-01"
  }'

# 測試獲取藥物列表
curl http://localhost:8000/api/medicine/
```

#### 詳細資訊測試
```bash
# 測試新增詳細資訊
curl -X POST http://localhost:8000/api/medicine/detailed/ \
  -H "Content-Type: application/json" \
  -d '{
    "medicine_name": "測試藥物",
    "medicine_data": {
      "基本資訊": {
        "名稱": "測試藥物",
        "劑量": "10毫克"
      },
      "外觀": {
        "顏色": "白色",
        "形狀": "圓形"
      }
    }
  }'
```

#### 處方管理測試
```bash
# 測試開立處方
curl -X POST http://localhost:8000/api/prescription/ \
  -H "Content-Type: application/json" \
  -d '{
    "patient_name": "測試病人",
    "doctor_name": "測試醫生",
    "medicines": [{
      "medicine_name": "測試藥物",
      "dosage": "10mg",
      "frequency": "每日三次",
      "duration": "7天"
    }]
  }'
```

### 2. 整合測試

#### 端到端工作流程測試
```bash
# 使用examples_package.py進行完整測試
python3 examples_package.py
```

#### ROS2整合測試
```bash
# 檢查ROS2資料匯出
cat medicine_data_for_ros2.json

# 測試ROS2服務狀態
curl http://localhost:8000/api/ros2/services/status
```

## 📊 測試檢查清單

### ✅ 基本功能檢查
- [ ] 伺服器啟動成功
- [ ] API測試端點回應正常
- [ ] 藥物CRUD操作正常
- [ ] 詳細資訊儲存正常
- [ ] 處方開立功能正常

### ✅ 網頁介面檢查
- [ ] Medicine.html載入正常
- [ ] doctor.html載入正常
- [ ] Prescription.html載入正常
- [ ] 表格功能正常
- [ ] 表單提交正常

### ✅ ROS2整合檢查
- [ ] ROS2客戶端連接正常
- [ ] 藥物資料發布正常
- [ ] 服務調用正常
- [ ] 資料格式正確

### ✅ 資料一致性檢查
- [ ] 基本藥物與詳細資訊對應
- [ ] 處方中藥物資訊正確
- [ ] JSON匯出資料完整
- [ ] 搜尋功能正確

## 🔧 故障排除

### 常見問題解決

1. **伺服器啟動失敗**
   ```bash
   # 檢查端口佔用
   lsof -i :8000
   
   # 殺死佔用進程
   pkill -f fixed_server
   ```

2. **API連接失敗**
   ```bash
   # 檢查伺服器狀態
   curl http://localhost:8000/api/test
   
   # 檢查日誌
   tail -f server.log
   ```

3. **網頁載入失敗**
   ```bash
   # 檢查靜態文件
   ls -la static/html/
   
   # 檢查檔案權限
   chmod 644 static/html/*.html
   ```

4. **ROS2整合問題**
   ```bash
   # 檢查ROS2環境
   echo $ROS_DOMAIN_ID
   
   # 檢查主題
   ros2 topic list
   ```

## 📈 效能監控

### 系統狀態監控
```bash
# 檢查系統統計
curl http://localhost:8000/ | jq .statistics

# 檢查記憶體使用
ps aux | grep fixed_server

# 檢查API回應時間
time curl http://localhost:8000/api/medicine/
```

## 📝 開發注意事項

1. **資料格式一致性**：確保前端送出的資料格式符合API要求
2. **錯誤處理**：實現適當的錯誤處理和用戶回饋
3. **效能考量**：大量資料時考慮分頁和快取
4. **安全性**：生產環境需要加入認證和授權
5. **可擴展性**：設計時考慮未來功能擴展需求

這個系統設計為模組化架構，各個組件可以獨立開發和測試，同時保持良好的整合性。
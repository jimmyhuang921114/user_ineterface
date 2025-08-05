# 🤖 ROS2 YAML藥物訂單系統使用指南

## 📋 系統概述

本系統提供完整的ROS2整合解決方案，用於生成指定格式的YAML藥物訂單文件。系統支援：

- ✅ **整合表格**: 同時填寫基本和詳細藥物資訊
- ✅ **ROS2節點**: 自動生成標準YAML格式
- ✅ **訂單覆蓋**: 新訂單自動覆蓋舊訂單
- ✅ **完整資訊**: 包含所有必要的藥物詳細資料
- ✅ **中文支援**: 完整的繁體中文支援

## 🎯 YAML輸出格式

系統生成的YAML格式完全符合您的需求：

```yaml
# 訂單ID: 000001
# 生成時間: 2025-08-05 15:38:19
# 精神好製藥

Antipsychotics:
  藥物基本資料:
    外觀類型: "藥片"
    存取位置: [1, 1]

  藥物詳細資料:
    名稱: "Antipsychotics"
    成分: "Antipsychotic compounds"
    分類: "精神科藥物"
    劑量: "5 毫克"
    服用方式: "口服 (Oral use)"
    單位劑量: "1 special pill"
    有效日期: "2027/08/02"
    適應症: "消除精神分裂症的陽性病徵，例如幻覺、妄想、思想混亂等。有助眠功效，適用於對其他藥物不適應者。"
    可能副作用: "嗜睡、頭暈、體重增加、口乾、便秘、姿勢性低血壓"
    條碼編號: "ANTI-123456789"
    外觀:
      顏色: "紅色條紋 白色外觀"
      形狀: "圓扁形"
```

## 🚀 系統啟動

### 1. 啟動後端API服務

```bash
cd user_interface
python3 fixed_server.py
```

服務啟動後會顯示：
```
🌐 醫院藥物管理系統已啟動
📋 主要頁面: http://localhost:8000/medicine_integrated.html
🤖 ROS2整合頁面: http://localhost:8000/simple_test.html
📖 API文檔: http://localhost:8000/docs
```

### 2. 啟動ROS2 YAML節點

```bash
python3 ros2_yaml_generator_node.py
```

節點會顯示：
```
🤖 YAML藥物訂單生成節點已啟動
📁 YAML輸出路徑: medicine_order_output.yaml
🚀 發布示例訂單...
```

## 🌐 使用整合表格

### 訪問整合表格頁面
```
http://localhost:8000/medicine_integrated.html
```

### 表格包含的欄位

#### 📋 基本藥物資料
- **藥物名稱** *(必填)*
- **數量**
- **使用天數**
- **存放位置**
- **製造商**
- **劑量**

#### 📊 詳細藥物資訊
- **藥物成分**: 主要成分名稱
- **藥物分類**: 例如 "3CL proteinase inhibitors"
- **服用方式**: 口服/注射/外用等
- **單位劑量**: 例如 "1 special pill"
- **藥物描述/適應症**: 詳細說明
- **可能副作用**: 副作用與注意事項
- **條碼編號**: 例如 "TEST-367842394"
- **外觀類型**: 藥片/膠囊/液體等
- **外觀顏色**: 藥物顏色描述
- **藥物形狀**: 圓扁形/膠囊形等
- **儲存條件**: 保存方式
- **有效期限**: 到期日期
- **特殊說明**: 其他備註

### 提交表格

1. 填寫完整的基本和詳細資料
2. 點擊 **"提交整合藥物資料"** 按鈕
3. 系統會同時：
   - 保存基本資料到 `medicine_basic_data.json`
   - 保存詳細資料到 `medicine_detailed_data.json`
   - 觸發ROS2節點生成YAML文件

## 🤖 ROS2節點功能

### 訂單接收

節點監聽 `/medicine_order_request` topic：

```bash
ros2 topic pub /medicine_order_request std_msgs/String \
'{"data": "{\"id\": \"000001\", \"order_data\": {\"medicine_1\": {\"amount\": 87, \"locate\": [1,1], \"name\": \"Antipsychotics\"}}}"}'
```

### YAML輸出

節點發布到 `/medicine_yaml_output` topic：

```bash
ros2 topic echo /medicine_yaml_output
```

### 自動覆蓋功能

- 每當收到新訂單時，節點會：
  1. 生成新的YAML內容
  2. 覆蓋寫入 `medicine_order_output.yaml`
  3. 發布到ROS2 topic
  4. 記錄新的訂單ID

## 📁 文件結構

```
/workspace/
├── ros2_yaml_generator_node.py      # ROS2 YAML生成節點
├── test_yaml_simple.py              # 測試腳本
├── medicine_order_output.yaml       # 生成的YAML文件
├── user_interface/
│   ├── fixed_server.py              # FastAPI後端
│   ├── static/html/
│   │   ├── medicine_integrated.html # 整合表格頁面
│   │   ├── simple_test.html         # API測試頁面
│   │   └── doctor.html              # 醫生界面
│   ├── medicine_basic_data.json     # 基本藥物資料
│   ├── medicine_detailed_data.json  # 詳細藥物資料
│   └── prescription_data.json       # 處方籤資料
└── ROS2_YAML_ORDER_SYSTEM_GUIDE.md  # 本指南
```

## 🔧 API端點

### 基本藥物資料
- `POST /api/medicine/basic` - 新增基本資料
- `GET /api/medicine/basic` - 獲取基本資料
- `GET /api/ros2/medicine/basic` - ROS2格式基本資料

### 詳細藥物資料
- `POST /api/medicine/detailed` - 新增詳細資料
- `GET /api/medicine/detailed` - 獲取詳細資料
- `GET /api/ros2/medicine/detailed` - ROS2格式詳細資料

### 整合資料
- `GET /api/ros2/medicine/integrated/{medicine_name}` - 整合資料

### 處方籤資料
- `GET /api/ros2/prescription` - ROS2格式處方籤資料

## 🧪 測試功能

### 運行完整測試
```bash
python3 test_yaml_simple.py
```

### 測試API端點
```bash
# 訪問測試頁面
http://localhost:8000/simple_test.html

# 或使用curl測試
curl http://localhost:8000/api/ros2/medicine/basic
curl http://localhost:8000/api/ros2/medicine/detailed
```

## 🎯 完整工作流程

1. **啟動系統**
   ```bash
   cd user_interface
   python3 fixed_server.py
   ```

2. **啟動ROS2節點**
   ```bash
   python3 ros2_yaml_generator_node.py
   ```

3. **填寫藥物資料**
   - 訪問: `http://localhost:8000/medicine_integrated.html`
   - 同時填寫基本和詳細資料
   - 提交表格

4. **查看結果**
   ```bash
   cat medicine_order_output.yaml
   ```

5. **ROS2整合** (如果有ROS2環境)
   ```bash
   # 發送訂單
   ros2 topic pub /medicine_order_request std_msgs/String '...'
   
   # 監聽輸出
   ros2 topic echo /medicine_yaml_output
   ```

## ⚠️ 重要注意事項

1. **覆蓋行為**: 新訂單會完全覆蓋舊的YAML文件
2. **中文支援**: 所有輸出都支援繁體中文
3. **資料驗證**: 表格會驗證必填欄位
4. **自動時間戳**: 每個YAML文件都包含生成時間
5. **錯誤處理**: 節點會優雅地處理API連接錯誤

## 🎉 功能特色

- ✅ **完全自動化**: 從表格填寫到YAML生成
- ✅ **標準格式**: 完全符合您指定的YAML結構
- ✅ **即時更新**: 新訂單立即覆蓋舊文件
- ✅ **豐富資訊**: 包含所有必要的藥物詳細資料
- ✅ **ROS2原生**: 專為ROS2環境設計
- ✅ **中文優化**: 完整的繁體中文支援
- ✅ **易於擴展**: 模組化設計，易於客製化

---

**🎯 您的所有需求已100%實現！**

現在您擁有一個完整的ROS2整合系統，可以：
- 使用整合表格同時填寫基本和詳細藥物資訊
- 自動生成指定格式的YAML文件
- 在新訂單時自動覆蓋舊文件
- 支援完整的藥物詳細資訊
- 確保功能正常運作
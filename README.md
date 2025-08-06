# 🏥 醫院藥物管理系統 v5.0.0

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.100+-green.svg)](https://fastapi.tiangolo.com/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble+-orange.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 📖 系統概述

這是一個基於 **FastAPI** 和 **ROS2** 的現代化醫院藥物管理系統，支援**多格式數據存儲**（JSON、YAML、SQL），提供完整的處方籤管理、藥物庫存控制和ROS2機器人整合功能。

### ✨ 核心特色
- 🎯 **處方籤管理**: 完整的處方籤開立、儲存和查詢（替代傳統病例管理）
- 🤖 **ROS2 深度整合**: 自動化訂單處理和狀態更新
- 💾 **多格式存儲**: 同時支援 JSON、YAML、SQL 三種數據格式
- 💊 **智能藥物管理**: 基本和詳細藥物資訊管理
- 📦 **即時庫存控制**: 即時庫存調整和追蹤
- 🔍 **AI 智能助手**: 智能藥物推薦和交互提示
- 🌐 **響應式介面**: 現代化響應式使用者介面

---

## 🚀 快速開始

### 📋 系統需求
- **Python**: 3.8 或更高版本
- **作業系統**: Windows, macOS, Linux  
- **記憶體**: 最少 1GB RAM
- **硬碟**: 最少 1GB 可用空間

### ⚡ 安裝步驟

#### 1. 安裝 Python 依賴
```bash
pip install fastapi uvicorn pydantic sqlalchemy sqlite3 pyyaml pathlib typing
```

#### 2. 啟動系統（多種方式）

**方式一：使用主服務器（推薦）**
```bash
cd user_interface
python3 fixed_server.py
```

**方式二：使用簡化啟動**
```bash
cd user_interface  
python3 main.py
```

**方式三：使用SQL服務器**
```bash
cd user_interface
python3 sql_server.py
```

#### 3. 訪問系統
開啟瀏覽器，前往：http://localhost:8000

---

## 🏗️ 系統架構

### 📊 多格式數據架構

```
數據流向: 用戶操作 → FastAPI → 多格式存儲 → 三種格式同步

┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   前端介面      │───▶│   FastAPI APIs   │───▶│ 多格式存儲系統  │
│                │    │                  │    │                │
│ • 醫生工作站    │    │ • 處方籤 API     │    │ • JSON 存儲     │
│ • 處方籤管理    │    │ • 藥物管理 API   │    │ • YAML 匯出     │
│ • 藥物管理      │    │ • ROS2 整合 API  │    │ • SQL 資料庫    │
│ • 整合管理      │    │ • 庫存控制 API   │    │                │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                        │
                                                        ▼
                                               ┌─────────────────┐
                                               │   ROS2 整合     │
                                               │                │
                                               │ • 訂單獲取     │
                                               │ • 狀態回饋     │
                                               │ • 機器人控制   │
                                               └─────────────────┘
```

### 🗃️ 數據存儲架構

#### 📁 JSON 格式（主要存儲）
```
user_interface/
├── medicine_basic_data.json     # 基本藥物資料
├── medicine_detailed_data.json  # 詳細藥物資料
├── prescription_data.json       # 處方籤資料
└── orders_data.json            # 訂單狀態記錄
```

#### 📄 YAML 格式（ROS2兼容）
```
user_interface/data/
├── basic_medicines.yaml         # 基本藥物 YAML
├── detailed_medicines.yaml      # 詳細藥物 YAML
├── prescriptions.yaml           # 處方籤 YAML
├── ros2_basic_medicines.yaml    # ROS2 基本藥物
└── ros2_detailed_medicines.yaml # ROS2 詳細藥物
```

#### 🗄️ SQL 格式（企業級）
```
user_interface/data/hospital_management.db
├── medicine_basic               # 基本藥物表
├── medicine_detailed            # 詳細藥物表
├── prescriptions               # 處方籤表
└── system_logs                 # 系統日誌表
```

---

## 🎯 主要功能

### 👨‍⚕️ 醫生工作站
**網址**: http://localhost:8000/doctor.html

✅ **核心功能**
- 🆔 自動病患編號生成（基於身份證號）
- ⏰ 自動處方時間生成
- 💊 智能藥物選擇下拉選單
- 📊 庫存狀態即時顯示
- 🤖 AI提示詞智能推薦
- ✅ 數據驗證和錯誤處理

✅ **處方籤流程**
```
1. 輸入病患基本資訊 → 2. 選擇藥物並設定劑量 → 3. 系統自動驗證庫存 → 4. 生成完整處方籤
```

### 📋 處方籤管理
**網址**: http://localhost:8000/Prescription.html

✅ **管理功能**
- 📊 處方籤統計儀表板
- 🔍 處方籤搜尋和篩選
- 👁️ 詳細資訊檢視
- 🔄 處方狀態更新
- 📈 歷史記錄追蹤
- 📤 數據匯出功能

### 💊 藥物管理
**網址**: http://localhost:8000/Medicine.html

✅ **藥物功能**
- 📦 庫存統計和警告
- 🔍 即時搜尋和篩選
- ➕ 新增藥物資料
- ✏️ 編輯藥物資訊
- 📊 庫存調整記錄
- 📈 使用量統計

### 🔧 整合管理
**網址**: http://localhost:8000/integrated_medicine_management.html

✅ **整合功能**
- 📋 四大管理模組
- 💊 統一藥物新增
- 📦 庫存批量操作
- 📊 完整統計分析
- 🔄 數據同步功能

---

## 🤖 ROS2 整合

### 📡 ROS2 API 端點

```http
# 獲取處方籤訂單
GET /api/ros2/orders                 # 所有ROS2訂單
GET /api/ros2/orders/{order_id}      # 特定訂單詳情
POST /api/ros2/status               # 更新訂單狀態
GET /api/ros2/prescription          # ROS2格式處方籤
```

### 🎯 ROS2 數據格式

#### 訂單格式
```json
{
  "order_id": "ORDER_0001",
  "prescription_id": 1,
  "patient_name": "病患姓名",
  "patient_id": "P123456",
  "doctor_name": "醫師姓名",
  "medicines": [
    {
      "medicine_name": "藥物名稱",
      "quantity": 2,
      "duration_days": 7,
      "notes": "使用說明"
    }
  ],
  "status": "pending",
  "created_at": "2025-08-06 15:30:00"
}
```

#### 狀態更新格式
```json
{
  "order_id": "ORDER_0001",
  "status": "processing|completed|failed",
  "message": "處理狀態訊息",
  "timestamp": "2025-08-06 15:30:00"
}
```

### 🤖 ROS2 節點範例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import requests

class HospitalOrderProcessor(Node):
    def __init__(self):
        super().__init__('hospital_order_processor')
        self.timer = self.create_timer(10.0, self.process_orders)
        
    def process_orders(self):
        # 獲取待處理訂單
        response = requests.get("http://localhost:8000/api/ros2/orders")
        orders = response.json()
        
        for order in orders['orders']:
            if order['status'] == 'pending':
                self.process_single_order(order)
    
    def process_single_order(self, order):
        order_id = order['order_id']
        
        # 更新為處理中
        self.update_status(order_id, "processing", "機器人開始處理")
        
        # 執行配送邏輯
        for medicine in order['medicines']:
            self.get_logger().info(f"配送: {medicine['medicine_name']}")
            
        # 更新為完成
        self.update_status(order_id, "completed", "配送完成")
    
    def update_status(self, order_id, status, message):
        requests.post("http://localhost:8000/api/ros2/status", json={
            "order_id": order_id,
            "status": status,
            "message": message
        })
```

---

## 📡 API 文檔

### 🔗 完整 API 列表

#### 系統 API
```http
GET  /api/health                    # 系統健康檢查
GET  /docs                          # API 文檔
GET  /favicon.ico                   # 網站圖標
```

#### 藥物管理 API  
```http
GET    /api/medicine/basic          # 獲取基本藥物
POST   /api/medicine/basic          # 新增基本藥物
PUT    /api/medicine/basic/{id}     # 更新基本藥物
DELETE /api/medicine/basic/{id}     # 刪除基本藥物

GET    /api/medicine/detailed       # 獲取詳細藥物
POST   /api/medicine/detailed       # 新增詳細藥物
PUT    /api/medicine/detailed/{id}  # 更新詳細藥物
DELETE /api/medicine/detailed/{id}  # 刪除詳細藥物

POST   /api/medicine/adjust-stock   # 調整藥物庫存
```

#### 處方籤管理 API
```http
GET    /api/prescription/           # 獲取所有處方籤
POST   /api/prescription/           # 創建新處方籤
PUT    /api/prescription/{id}       # 更新處方籤
DELETE /api/prescription/{id}       # 刪除處方籤
```

#### ROS2 整合 API
```http
GET    /api/ros2/orders             # 獲取ROS2訂單
GET    /api/ros2/orders/{id}        # 獲取特定訂單
POST   /api/ros2/status             # 更新訂單狀態
GET    /api/ros2/prescription       # ROS2格式處方籤
```

---

## 💾 多格式存儲系統

### 🔄 自動同步機制

系統採用**多格式同步存儲**，每次數據操作都會自動同步到三種格式：

```python
# 數據保存時自動同步
def save_prescriptions(data):
    # 1. 保存到JSON（主要格式）
    with open("prescription_data.json", "w") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    
    # 2. 同步到YAML（ROS2兼容）
    multi_storage.save_prescriptions(data)
    
    # 3. 同步到SQL（企業級）
    # 自動執行，無需額外操作
```

### 📊 數據一致性驗證

```bash
# 驗證數據一致性
cd user_interface
python3 -c "
from multi_format_storage import get_storage
storage = get_storage()
consistency = storage.validate_data_consistency()
print(f'數據一致性: {consistency}')
"
```

### 🔧 手動同步（如需要）

```bash
# 手動同步所有格式
cd user_interface  
python3 multi_format_storage.py
```

---

## 🧪 測試功能

### 🌐 網頁測試
- **ROS2測試中心**: http://localhost:8000/ros2_test.html
- **功能測試頁面**: http://localhost:8000/test_all_functions.html

### ✅ 自動化測試
```bash
# 運行系統健康檢查
cd user_interface
python3 system_check.py
```

### 🧪 手動測試流程
1. **處方籤測試**: 開立 → 保存 → 查看 → ROS2讀取
2. **藥物管理測試**: 新增 → 編輯 → 庫存調整 → 統計查看
3. **ROS2測試**: 訂單獲取 → 狀態更新 → 完成流程

---

## 📁 專案結構

```
醫院藥物管理系統/
├── user_interface/                 # 主要應用程式
│   ├── fixed_server.py            # 主服務器（JSON + 多格式）
│   ├── sql_server.py              # SQL版本服務器
│   ├── main.py                    # 簡化啟動入口
│   ├── multi_format_storage.py    # 多格式存儲系統 ⭐
│   ├── yaml_storage.py            # YAML處理模組
│   ├── database.py                # SQL資料庫模型
│   ├── system_check.py            # 系統健康檢查
│   │
│   ├── static/                    # 靜態檔案
│   │   ├── html/                  # HTML頁面
│   │   │   ├── doctor.html        # 醫生工作站
│   │   │   ├── Prescription.html  # 處方籤管理
│   │   │   ├── Medicine.html      # 藥物管理
│   │   │   └── integrated_medicine_management.html # 整合管理
│   │   ├── css/                   # 樣式表
│   │   └── js/                    # JavaScript邏輯
│   │
│   ├── data/                      # 多格式數據儲存 ⭐
│   │   ├── *.json                 # JSON數據檔案
│   │   ├── *.yaml                 # YAML數據檔案
│   │   └── hospital_management.db # SQL資料庫
│   │
│   ├── ros2_test.html             # ROS2測試頁面
│   ├── test_all_functions.html    # 功能測試頁面
│   └── *.json                     # 主要數據檔案
│
├── README.md                      # 專案說明文檔
├── SYSTEM_STATUS.md              # 系統狀態報告
└── system_check.py               # 系統檢查腳本
```

---

## 🚨 故障排除

### 🔧 常見問題

#### 1. 服務器無法啟動
**問題**: `ModuleNotFoundError` 或連接埠衝突
```bash
# 解決方案
pip install fastapi uvicorn pydantic sqlalchemy pyyaml
kill -9 $(lsof -ti:8000)  # 強制終止佔用連接埠的程序
```

#### 2. 數據格式不一致
**問題**: JSON、YAML、SQL數據不同步
```bash
# 解決方案
cd user_interface
python3 multi_format_storage.py  # 重新同步所有格式
```

#### 3. ROS2連接問題
**問題**: 無法獲取ROS2訂單
```bash
# 檢查方案
curl http://localhost:8000/api/ros2/orders
curl http://localhost:8000/api/health
```

#### 4. 處方籤無法保存
**問題**: 處方籤提交後沒有顯示
```bash
# 檢查處方籤數據
curl http://localhost:8000/api/prescription/
# 檢查數據文件
ls -la user_interface/prescription_data.json
```

### 📊 系統監控
```bash
# 檢查系統狀態
python3 system_check.py

# 檢查數據一致性
python3 -c "
from multi_format_storage import get_storage
print(get_storage().validate_data_consistency())
"

# 檢查日誌
tail -f /var/log/hospital_system.log  # 如果有配置日誌
```

---

## 📈 版本歷史

### v5.0.0 (目前版本) ⭐
- **🔄 多格式存儲**: 新增JSON、YAML、SQL三格式同步
- **🗑️ 系統清理**: 移除無用檔案和重複功能
- **💾 數據優化**: 改進數據存儲和讀取效率
- **🤖 ROS2增強**: 完善ROS2整合和API端點
- **📋 處方籤專精**: 專注處方籤管理（移除病例概念）

### v4.0.0 
- **✅ 錯誤修復**: 修復JavaScript null錯誤
- **🔧 API改進**: 完善Pydantic數據驗證
- **🤖 ROS2整合**: 新增完整ROS2訂單API
- **🧪 測試增強**: 新增全面測試頁面

### v3.0.0
- **🎯 功能整合**: 統一導航系統
- **🔄 表單優化**: 改進使用者體驗
- **👨‍⚕️ 工作台精簡**: 專注核心功能
- **🚀 ROS2服務**: 增強ROS2服務支援

---

## 🎯 特色總結

✨ **五個核心頁面，完整功能覆蓋**  
✨ **三種數據格式，彈性存儲選擇**  
✨ **ROS2深度整合，支援機器人應用**  
✨ **AI智能助手，提升醫療決策品質**  
✨ **現代化介面，優秀的使用者體驗**  
✨ **處方籤專精，簡化工作流程**  

---

## 📞 支援與授權

### 🤝 技術支援
如需技術支援或回報問題：
1. 檢查本文檔的故障排除章節
2. 執行系統健康檢查 (`python3 system_check.py`)
3. 查看服務器日誌
4. 提供詳細的錯誤訊息和重現步驟

### 📄 授權條款
本專案採用 **MIT 授權條款**

---

**🏥 醫院藥物管理系統 v5.0.0 - 為現代醫療提供專業的數位化解決方案！**

*最後更新: 2025-08-06*  
*版本: v5.0.0*  
*相容性: Python 3.8+, FastAPI 0.68+*
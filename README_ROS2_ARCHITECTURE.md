# 🤖 醫院藥物管理系統 - ROS2架構完整說明

## 🏗️ **整體架構概覽**

本系統採用 **串行單筆處理模式**，確保每筆訂單完整處理後再處理下一筆，包含完整的處方建立、庫存更新、機械手臂控制等流程。

```
FastAPI Web Server ←→ ROS2 Medicine Management Client ←→ ROS2 Order Processor ←→ 機械手臂控制
       ↓                           ↓                              ↓
   處方籤資料庫              自動輪詢查詢                    完整訂單處理
   狀態更新記錄              串行處理控制                    硬體設備控制
```

---

## 🔄 **核心處理流程**

### **串行處理循環**
```
1. 🔍 查詢 FastAPI 是否有新訂單
   ↓
2. ✅ 如果有新訂單 → 處理一筆完整流程
   ↓
3. ⏳ 等待處理完成（包含所有步驟）
   ↓
4. 🔄 處理完成後，回到步驟1查詢下一筆
   ↓
5. 📊 無新訂單 → 等待，繼續監控
```

### **單筆訂單完整處理流程**
```
📋 收到處方籤
   ↓
🔍 驗證訂單資料
   ↓
📦 檢查藥物庫存
   ↓
📄 建立處方記錄
   ↓
🔄 更新庫存數量
   ↓
🤖 控制機械手臂取藥
   ↓
📊 回報處理狀態給 FastAPI
   ↓
✅ 完成，準備處理下一筆
```

---

## 📦 **ROS2 包架構說明**

### **1. 🔧 medicine_interfaces**
**自訂介面定義包**

```
medicine_interfaces/
├── srv/
│   ├── MedicineOrder.srv      # 藥物訂單服務介面
│   └── GetMedicineInfo.srv    # 藥物資訊查詢介面
├── msg/
│   ├── MedicineBasic.msg      # 基本藥物資訊訊息
│   ├── MedicineDetailed.msg   # 詳細藥物資訊訊息
│   └── OrderResponse.msg      # 訂單回應訊息
├── CMakeLists.txt
└── package.xml
```

**功能說明:**
- 定義所有 ROS2 節點間通訊的介面
- 統一訊息格式，確保資料一致性
- 支援服務 (request/response) 和訊息 (publish/subscribe)

**主要介面:**
- `MedicineOrder.srv`: 處理藥物訂單的服務介面
- `GetMedicineInfo.srv`: 查詢藥物資訊的服務介面

---

### **2. 🏥 medicine_management_client**
**自動輪詢與串行處理客戶端**

```
medicine_management_client/
├── medicine_management_client/
│   ├── __init__.py
│   └── medicine_management_client.py    # 主要客戶端邏輯
├── setup.py
├── package.xml
└── resource/
    └── medicine_management_client
```

**核心功能:**
- 🔍 **自動查詢**: 定期查詢 FastAPI 是否有新處方籤
- 🎯 **串行處理**: 一次只處理一筆訂單，確保完整性
- 📡 **狀態管控**: 控制處理狀態，避免重複處理
- 🔄 **循環監控**: 處理完成後自動查詢下一筆

**處理邏輯:**
```python
class MedicineManagementClient(Node):
    def check_for_new_batch(self):
        """查詢是否有新訂單需要處理（一次處理一筆）"""
        
    def process_single_prescription(self, prescription):
        """處理單筆處方籤"""
        
    def process_prescription_sync(self, prescription):
        """同步處理單個處方籤，等待完成後再處理下一筆"""
```

**啟動方式:**
```bash
ros2 run medicine_management_client medicine_management_client
```

---

### **3. ⚙️ medicine_order_processor**
**訂單處理執行器**

```
medicine_order_processor/
├── scripts/
│   └── order_processor_node.py         # 訂單處理主節點
├── package.xml
└── setup.py
```

**核心功能:**
- 📋 **訂單驗證**: 檢查處方籤資料完整性
- 📦 **庫存檢查**: 驗證藥物庫存是否充足
- 🏥 **處方建立**: 呼叫 FastAPI 建立正式處方記錄
- 🔄 **庫存更新**: 更新藥物庫存數量
- 🤖 **設備控制**: 控制機械手臂等硬體設備
- 📊 **狀態回報**: 處理完成後回報狀態給 FastAPI

**主要服務:**
- 提供 `medicine_order` 服務
- 接收訂單請求並執行完整處理流程

**處理步驟:**
```python
def medicine_order_callback(self, request, response):
    # 1. 驗證訂單
    validation_result = self.validate_order(request)
    
    # 2. 檢查庫存
    stock_check_result = self.check_medicine_stock(request)
    
    # 3. 建立處方
    prescription_result = self.create_prescription_via_api(request)
    
    # 4. 更新庫存
    stock_update_result = self.update_stock_via_api(request)
    
    # 5. 回報狀態
    self.report_status_to_web(order_id, "completed", message, request)
```

**啟動方式:**
```bash
ros2 run medicine_order_processor order_processor_node
```

---

### **4. 📊 medicine_info_provider**
**藥物資訊提供服務**

```
medicine_info_provider/
├── scripts/
│   └── medicine_info_node.py           # 藥物資訊服務節點
├── package.xml
└── setup.py
```

**功能說明:**
- 🔍 **資訊查詢**: 提供藥物基本和詳細資訊查詢
- 📄 **YAML 整合**: 讀取 YAML 格式的藥物資料
- 🌐 **API 整合**: 從 FastAPI 獲取最新藥物資訊

**主要服務:**
- `get_medicine_info`: 查詢藥物資訊服務

---

## 🌐 **FastAPI 整合**

### **新增的 API 端點**

#### **POST /api/prescription/status-update**
**ROS2 狀態回報端點**

```python
@app.post("/api/prescription/status-update")
async def update_prescription_status(status_data: dict):
    """接收ROS2節點的處方籤狀態更新"""
```

**功能:**
- 接收 ROS2 處理完成的狀態更新
- 更新處方籤的 ROS2 處理狀態
- 記錄處理歷史和詳細資訊

**請求格式:**
```json
{
  "order_id": "ORDER_1733123456_測試病人",
  "status": "completed",
  "message": "訂單處理成功，耗時 3.45 秒",
  "timestamp": "2024-12-02T10:30:45",
  "patient_name": "測試病人",
  "patient_id": "P20241202001",
  "medicine_count": 2,
  "medicines": [
    {
      "name": "測試藥物",
      "quantity": 30,
      "dosage": "30個",
      "status": "completed"
    }
  ]
}
```

---

## 🚀 **完整系統啟動指南**

### **1. 環境準備**
```bash
# 安裝 ROS2 依賴
sudo apt update
sudo apt install ros-humble-desktop

# 設置環境
source /opt/ros/humble/setup.bash

# 建置自訂介面包
cd ros2_packages/medicine_interfaces
colcon build
source install/setup.bash
```

### **2. 啟動 FastAPI 服務器**
```bash
cd user_interface
python3 fixed_server.py

# 服務器將運行在 http://localhost:8000
```

### **3. 啟動 ROS2 節點**

**終端 1: 啟動訂單處理器**
```bash
cd ros2_packages/medicine_order_processor
ros2 run medicine_order_processor order_processor_node
```

**終端 2: 啟動自動輪詢客戶端**
```bash
cd ros2_packages/medicine_management_client
ros2 run medicine_management_client medicine_management_client
```

**終端 3: 啟動藥物資訊服務 (可選)**
```bash
cd ros2_packages/medicine_info_provider
ros2 run medicine_info_provider medicine_info_node
```

### **4. 驗證系統運作**

**查看 ROS2 節點狀態:**
```bash
ros2 node list
ros2 service list
ros2 topic list
```

**查看日誌:**
```bash
ros2 topic echo /prescription_batch_status
```

---

## 📊 **監控和除錯**

### **ROS2 系統監控**

**查看節點狀態:**
```bash
# 查看所有節點
ros2 node list

# 查看節點資訊
ros2 node info /medicine_management_client
ros2 node info /order_processor_node
```

**查看服務狀態:**
```bash
# 查看所有服務
ros2 service list

# 測試服務
ros2 service type /medicine_order
ros2 interface show medicine_interfaces/srv/MedicineOrder
```

**監控訊息:**
```bash
# 監控處理狀態
ros2 topic echo /prescription_batch_status

# 查看系統日誌
ros2 log level /medicine_management_client debug
```

### **FastAPI 監控**

**API 文檔:**
```
http://localhost:8000/docs
```

**測試端點:**
```bash
# 查詢處方籤
curl http://localhost:8000/api/prescription/

# 查詢藥物
curl http://localhost:8000/api/medicine/basic
```

---

## 🔧 **系統配置**

### **關鍵參數設置**

**客戶端配置 (`medicine_management_client.py`):**
```python
self.api_base_url = "http://localhost:8000/api"     # FastAPI 地址
self.batch_check_interval = 5.0                      # 查詢間隔 (秒)
```

**處理器配置 (`order_processor_node.py`):**
```python
self.api_base_url = "http://localhost:8000/api"     # FastAPI 地址
timeout_sec = 30.0                                   # 服務超時時間
```

---

## ⚡ **性能優化建議**

### **處理效率**
- ✅ **串行處理**: 確保每筆訂單完整處理，避免衝突
- ✅ **超時控制**: 設置合理的服務超時時間
- ✅ **錯誤恢復**: 處理失敗後自動繼續下一筆

### **資源管理**
- 📊 **記憶體使用**: 避免累積大量處理記錄
- 🔄 **連線管理**: 複用 HTTP 連線
- 📝 **日誌管理**: 適當的日誌級別設置

---

## 🛠️ **故障排除**

### **常見問題**

#### **Q: ROS2 節點無法找到服務**
**A:** 
```bash
# 檢查節點是否運行
ros2 node list

# 檢查服務是否可用
ros2 service list | grep medicine_order

# 重新啟動節點
```

#### **Q: FastAPI 連線失敗**
**A:**
```bash
# 檢查 FastAPI 是否運行
curl http://localhost:8000/api/medicine/basic

# 檢查防火牆設置
sudo ufw status

# 檢查埠號是否被占用
netstat -tulpn | grep :8000
```

#### **Q: 處方籤重複處理**
**A:**
- 檢查 `processed_prescriptions` 記錄
- 確認處方籤 ID 生成邏輯
- 查看處理狀態標記

### **除錯指令**

**詳細日誌:**
```bash
ros2 run medicine_management_client medicine_management_client --ros-args --log-level debug
```

**服務測試:**
```bash
# 手動測試訂單處理服務
ros2 service call /medicine_order medicine_interfaces/srv/MedicineOrder "{
  order_id: 'TEST_001',
  patient_name: '測試病人',
  patient_id: 'P001',
  doctor_name: '測試醫生',
  medicine_names: ['測試藥物'],
  dosages: ['30個'],
  frequencies: ['依醫囑'],
  quantities: [30],
  notes: '測試訂單',
  timestamp: 1733123456
}"
```

---

## 🎯 **系統特色**

### **✅ 可靠性**
- 🔒 **串行處理**: 避免並發衝突
- 🔄 **自動恢復**: 錯誤後自動繼續
- 📊 **狀態追蹤**: 完整的處理狀態記錄

### **✅ 擴展性**
- 🔌 **模組化設計**: 各功能獨立包裝
- 🌐 **標準介面**: 使用 ROS2 標準通訊
- 📦 **容器化友好**: 支援 Docker 部署

### **✅ 監控性**
- 📊 **實時狀態**: ROS2 topic 即時監控
- 📝 **詳細日誌**: 完整的處理記錄
- 🌐 **Web 界面**: FastAPI 提供 Web 監控

---

## 📈 **未來擴展**

### **硬體整合**
- 🤖 **機械手臂控制**: 擴展硬體控制功能
- 📦 **自動倉儲**: 整合倉儲管理系統
- 🔍 **視覺辨識**: 藥物識別和驗證

### **智能化功能**
- 🧠 **AI 推薦**: 基於提示詞的智能推薦
- 📊 **預測分析**: 庫存和需求預測
- 🔄 **自動補貨**: 智能補貨建議

---

**🎉 完整的 ROS2 醫院藥物管理系統已就緒！**

*更新時間: 2024-12-02*  
*版本: v3.0.0 - ROS2 完整整合版*
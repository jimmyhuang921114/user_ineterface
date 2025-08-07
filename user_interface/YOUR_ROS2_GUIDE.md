# 🤖 您的 ROS2 自動推送訂單系統指南

## 🎯 您的完整解決方案

**✅ 自動推送訂單給您的 ROS2 節點**  
**✅ 一次一個，等結束再進行下一個**  
**✅ 能夠告訴網站完成了**  
**✅ 查看訂單內容和接收節點**

---

## 📁 必要檔案清單

### ⭐ 核心檔案 (必須保留)
```
user_interface/
├── database_final.py              # 資料庫模型
├── simple_server_final.py         # Web 服務器
├── ros2_order_pusher.py           # 自動推送器
├── your_ros2_node.py              # 您的 ROS2 節點
├── start_your_system.py           # 專用啟動器
├── test_order_flow.py             # 測試工具
└── static/                        # Web 界面
    ├── integrated_medicine_management.html
    ├── doctor.html
    ├── Prescription.html
    ├── css/
    └── js/
```

### ❌ 可以刪除的檔案
```
# ROS2 服務模式 (您不需要)
- ros2_services_interface.py
- ros2_client_example.py
- ROS2_SERVICES_GUIDE.md

# 藥物查詢服務 (可選)
- ros2_medicine_detail_service.py
- medicine_client_example.py
- MEDICINE_DETAIL_SERVICE_GUIDE.md

# 舊版檔案
- ros2_interface_final.py
- integration_example.py
- start_final_server.py
- start_complete_system.py
- start_system_modes.py

# 文檔檔案 (可選保留)
- ESSENTIAL_FILES_ANALYSIS.md
- FINAL_SYSTEM_GUIDE.md
- WEB_ROS2_ARCHITECTURE.md
- README.md

# 資料庫和日誌
- hospital_medicine_final.db (會自動創建)
- hospital_system_ros2_real.log
```

---

## 🚀 快速開始

### 1️⃣ 啟動您的系統
```bash
cd user_interface
python3 start_your_system.py
```

### 2️⃣ 系統啟動後您會看到
```
🎉 您的自動推送訂單系統已啟動！
============================================================

📋 系統功能:
✅ 自動監控新處方籤
✅ 自動推送 YAML 訂單到您的 ROS2 節點
✅ 一次處理一個訂單
✅ 處理完成後自動進行下一個
✅ 完整的 Web 管理界面

🌐 Web 界面:
• 藥物管理: http://localhost:8001/integrated_medicine_management.html
• 醫生工作台: http://localhost:8001/doctor.html
• 處方籤管理: http://localhost:8001/Prescription.html
```

### 3️⃣ 測試系統
```bash
# 在另一個終端機
python3 test_order_flow.py basic
```

---

## 📋 如何查看訂單內容

### 🔍 您的 ROS2 節點會收到完整信息
當有新訂單時，您的節點會打印：

```
==================================================
📋 訂單詳細內容:
==================================================
🔖 YAML 格式:
   order_id: "000001"
   prescription_id: 1
   patient_name: "張三"
   medicine:
     - name: 阿斯匹靈
       amount: 10
       locate: [2, 3]
       prompt: tablet

------------------------------
🆔 訂單 ID: 000001
📝 處方籤 ID: 1
👤 病患姓名: 張三
💊 藥物清單 (1 項):
   1. 藥物: 阿斯匹靈
      數量: 10
      位置: 第2排第3列
      類型: tablet
==================================================
```

### 🌐 Web 界面查看
1. **處方籤管理**: http://localhost:8001/Prescription.html
   - 查看所有處方籤狀態
   - 實時監控處理進度

2. **藥物管理**: http://localhost:8001/integrated_medicine_management.html
   - 查看所有可用藥物
   - 管理庫存數量

---

## 🤖 如何接收和處理訂單

### 📨 自動接收機制
您的節點會自動接收推送的訂單，無需手動查詢：

```python
def process_order(self, order_dict: Dict[str, Any], yaml_order: str):
    """
    自動接收訂單 (由系統調用)
    """
    order_id = order_dict['order_id']
    medicines = order_dict.get('medicine', [])
    
    # 自動顯示訂單詳情
    self.print_order_details(order_dict, yaml_order)
    
    # 開始異步處理
    threading.Thread(target=self._process_order_async, args=(order_dict,)).start()
```

### 🔧 實現您的機器人邏輯
在 `your_ros2_node.py` 的 `process_medicine()` 中添加：

```python
def process_medicine(self, medicine: Dict[str, Any], index: int, total: int):
    """
    處理單個藥物 - 在這裡實現您的機器人邏輯！
    """
    name = medicine.get('name', 'N/A')
    amount = medicine.get('amount', 0)
    locate = medicine.get('locate', [0, 0])  # [row, col]
    prompt = medicine.get('prompt', 'unknown')  # tablet/capsule/white_circle_box
    
    # ============================================
    # 添加您的機器人控制代碼！
    # ============================================
    
    # 1. 移動到指定位置
    self.move_robot_to_position(locate[0], locate[1])
    
    # 2. 根據類型選擇抓取方式
    if prompt == 'tablet':
        self.pick_tablet(amount)
    elif prompt == 'capsule':
        self.pick_capsule(amount)
    elif prompt == 'white_circle_box':
        self.pick_box(amount)
    
    # 3. 放置到分配區域
    self.place_medicine()
```

### ✅ 告知完成狀態
系統會自動調用完成函數：

```python
def complete_order(self, order_id: str):
    """告知網站訂單已完成"""
    success = self._order_pusher.complete_order(order_id)
    if success:
        self.get_logger().info(f"✅ 已告知網站訂單 {order_id} 完成")
        self.get_logger().info("🔄 系統將自動處理下一個訂單...")
```

---

## 🔄 工作流程詳解

### 📊 完整流程圖
```
醫生開立處方籤 (Web) 
    ↓
存入資料庫 (status: pending)
    ↓
ros2_order_pusher 監控到新訂單
    ↓
轉換為 YAML 格式
    ↓
推送到您的 ROS2 節點
    ↓
your_ros2_node.process_order() 被調用
    ↓
顯示訂單詳情
    ↓
process_medicine() 處理每個藥物
    ↓
complete_order() 告知完成
    ↓
更新資料庫 (status: completed)
    ↓
自動檢查並處理下一個訂單
```

### 🔒 一次一個機制
- ✅ `ros2_order_pusher` 使用 `ros2_busy` 標誌
- ✅ 只有完成當前訂單才會處理下一個
- ✅ 防止訂單重複或並發處理

---

## 🧪 測試方法

### 1️⃣ 基本測試
```bash
python3 test_order_flow.py basic
```
**結果**: 創建一個測試處方籤，觀察自動推送

### 2️⃣ 批量測試
```bash
python3 test_order_flow.py batch 3
```
**結果**: 創建 3 個處方籤，驗證一次一個處理

### 3️⃣ Web 界面測試
1. 開啟 http://localhost:8001/integrated_medicine_management.html
2. 新增測試藥物
3. 開啟 http://localhost:8001/doctor.html  
4. 開立處方籤
5. 觀察終端機的訂單推送

### 4️⃣ 狀態監控
```bash
python3 test_order_flow.py list
```
**結果**: 查看所有處方籤狀態

---

## 📄 訂單格式詳解

### 🔖 YAML 格式
```yaml
order_id: "000001"           # 唯一訂單識別碼
prescription_id: 1           # 處方籤 ID
patient_name: "張三"         # 病患姓名
medicine:                    # 藥物清單
  - name: 阿斯匹靈           # 藥物名稱
    amount: 10               # 需要數量
    locate: [2, 3]           # 位置 [排, 列]
    prompt: tablet           # 類型提示
```

### 🏷️ 藥物類型 (prompt)
- **`tablet`**: 錠劑/藥片
- **`capsule`**: 膠囊
- **`white_circle_box`**: 白色圓盒包裝

### 📍 位置格式 (locate)
- `[2, 3]` = 第 2 排，第 3 列
- 從 1 開始計數
- 您可以根據實際藥櫃佈局調整

---

## 🔧 客製化設定

### 🎛️ 修改推送頻率
在 `ros2_order_pusher.py` 中：
```python
# 修改檢查間隔 (預設 3 秒)
self.check_interval = 3
```

### 🏷️ 修改藥物位置邏輯
在 `ros2_order_pusher.py` 的 `_convert_prescription_to_order()` 中：
```python
# 客製化位置分配邏輯
locate = [row, col]  # 根據您的藥櫃設計
```

### 📝 修改節點名稱
在 `your_ros2_node.py` 中：
```python
super().__init__('your_custom_node_name')
```

---

## 🛠️ 故障排除

### ❌ 問題 1: 沒有收到訂單
**檢查**:
```bash
# 檢查系統狀態
curl "http://localhost:8001/api/system/status"

# 檢查處方籤
python3 test_order_flow.py list
```

### ❌ 問題 2: 訂單卡住不進行下一個
**原因**: 沒有調用 `complete_order()`
**解決**: 確保在 `_process_order_async()` 最後調用

### ❌ 問題 3: ROS2 節點無法啟動
**檢查**:
```bash
# 檢查 ROS2 環境
ros2 --version
source /opt/ros/humble/setup.bash  # 或您的 ROS2 版本
```

---

## 📊 監控和調試

### 📈 實時狀態監控
- **Web 界面**: http://localhost:8001/Prescription.html
- **API 查詢**: `curl "http://localhost:8001/api/prescription/"`
- **節點日誌**: 查看終端機輸出

### 🔍 調試技巧
```bash
# 持續監控處方籤狀態
watch -n 2 "curl -s http://localhost:8001/api/prescription/ | jq '.[] | {id, status, patient_name}'"

# 手動完成處方籤
python3 test_order_flow.py complete 1

# 重置處方籤狀態
python3 test_order_flow.py reset 1 pending
```

---

## 🎯 總結

### ✅ 您現在擁有：
1. **自動推送系統** - 無需手動查詢訂單
2. **一次一個處理** - 防止訂單衝突
3. **完整狀態管理** - 自動更新處理狀態
4. **詳細訂單信息** - YAML 格式完整數據
5. **Web 管理界面** - 方便監控和測試
6. **測試工具** - 完整的測試支援

### 🔧 下一步：
1. 在 `your_ros2_node.py` 中實現您的機器人邏輯
2. 移除模擬延遲 (`time.sleep()`)
3. 根據實際藥櫃調整位置邏輯
4. 測試完整流程

**🎊 您的自動推送訂單系統已完全準備就緒！**
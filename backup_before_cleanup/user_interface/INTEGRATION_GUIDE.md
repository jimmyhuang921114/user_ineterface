# 🤖 整合您的 ROS2 節點 - 完整指南

## 🎯 系統架構

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Web 界面      │    │   適配器 API     │    │  您的 ROS2 節點 │
│  (port 8001)    │◄──►│  (port 8002)     │◄──►│                 │
│                 │    │                  │    │ OrderHandlerNode │
│ - 藥物管理      │    │ - 訂單拉取       │    │                 │
│ - 處方籤開立    │    │ - 進度回報       │    │ + Medicine      │
│ - 狀態監控      │    │ - 完成通知       │    │   Detail Service │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 📦 檔案清單

### ⭐ 核心整合檔案
1. **`integration_with_your_node.py`** - 適配器系統，連接我們的 Web 系統與您的 ROS2 節點
2. **`medicine_detail_service_node.py`** - 藥物詳細資訊查詢 ROS2 服務
3. **您的 `OrderHandlerNode`** - 您現有的訂單處理節點

### 📋 我們系統的必要檔案
- `database_final.py` - 資料庫模型
- `simple_server_final.py` - Web 服務器
- `static/` - Web 界面

## 🚀 快速啟動

### 1️⃣ 啟動整合系統
```bash
# 終端機 1: 啟動適配器系統
cd /path/to/user_interface
python3 integration_with_your_node.py
```

### 2️⃣ 啟動您的 ROS2 節點
```bash
# 終端機 2: 設置環境變數並啟動您的節點
source /opt/ros/humble/setup.bash

# 設置適配器 API 位址
export ORDER_BASE_URL='http://127.0.0.1:8002'
export ORDER_PULL_URL='http://127.0.0.1:8002/api/order/next'
export ORDER_PULL_INTERVAL='3'
export ORDER_PROGRESS_PATH='/api/order/progress'
export ORDER_COMPLETE_PATH='/api/order/complete'

# 啟動您的節點
python3 your_order_handler_node.py
```

### 3️⃣ (可選) 啟動藥物查詢服務
```bash
# 終端機 3: 藥物詳細資訊查詢服務
source /opt/ros/humble/setup.bash
export MEDICINE_BASE_URL='http://127.0.0.1:8001'
python3 medicine_detail_service_node.py
```

## 📋 您的節點會收到的 YAML 格式

當有新處方籤時，您的 `OrderHandlerNode` 會通過 HTTP 拉取到：

```yaml
order_id: "000001"
prescription_id: 1
patient_name: "張三"
medicine:
  - name: 阿斯匹靈
    amount: 10
    locate: [2, 3]
    prompt: tablet
  - name: 維他命C
    amount: 5
    locate: [1, 5]
    prompt: capsule
```

## 🔧 您需要做的修改

### 1️⃣ 環境變數 (必要)
在啟動您的節點前設置：
```bash
export ORDER_BASE_URL='http://127.0.0.1:8002'
export ORDER_PULL_URL='http://127.0.0.1:8002/api/order/next'
export ORDER_PROGRESS_PATH='/api/order/progress'
export ORDER_COMPLETE_PATH='/api/order/complete'
```

### 2️⃣ 在您的 `_process_medicine()` 中實現機器人邏輯
```python
def _process_medicine(self, order_id: str, med: Dict[str, Any], idx: int, total: int):
    name = med.get('name', 'N/A')
    amount = med.get('amount', 0)
    locate = med.get('locate', [0, 0])  # [row, col]
    prompt = med.get('prompt', 'unknown')  # tablet/capsule/white_circle_box
    
    # ===== 在這裡實現您的機器人流程 =====
    # 1. 移動到藥櫃位置
    self.move_to_position(locate[0], locate[1])
    
    # 2. 根據藥物類型選擇抓取方式
    if prompt == 'tablet':
        self.pick_tablets(name, amount)
    elif prompt == 'capsule':
        self.pick_capsules(name, amount)
    elif prompt == 'white_circle_box':
        self.pick_box(name, amount)
    
    # 3. 放置到配藥區
    self.place_medicine_in_container()
```

### 3️⃣ (可選) 查詢藥物詳細資訊
如果您需要更多藥物資訊，可以在處理藥物前查詢：

```python
# 在您的節點中添加 service client
self.medicine_detail_client = self.create_client(
    MedicineDetail, '/hospital/get_medicine_detail'
)

def get_medicine_details(self, medicine_name: str) -> Dict[str, Any]:
    """查詢藥物詳細資訊"""
    request = MedicineDetail.Request()
    request.name = medicine_name
    
    future = self.medicine_detail_client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    
    response = future.result()
    if response.success:
        return yaml.safe_load(response.detail)
    else:
        self.get_logger().warn(f"查詢藥物失敗: {medicine_name}")
        return {}
```

## 🌐 Web 界面使用

啟動系統後，您可以通過以下界面管理：

### 藥物管理
**網址**: http://localhost:8001/integrated_medicine_management.html
- 新增/編輯/刪除藥物
- 管理庫存數量
- 設置藥物位置

### 醫生工作台  
**網址**: http://localhost:8001/doctor.html
- 開立新處方籤
- 選擇藥物和數量
- 自動推送到您的 ROS2 節點

### 處方籤管理
**網址**: http://localhost:8001/Prescription.html
- 查看所有處方籤狀態
- 監控處理進度
- 手動更新狀態

### 適配器狀態
**網址**: http://localhost:8002/api/order/status
- 查看當前處理的訂單
- 監控系統狀態

## 🧪 測試流程

### 1️⃣ 基本測試
```bash
# 創建測試處方籤
python3 test_order_flow.py basic
```

### 2️⃣ Web 界面測試
1. 開啟 http://localhost:8001/integrated_medicine_management.html
2. 新增測試藥物：
   - 名稱: 測試藥物A
   - 描述: 用於測試
   - 庫存: 100
3. 開啟 http://localhost:8001/doctor.html
4. 開立處方籤：
   - 病患: 測試病患
   - 選擇: 測試藥物A
   - 數量: 5
5. 觀察您的 ROS2 節點接收訂單並處理

### 3️⃣ 藥物查詢測試
```bash
# 查詢單個藥物
ros2 service call /hospital/get_medicine_detail tm_robot_if/srv/MedicineDetail "{name: '阿斯匹靈'}"

# 獲取所有藥物
ros2 service call /hospital/get_all_medicines tm_robot_if/srv/MedicineDetail "{name: ''}"

# 搜尋藥物
ros2 service call /hospital/search_medicines tm_robot_if/srv/MedicineDetail "{name: '感冒'}"
```

## 📊 訂單處理流程

```
1. 醫生在 Web 界面開立處方籤
   ↓
2. 處方籤存入資料庫 (status: pending)
   ↓
3. 您的節點定期拉取 (每 3 秒)
   ↓
4. 適配器轉換格式並回傳 YAML 訂單
   ↓
5. 您的節點處理藥物 (一次一個)
   ↓
6. 回報進度到適配器
   ↓
7. 完成後回報 success/failed
   ↓
8. 適配器更新資料庫狀態 (completed/failed)
   ↓
9. 處理下一個訂單
```

## 🛠️ 故障排除

### 問題 1: 節點無法拉取訂單
**檢查**:
```bash
# 確認環境變數
echo $ORDER_BASE_URL
echo $ORDER_PULL_URL

# 測試 API 連通性
curl http://127.0.0.1:8002/api/order/next
```

### 問題 2: 藥物查詢服務無回應
**檢查**:
```bash
# 確認服務是否運行
ros2 service list | grep hospital

# 測試服務
ros2 service call /hospital/get_medicine_detail tm_robot_if/srv/MedicineDetail "{name: 'test'}"
```

### 問題 3: 訂單處理後狀態未更新
**檢查**:
- 確認您的節點正確調用 `_report_complete()`
- 檢查適配器日誌
- 查看處方籤管理界面的狀態

## 🔧 客製化設定

### 調整位置邏輯
在 `integration_with_your_node.py` 中修改：
```python
def generate_location(self, index: int) -> List[int]:
    """根據您的藥櫃配置調整"""
    # 您的自定義邏輯
    row = your_custom_row_logic(index)
    col = your_custom_col_logic(index)
    return [row, col]
```

### 調整藥物類型判斷
```python
def determine_prompt(self, medicine_name: str) -> str:
    """根據您的藥物分類調整"""
    # 您的自定義邏輯
    if your_custom_logic(medicine_name):
        return 'your_custom_type'
    return 'tablet'
```

### 調整拉取頻率
```bash
export ORDER_PULL_INTERVAL='5'  # 改為 5 秒拉取一次
```

## 📚 API 參考

### 適配器 API (port 8002)

#### GET /api/order/next
拉取下一個待處理訂單
- **回應**: 204 (無訂單) 或 200 + YAML 訂單

#### POST /api/order/progress
回報處理進度
```json
{
  "order_id": "000001",
  "stage": "processing",
  "message": "處理中...",
  "item": "阿斯匹靈",
  "index": 1,
  "total": 3
}
```

#### POST /api/order/complete
回報完成狀態
```json
{
  "order_id": "000001",
  "status": "success",
  "details": "所有藥物已完成"
}
```

### Medicine Detail Service

#### /hospital/get_medicine_detail
查詢單個藥物詳細資訊
- **請求**: `{name: "藥物名稱"}`
- **回應**: YAML 格式的藥物詳細資訊

#### /hospital/get_all_medicines  
獲取所有可用藥物列表
- **請求**: `{name: ""}`
- **回應**: YAML 格式的藥物列表

#### /hospital/search_medicines
搜尋藥物 (模糊匹配)
- **請求**: `{name: "搜尋關鍵字"}`
- **回應**: YAML 格式的搜尋結果

## 🎉 完成！

現在您的 ROS2 節點已經完全整合到我們的醫院藥物管理系統中：

✅ **自動拉取 YAML 訂單**  
✅ **一次處理一個，等完成再下一個**  
✅ **回報進度和完成狀態給網站**  
✅ **查詢藥物詳細資訊**  
✅ **完整的 Web 管理界面**  

**您只需要在 `_process_medicine()` 中實現您的機器人邏輯即可！** 🎊
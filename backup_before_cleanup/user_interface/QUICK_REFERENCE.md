# 🚀 醫院藥物管理系統 - 快速參考

## 📋 系統模式總覽

### 1️⃣ Python 推送模式 (推薦)
**特點**: 系統主動推送 YAML 訂單到您的 Python 函數
```bash
python3 start_system_modes.py
# 選擇 1 - Python 推送模式
```

**您會收到**:
```python
def your_process_order(order_dict, yaml_order):
    # order_dict = 解析後的訂單字典
    # yaml_order = 原始 YAML 字符串
    order_id = order_dict['order_id']
    
    # 處理訂單邏輯
    for medicine in order_dict.get('medicine', []):
        name = medicine['name']
        amount = medicine['amount']
        locate = medicine['locate']  # [row, col]
        prompt = medicine['prompt']  # tablet/capsule/white_circle_box
        
        # 您的機器人邏輯
        
    # 重要：完成後必須調用
    self._order_pusher.complete_order(order_id)
```

### 2️⃣ ROS2 服務模式
**特點**: 標準 ROS2 服務和 Topic，YAML 格式通信
```bash
python3 start_system_modes.py
# 選擇 2 - ROS2 服務模式
```

**使用方法**:
```bash
# 獲取訂單
ros2 service call /hospital/get_order std_srvs/srv/Empty

# 監聽訂單數據
ros2 topic echo /hospital/order_data

# 完成訂單
ros2 service call /hospital/complete_order std_srvs/srv/Empty
```

### 3️⃣ 僅 Web 系統
**特點**: 只啟動 Web 界面，手動測試
```bash
python3 start_system_modes.py
# 選擇 3 - 僅 Web 系統
```

## 🌐 Web 界面 (所有模式都包含)

| 功能 | 網址 | 用途 |
|------|------|------|
| 藥物管理 | http://localhost:8001/integrated_medicine_management.html | 新增、管理藥物 |
| 醫生工作台 | http://localhost:8001/doctor.html | 開立處方籤 |
| 處方籤管理 | http://localhost:8001/Prescription.html | 查看處方籤狀態 |
| API 文檔 | http://localhost:8001/docs | FastAPI 自動文檔 |

## 💊 專用藥物詳細資料服務

**啟動**:
```bash
python3 ros2_medicine_detail_service.py
```

**查詢藥物**:
```bash
# Topic 方式
ros2 topic pub /medicine/detail_request std_msgs/msg/String 'data: "阿斯匹靈"'
ros2 topic echo /medicine/detail_response

# Python 客戶端
python3 medicine_client_example.py
```

**YAML 回應格式**:
```yaml
name: 阿斯匹靈
found: true
description: "解熱鎮痛藥，用於緩解頭痛、發燒等症狀"
category: "解熱鎮痛藥"
unit_dose: "500mg"
stock_quantity: 100
```

## 📄 訂單 YAML 格式

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

## 🧪 測試工具

### 自動測試腳本
```bash
# 基本測試
python3 test_order_flow.py basic

# 批量測試
python3 test_order_flow.py batch 5

# 查看所有處方籤
python3 test_order_flow.py list

# 手動完成處方籤
python3 test_order_flow.py complete 1

# 重置處方籤狀態
python3 test_order_flow.py reset 1 pending
```

### Web 界面測試
1. **新增藥物**: http://localhost:8001/integrated_medicine_management.html
2. **開立處方籤**: http://localhost:8001/doctor.html
3. **監控狀態**: http://localhost:8001/Prescription.html

### API 測試
```bash
# 創建處方籤
curl -X POST "http://localhost:8001/api/prescription/" \
-H "Content-Type: application/json" \
-d '{
  "patient_name": "測試病患",
  "medicines": [{"name": "測試藥物", "amount": 5}]
}'

# 查看處方籤
curl "http://localhost:8001/api/prescription/"

# 手動完成
curl -X PUT "http://localhost:8001/api/prescription/1/status" \
-H "Content-Type: application/json" \
-d '{"status": "completed"}'
```

## 🔄 工作流程

### 自動推送流程
```
醫生開立處方籤 → 存入資料庫 (pending) → 
自動推送/獲取 → 您的處理 → 調用完成 → 
更新狀態 (completed) → 處理下一個
```

### 手動測試流程
```
1. 啟動系統 → 2. 新增測試藥物 → 3. 開立處方籤 → 
4. 觀察推送 → 5. 手動完成 (如需要)
```

## ⚡ 快速指令

### 系統啟動
```bash
# 多模式啟動 (推薦)
python3 start_system_modes.py

# 完整系統 (Python 推送模式)
python3 start_complete_system.py

# 僅服務器
python3 simple_server_final.py
```

### ROS2 命令
```bash
# 檢查服務
ros2 service list | grep hospital

# 檢查 Topic
ros2 topic list | grep hospital

# 查詢藥物
ros2 topic pub /medicine/detail_request std_msgs/msg/String 'data: "藥物名稱"'
```

### API 快速檢查
```bash
# 系統狀態
curl "http://localhost:8001/api/system/status"

# 處方籤列表
curl "http://localhost:8001/api/prescription/"

# 藥物列表
curl "http://localhost:8001/api/medicine/detailed"
```

## 🛠️ 故障排除

### 問題 1: 訂單沒有自動推送
**檢查**:
- 系統是否運行在正確模式
- 處方籤狀態是否為 'pending'
- 推送器是否正常運行

**解決**:
```bash
# 檢查系統狀態
curl "http://localhost:8001/api/system/status"

# 手動測試
python3 test_order_flow.py basic
```

### 問題 2: ROS2 服務無回應
**檢查**:
```bash
ros2 node list
ros2 service list
ros2 topic list
```

### 問題 3: Web 界面錯誤
**檢查**:
- 服務器是否運行在 port 8001
- 瀏覽器控制台錯誤
- API 端點是否可用

## 🎯 最佳實踐

### 開發建議
1. **先用 Web 界面**測試基本功能
2. **使用測試腳本**驗證自動化流程
3. **監控日誌**了解系統行為
4. **逐步整合**到您的 ROS2 系統

### 生產使用
1. **模式 1**: 適合直接 Python 整合
2. **模式 2**: 適合標準 ROS2 節點
3. **監控機制**: 使用處方籤管理界面
4. **錯誤處理**: 實現完整的異常處理

## 📚 詳細文檔

- [ORDER_FLOW_GUIDE.md](ORDER_FLOW_GUIDE.md) - 詳細流程說明
- [MEDICINE_DETAIL_SERVICE_GUIDE.md](MEDICINE_DETAIL_SERVICE_GUIDE.md) - 藥物查詢服務
- [ROS2_SERVICES_GUIDE.md](ROS2_SERVICES_GUIDE.md) - ROS2 服務接口
- [WEB_ROS2_ARCHITECTURE.md](WEB_ROS2_ARCHITECTURE.md) - 系統架構圖

**🎊 現在您可以快速開始使用醫院藥物管理系統！**
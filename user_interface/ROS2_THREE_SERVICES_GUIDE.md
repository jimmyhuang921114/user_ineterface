# 醫院 ROS2 三服務接口使用指南

## 概述

本系統提供三個專門的 ROS2 服務接口，滿足您的需求：

1. **訂單服務 (Order Service)** - 一次只處理一個 client 請求
2. **基本藥物服務 (Basic Medicine Service)** - 持續獲取基本藥物資訊
3. **詳細藥物服務 (Detailed Medicine Service)** - 持續獲取詳細藥物資訊

## 🚀 快速開始

### 1. 導入和初始化

```python
from ros2_service_interfaces import HospitalROS2ServiceManager

# 創建服務管理器
manager = HospitalROS2ServiceManager()

# 啟動持續服務
manager.start_continuous_services()
```

### 2. 使用三個服務

```python
# 📦 訂單服務 - 每筆完成後再送下一筆
manager.send_order([
    {"name": "阿司匹林", "quantity": 2},
    {"name": "維他命C", "quantity": 1}
], {
    "patient_name": "張三",
    "doctor_name": "李醫師"
})

# 💊 基本藥物服務 - 持續查詢
manager.query_basic_medicine("阿司匹林")

# 🔬 詳細藥物服務 - 持續查詢
manager.query_detailed_medicine("布洛芬")
```

## 📦 訂單服務 (Order Service)

### 特點
- ✅ **單一請求處理**：一次只處理一個 client 請求
- ✅ **忙碌拒絕機制**：處理中時拒絕新請求
- ✅ **狀態追蹤**：即時監控處理狀態

### 使用方法

```python
# 發送訂單
manager.send_order(
    medicines=[
        {"name": "藥物名稱", "quantity": 數量},
        {"name": "藥物名稱2", "quantity": 數量2}
    ],
    patient_info={
        "patient_name": "患者姓名",
        "doctor_name": "醫生姓名"
    }
)

# 檢查訂單狀態
status = manager.get_service_status()
order_status = status['order_service']
print(f"當前處理: {order_status['current_order']}")
print(f"可接受新訂單: {order_status['ready_for_new_order']}")
```

### ROS2 Topic
- **發布**: `hospital/order_request`
- **訂閱**: `hospital/order_completed`

## 💊 基本藥物服務 (Basic Medicine Service)

### 特點
- ✅ **持續服務**：可連續查詢
- ✅ **快速響應**：僅返回基本資訊
- ✅ **低延遲**：適合頻繁查詢

### 返回資訊
- 藥物 ID
- 藥物名稱
- 庫存數量
- 儲存位置
- 製造商
- 劑量

### 使用方法

```python
# 查詢基本藥物資訊
result = manager.query_basic_medicine("阿司匹林")

# HTTP 模式下可直接獲取結果
if result:
    medicines = result.get('medicines', [])
    for med in medicines:
        print(f"藥物: {med['name']}")
        print(f"庫存: {med['amount']}")
        print(f"位置: {med['position']}")
```

### ROS2 Topic
- **發布**: `hospital/basic_medicine_request`
- **訂閱**: `hospital/basic_medicine_response`

## 🔬 詳細藥物服務 (Detailed Medicine Service)

### 特點
- ✅ **持續服務**：可連續查詢
- ✅ **完整資訊**：包含所有藥物詳情
- ✅ **專業資料**：適合醫療決策

### 返回資訊
- 藥物描述
- 活性成分
- 藥物類別
- 使用方法
- 單位劑量
- 副作用
- 儲存條件
- 有效期限
- 條碼
- 外觀描述
- 備註

### 使用方法

```python
# 查詢詳細藥物資訊
result = manager.query_detailed_medicine("布洛芬")

# HTTP 模式下可直接獲取結果
if result:
    details = result.get('detailed_medicines', [])
    for detail in details:
        print(f"描述: {detail['description']}")
        print(f"成分: {detail['ingredient']}")
        print(f"用法: {detail['usage_method']}")
        print(f"副作用: {detail['side_effects']}")
```

### ROS2 Topic
- **發布**: `hospital/detailed_medicine_request`
- **訂閱**: `hospital/detailed_medicine_response`

## 🔄 併發使用

三個服務可以同時使用，互不干擾：

```python
# 同時使用三個服務
manager.send_order([{"name": "阿司匹林", "quantity": 2}])
manager.query_basic_medicine("布洛芬")
manager.query_detailed_medicine("維他命C")

# 持續查詢不會影響訂單處理
for medicine in ["阿司匹林", "布洛芬", "維他命C"]:
    manager.query_basic_medicine(medicine)
    manager.query_detailed_medicine(medicine)
```

## 📊 服務狀態監控

```python
# 獲取所有服務狀態
status = manager.get_service_status()

print("訂單服務:")
print(f"  當前訂單: {status['order_service']['current_order']}")
print(f"  處理中: {status['order_service']['processing']}")
print(f"  可接受新訂單: {status['order_service']['ready_for_new_order']}")

print("基本藥物服務:")
print(f"  運行狀態: {status['basic_service']['running']}")

print("詳細藥物服務:")
print(f"  運行狀態: {status['detailed_service']['running']}")
```

## 🎯 使用場景

### 訂單服務適用於：
- 處方籤處理
- 藥物配送
- 庫存扣減
- 工作流程控制

### 基本藥物服務適用於：
- 快速庫存查詢
- 位置確認
- 簡單資訊顯示
- 高頻率查詢

### 詳細藥物服務適用於：
- 醫療決策支援
- 藥物諮詢
- 處方開立
- 完整資訊展示

## 🛠️ 技術實現

### ROS2 模式
- 使用 `rclpy` 進行通訊
- 支援發布/訂閱模式
- 低延遲、高可靠性

### HTTP 模式
- 自動降級為 HTTP API
- 保持相同的接口
- 易於調試和測試

## 📝 測試

```bash
# 運行測試腳本
python3 test_ros2_three_services.py

# 運行簡單範例
python3 ros2_service_interfaces.py
```

## 🔧 配置

```python
# 自訂服務器位址
manager = HospitalROS2ServiceManager(base_url="http://your-server:8001")

# 手動控制持續服務
manager.basic_service.start_continuous_service()
manager.detailed_service.start_continuous_service()

# 停止服務
manager.stop_continuous_services()
```

## 💡 最佳實踐

1. **訂單服務**：
   - 一次只能處理一個請求，處理中時會拒絕新請求
   - 等待當前訂單完成後再發送新訂單

2. **基本藥物服務**：
   - 用於頻繁的快速查詢
   - 適合儀表板和狀態顯示

3. **詳細藥物服務**：
   - 按需查詢，避免不必要的資料傳輸
   - 適合詳細頁面和報告

4. **併發使用**：
   - 三個服務獨立運行，可安全併發
   - 根據實際需求選擇合適的服務

## 🚨 注意事項

- 訂單服務會自動扣減庫存
- 持續服務需要手動啟動和停止
- ROS2 模式需要正確的環境配置
- HTTP 模式作為備選方案，功能完全相同
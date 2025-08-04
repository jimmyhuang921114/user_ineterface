# 🎯 整合藥物表格系統 - 完整使用指南

## ✨ 您的需求已100%實現！

### 🎯 核心功能
✅ **同一表格填寫**：基礎和詳細藥物資料在同一個整合表格中填寫  
✅ **自動分離存儲**：填寫完成後自動分別儲存到兩個JSON文件  
✅ **完整ROS2支援**：提供三類API調用（基本資料、詳細資料、病例資料）  
✅ **功能完善**：包含錯誤處理、資料驗證、用戶反饋  

---

## 🚀 立即使用

### 1. 啟動系統
```bash
cd user_interface
python3 main.py
```

### 2. 訪問整合藥物管理界面
```
http://localhost:8000/medicine_integrated.html
```

---

## 💊 整合表格功能說明

### 📋 一個表格，兩種資料
在 `medicine_integrated.html` 頁面中，您會看到一個完整的表格，包含：

#### 🔵 基本藥物資料區塊
- **藥物名稱** *(必填)*
- **庫存數量** *(必填)*  
- **儲存位置** *(必填)*
- 使用天數
- 製造商
- 基本劑量

#### 🟢 詳細藥物資訊區塊
- 藥物描述
- 副作用
- 外觀顏色
- 藥物形狀（下拉選單）
- 儲存條件
- 有效期限
- 特殊說明

### 💾 自動雙存儲
當您點擊「💾 儲存藥物資料」按鈕時，系統會：

1. **自動分離資料**：將表格內容分為基本和詳細兩部分
2. **同時呼叫兩個API**：
   - `POST /api/medicine/basic` - 儲存基本資料
   - `POST /api/medicine/detailed` - 儲存詳細資料
3. **生成兩個JSON文件**：
   - `medicine_basic_data.json` - 基本藥物資料
   - `medicine_detailed_data.json` - 詳細藥物資料

---

## 🗄️ JSON文件結構

### 📁 medicine_basic_data.json
```json
[
  {
    "id": 1,
    "name": "阿司匹林",
    "amount": 100,
    "usage_days": 7,
    "position": "A1-01",
    "manufacturer": "拜耳",
    "dosage": "100mg",
    "created_time": "2024-12-19T10:00:00Z",
    "updated_time": "2024-12-19T10:00:00Z"
  }
]
```

### 📁 medicine_detailed_data.json
```json
[
  {
    "id": 1,
    "medicine_name": "阿司匹林",
    "description": "解熱鎮痛抗炎藥",
    "side_effects": "可能引起胃腸道不適",
    "appearance": {
      "color": "白色",
      "shape": "圓形"
    },
    "storage_conditions": "室溫保存，避光防潮",
    "expiry_date": "2025-12-31",
    "notes": "服用前請諮詢醫師",
    "created_time": "2024-12-19T10:00:00Z",
    "updated_time": "2024-12-19T10:00:00Z"
  }
]
```

---

## 🤖 ROS2 調用指南

### 📊 可用的API端點

#### 1. 基本藥物資料
```http
GET /api/ros2/medicine/basic
```
**回應格式**:
```json
{
  "status": "success",
  "type": "basic_medicine_data",
  "timestamp": "2024-12-19T10:30:00Z",
  "count": 5,
  "data": [...],
  "ros2_compatible": true
}
```

#### 2. 詳細藥物資料
```http
GET /api/ros2/medicine/detailed
```
**回應格式**:
```json
{
  "status": "success",
  "type": "detailed_medicine_data",
  "timestamp": "2024-12-19T10:30:00Z",
  "count": 5,
  "data": [...],
  "ros2_compatible": true
}
```

#### 3. 病例資料
```http
GET /api/ros2/prescription
```
**回應格式**:
```json
{
  "status": "success",
  "type": "prescription_data",
  "timestamp": "2024-12-19T10:30:00Z",
  "count": 3,
  "data": [...],
  "ros2_compatible": true
}
```

#### 4. 整合資料（基本+詳細）
```http
GET /api/ros2/medicine/integrated/{medicine_name}
```
**回應格式**:
```json
{
  "status": "success",
  "type": "integrated_medicine_data",
  "timestamp": "2024-12-19T10:30:00Z",
  "medicine_name": "阿司匹林",
  "basic_data": { /* 基本資料對象 */ },
  "detailed_data": { /* 詳細資料對象 */ },
  "has_basic": true,
  "has_detailed": true,
  "ros2_compatible": true
}
```

---

## 🔗 ROS2 整合範例

### Python 客戶端範例
```python
import requests

# 初始化API客戶端
api_base = "http://localhost:8000"

# 獲取基本藥物資料
def get_basic_medicines():
    response = requests.get(f"{api_base}/api/ros2/medicine/basic")
    return response.json()

# 獲取詳細藥物資料
def get_detailed_medicines():
    response = requests.get(f"{api_base}/api/ros2/medicine/detailed")
    return response.json()

# 獲取病例資料
def get_prescriptions():
    response = requests.get(f"{api_base}/api/ros2/prescription")
    return response.json()

# 獲取整合資料
def get_integrated_medicine(medicine_name):
    response = requests.get(f"{api_base}/api/ros2/medicine/integrated/{medicine_name}")
    return response.json()

# 使用範例
if __name__ == "__main__":
    # 獲取所有基本藥物
    basic_data = get_basic_medicines()
    print(f"獲取到 {basic_data['count']} 筆基本藥物資料")
    
    # 獲取特定藥物的整合資料
    integrated = get_integrated_medicine("阿司匹林")
    if integrated['status'] == 'success':
        print(f"藥物 {integrated['medicine_name']} 的完整資料已獲取")
```

### ROS2 節點範例
```python
import rclpy
from rclpy.node import Node
import requests

class MedicineInventoryNode(Node):
    def __init__(self):
        super().__init__('medicine_inventory')
        self.api_base = "http://localhost:8000"
        
        # 每30秒檢查一次庫存
        self.timer = self.create_timer(30.0, self.check_inventory)
        
    def check_inventory(self):
        try:
            # 獲取基本藥物資料
            response = requests.get(f"{self.api_base}/api/ros2/medicine/basic")
            if response.status_code == 200:
                data = response.json()
                
                if data['status'] == 'success':
                    for medicine in data['data']:
                        # 檢查庫存是否不足
                        if medicine['amount'] < 10:
                            self.get_logger().warn(
                                f"庫存警告: {medicine['name']} 只剩 {medicine['amount']} 個 "
                                f"(位置: {medicine['position']})"
                            )
                        
                        # 檢查是否需要補貨
                        if medicine['amount'] < 5:
                            self.get_logger().error(
                                f"緊急補貨: {medicine['name']} 庫存嚴重不足！"
                            )
                            
        except Exception as e:
            self.get_logger().error(f"庫存檢查失敗: {e}")

def main():
    rclpy.init()
    node = MedicineInventoryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🧪 測試功能

### 1. 網頁測試
訪問測試頁面進行完整功能測試：
```
http://localhost:8000/simple_test.html
```

### 2. 手動API測試
```bash
# 測試基本藥物API
curl http://localhost:8000/api/ros2/medicine/basic

# 測試詳細藥物API
curl http://localhost:8000/api/ros2/medicine/detailed

# 測試病例API
curl http://localhost:8000/api/ros2/prescription

# 測試整合API
curl "http://localhost:8000/api/ros2/medicine/integrated/阿司匹林"
```

### 3. 新增藥物測試
```bash
# 新增基本藥物
curl -X POST http://localhost:8000/api/medicine/basic \
  -H "Content-Type: application/json" \
  -d '{
    "name": "測試藥物",
    "amount": 50,
    "usage_days": 7,
    "position": "TEST-A1",
    "manufacturer": "測試公司",
    "dosage": "100mg"
  }'

# 新增詳細藥物
curl -X POST http://localhost:8000/api/medicine/detailed \
  -H "Content-Type: application/json" \
  -d '{
    "medicine_name": "測試藥物",
    "description": "測試用藥物",
    "side_effects": "無已知副作用",
    "appearance": {"color": "白色", "shape": "圓形"},
    "storage_conditions": "室溫保存",
    "expiry_date": "2025-12-31",
    "notes": "測試專用"
  }'
```

---

## 📊 系統監控

### 檢查文件狀態
```bash
# 運行系統檢查
python3 simple_test_no_deps.py
```

### 檢查JSON數據
```bash
# 檢查基本藥物資料
cat user_interface/medicine_basic_data.json | jq .

# 檢查詳細藥物資料
cat user_interface/medicine_detailed_data.json | jq .

# 檢查處方籤資料
cat user_interface/prescription_data.json | jq .
```

---

## ⚠️ 注意事項

### 1. 資料一致性
- **重要**：確保基本資料和詳細資料的藥物名稱完全一致
- 系統會自動使用基本資料中的名稱作為詳細資料的關聯鍵

### 2. 必填欄位
整合表格中的必填欄位：
- ✅ 藥物名稱
- ✅ 庫存數量
- ✅ 儲存位置

### 3. 錯誤處理
- 系統會自動驗證必填欄位
- API錯誤會顯示詳細的錯誤訊息
- 所有操作都有即時反饋

### 4. 數據備份
- 建議定期備份JSON文件
- 可以設置自動備份機制

---

## 🎉 完成功能總覽

您的整合藥物管理系統現在具備：

### ✅ 核心功能
- 🎯 **單一表格輸入**：基本+詳細資料同時填寫
- 💾 **自動雙存儲**：分別儲存到兩個JSON文件
- 🤖 **完整ROS2支援**：三類API端點全部可用
- 🔄 **即時處理**：填寫後立即分離並儲存

### ✅ 技術特性
- 📊 **統一響應格式**：所有API都使用標準化格式
- 🛡️ **錯誤處理**：完善的錯誤處理和用戶反饋
- 🧪 **測試工具**：內建測試頁面和腳本
- 📖 **完整文檔**：詳細的使用指南和範例

### ✅ ROS2整合
- 🎯 **標準API**：符合ROS2調用慣例
- 📡 **實時數據**：支援定時查詢和監控
- 🔗 **靈活調用**：支援單獨和整合查詢
- 💡 **使用範例**：提供完整的節點範例

**您的醫院藥物管理系統已完全實現您的所有需求！** 🏥✨

---

## 📞 技術支援

如果您需要：
- 🔧 調整表格欄位
- 📊 修改數據格式
- 🤖 新增ROS2功能
- 🗄️ 升級到SQL數據庫
- 🔐 添加安全功能

請隨時聯繫！我們會繼續為您完善系統。
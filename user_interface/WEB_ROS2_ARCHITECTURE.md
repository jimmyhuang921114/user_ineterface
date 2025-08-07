# Web 系統與 ROS2 整合架構圖

## 🏗️ 整體系統架構

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                               醫院藥物管理系統                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐       │
│  │   醫生工作台      │    │   藥物管理界面    │    │   處方籤管理      │       │
│  │  doctor.html     │    │  integrated_     │    │ Prescription.html │       │
│  │                  │    │  medicine_       │    │                  │       │
│  │ • 開立處方籤      │    │ • 管理庫存       │    │ • 查看處方籤列表  │       │
│  │ • 選擇藥物       │    │ • 新增藥物       │    │ • 監控處理狀態    │       │
│  │ • 設定劑量       │    │ • 查看詳細資訊    │    │ • 手動狀態更新    │       │
│  └──────────────────┘    └──────────────────┘    └──────────────────┘       │
│           │               │ • 自動轉換為訂單                        │             │
│           ▼               └──────────────────┘             ▼                │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      FastAPI 後端服務器                            │   │
│  │                   (simple_server_final.py)                        │   │
│  │                                                                    │   │
│  │  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ │   │
│  │  │   藥物 API   │ │  處方籤 API  │ │  ROS2 API   │ │  系統 API    │ │   │
│  │  │              │ │              │ │              │ │              │ │   │
│  │  │ • GET /basic │ │ • POST /     │ │ • POST /     │ │ • GET /      │ │   │
│  │  │ • GET /      │ │ • GET /      │ │   query-     │ │   status     │ │   │
│  │  │   detailed   │ │ • PUT /      │ │   detail     │ │              │ │   │
│  │  │ • POST /     │ │   {id}/      │ │ • POST /     │ │              │ │   │
│  │  │   unified    │ │   status     │ │   process-   │ │              │ │   │
│  │  │              │ │              │ │   order      │ │              │ │   │
│  │  └──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘ │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│           │                            │                                     │
│           ▼                            ▼                                     │
│  ┌─────────────────────┐    ┌─────────────────────────────────────────┐     │
│  │     SQLite 資料庫    │    │        ROS2 訂單推送器                  │     │
│  │ hospital_medicine_  │    │    (ros2_order_pusher.py)              │     │
│  │ final.db            │    │                                         │     │
│  │                     │    │ • 監控新處方籤                          │     │
│  │ • 藥物基本資料       │    │ • 自動轉換為訂單                        │     │
│  │ • 藥物詳細資料       │    │ • 單一訂單處理                          │     │
│  │ • 處方籤記錄        │    │ • 等待 ROS2 完成                        │     │
│  │ • 狀態追蹤          │    │ • 格式化 YAML 輸出                      │     │
│  └─────────────────────┘    └─────────────────────────────────────────┘     │
│                                            │                                 │
└────────────────────────────────────────────┼─────────────────────────────────┘
                                             │
                                             ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            您的 ROS2 系統                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                        ROS2 整合層                                   │   │
│  │                                                                      │   │
│  │  ┌────────────────┐    ┌─────────────────┐    ┌─────────────────┐   │   │
│  │  │   訂單接收器    │    │   狀態回報器     │    │   完成通知器     │   │   │
│  │  │                │    │                 │    │                 │   │   │
│  │  │ def receive_   │    │ def report_     │    │ def complete_   │   │   │
│  │  │ order(yaml):   │    │ status():       │    │ order(id):      │   │   │
│  │  │   # 解析訂單    │    │   # 回報進度     │    │   # 通知完成     │   │   │
│  │  │   # 呼叫機器人  │    │   # 更新狀態     │    │   # 重置狀態     │   │   │
│  │  └────────────────┘    └─────────────────┘    └─────────────────┘   │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                     │                                       │
│                                     ▼                                       │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                         ROS2 控制層                                  │   │
│  │                                                                      │   │
│  │  ┌────────────────┐    ┌─────────────────┐    ┌─────────────────┐   │   │
│  │  │  路徑規劃節點   │    │   機械臂控制     │    │   視覺識別節點   │   │   │
│  │  │                │    │                 │    │                 │   │   │
│  │  │ • 計算最佳路徑  │    │ • 抓取藥物      │    │ • 識別藥物類型   │   │   │
│  │  │ • 避障導航     │    │ • 精確定位      │    │ • 確認位置      │   │   │
│  │  │ • 移動控制     │    │ • 力控抓握      │    │ • 品質檢查      │   │   │
│  │  └────────────────┘    └─────────────────┘    └─────────────────┘   │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                     │                                       │
│                                     ▼                                       │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                        硬體設備層                                    │   │
│  │                                                                      │   │
│  │  ┌────────────────┐    ┌─────────────────┐    ┌─────────────────┐   │   │
│  │  │   移動平台      │    │   機械臂        │    │   感測器組      │   │   │
│  │  │                │    │                 │    │                 │   │   │
│  │  │ • 輪式底盤     │    │ • 6軸機械臂     │    │ • 攝影機        │   │   │
│  │  │ • 馬達驅動     │    │ • 夾爪器       │    │ • 雷射雷達      │   │   │
│  │  │ • 編碼器       │    │ • 關節感測器    │    │ • 超音波感測器  │   │   │
│  │  └────────────────┘    └─────────────────┘    └─────────────────┘   │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🔄 資料流程圖

```
醫生開立處方籤 → FastAPI 後端 → 資料庫儲存 → 訂單推送器監控
                                                      │
                                                      ▼
您的 ROS2 系統 ← YAML 格式訂單 ← 自動轉換 ← 新處方籤檢測
      │
      ▼
   開始執行任務
      │
      ▼
   完成後回調 → pusher.complete_order(order_id) → 更新處方籤狀態
      │
      ▼
   準備處理下一個訂單
```

## 🏃‍♂️ 實際整合流程

### 1. 初始設定
```python
from ros2_order_pusher import OrderPusher

def your_ros2_callback(order_dict, yaml_order):
    """您的 ROS2 處理函數"""
    order_id = order_dict['order_id']
    medicines = order_dict['medicines']
    
    # 呼叫您的 ROS2 系統
    your_ros2_system.process_order(yaml_order)
    
    # 在完成後調用
    # pusher.complete_order(order_id)

# 建立推送器
pusher = OrderPusher(callback_func=your_ros2_callback)
pusher.start_monitoring()
```

### 2. 訂單格式
您會收到的 YAML 格式：
```yaml
order_id: "000001"
medicine:
  - name: Antipsychotics
    amount: 87
    locate: [9, 6]
    prompt: white_circle_box
  - name: 測試藥物B
    amount: 212
    locate: [1, 3]
    prompt: tablet
```

### 3. 完成通知
```python
# 您的 ROS2 系統完成處理後
success = pusher.complete_order("000001")
if success:
    print("訂單完成，可處理下一個")
```

## 🔧 您需要實作的部分

### ROS2 節點架構
```python
import rclpy
from rclpy.node import Node
from ros2_order_pusher import OrderPusher

class HospitalRobotNode(Node):
    def __init__(self):
        super().__init__('hospital_robot')
        self.pusher = OrderPusher(callback_func=self.process_order)
        self.current_order_id = None
        
    def process_order(self, order_dict, yaml_order):
        """處理從 Web 系統收到的訂單"""
        self.current_order_id = order_dict['order_id']
        medicines = order_dict['medicines']
        
        self.get_logger().info(f"收到訂單: {self.current_order_id}")
        
        # 開始處理每個藥物
        for medicine in medicines:
            self.process_medicine(medicine)
        
        # 完成後通知系統
        self.pusher.complete_order(self.current_order_id)
        
    def process_medicine(self, medicine):
        """處理單個藥物"""
        name = medicine['name']
        amount = medicine['amount']
        locate = medicine['locate']  # [row, col]
        prompt = medicine['prompt']  # 藥物類型
        
        # 1. 導航到位置
        self.navigate_to_location(locate)
        
        # 2. 識別和抓取藥物
        self.pick_medicine(name, amount, prompt)
        
        # 3. 運送到分配點
        self.deliver_medicine()
        
    def navigate_to_location(self, locate):
        """導航到指定位置"""
        row, col = locate
        # 實作您的導航邏輯
        pass
        
    def pick_medicine(self, name, amount, prompt):
        """抓取指定藥物"""
        # 根據 prompt 類型調整抓取策略
        if prompt == 'tablet':
            # 片劑抓取邏輯
            pass
        elif prompt == 'capsule':
            # 膠囊抓取邏輯
            pass
        elif prompt == 'white_circle_box':
            # 盒裝藥物抓取邏輯
            pass
            
    def deliver_medicine(self):
        """運送藥物到目標位置"""
        # 實作運送邏輯
        pass
```

## 📊 通訊協定

### Web → ROS2
- **格式**: YAML 字符串
- **觸發**: 新處方籤自動推送
- **頻率**: 單一訂單處理
- **內容**: 訂單ID + 藥物清單 + 位置 + 類型

### ROS2 → Web
- **方法**: `pusher.complete_order(order_id)`
- **時機**: 訂單完成後
- **效果**: 更新處方籤狀態 + 允許下一個訂單

## 🎯 關鍵特性

1. **單一訂單處理**: 確保一次只處理一個訂單
2. **自動監控**: 無需手動查詢，自動推送新訂單
3. **狀態同步**: ROS2 完成後自動更新 Web 系統狀態
4. **錯誤處理**: 異常情況下自動重置狀態
5. **擴展性**: 易於添加新的藥物類型和處理邏輯

這個架構確保了 Web 系統與您的 ROS2 系統之間的無縫整合，並且符合您"等我 ROS2 完成後再繼續發送下一單"的需求。
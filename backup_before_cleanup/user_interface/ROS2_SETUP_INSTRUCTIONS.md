# 🤖 ROS2 自動推送訂單系統 - 設置說明

## 🎯 系統功能
- ✅ **自動推送 YAML 訂單給您的 ROS2 節點**
- ✅ **一次一個，等結束再進行下一個**
- ✅ **能夠告訴網站完成了**
- ✅ **查看完整訂單內容**

## 📦 完整代碼包內容

### ⭐ 核心檔案 (8 個)
```
user_interface/
├── database_final.py              # 資料庫模型
├── simple_server_final.py         # FastAPI Web 服務器
├── ros2_order_pusher.py           # 自動推送器
├── your_ros2_node.py              # 您的 ROS2 節點模板
├── start_your_system.py           # 系統啟動器
├── test_order_flow.py             # 測試工具
├── YOUR_ROS2_GUIDE.md             # 完整使用指南
├── ROS2_SETUP_INSTRUCTIONS.md    # 此檔案
└── static/                        # Web 界面
    ├── integrated_medicine_management.html
    ├── doctor.html
    ├── Prescription.html
    ├── css/
    └── js/
```

## 🚀 在您的 ROS2 Humble 環境中使用

### 1️⃣ 設置 ROS2 環境
```bash
# 在您的終端機中
source /opt/ros/humble/setup.bash
# 或者您的自定義路徑
source ~/your_ros2_path/setup.bash

# 確認 ROS2 可用
ros2 --version
```

### 2️⃣ 安裝 Python 依賴
```bash
pip3 install fastapi uvicorn sqlalchemy pydantic requests pyyaml
```

### 3️⃣ 複製檔案到您的環境
將所有檔案複製到您的工作目錄：
```bash
# 複製核心檔案
cp database_final.py your_workspace/
cp simple_server_final.py your_workspace/
cp ros2_order_pusher.py your_workspace/
cp your_ros2_node.py your_workspace/
cp start_your_system.py your_workspace/
cp test_order_flow.py your_workspace/
cp YOUR_ROS2_GUIDE.md your_workspace/

# 複製 Web 界面
cp -r static/ your_workspace/
```

### 4️⃣ 啟動系統
```bash
cd your_workspace

# 確保 ROS2 環境已設置
source /opt/ros/humble/setup.bash

# 啟動系統
python3 start_your_system.py
```

### 5️⃣ 測試系統
```bash
# 在另一個終端機
python3 test_order_flow.py basic
```

## 📄 您會收到的 YAML 訂單格式

當有新處方籤時，您的 ROS2 節點會自動收到：

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

## 🔧 實現您的機器人邏輯

在 `your_ros2_node.py` 的 `process_medicine()` 函數中添加您的代碼：

```python
def process_medicine(self, medicine: Dict[str, Any], index: int, total: int):
    """處理單個藥物 - 在這裡實現您的機器人邏輯！"""
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

## 🌐 Web 界面
啟動後可以訪問：

- **藥物管理**: http://localhost:8001/integrated_medicine_management.html
- **醫生工作台**: http://localhost:8001/doctor.html
- **處方籤管理**: http://localhost:8001/Prescription.html
- **API 文檔**: http://localhost:8001/docs

## 🔄 工作流程

```
醫生開立處方籤 (Web界面)
    ↓
存入資料庫 (status: pending)
    ↓
ros2_order_pusher 自動監控到新訂單
    ↓
轉換為 YAML 格式
    ↓
推送到您的 ROS2 節點 (your_ros2_node.py)
    ↓
自動顯示完整訂單內容
    ↓
處理每個藥物 (process_medicine)
    ↓
完成後自動告知網站 (complete_order)
    ↓
更新資料庫狀態為 completed
    ↓
自動檢查並處理下一個訂單
```

## 🧪 測試方法

### 自動測試
```bash
python3 test_order_flow.py basic          # 基本測試
python3 test_order_flow.py batch 3        # 批量測試
python3 test_order_flow.py list           # 查看處方籤
python3 test_order_flow.py complete 1     # 手動完成
```

### Web 界面測試
1. 新增藥物: http://localhost:8001/integrated_medicine_management.html
2. 開立處方籤: http://localhost:8001/doctor.html
3. 觀察終端機的自動推送
4. 監控狀態: http://localhost:8001/Prescription.html

## 🛠️ 故障排除

### ROS2 環境問題
```bash
# 檢查 ROS2 環境
echo $ROS_DISTRO
ros2 node list

# 重新設置環境
source /opt/ros/humble/setup.bash
```

### Python 依賴問題
```bash
# 安裝缺少的套件
pip3 install missing_package_name
```

### 端口衝突
```bash
# 檢查端口 8001 是否被佔用
netstat -tulpn | grep :8001

# 終止佔用的進程
sudo kill -9 <PID>
```

## 📚 詳細說明

查看 `YOUR_ROS2_GUIDE.md` 獲得：
- 完整的使用指南
- 客製化設定方法
- 詳細的故障排除
- API 文檔說明

## 🎉 立即開始！

1. **複製檔案** 到您的 ROS2 環境
2. **設置 ROS2** 環境變數
3. **啟動系統**: `python3 start_your_system.py`
4. **測試**: `python3 test_order_flow.py basic`
5. **實現邏輯** 在 `your_ros2_node.py` 中

**您的自動推送 ROS2 訂單系統已準備就緒！** 🎊
  ROS 醫院藥物管理系統

  快速啟動

  啟動 Web 系統和適配器
```bash
cd user_interface
python integration_with_your_node.py
```

  啟動您的 ROS 節點
```bash
 新終端機
source /opt/ros/humble/setup.bash
export ORDER_BASE_URL='http://...:'
python your_order_handler_node.py
```

  (可選) 啟動藥物查詢服務
```bash
 新終端機  
source /opt/ros/humble/setup.bash
python user_interface/medicine_detail_service_node.py
```

  YAML 訂單格式

您的 ROS 節點會收到：
```yaml
order_id: ""
prescription_id: 
patient_name: "張三"
medicine:
  - name: 阿斯匹靈
    amount: 
    locate: [, ]
    prompt: tablet
```

  Web 界面

- 藥物管理: http://localhost:/integrated_medicine_management.html
- 醫生工作台: http://localhost:/doctor.html  
- 處方籤管理: http://localhost:/Prescription.html

  ROS 藥物查詢服務

```bash
ros service call /hospital/get_medicine_detail tm_robot_if/srv/MedicineDetail "{name: '阿斯匹靈'}"
ros service call /hospital/get_all_medicines tm_robot_if/srv/MedicineDetail "{name: ''}"
ros service call /hospital/search_medicines tm_robot_if/srv/MedicineDetail "{name: '感冒'}"
```

  必要檔案

- `user_interface/database_final.py` - 資料庫模型
- `user_interface/simple_server_final.py` - Web 伺服器
- `user_interface/integration_with_your_node.py` - ROS 適配器
- `user_interface/medicine_detail_service_node.py` - 藥物查詢服務
- `user_interface/test_order_flow.py` - 測試工具
- `user_interface/static/` - Web 界面檔案
- `user_interface/hospital_medicine_final.db` - 資料庫

  在您的節點中實現

```python
def _process_medicine(self, order_id: str, med: Dict[str, Any], idx: int, total: int):
    name = med.get('name')
    amount = med.get('amount') 
    locate = med.get('locate')   [row, col]
    prompt = med.get('prompt')   tablet/capsule/white_circle_box
    
     在這裡實現您的機器人邏輯
     . 移動到 locate 位置
     . 根據 prompt 抓取 amount 數量的 name 藥物
     . 放置到配藥區
```

  測試

```bash
cd user_interface
python test_order_flow.py basic
```

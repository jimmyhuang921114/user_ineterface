#!/usr/bin/env python3
"""
ROS2 整合範例 - 醫院藥物管理系統
展示如何在ROS2節點中調用藥物管理API
"""

import requests
import json
from datetime import datetime
import time

class HospitalMedicineClient:
    """醫院藥物管理系統客戶端 - ROS2整合"""
    
    def __init__(self, base_url="http://localhost:8000"):
        self.base_url = base_url
        self.session = requests.Session()
        
    def get_basic_medicines(self):
        """獲取基本藥物資料"""
        try:
            response = self.session.get(f"{self.base_url}/api/ros2/medicine/basic")
            if response.status_code == 200:
                data = response.json()
                print(f"✅ 成功獲取 {data['count']} 筆基本藥物資料")
                return data
            else:
                print(f"❌ 獲取基本藥物資料失敗: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ 連接錯誤: {e}")
            return None
    
    def get_detailed_medicines(self):
        """獲取詳細藥物資料"""
        try:
            response = self.session.get(f"{self.base_url}/api/ros2/medicine/detailed")
            if response.status_code == 200:
                data = response.json()
                print(f"✅ 成功獲取 {data['count']} 筆詳細藥物資料")
                return data
            else:
                print(f"❌ 獲取詳細藥物資料失敗: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ 連接錯誤: {e}")
            return None
    
    def get_prescriptions(self):
        """獲取病例資料"""
        try:
            response = self.session.get(f"{self.base_url}/api/ros2/prescription")
            if response.status_code == 200:
                data = response.json()
                print(f"✅ 成功獲取 {data['count']} 筆病例資料")
                return data
            else:
                print(f"❌ 獲取病例資料失敗: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ 連接錯誤: {e}")
            return None
    
    def get_integrated_medicine(self, medicine_name):
        """獲取整合藥物資料（基本+詳細）"""
        try:
            response = self.session.get(f"{self.base_url}/api/ros2/medicine/integrated/{medicine_name}")
            if response.status_code == 200:
                data = response.json()
                if data['status'] == 'success':
                    print(f"✅ 成功獲取藥物 '{medicine_name}' 的整合資料")
                    print(f"   - 有基本資料: {data['has_basic']}")
                    print(f"   - 有詳細資料: {data['has_detailed']}")
                else:
                    print(f"❌ 找不到藥物 '{medicine_name}'")
                return data
            else:
                print(f"❌ 獲取整合藥物資料失敗: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ 連接錯誤: {e}")
            return None
    
    def add_basic_medicine(self, medicine_data):
        """新增基本藥物資料"""
        try:
            response = self.session.post(
                f"{self.base_url}/api/medicine/basic",
                json=medicine_data,
                headers={'Content-Type': 'application/json'}
            )
            if response.status_code == 200:
                result = response.json()
                print(f"✅ 成功新增基本藥物: {medicine_data['name']}")
                return result
            else:
                print(f"❌ 新增基本藥物失敗: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ 連接錯誤: {e}")
            return None
    
    def add_detailed_medicine(self, medicine_data):
        """新增詳細藥物資料"""
        try:
            response = self.session.post(
                f"{self.base_url}/api/medicine/detailed",
                json=medicine_data,
                headers={'Content-Type': 'application/json'}
            )
            if response.status_code == 200:
                result = response.json()
                print(f"✅ 成功新增詳細藥物: {medicine_data['medicine_name']}")
                return result
            else:
                print(f"❌ 新增詳細藥物失敗: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ 連接錯誤: {e}")
            return None

def simulate_ros2_node():
    """模擬ROS2節點運行"""
    print("🤖 =================================================")
    print("🤖 ROS2 醫院藥物管理系統整合範例")
    print("🤖 =================================================")
    
    # 初始化客戶端
    client = HospitalMedicineClient()
    
    # 測試資料
    test_basic_medicine = {
        "name": "ROS2測試藥物",
        "amount": 50,
        "usage_days": 7,
        "position": "ROS2-A1",
        "manufacturer": "ROS2製藥",
        "dosage": "100mg"
    }
    
    test_detailed_medicine = {
        "medicine_name": "ROS2測試藥物",
        "description": "這是由ROS2系統新增的測試藥物",
        "side_effects": "可能引起輕微頭暈",
        "appearance": {
            "color": "白色",
            "shape": "圓形"
        },
        "storage_conditions": "室溫保存",
        "expiry_date": "2025-12-31",
        "notes": "ROS2整合測試專用"
    }
    
    print("\n🔄 開始ROS2整合測試...")
    
    # 1. 新增測試資料
    print("\n📝 1. 新增測試藥物資料...")
    client.add_basic_medicine(test_basic_medicine)
    time.sleep(0.5)
    client.add_detailed_medicine(test_detailed_medicine)
    time.sleep(0.5)
    
    # 2. 獲取基本藥物資料
    print("\n📋 2. 獲取基本藥物資料...")
    basic_data = client.get_basic_medicines()
    if basic_data:
        print(f"   資料類型: {basic_data['type']}")
        print(f"   時間戳: {basic_data['timestamp']}")
        print(f"   ROS2相容: {basic_data['ros2_compatible']}")
    
    # 3. 獲取詳細藥物資料
    print("\n📊 3. 獲取詳細藥物資料...")
    detailed_data = client.get_detailed_medicines()
    if detailed_data:
        print(f"   資料類型: {detailed_data['type']}")
        print(f"   時間戳: {detailed_data['timestamp']}")
        print(f"   ROS2相容: {detailed_data['ros2_compatible']}")
    
    # 4. 獲取病例資料
    print("\n🏥 4. 獲取病例資料...")
    prescription_data = client.get_prescriptions()
    if prescription_data:
        print(f"   資料類型: {prescription_data['type']}")
        print(f"   時間戳: {prescription_data['timestamp']}")
        print(f"   ROS2相容: {prescription_data['ros2_compatible']}")
    
    # 5. 獲取整合藥物資料
    print("\n🔗 5. 獲取整合藥物資料...")
    integrated_data = client.get_integrated_medicine("ROS2測試藥物")
    if integrated_data and integrated_data['status'] == 'success':
        print(f"   資料類型: {integrated_data['type']}")
        print(f"   時間戳: {integrated_data['timestamp']}")
        print(f"   ROS2相容: {integrated_data['ros2_compatible']}")
        
        # 顯示整合資料詳情
        if integrated_data['has_basic']:
            basic_info = integrated_data['basic_data']
            print(f"   基本資料: {basic_info['name']} - {basic_info['amount']}單位")
        
        if integrated_data['has_detailed']:
            detailed_info = integrated_data['detailed_data']
            print(f"   詳細資料: {detailed_info['description']}")
    
    print("\n✅ ROS2整合測試完成!")
    print("🤖 =================================================")

def demonstrate_ros2_usage():
    """展示ROS2使用範例"""
    print("\n💡 ROS2使用範例:")
    print("""
# 在ROS2節點中使用範例:

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MedicineManagerNode(Node):
    def __init__(self):
        super().__init__('medicine_manager')
        self.client = HospitalMedicineClient()
        
        # 創建定時器，每10秒檢查一次藥物庫存
        self.timer = self.create_timer(10.0, self.check_medicine_inventory)
        
        # 創建發布器，發布低庫存警告
        self.publisher = self.create_publisher(String, 'medicine_alerts', 10)
    
    def check_medicine_inventory(self):
        # 獲取基本藥物資料
        basic_data = self.client.get_basic_medicines()
        if basic_data and basic_data['status'] == 'success':
            for medicine in basic_data['data']:
                if medicine['amount'] < 10:  # 庫存少於10個
                    alert_msg = String()
                    alert_msg.data = f"警告: {medicine['name']} 庫存不足 ({medicine['amount']})"
                    self.publisher.publish(alert_msg)
                    self.get_logger().warn(alert_msg.data)

def main(args=None):
    rclpy.init(args=args)
    medicine_manager = MedicineManagerNode()
    rclpy.spin(medicine_manager)
    medicine_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    """)

if __name__ == "__main__":
    # 運行模擬測試
    simulate_ros2_node()
    
    # 展示使用範例
    demonstrate_ros2_usage()
    
    print("\n📚 詳細文檔:")
    print("   API文檔: http://localhost:8000/docs")
    print("   測試頁面: http://localhost:8000/simple_test.html")
    print("   整合管理: http://localhost:8000/medicine_integrated.html")
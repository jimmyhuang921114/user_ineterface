#!/usr/bin/env python3
"""
ROS2 客戶端示例 - 展示如何使用醫院服務
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
import yaml
import time
import threading

class HospitalClient(Node):
    """醫院 ROS2 客戶端"""
    
    def __init__(self):
        super().__init__('hospital_client')
        
        # 創建服務客戶端
        self.get_order_client = self.create_client(Empty, 'hospital/get_order')
        self.complete_order_client = self.create_client(Empty, 'hospital/complete_order')
        
        # 創建發布者
        self.medicine_request_pub = self.create_publisher(String, 'hospital/medicine_request', 10)
        
        # 創建訂閱者
        self.order_sub = self.create_subscription(
            String, 'hospital/order_data', self.order_callback, 10)
        self.medicine_sub = self.create_subscription(
            String, 'hospital/medicine_data', self.medicine_callback, 10)
        self.status_sub = self.create_subscription(
            String, 'hospital/status', self.status_callback, 10)
        
        # 狀態變數
        self.current_order = None
        self.is_processing = False
        
        self.get_logger().info("🤖 醫院客戶端已啟動")

    def wait_for_services(self):
        """等待服務可用"""
        self.get_logger().info("⏳ 等待服務可用...")
        
        while not self.get_order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ 等待 hospital/get_order 服務...")
            
        while not self.complete_order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ 等待 hospital/complete_order 服務...")
            
        self.get_logger().info("✅ 所有服務已可用")

    def get_new_order(self):
        """獲取新訂單"""
        try:
            self.get_logger().info("📋 請求新訂單...")
            
            request = Empty.Request()
            future = self.get_order_client.call_async(request)
            
            # 等待回應
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                self.get_logger().info("✅ 訂單請求已發送")
                return True
            else:
                self.get_logger().error("❌ 訂單請求失敗")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ 獲取訂單時發生錯誤: {e}")
            return False

    def query_medicine_detail(self, medicine_name: str):
        """查詢藥物詳細資訊"""
        try:
            self.get_logger().info(f"💊 查詢藥物詳細資訊: {medicine_name}")
            
            msg = String()
            msg.data = medicine_name
            self.medicine_request_pub.publish(msg)
            
            self.get_logger().info("✅ 藥物查詢請求已發送")
            
        except Exception as e:
            self.get_logger().error(f"❌ 查詢藥物時發生錯誤: {e}")

    def complete_current_order(self):
        """完成當前訂單"""
        try:
            if not self.current_order:
                self.get_logger().warn("⚠️ 沒有正在處理的訂單")
                return False
                
            self.get_logger().info(f"✅ 標記訂單完成: {self.current_order['order_id']}")
            
            request = Empty.Request()
            future = self.complete_order_client.call_async(request)
            
            # 等待回應
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                self.get_logger().info("✅ 訂單完成通知已發送")
                self.current_order = None
                self.is_processing = False
                return True
            else:
                self.get_logger().error("❌ 訂單完成通知失敗")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ 完成訂單時發生錯誤: {e}")
            return False

    def order_callback(self, msg):
        """處理訂單數據 (YAML 格式)"""
        try:
            # 解析 YAML 格式的訂單數據
            order_data = yaml.safe_load(msg.data)
            self.current_order = order_data
            self.is_processing = True
            
            self.get_logger().info("📋 收到新訂單!")
            self.get_logger().info(f"   訂單ID: {order_data['order_id']}")
            self.get_logger().info(f"   病患: {order_data.get('patient_name', 'N/A')}")
            
            medicines = order_data.get('medicine', [])
            self.get_logger().info(f"   藥物數量: {len(medicines)}")
            
            # 顯示原始 YAML 格式
            print("\n" + "="*60)
            print(f"📋 收到訂單 YAML 格式:")
            print("="*60)
            print(msg.data)
            print("="*60)
            
            # 顯示解析後的訂單詳細資訊
            print(f"📋 訂單 {order_data['order_id']} 詳細資訊:")
            print("="*60)
            
            for i, med in enumerate(medicines, 1):
                print(f"藥物 {i}:")
                print(f"  名稱: {med['name']}")
                print(f"  數量: {med['amount']}")
                print(f"  位置: {med['locate']}")
                print(f"  類型: {med['prompt']}")
                print()
            
            # 開始處理訂單
            self.process_order(order_data)
            
        except Exception as e:
            self.get_logger().error(f"❌ 處理訂單數據時發生錯誤: {e}")
            self.get_logger().error(f"   原始數據: {msg.data}")

    def medicine_callback(self, msg):
        """處理藥物詳細資訊 (YAML 格式)"""
        try:
            # 解析 YAML 格式的藥物數據
            medicine_data = yaml.safe_load(msg.data)
            
            if medicine_data.get('error'):
                self.get_logger().warn(f"⚠️ {medicine_data['error']}")
            else:
                self.get_logger().info(f"💊 收到藥物詳細資訊: {medicine_data['name']}")
                
                print("\n" + "="*40)
                print(f"💊 藥物詳細資訊 YAML 格式:")
                print("="*40)
                print(msg.data)
                print("="*40)
                
        except Exception as e:
            self.get_logger().error(f"❌ 處理藥物數據時發生錯誤: {e}")
            self.get_logger().error(f"   原始數據: {msg.data}")

    def status_callback(self, msg):
        """處理系統狀態 (YAML 格式)"""
        try:
            # 解析 YAML 格式的狀態數據
            status_data = yaml.safe_load(msg.data)
            
            status = status_data.get('status', '')
            order_id = status_data.get('order_id', '')
            
            if status == 'order_received':
                self.get_logger().info(f"📋 系統狀態: 訂單 {order_id} 已接收")
            elif status == 'order_completed':
                self.get_logger().info(f"✅ 系統狀態: 訂單 {order_id} 已完成")
            else:
                self.get_logger().info(f"ℹ️ 系統狀態: {status}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 處理狀態數據時發生錯誤: {e}")
            self.get_logger().error(f"   原始數據: {msg.data}")

    def process_order(self, order_data):
        """處理訂單（您的機器人邏輯）"""
        order_id = order_data['order_id']
        medicines = order_data.get('medicine', [])  # YAML 格式中是 'medicine' 不是 'medicines'
        
        self.get_logger().info(f"🤖 開始處理訂單: {order_id}")
        
        # 模擬處理每個藥物
        for i, medicine in enumerate(medicines, 1):
            self.get_logger().info(f"🔄 處理藥物 {i}/{len(medicines)}: {medicine['name']}")
            
            # 1. 導航到位置
            locate = medicine['locate']
            self.get_logger().info(f"🚀 導航到位置 [{locate[0]}, {locate[1]}]")
            time.sleep(1)  # 模擬移動時間
            
            # 2. 查詢藥物詳細資訊（如果需要）
            self.query_medicine_detail(medicine['name'])
            time.sleep(1)
            
            # 3. 抓取藥物
            prompt = medicine['prompt']
            amount = medicine['amount']
            self.get_logger().info(f"🤖 抓取 {amount} 個 {prompt} 類型的藥物")
            time.sleep(2)  # 模擬抓取時間
            
            # 4. 運送藥物
            self.get_logger().info("📦 運送藥物到分配點")
            time.sleep(1)  # 模擬運送時間
        
        self.get_logger().info(f"✅ 訂單 {order_id} 處理完成")
        
        # 通知系統訂單完成
        self.complete_current_order()

    def interactive_mode(self):
        """互動模式"""
        print("\n🤖 醫院 ROS2 客戶端 - 互動模式")
        print("="*50)
        print("可用指令:")
        print("  1 - 獲取新訂單")
        print("  2 - 查詢藥物詳細資訊")
        print("  3 - 完成當前訂單")
        print("  4 - 顯示當前訂單")
        print("  q - 退出")
        print("="*50)
        
        while True:
            try:
                cmd = input("\n請輸入指令: ").strip()
                
                if cmd == '1':
                    self.get_new_order()
                elif cmd == '2':
                    medicine_name = input("請輸入藥物名稱: ").strip()
                    if medicine_name:
                        self.query_medicine_detail(medicine_name)
                elif cmd == '3':
                    self.complete_current_order()
                elif cmd == '4':
                    if self.current_order:
                        print(f"\n當前訂單: {self.current_order['order_id']}")
                        print(f"處理狀態: {'處理中' if self.is_processing else '待處理'}")
                    else:
                        print("\n目前沒有訂單")
                elif cmd.lower() == 'q':
                    break
                else:
                    print("無效指令，請重新輸入")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"錯誤: {e}")


def main():
    """主函數"""
    rclpy.init()
    
    try:
        client = HospitalClient()
        
        # 等待服務可用
        client.wait_for_services()
        
        print("\n🤖 客戶端已準備就緒!")
        print("您可以選擇:")
        print("  1. 自動模式 - 自動獲取並處理訂單")
        print("  2. 互動模式 - 手動控制")
        
        mode = input("請選擇模式 (1/2): ").strip()
        
        if mode == '1':
            # 自動模式
            print("\n🔄 自動模式啟動，每 10 秒檢查新訂單...")
            
            def auto_check():
                while True:
                    try:
                        if not client.is_processing:
                            client.get_new_order()
                        time.sleep(10)
                    except Exception as e:
                        client.get_logger().error(f"自動檢查錯誤: {e}")
            
            auto_thread = threading.Thread(target=auto_check, daemon=True)
            auto_thread.start()
            
            rclpy.spin(client)
            
        elif mode == '2':
            # 互動模式
            spin_thread = threading.Thread(target=lambda: rclpy.spin(client), daemon=True)
            spin_thread.start()
            
            client.interactive_mode()
        else:
            print("無效選擇，使用互動模式")
            spin_thread = threading.Thread(target=lambda: rclpy.spin(client), daemon=True)
            spin_thread.start()
            
            client.interactive_mode()
            
    except KeyboardInterrupt:
        print("\n🛑 正在停止客戶端...")
    except Exception as e:
        print(f"❌ 客戶端發生錯誤: {e}")
    finally:
        try:
            client.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("✅ 客戶端已停止")


if __name__ == '__main__':
    main()
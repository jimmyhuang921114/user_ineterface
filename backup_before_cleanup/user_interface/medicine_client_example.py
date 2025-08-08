#!/usr/bin/env python3
"""
藥物詳細資料查詢客戶端示例
展示如何查詢藥物詳細資訊
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
import yaml
import time
import sys

class MedicineDetailClient(Node):
    """藥物詳細資料查詢客戶端"""
    
    def __init__(self):
        super().__init__('medicine_detail_client')
        
        # 創建發布者 - 發送查詢請求
        self.request_publisher = self.create_publisher(
            String, 'medicine/detail_request', 10)
        
        # 創建訂閱者 - 接收詳細資料
        self.response_subscriber = self.create_subscription(
            String, 'medicine/detail_response', self.response_callback, 10)
        
        # 創建服務客戶端
        self.service_client = self.create_client(Empty, 'medicine/get_detail')
        
        # 狀態變數
        self.last_response = None
        self.waiting_for_response = False
        
        self.get_logger().info("💊 藥物詳細資料客戶端已啟動")

    def query_medicine(self, medicine_name: str):
        """查詢藥物詳細資料 (Topic 方式)"""
        self.get_logger().info(f"🔍 查詢藥物: {medicine_name}")
        
        # 發送查詢請求
        msg = String()
        msg.data = medicine_name
        self.request_publisher.publish(msg)
        
        self.waiting_for_response = True
        self.last_response = None
        
        # 等待回應
        start_time = time.time()
        timeout = 10.0  # 10秒超時
        
        while self.waiting_for_response and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.last_response:
            return self.last_response
        else:
            self.get_logger().warn(f"⏰ 查詢 {medicine_name} 超時")
            return None

    def query_medicine_service(self, medicine_name: str):
        """查詢藥物詳細資料 (Service 方式)"""
        self.get_logger().info(f"🔧 服務查詢藥物: {medicine_name}")
        
        # 先用 Topic 設置查詢名稱
        msg = String()
        msg.data = medicine_name
        self.request_publisher.publish(msg)
        
        time.sleep(1)  # 等待設置完成
        
        # 等待服務可用
        if not self.service_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ 藥物詳細資料服務不可用")
            return None
        
        # 調用服務
        request = Empty.Request()
        future = self.service_client.call_async(request)
        
        # 等待服務回應
        start_time = time.time()
        timeout = 10.0
        self.waiting_for_response = True
        self.last_response = None
        
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if future.done():
            # 等待 Topic 回應
            while self.waiting_for_response and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                
            return self.last_response
        else:
            self.get_logger().warn(f"⏰ 服務查詢 {medicine_name} 超時")
            return None

    def response_callback(self, msg):
        """處理查詢回應"""
        try:
            # 解析 YAML 格式的回應
            response_data = yaml.safe_load(msg.data)
            
            medicine_name = response_data.get('name', 'Unknown')
            found = response_data.get('found', False)
            
            if found:
                self.get_logger().info(f"✅ 找到藥物: {medicine_name}")
                
                # 顯示詳細資訊
                print("\n" + "="*50)
                print(f"💊 藥物詳細資料: {medicine_name}")
                print("="*50)
                print("📄 YAML 格式:")
                print(msg.data)
                print("="*50)
                
                # 解析並顯示關鍵資訊
                description = response_data.get('description', 'N/A')
                category = response_data.get('category', 'N/A')
                unit_dose = response_data.get('unit_dose', 'N/A')
                stock_quantity = response_data.get('stock_quantity', 0)
                
                print(f"📝 描述: {description}")
                print(f"🏷️  分類: {category}")
                print(f"💊 劑量: {unit_dose}")
                print(f"📦 庫存: {stock_quantity}")
                print("="*50)
                
            else:
                error = response_data.get('error', '未知錯誤')
                self.get_logger().warn(f"❌ 藥物查詢失敗: {medicine_name} - {error}")
                print(f"\n❌ 未找到藥物: {medicine_name}")
                print(f"錯誤: {error}")
            
            self.last_response = response_data
            self.waiting_for_response = False
            
        except Exception as e:
            self.get_logger().error(f"❌ 處理回應時發生錯誤: {e}")
            self.get_logger().error(f"原始資料: {msg.data}")
            self.waiting_for_response = False

    def interactive_mode(self):
        """互動模式"""
        print("\n💊 藥物詳細資料查詢客戶端 - 互動模式")
        print("="*60)
        print("可用指令:")
        print("  1 - Topic 方式查詢藥物")
        print("  2 - Service 方式查詢藥物")
        print("  3 - 批量查詢常用藥物")
        print("  q - 退出")
        print("="*60)
        
        while True:
            try:
                cmd = input("\n請輸入指令: ").strip()
                
                if cmd == '1':
                    medicine_name = input("請輸入藥物名稱: ").strip()
                    if medicine_name:
                        result = self.query_medicine(medicine_name)
                        if not result:
                            print("❌ 查詢失敗或超時")
                            
                elif cmd == '2':
                    medicine_name = input("請輸入藥物名稱: ").strip()
                    if medicine_name:
                        result = self.query_medicine_service(medicine_name)
                        if not result:
                            print("❌ 服務查詢失敗或超時")
                            
                elif cmd == '3':
                    # 批量查詢常用藥物
                    common_medicines = [
                        "阿斯匹靈", "維他命C", "感冒藥", 
                        "止痛藥", "胃藥", "測試藥物"
                    ]
                    
                    print(f"\n🔍 批量查詢 {len(common_medicines)} 種常用藥物...")
                    for med in common_medicines:
                        print(f"\n查詢: {med}")
                        result = self.query_medicine(med)
                        if result and result.get('found'):
                            print(f"✅ {med}: {result.get('description', 'N/A')}")
                        else:
                            print(f"❌ {med}: 未找到")
                        time.sleep(0.5)  # 避免過於頻繁的請求
                        
                elif cmd.lower() == 'q':
                    break
                else:
                    print("❌ 無效指令，請重新輸入")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ 發生錯誤: {e}")

    def test_specific_medicines(self, medicine_names: list):
        """測試特定藥物列表"""
        print(f"\n🧪 測試 {len(medicine_names)} 種藥物...")
        
        results = []
        for name in medicine_names:
            print(f"\n🔍 查詢: {name}")
            result = self.query_medicine(name)
            results.append((name, result))
            time.sleep(0.5)
        
        # 總結結果
        print("\n📊 查詢結果總結:")
        print("="*50)
        found_count = 0
        for name, result in results:
            if result and result.get('found'):
                found_count += 1
                print(f"✅ {name}")
            else:
                print(f"❌ {name}")
        
        print(f"\n📈 成功率: {found_count}/{len(medicine_names)} ({100*found_count/len(medicine_names):.1f}%)")
        return results


def main():
    """主函數"""
    rclpy.init()
    
    try:
        client = MedicineDetailClient()
        
        print("💊 藥物詳細資料查詢客戶端已啟動")
        print("⏳ 等待服務可用...")
        
        # 等待一下讓系統初始化
        time.sleep(2)
        
        if len(sys.argv) > 1:
            # 命令行模式
            medicine_name = " ".join(sys.argv[1:])
            print(f"🔍 命令行查詢: {medicine_name}")
            
            result = client.query_medicine(medicine_name)
            if result:
                if result.get('found'):
                    print("✅ 查詢成功")
                else:
                    print("❌ 未找到藥物")
            else:
                print("❌ 查詢失敗")
                
        else:
            # 互動模式
            print("\n您可以選擇:")
            print("  1. 互動模式 - 手動輸入藥物名稱")
            print("  2. 測試模式 - 自動測試常用藥物")
            
            mode = input("請選擇模式 (1/2): ").strip()
            
            if mode == '2':
                # 測試模式
                test_medicines = [
                    "阿斯匹靈", "維他命C", "感冒藥", "測試藥物",
                    "不存在的藥物", "aspirin", "vitamin"
                ]
                client.test_specific_medicines(test_medicines)
            else:
                # 互動模式
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
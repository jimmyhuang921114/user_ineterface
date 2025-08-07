#!/usr/bin/env python3
"""
Web 系統與 ROS2 整合示例
展示如何將訂單推送器與您的 ROS2 系統整合
"""

import time
import threading
import logging
from ros2_order_pusher import OrderPusher

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("integration_example")

class YourROS2System:
    """
    這是您的 ROS2 系統模擬
    請將此類替換為您實際的 ROS2 節點
    """
    
    def __init__(self):
        self.robot_busy = False
        
    def navigate_to_location(self, locate):
        """導航到指定位置"""
        row, col = locate
        logger.info(f"🚀 導航到位置 [{row}, {col}]")
        time.sleep(2)  # 模擬移動時間
        
    def pick_medicine(self, name, amount, prompt):
        """抓取藥物"""
        logger.info(f"🤖 抓取藥物: {name} x{amount} (類型: {prompt})")
        
        # 根據不同類型調整抓取策略
        if prompt == 'tablet':
            logger.info("   使用精密夾爪抓取片劑")
        elif prompt == 'capsule':
            logger.info("   使用軟質夾爪抓取膠囊")
        elif prompt == 'white_circle_box':
            logger.info("   使用大夾爪抓取盒裝藥物")
        else:
            logger.info("   使用預設抓取模式")
            
        time.sleep(3)  # 模擬抓取時間
        
    def deliver_medicine(self):
        """運送藥物"""
        logger.info("📦 運送藥物到分配點")
        time.sleep(2)  # 模擬運送時間
        
    def process_order(self, order_dict, yaml_order):
        """處理訂單的主要函數"""
        if self.robot_busy:
            logger.warning("機器人忙碌中，無法處理新訂單")
            return
            
        self.robot_busy = True
        order_id = order_dict['order_id']
        medicines = order_dict['medicines']
        
        logger.info(f"📋 開始處理訂單: {order_id}")
        logger.info(f"📄 訂單內容:\n{yaml_order}")
        
        try:
            # 處理每個藥物
            for i, medicine in enumerate(medicines, 1):
                logger.info(f"處理藥物 {i}/{len(medicines)}")
                
                # 1. 導航到位置
                self.navigate_to_location(medicine['locate'])
                
                # 2. 抓取藥物
                self.pick_medicine(
                    medicine['name'], 
                    medicine['amount'], 
                    medicine['prompt']
                )
                
                # 3. 運送藥物
                self.deliver_medicine()
                
            logger.info(f"✅ 訂單 {order_id} 處理完成")
            
            # 重要：通知 Web 系統訂單已完成
            # 這會允許系統處理下一個訂單
            global order_pusher
            if order_pusher:
                success = order_pusher.complete_order(order_id)
                if success:
                    logger.info("✅ 已通知 Web 系統訂單完成")
                else:
                    logger.error("❌ 通知 Web 系統失敗")
                    
        except Exception as e:
            logger.error(f"❌ 處理訂單時發生錯誤: {e}")
            # 錯誤情況下也要重置狀態
            if order_pusher:
                order_pusher.complete_order(order_id)
                
        finally:
            self.robot_busy = False

def main():
    """主函數"""
    print("🏥 醫院藥物管理系統 - ROS2 整合示例")
    print("=" * 60)
    print("此示例展示如何將 Web 系統與您的 ROS2 系統整合")
    print("")
    
    # 建立您的 ROS2 系統實例
    ros2_system = YourROS2System()
    
    # 建立訂單推送器，傳入您的處理函數
    global order_pusher
    order_pusher = OrderPusher(
        fastapi_base_url="http://localhost:8001",
        callback_func=ros2_system.process_order
    )
    
    try:
        # 開始監控新訂單
        order_pusher.start_monitoring()
        
        print("✅ 系統已啟動，開始監控新處方籤...")
        print("📋 請到 Web 界面創建處方籤來測試功能:")
        print("   • 藥物管理: http://localhost:8001/integrated_medicine_management.html")
        print("   • 醫生工作台: http://localhost:8001/doctor.html")
        print("   • 處方籤管理: http://localhost:8001/Prescription.html")
        print("")
        print("🔄 系統特性:")
        print("   • 自動檢測新處方籤")
        print("   • 一次只處理一個訂單")
        print("   • 等待 ROS2 完成後才處理下一個")
        print("   • 自動更新處方籤狀態")
        print("")
        print("🛑 按 Ctrl+C 停止系統")
        
        # 主循環 - 顯示系統狀態
        while True:
            time.sleep(5)
            status = order_pusher.get_status()
            
            # 顯示狀態
            print(f"\r⏱️ 監控中... " + 
                  f"已處理: {status['processed_count']} | " +
                  f"ROS2狀態: {'忙碌' if status['ros2_busy'] else '空閒'} | " +
                  f"當前訂單: {status['current_order_id'] or '無'}", 
                  end="", flush=True)
                  
    except KeyboardInterrupt:
        print("\n\n🛑 正在停止系統...")
        order_pusher.stop_monitoring()
        print("✅ 系統已停止")

if __name__ == "__main__":
    main()
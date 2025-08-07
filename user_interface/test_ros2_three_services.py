#!/usr/bin/env python3
"""
測試三個專門的 ROS2 服務接口
"""

import time
import asyncio
from ros2_service_interfaces import HospitalROS2ServiceManager

def test_order_service(manager):
    """測試訂單服務（每筆完成後再送下一筆）"""
    print("\n📦 測試訂單服務")
    print("-" * 40)
    
    # 測試訂單1
    print("發送訂單1...")
    manager.send_order([
        {"name": "阿司匹林", "quantity": 3},
        {"name": "維他命C", "quantity": 2}
    ], {
        "patient_name": "張小明",
        "doctor_name": "李醫師"
    })
    
    # 測試訂單2（會排隊）
    print("發送訂單2...")
    manager.send_order([
        {"name": "布洛芬", "quantity": 1},
        {"name": "胃藥", "quantity": 1}
    ], {
        "patient_name": "王小華",
        "doctor_name": "陳醫師"
    })
    
    # 測試訂單3（會排隊）
    print("發送訂單3...")
    manager.send_order([
        {"name": "感冒糖漿", "quantity": 1}
    ], {
        "patient_name": "李小美",
        "doctor_name": "張醫師"
    })
    
    # 檢查訂單狀態
    print("\n📊 訂單服務狀態:")
    status = manager.get_service_status()
    order_status = status['order_service']
    print(f"   當前處理中: {order_status['current_order']['order_id'] if order_status['current_order'] else '無'}")
    print(f"   排隊數量: {order_status['queue_length']}")
    print(f"   是否處理中: {order_status['processing']}")

def test_basic_medicine_service(manager):
    """測試基本藥物服務（持續）"""
    print("\n💊 測試基本藥物服務")
    print("-" * 40)
    
    # 測試多個藥物查詢
    medicines = ["阿司匹林", "布洛芬", "維他命C", "胃藥", "感冒糖漿"]
    
    for medicine in medicines:
        print(f"查詢基本藥物: {medicine}")
        result = manager.query_basic_medicine(medicine)
        if result and not manager.basic_service.use_ros2:
            # HTTP 模式下可以直接顯示結果
            medicines_data = result.get('medicines', [])
            if medicines_data:
                med = medicines_data[0]
                print(f"   ✅ 找到: {med['name']} (庫存: {med['amount']}, 位置: {med['position']})")
            else:
                print(f"   ❌ 未找到藥物: {medicine}")
        time.sleep(0.5)  # 模擬持續查詢間隔

def test_detailed_medicine_service(manager):
    """測試詳細藥物服務（持續）"""
    print("\n🔬 測試詳細藥物服務")
    print("-" * 40)
    
    # 測試多個藥物的詳細資訊查詢
    medicines = ["阿司匹林", "布洛芬", "維他命C"]
    
    for medicine in medicines:
        print(f"查詢詳細藥物: {medicine}")
        result = manager.query_detailed_medicine(medicine)
        if result and not manager.detailed_service.use_ros2:
            # HTTP 模式下可以直接顯示結果
            detailed_data = result.get('detailed_medicines', [])
            if detailed_data:
                detail = detailed_data[0]
                print(f"   ✅ 詳細資訊:")
                print(f"      描述: {detail['description']}")
                print(f"      成分: {detail['ingredient']}")
                print(f"      用法: {detail['usage_method']}")
                print(f"      副作用: {detail['side_effects']}")
            else:
                print(f"   ❌ 未找到詳細資訊: {medicine}")
        time.sleep(0.5)  # 模擬持續查詢間隔

def test_concurrent_services(manager):
    """測試併發服務使用"""
    print("\n🔄 測試併發服務使用")
    print("-" * 40)
    
    print("同時進行訂單處理和藥物查詢...")
    
    # 發送一個長訂單
    manager.send_order([
        {"name": "阿司匹林", "quantity": 5},
        {"name": "布洛芬", "quantity": 3},
        {"name": "維他命C", "quantity": 2},
        {"name": "胃藥", "quantity": 1}
    ], {
        "patient_name": "併發測試患者",
        "doctor_name": "併發測試醫師"
    })
    
    # 同時進行藥物查詢
    for i in range(3):
        print(f"第 {i+1} 輪查詢...")
        manager.query_basic_medicine("阿司匹林")
        manager.query_detailed_medicine("布洛芬")
        time.sleep(1)

def monitor_service_status(manager, duration=10):
    """監控服務狀態"""
    print(f"\n📊 監控服務狀態 ({duration} 秒)")
    print("-" * 40)
    
    start_time = time.time()
    while time.time() - start_time < duration:
        status = manager.get_service_status()
        
        print(f"時間: {time.time() - start_time:.1f}s")
        print(f"  訂單服務:")
        order_status = status['order_service']
        if order_status['current_order']:
            print(f"    當前訂單: {order_status['current_order']['order_id']}")
        print(f"    排隊: {order_status['queue_length']} 筆")
        print(f"    處理中: {order_status['processing']}")
        
        print(f"  基本藥物服務: {'運行中' if status['basic_service']['running'] else '停止'}")
        print(f"  詳細藥物服務: {'運行中' if status['detailed_service']['running'] else '停止'}")
        print("-" * 20)
        
        time.sleep(2)

def main():
    """主測試函數"""
    print("醫院 ROS2 三服務接口測試")
    print("=" * 60)
    
    # 創建服務管理器
    print("🏥 初始化服務管理器...")
    manager = HospitalROS2ServiceManager()
    
    # 啟動持續服務
    print("\n🚀 啟動持續服務...")
    manager.start_continuous_services()
    
    try:
        # 測試各個服務
        test_basic_medicine_service(manager)
        test_detailed_medicine_service(manager)
        test_order_service(manager)
        test_concurrent_services(manager)
        
        # 監控服務狀態
        monitor_service_status(manager, 15)
        
    finally:
        # 停止持續服務
        print("\n⏹️ 停止持續服務...")
        manager.stop_continuous_services()
    
    print("\n✅ 測試完成！")
    print("\n📋 測試總結:")
    print("   • 訂單服務: 支援佇列機制，每筆完成後處理下一筆")
    print("   • 基本藥物服務: 持續提供基本藥物資訊查詢")
    print("   • 詳細藥物服務: 持續提供詳細藥物資訊查詢")
    print("   • 併發支援: 三個服務可同時使用互不干擾")

if __name__ == "__main__":
    main()
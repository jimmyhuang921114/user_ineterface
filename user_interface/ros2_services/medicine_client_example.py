#!/usr/bin/env python3
"""
ROS2 Medicine Service Client Example
ROS2 藥物服務客戶端示例
"""

import rclpy
from rclpy.node import Node
from rclpy.client import Client
import time

class MedicineClientExample(Node):
    def __init__(self):
        super().__init__('medicine_client_example')
        
        # 創建服務客戶端
        self.basic_medicine_client = self.create_client(
            dict,  # 臨時使用 dict，實際應該是 GetBasicMedicine
            'get_basic_medicine'
        )
        
        self.detailed_medicine_client = self.create_client(
            dict,  # 臨時使用 dict，實際應該是 GetDetailedMedicine
            'get_detailed_medicine'
        )
        
        self.get_logger().info('藥物服務客戶端已啟動')

    def wait_for_services(self):
        """等待服務可用"""
        self.get_logger().info('等待藥物服務...')
        
        while not self.basic_medicine_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('基本藥物服務不可用，等待中...')
        
        while not self.detailed_medicine_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('詳細藥物服務不可用，等待中...')
        
        self.get_logger().info('所有服務已就緒!')

    async def get_all_basic_medicines(self):
        """獲取所有基本藥物"""
        self.get_logger().info('請求所有基本藥物資訊...')
        
        # 創建請求 (臨時用字典，實際應該是 GetBasicMedicine.Request())
        request = type('Request', (), {
            'medicine_name': '',
            'medicine_id': 0,
            'get_all': True
        })()
        
        try:
            future = self.basic_medicine_client.call_async(request)
            response = await future
            
            if response.success:
                self.get_logger().info(f'✅ {response.message}')
                for i, medicine in enumerate(response.medicines, 1):
                    self.get_logger().info(f'  {i}. {medicine.name} (ID: {medicine.id})')
                    self.get_logger().info(f'     庫存: {medicine.amount} | 位置: {medicine.position}')
                    self.get_logger().info(f'     製造商: {medicine.manufacturer} | 劑量: {medicine.dosage}')
            else:
                self.get_logger().error(f'❌ {response.message}')
                
        except Exception as e:
            self.get_logger().error(f'請求失敗: {str(e)}')

    async def get_medicine_by_name(self, medicine_name: str):
        """按名稱獲取藥物"""
        self.get_logger().info(f'按名稱搜尋藥物: {medicine_name}')
        
        # 基本藥物查詢
        request = type('Request', (), {
            'medicine_name': medicine_name,
            'medicine_id': 0,
            'get_all': False
        })()
        
        try:
            future = self.basic_medicine_client.call_async(request)
            response = await future
            
            if response.success and response.medicines:
                medicine = response.medicines[0]
                self.get_logger().info(f'✅ 找到基本藥物: {medicine.name}')
                self.get_logger().info(f'   ID: {medicine.id} | 庫存: {medicine.amount}')
                
                # 同時獲取詳細資訊
                await self.get_detailed_medicine_by_name(medicine_name)
            else:
                self.get_logger().warning(f'未找到藥物: {medicine_name}')
                
        except Exception as e:
            self.get_logger().error(f'查詢失敗: {str(e)}')

    async def get_detailed_medicine_by_name(self, medicine_name: str):
        """按名稱獲取詳細藥物資訊"""
        self.get_logger().info(f'獲取詳細藥物資訊: {medicine_name}')
        
        request = type('Request', (), {
            'medicine_name': medicine_name,
            'medicine_id': 0,
            'get_all': False,
            'include_basic': True
        })()
        
        try:
            future = self.detailed_medicine_client.call_async(request)
            response = await future
            
            if response.success and response.detailed_medicines:
                detailed = response.detailed_medicines[0]
                basic = response.basic_medicines[0] if response.basic_medicines else None
                
                self.get_logger().info(f'✅ 詳細藥物資訊:')
                self.get_logger().info(f'   藥物ID: {detailed.medicine_id}')
                self.get_logger().info(f'   描述: {detailed.description}')
                self.get_logger().info(f'   成分: {detailed.ingredient}')
                self.get_logger().info(f'   分類: {detailed.category}')
                self.get_logger().info(f'   用法: {detailed.usage_method}')
                self.get_logger().info(f'   副作用: {detailed.side_effects}')
                
                if basic:
                    self.get_logger().info(f'   當前庫存: {basic.amount}')
                    self.get_logger().info(f'   儲存位置: {basic.position}')
            else:
                self.get_logger().warning(f'未找到詳細資訊: {medicine_name}')
                
        except Exception as e:
            self.get_logger().error(f'查詢詳細資訊失敗: {str(e)}')

    async def get_medicine_by_id(self, medicine_id: int):
        """按 ID 獲取藥物"""
        self.get_logger().info(f'按 ID 查詢藥物: {medicine_id}')
        
        # 基本資訊查詢
        basic_request = type('Request', (), {
            'medicine_name': '',
            'medicine_id': medicine_id,
            'get_all': False
        })()
        
        # 詳細資訊查詢
        detailed_request = type('Request', (), {
            'medicine_name': '',
            'medicine_id': medicine_id,
            'get_all': False,
            'include_basic': True
        })()
        
        try:
            # 同時發送兩個請求
            basic_future = self.basic_medicine_client.call_async(basic_request)
            detailed_future = self.detailed_medicine_client.call_async(detailed_request)
            
            basic_response = await basic_future
            detailed_response = await detailed_future
            
            if basic_response.success and basic_response.medicines:
                basic = basic_response.medicines[0]
                self.get_logger().info(f'✅ 基本資訊: {basic.name} (ID: {basic.id})')
                self.get_logger().info(f'   庫存: {basic.amount} | 位置: {basic.position}')
            
            if detailed_response.success and detailed_response.detailed_medicines:
                detailed = detailed_response.detailed_medicines[0]
                self.get_logger().info(f'✅ 詳細資訊:')
                self.get_logger().info(f'   成分: {detailed.ingredient}')
                self.get_logger().info(f'   用法: {detailed.usage_method}')
                self.get_logger().info(f'   副作用: {detailed.side_effects}')
            
            if not basic_response.success and not detailed_response.success:
                self.get_logger().error(f'❌ 未找到 ID 為 {medicine_id} 的藥物')
                
        except Exception as e:
            self.get_logger().error(f'查詢失敗: {str(e)}')

async def run_examples():
    """運行示例"""
    rclpy.init()
    
    client = MedicineClientExample()
    
    try:
        # 等待服務
        client.wait_for_services()
        
        # 示例 1: 獲取所有基本藥物
        print("\n" + "="*50)
        print("示例 1: 獲取所有基本藥物")
        print("="*50)
        await client.get_all_basic_medicines()
        
        # 示例 2: 按名稱搜尋
        print("\n" + "="*50)
        print("示例 2: 按名稱搜尋藥物")
        print("="*50)
        await client.get_medicine_by_name("普拿疼")
        
        # 示例 3: 按 ID 查詢
        print("\n" + "="*50)
        print("示例 3: 按 ID 查詢藥物")
        print("="*50)
        await client.get_medicine_by_id(3)
        
        print("\n✅ 所有示例執行完成!")
        
    except KeyboardInterrupt:
        client.get_logger().info('收到停止信號...')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(run_examples())
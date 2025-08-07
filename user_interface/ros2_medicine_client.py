#!/usr/bin/env python3
"""
ROS2 醫院藥物管理客戶端包
提供完整的 ROS2 service 介面，可獨立使用

使用方式:
    from ros2_medicine_client import MedicineROS2Client
    
    client = MedicineROS2Client()
    await client.initialize()
    
    # 查詢藥物
    result = await client.query_medicine("阿司匹林")
    
    # 處理訂單
    orders = await client.get_pending_orders()
    await client.execute_order(1)
"""

import asyncio
import json
import logging
from typing import Dict, List, Optional, Any
from datetime import datetime
import time # Added for time.sleep

# 嘗試導入 ROS2
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("警告: ROS2 不可用，將使用 HTTP 模式")

# HTTP 客戶端 (備用模式)
try:
    import requests
    HTTP_AVAILABLE = True
except ImportError:
    HTTP_AVAILABLE = False
    print("警告: requests 不可用，ROS2 模式將無法回退到 HTTP")

class MedicineROS2Client:
    """
    醫院藥物管理 ROS2 客戶端
    
    提供統一的介面來與醫院藥物管理系統互動，
    支援 ROS2 和 HTTP 兩種模式
    """
    
    def __init__(self, 
                 base_url: str = "http://localhost:8001",
                 node_name: str = "medicine_client",
                 use_ros2: bool = True):
        """
        初始化客戶端
        
        Args:
            base_url: HTTP API 基礎 URL
            node_name: ROS2 節點名稱
            use_ros2: 是否優先使用 ROS2
        """
        self.base_url = base_url
        self.node_name = node_name
        self.use_ros2 = use_ros2 and ROS2_AVAILABLE
        self.use_http = HTTP_AVAILABLE
        
        # ROS2 相關
        self.node = None
        self.medicine_publisher = None
        self.order_publisher = None
        self.response_subscriber = None
        self.status_publisher = None # Added for new services
        
        # 設置日誌
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # 響應回調存儲
        self._pending_requests = {}
        
    async def initialize(self):
        """初始化客戶端"""
        if self.use_ros2:
            try:
                await self._initialize_ros2()
                self.logger.info("ROS2 客戶端初始化成功")
            except Exception as e:
                self.logger.warning(f"ROS2 初始化失敗: {e}")
                self.use_ros2 = False
        
        if not self.use_ros2 and not self.use_http:
            raise RuntimeError("無可用的通訊方式 (ROS2 或 HTTP)")
        
        if not self.use_ros2:
            self.logger.info("使用 HTTP 模式")
    
    async def _initialize_ros2(self):
        """初始化 ROS2 節點"""
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 不可用")
        
        rclpy.init()
        self.node = Node(self.node_name)
        
        # 創建發布器
        self.medicine_publisher = self.node.create_publisher(
            String, 'hospital/medicine_query', 10
        )
        self.order_publisher = self.node.create_publisher(
            String, 'hospital/order_request', 10
        )
        self.status_publisher = self.node.create_publisher( # Initialize status publisher
            String, 'hospital/status_request', 10
        )
        
        # 創建訂閱器接收響應
        self.response_subscriber = self.node.create_subscription(
            String, 'hospital/response', self._response_callback, 10
        )
        
        # 在背景執行 ROS2 spin
        asyncio.create_task(self._ros2_spin())
    
    async def _ros2_spin(self):
        """在背景執行 ROS2 spin"""
        while rclpy.ok() and self.use_ros2:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            await asyncio.sleep(0.01)
    
    def _response_callback(self, msg):
        """處理 ROS2 響應"""
        try:
            data = json.loads(msg.data)
            request_id = data.get('request_id')
            if request_id in self._pending_requests:
                future = self._pending_requests.pop(request_id)
                future.set_result(data)
        except Exception as e:
            self.logger.error(f"響應處理錯誤: {e}")
    
    def _generate_request_id(self) -> str:
        """生成請求 ID"""
        return f"{self.node_name}_{datetime.now().timestamp()}"
    
    async def _send_ros2_request(self, topic_publisher, data: Dict) -> Dict:
        """發送 ROS2 請求並等待響應"""
        request_id = self._generate_request_id()
        data['request_id'] = request_id
        
        # 創建 Future 來等待響應
        future = asyncio.Future()
        self._pending_requests[request_id] = future
        
        # 發送請求
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        topic_publisher.publish(msg)
        
        # 等待響應 (最多 10 秒)
        try:
            result = await asyncio.wait_for(future, timeout=10.0)
            return result
        except asyncio.TimeoutError:
            self._pending_requests.pop(request_id, None)
            raise TimeoutError("ROS2 請求超時")
    
    async def _send_http_request(self, method: str, endpoint: str, data: Dict = None) -> Dict:
        """發送 HTTP 請求"""
        if not self.use_http:
            raise RuntimeError("HTTP 不可用")
        
        url = f"{self.base_url}{endpoint}"
        
        try:
            if method.upper() == "GET":
                response = requests.get(url, timeout=10)
            elif method.upper() == "POST":
                response = requests.post(url, json=data, timeout=10)
            elif method.upper() == "PUT":
                response = requests.put(url, json=data, timeout=10)
            else:
                raise ValueError(f"不支援的 HTTP 方法: {method}")
            
            response.raise_for_status()
            return response.json()
        
        except requests.RequestException as e:
            self.logger.error(f"HTTP 請求失敗: {e}")
            raise
    
    async def _make_request(self, ros2_data: Dict = None, http_method: str = "GET", 
                          http_endpoint: str = "", http_data: Dict = None) -> Dict:
        """統一的請求方法，自動選擇 ROS2 或 HTTP"""
        if self.use_ros2 and ros2_data:
            try:
                return await self._send_ros2_request(self.medicine_publisher, ros2_data)
            except Exception as e:
                self.logger.warning(f"ROS2 請求失敗，回退到 HTTP: {e}")
                self.use_ros2 = False
        
        if self.use_http:
            return await self._send_http_request(http_method, http_endpoint, http_data)
        
        raise RuntimeError("無可用的通訊方式")
    
    # =============================================================================
    # 公共 API 方法
    # =============================================================================
    
    async def query_medicine(self, medicine_name: str, include_detailed: bool = True) -> Dict:
        """
        查詢藥物資訊
        
        Args:
            medicine_name: 藥物名稱
            include_detailed: 是否包含詳細資訊
            
        Returns:
            藥物資訊字典
        """
        ros2_data = {
            "action": "query_medicine",
            "medicine_name": medicine_name,
            "include_detailed": include_detailed
        }
        
        http_data = {
            "medicine_name": medicine_name,
            "include_detailed": include_detailed
        }
        
        return await self._make_request(
            ros2_data=ros2_data,
            http_method="POST",
            http_endpoint="/api/ros2/query-medicine",
            http_data=http_data
        )
    
    async def query_medicine_by_id(self, medicine_id: int, include_detailed: bool = True) -> Dict:
        """
        根據 ID 查詢藥物資訊
        
        Args:
            medicine_id: 藥物 ID
            include_detailed: 是否包含詳細資訊
            
        Returns:
            藥物資訊字典
        """
        ros2_data = {
            "action": "query_medicine",
            "medicine_id": medicine_id,
            "include_detailed": include_detailed
        }
        
        http_data = {
            "medicine_id": medicine_id,
            "include_detailed": include_detailed
        }
        
        return await self._make_request(
            ros2_data=ros2_data,
            http_method="POST",
            http_endpoint="/api/ros2/query-medicine",
            http_data=http_data
        )
    
    async def batch_query_medicines(self, medicine_names: List[str] = None, 
                                  medicine_ids: List[int] = None,
                                  include_detailed: bool = True) -> Dict:
        """
        批量查詢藥物資訊
        
        Args:
            medicine_names: 藥物名稱列表
            medicine_ids: 藥物 ID 列表
            include_detailed: 是否包含詳細資訊
            
        Returns:
            批量查詢結果
        """
        ros2_data = {
            "action": "batch_query_medicines",
            "medicine_names": medicine_names,
            "medicine_ids": medicine_ids,
            "include_detailed": include_detailed
        }
        
        http_data = {
            "medicine_names": medicine_names,
            "medicine_ids": medicine_ids,
            "include_detailed": include_detailed
        }
        
        return await self._make_request(
            ros2_data=ros2_data,
            http_method="POST",
            http_endpoint="/api/ros2/batch-query-medicines",
            http_data=http_data
        )
    
    async def get_pending_orders(self) -> Dict:
        """
        獲取待處理訂單
        
        Returns:
            待處理訂單列表
        """
        ros2_data = {
            "action": "get_pending_orders"
        }
        
        return await self._make_request(
            ros2_data=ros2_data,
            http_method="GET",
            http_endpoint="/api/ros2/pending-orders"
        )
    
    async def request_order_confirmation(self, prescription_id: int) -> Dict:
        """
        請求訂單確認
        
        Args:
            prescription_id: 處方籤 ID
            
        Returns:
            確認結果
        """
        ros2_data = {
            "action": "request_order_confirmation",
            "prescription_id": prescription_id
        }
        
        http_data = {
            "prescription_id": prescription_id
        }
        
        return await self._make_request(
            ros2_data=ros2_data,
            http_method="POST",
            http_endpoint="/api/ros2/request-order-confirmation",
            http_data=http_data
        )
    
    async def execute_order(self, prescription_id: int) -> Dict:
        """
        執行訂單
        
        Args:
            prescription_id: 處方籤 ID
            
        Returns:
            執行結果
        """
        ros2_data = {
            "action": "execute_order",
            "prescription_id": prescription_id
        }
        
        http_data = {
            "prescription_id": prescription_id
        }
        
        return await self._make_request(
            ros2_data=ros2_data,
            http_method="POST",
            http_endpoint="/api/ros2/confirm-and-execute-order",
            http_data=http_data
        )
    
    async def complete_order(self, prescription_id: int) -> Dict:
        """
        完成訂單
        
        Args:
            prescription_id: 處方籤 ID
            
        Returns:
            完成結果
        """
        ros2_data = {
            "action": "complete_order",
            "prescription_id": prescription_id
        }
        
        http_data = {
            "prescription_id": prescription_id
        }
        
        return await self._make_request(
            ros2_data=ros2_data,
            http_method="POST",
            http_endpoint="/api/ros2/complete-order",
            http_data=http_data
        )
    
    def get_system_status(self):
        """獲取系統狀態"""
        try:
            response = requests.get(f"{self.base_url}/api/system/status", timeout=5)
            return response.json() if response.status_code == 200 else None
        except Exception as e:
            print(f"獲取系統狀態失敗: {e}")
            return None

    def get_pending_orders(self):
        """獲取待處理訂單"""
        try:
            response = requests.get(f"{self.base_url}/api/ros2/pending-orders", timeout=5)
            return response.json() if response.status_code == 200 else None
        except Exception as e:
            print(f"獲取待處理訂單失敗: {e}")
            return None

    # 新增：分離的基本藥物服務
    def query_basic_medicine(self, medicine_name=None, medicine_id=None, get_all=False):
        """查詢基本藥物資訊 (分離服務)"""
        try:
            query_data = {}
            if medicine_name:
                query_data["medicine_name"] = medicine_name
            elif medicine_id:
                query_data["medicine_id"] = medicine_id
            elif get_all:
                query_data = {}  # 空查詢獲取所有
            
            if self.use_ros2 and ROS2_AVAILABLE: # Changed from self.ros2_available to ROS2_AVAILABLE
                # ROS2 方式 - 發布查詢請求
                msg = String()
                msg.data = json.dumps({
                    "type": "basic_medicine_request",
                    "query": query_data
                })
                self.medicine_publisher.publish(msg)
                print(f"🤖 [ROS2] 基本藥物查詢請求已發送: {query_data}")
                time.sleep(0.1)  # 等待響應
                return {"status": "sent", "mode": "ROS2"}
            else:
                # HTTP 方式
                response = requests.post(
                    f"{self.base_url}/api/ros2/service/basic-medicine",
                    json=query_data,
                    timeout=10
                )
                
                if response.status_code == 200:
                    result = response.json()
                    print(f"🌐 [HTTP] 基本藥物查詢成功: {result['message']}")
                    return result
                else:
                    print(f"❌ [HTTP] 基本藥物查詢失敗: {response.status_code}")
                    return None
                    
        except Exception as e:
            print(f"基本藥物查詢失敗: {e}")
            return None

    # 新增：分離的詳細藥物服務
    def query_detailed_medicine(self, medicine_name=None, medicine_id=None, get_all=False, include_basic=False):
        """查詢詳細藥物資訊 (分離服務)"""
        try:
            query_data = {"include_detailed": include_basic}
            if medicine_name:
                query_data["medicine_name"] = medicine_name
            elif medicine_id:
                query_data["medicine_id"] = medicine_id
            elif get_all:
                pass  # 空查詢獲取所有
            
            if self.use_ros2 and ROS2_AVAILABLE: # Changed from self.ros2_available to ROS2_AVAILABLE
                # ROS2 方式 - 發布查詢請求
                msg = String()
                msg.data = json.dumps({
                    "type": "detailed_medicine_request",
                    "query": query_data
                })
                self.medicine_publisher.publish(msg)
                print(f"🤖 [ROS2] 詳細藥物查詢請求已發送: {query_data}")
                time.sleep(0.1)  # 等待響應
                return {"status": "sent", "mode": "ROS2"}
            else:
                # HTTP 方式
                response = requests.post(
                    f"{self.base_url}/api/ros2/service/detailed-medicine",
                    json=query_data,
                    timeout=10
                )
                
                if response.status_code == 200:
                    result = response.json()
                    print(f"🌐 [HTTP] 詳細藥物查詢成功: {result['message']}")
                    return result
                else:
                    print(f"❌ [HTTP] 詳細藥物查詢失敗: {response.status_code}")
                    return None
                    
        except Exception as e:
            print(f"詳細藥物查詢失敗: {e}")
            return None

    # 新增：ROS2 服務狀態檢查
    def check_ros2_service_status(self):
        """檢查 ROS2 服務狀態"""
        try:
            response = requests.get(f"{self.base_url}/api/ros2/service/status", timeout=5)
            if response.status_code == 200:
                result = response.json()
                print(f"🔍 ROS2 服務狀態: {result['message']}")
                print(f"   - ROS2 可用: {result['ros2_available']}")
                print(f"   - 節點活躍: {result['node_active']}")
                return result
            else:
                print(f"❌ 服務狀態檢查失敗: {response.status_code}")
                return None
        except Exception as e:
            print(f"服務狀態檢查錯誤: {e}")
            return None
    
    # =============================================================================
    # 高級工作流程方法
    # =============================================================================
    
    async def full_order_workflow(self, prescription_id: int) -> Dict:
        """
        完整的訂單處理工作流程
        
        Args:
            prescription_id: 處方籤 ID
            
        Returns:
            工作流程結果
        """
        try:
            # 1. 請求確認
            self.logger.info(f"開始處理訂單 {prescription_id}")
            confirm_result = await self.request_order_confirmation(prescription_id)
            
            if not confirm_result.get('success', True):
                return {"success": False, "message": "訂單確認失敗", "details": confirm_result}
            
            # 2. 執行訂單
            execute_result = await self.execute_order(prescription_id)
            
            if not execute_result.get('success', True):
                return {"success": False, "message": "訂單執行失敗", "details": execute_result}
            
            # 3. 模擬處理時間
            await asyncio.sleep(2)
            
            # 4. 完成訂單
            complete_result = await self.complete_order(prescription_id)
            
            self.logger.info(f"訂單 {prescription_id} 處理完成")
            
            return {
                "success": True,
                "message": "訂單處理完成",
                "prescription_id": prescription_id,
                "details": {
                    "confirm": confirm_result,
                    "execute": execute_result,
                    "complete": complete_result
                }
            }
            
        except Exception as e:
            self.logger.error(f"訂單工作流程失敗: {e}")
            return {"success": False, "message": f"工作流程錯誤: {str(e)}"}
    
    async def process_all_pending_orders(self) -> List[Dict]:
        """
        處理所有待處理訂單
        
        Returns:
            處理結果列表
        """
        try:
            # 獲取待處理訂單
            orders_result = await self.get_pending_orders()
            orders = orders_result.get('orders', [])
            
            if not orders:
                return [{"message": "沒有待處理訂單"}]
            
            results = []
            for order in orders:
                prescription_id = order.get('prescription_id')
                if prescription_id:
                    result = await self.full_order_workflow(prescription_id)
                    results.append(result)
            
            return results
            
        except Exception as e:
            self.logger.error(f"批量處理訂單失敗: {e}")
            return [{"success": False, "message": f"批量處理錯誤: {str(e)}"}]
    
    # =============================================================================
    # 清理方法
    # =============================================================================
    
    async def shutdown(self):
        """關閉客戶端"""
        if self.use_ros2 and self.node:
            self.node.destroy_node()
            rclpy.shutdown()
        
        self.logger.info("客戶端已關閉")


# =============================================================================
# 使用範例和測試
# =============================================================================

def example_usage():
    """示例使用方法"""
    print("醫院藥物管理系統 - ROS2 客戶端示例")
    print("=" * 60)
    
    # 創建客戶端實例
    client = MedicineROS2Client(use_ros2=False)  # 使用 HTTP 模式進行示例
    
    print("\n🔧 1. 檢查系統狀態")
    system_status = client.get_system_status()
    if system_status:
        print(f"✅ 系統狀態: {system_status.get('message', 'Unknown')}")
    
    print("\n🔧 2. 檢查 ROS2 服務狀態")
    ros2_status = client.check_ros2_service_status()
    if ros2_status:
        print(f"✅ ROS2 服務: {ros2_status.get('message', 'Unknown')}")
    
    print("\n🧪 3. 測試分離的基本藥物服務")
    
    # 3.1 獲取所有基本藥物
    print("   3.1 獲取所有基本藥物")
    basic_all = client.query_basic_medicine(get_all=True)
    if basic_all and basic_all.get('success'):
        medicines = basic_all.get('medicines', [])
        print(f"   ✅ 找到 {len(medicines)} 種基本藥物")
        for med in medicines[:2]:  # 顯示前兩個
            print(f"      • {med['name']} (庫存: {med['amount']}, 位置: {med['position']})")
    
    # 3.2 按名稱查詢基本藥物
    print("   3.2 按名稱查詢基本藥物")
    basic_aspirin = client.query_basic_medicine(medicine_name="阿司匹林")
    if basic_aspirin and basic_aspirin.get('success'):
        medicines = basic_aspirin.get('medicines', [])
        if medicines:
            med = medicines[0]
            print(f"   ✅ 找到: {med['name']} (製造商: {med['manufacturer']}, 劑量: {med['dosage']})")
    
    print("\n🔬 4. 測試分離的詳細藥物服務")
    
    # 4.1 獲取所有詳細藥物
    print("   4.1 獲取所有詳細藥物")
    detailed_all = client.query_detailed_medicine(get_all=True)
    if detailed_all and detailed_all.get('success'):
        detailed_meds = detailed_all.get('detailed_medicines', [])
        print(f"   ✅ 找到 {len(detailed_meds)} 種詳細藥物")
        if detailed_meds:
            detail = detailed_meds[0]
            print(f"      • {detail['description']} (成分: {detail['ingredient']})")
    
    # 4.2 按名稱查詢詳細藥物
    print("   4.2 按名稱查詢詳細藥物")
    detailed_ibuprofen = client.query_detailed_medicine(medicine_name="布洛芬")
    if detailed_ibuprofen and detailed_ibuprofen.get('success'):
        detailed_meds = detailed_ibuprofen.get('detailed_medicines', [])
        if detailed_meds:
            detail = detailed_meds[0]
            print(f"   ✅ 詳細資訊:")
            print(f"      - 描述: {detail['description']}")
            print(f"      - 用法: {detail['usage_method']}")
            print(f"      - 劑量: {detail['unit_dose']} mg")
            print(f"      - 副作用: {detail['side_effects']}")
    
    # 4.3 包含基本資訊的詳細查詢
    print("   4.3 包含基本資訊的詳細查詢")
    detailed_with_basic = client.query_detailed_medicine(medicine_name="維他命C", include_basic=True)
    if detailed_with_basic and detailed_with_basic.get('success'):
        detailed_count = len(detailed_with_basic.get('detailed_medicines', []))
        basic_count = len(detailed_with_basic.get('basic_medicines', []))
        print(f"   ✅ 查詢成功: {detailed_count} 詳細 + {basic_count} 基本")
    
    print("\n🚀 5. 比較服務差異")
    medicine_name = "胃藥"
    
    # 基本服務
    print(f"   5.1 查詢 '{medicine_name}' 基本資訊")
    basic_result = client.query_basic_medicine(medicine_name=medicine_name)
    if basic_result and basic_result.get('success') and basic_result.get('medicines'):
        med = basic_result['medicines'][0]
        print(f"   📋 基本: {med['name']} - {med['dosage']} (庫存: {med['amount']})")
    
    # 詳細服務
    print(f"   5.2 查詢 '{medicine_name}' 詳細資訊")
    detailed_result = client.query_detailed_medicine(medicine_name=medicine_name)
    if detailed_result and detailed_result.get('success') and detailed_result.get('detailed_medicines'):
        detail = detailed_result['detailed_medicines'][0]
        print(f"   🔬 詳細: {detail['description']}")
        print(f"       成分: {detail['ingredient']}")
        print(f"       類別: {detail['category']}")
    
    print("\n💡 使用建議:")
    print("   • query_basic_medicine(): 快速獲取庫存、位置等基本資訊")
    print("   • query_detailed_medicine(): 獲取成分、用法、副作用等詳細資訊")
    print("   • 根據使用場景選擇合適的服務，提高效率")
    print("   • ROS2 節點可以分別訂閱這兩個服務的響應")
    
    print("\n🎯 6. 原有功能（訂單管理）依然可用")
    
    # 原有的訂單功能示例
    pending_orders = client.get_pending_orders()
    if pending_orders:
        print(f"   📋 待處理訂單: {len(pending_orders.get('orders', []))} 個")
    
    print("\n✅ 示例完成！")
    print("現在您可以使用分離的基本和詳細藥物 ROS2 服務了。")


if __name__ == "__main__":
    """直接執行時運行範例"""
    example_usage()
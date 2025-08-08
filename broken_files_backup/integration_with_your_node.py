#!/usr/bin/env python3
"""
Integration adapter for your OrderHandlerNode with our automated push system
This adapter will:
1. Start our Web server and pusher
2. Provide HTTP API for your node to pull orders
3. Receive progress and completion status from your node
"""

import subprocess
import sys
import time
import signal
import threading
import os
import json
from typing import List, Dict, Any
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
import uvicorn

 # Import our system
from ros_order_pusher import OrderPusher
from simple_server_final import app as main_app
import yaml


class YourNodeIntegration:
# """整合您的 ROS 節點"""
    
    def __init__(self):
        self.processes: List[subprocess.Popen] = []
        self.order_pusher = None
        self.shutdown_event = threading.Event()
        
         # Create adapter API
        self.adapter_app = FastAPI(title="ROS Node Adapter API")
        self.setup_adapter_routes()
        
         # Store current order processing status
        self.current_orders = {}  # order_id -> status info
        
    def setup_adapter_routes(self):
        """Setup adapter API routes"""
        
        @self.adapter_app.get("/api/order/next")
        async def get_next_order():
# """您的節點調用此 API 來拉取下一個訂單"""
            try:
                if not self.order_pusher:
                    raise HTTPException(status_code=503, detail="Order pusher not ready")
                
                 # Check for new pending orders
                import requests
                response = requests.get("http://localhost:/api/prescription/")
                prescriptions = response.json()
# 找到最舊的 pending 處方籤
                pending = [p for p in prescriptions if p.get('status') == 'pending']
                if not pending:
                    return JSONResponse(status_code=204, content={})  # No content
# 取最舊的一個
                oldest = min(pending, key=lambda x: x.get('id', 0))
# 轉換為您的節點需要的格式
                order = await self.convert_prescription_to_order(oldest)
# 標記為 processing
                requests.put(
                    f"http://localhost:/api/prescription/{oldest['id']}/status",
                    json={"status": "processing"}
                )
# 記錄狀態
                self.current_orders[order['order_id']] = {
                    'prescription_id': oldest['id'],
                    'status': 'processing',
                    'start_time': time.time()
                }
                
                return {
                    "order": order,
                    "yaml": yaml.safe_dump(order, allow_unicode=True)
                }
                
            except Exception as e:
                print(f" Failed to get order: {e}")
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.adapter_app.post("/api/order/progress")
        async def report_progress(payload: Dict[str, Any]):
# """接收您的節點回報的進度"""
            order_id = payload.get('order_id')
            stage = payload.get('stage')
            message = payload.get('message')
# print(f" 訂單 {order_id} 進度: [{stage}] {message}")
            
            if order_id in self.current_orders:
                self.current_orders[order_id]['last_update'] = time.time()
                self.current_orders[order_id]['stage'] = stage
                self.current_orders[order_id]['message'] = message
            
            return {"status": "received"}
        
        @self.adapter_app.post("/api/order/complete")
        async def report_complete(payload: Dict[str, Any]):
# """接收您的節點回報的完成狀態"""
            order_id = payload.get('order_id')
            status = payload.get('status')   success / failed
            details = payload.get('details', '')
# print(f" 訂單 {order_id} 完成: {status} - {details}")
            
            if order_id in self.current_orders:
                order_info = self.current_orders[order_id]
                prescription_id = order_info['prescription_id']
# 更新處方籤狀態
                final_status = 'completed' if status == 'success' else 'failed'
                
                try:
                    import requests
                    response = requests.put(
                        f"http://localhost:/api/prescription/{prescription_id}/status",
                        json={"status": final_status}
                    )
                    if response.status_code == :
# print(f" 處方籤 {prescription_id} 狀態已更新為: {final_status}")
                    else:
# print(f" 更新處方籤狀態失敗: {response.status_code}")
                except Exception as e:
# print(f" 更新處方籤狀態時發生錯誤: {e}")
# 清理記錄
                del self.current_orders[order_id]
            
            return {"status": "completed"}
        
        @self.adapter_app.get("/api/order/status")
        async def get_status():
            """Get current order status"""
            return {
                "current_orders": self.current_orders,
                "total_processing": len(self.current_orders)
            }
    
    async def convert_prescription_to_order(self, prescription: Dict[str, Any]) -> Dict[str, Any]:
# """將處方籤轉換為您的節點需要的訂單格式"""
        order = {
            "order_id": f"{prescription['id']:d}",
            "prescription_id": prescription['id'],
            "patient_name": prescription.get('patient_name', 'Unknown'),
            "medicine": []
        }
        
        medicines = prescription.get('medicines', [])
        for i, med in enumerate(medicines):
            medicine_item = {
                "name": med.get('name', 'Unknown'),
                "amount": med.get('amount', ),
# "locate": self.generate_location(i),   可以根據實際藥櫃配置調整
                "prompt": self.determine_prompt(med.get('name', ''))
            }
            order["medicine"].append(medicine_item)
        
        return order
    
    def generate_location(self, index: int) -> List[int]:
# """生成藥物位置 [row, col]"""
# 簡單的位置分配邏輯，您可以根據實際藥櫃調整
# row = (index // ) +    每排最多  個
        col = (index % ) + 
        return [row, col]
    
    def determine_prompt(self, medicine_name: str) -> str:
# """根據藥物名稱決定類型提示"""
        name_lower = medicine_name.lower()
# if any(word in name_lower for word in ['膠囊', 'capsule']):
            return 'capsule'
# elif any(word in name_lower for word in ['盒', 'box']):
            return 'white_circle_box'
        else:
# return 'tablet'   預設為錠劑
    
    def start_web_server(self):
# """啟動主要的 Web 服務器"""
# print(" 啟動主要 Web 服務器...")
        
        try:
            process = subprocess.Popen([
                sys.executable, 'simple_server_final.py'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            self.processes.append(process)
# time.sleep()   等待啟動
            
            if process.poll() is None:
# print(" 主要 Web 服務器啟動成功 (port )")
                return True
            else:
# print(" 主要 Web 服務器啟動失敗")
                return False
                
        except Exception as e:
# print(f" 啟動主要 Web 服務器時發生錯誤: {e}")
            return False
    
    def start_adapter_server(self):
# """啟動適配器服務器"""
# print(" 啟動適配器服務器...")
        
        def run_adapter():
            uvicorn.run(self.adapter_app, host="...", port=, log_level="info")
        
        adapter_thread = threading.Thread(target=run_adapter, daemon=True)
        adapter_thread.start()
# time.sleep()   等待啟動
# print(" 適配器服務器啟動成功 (port )")
        return True
    
    def print_integration_info(self):
# """顯示整合信息"""
        print("\n" + "="  )
# print(" 您的 ROS 節點整合系統已啟動！")
        print("="  )
# print("\n 系統架構:")
        print("┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐")
# print("│   Web 界面      │    │   適配器 API     │    │  您的 ROS 節點 │")
        print("│  (port )    │◄──►│  (port )     │◄──►│                 │")
        print("│                 │    │                  │    │ OrderHandlerNode │")
        print("└─────────────────┘    └──────────────────┘    └─────────────────┘")
# print("\n 您的節點需要的環境變數:")
        print("export ORDER_BASE_URL='http://...:'")
        print("export ORDER_PULL_URL='http://...:/api/order/next'")
        print("export ORDER_PULL_INTERVAL=''")
        print("export ORDER_PROGRESS_PATH='/api/order/progress'")
        print("export ORDER_COMPLETE_PATH='/api/order/complete'")
# print("\n 啟動您的 ROS 節點:")
# print(" 在新的終端機中")
        print("source /opt/ros/humble/setup.bash")
        print("export ORDER_BASE_URL='http://...:'")
        print("python your_order_handler_node.py")
# print("\n Web 界面:")
# print("• 藥物管理: http://localhost:/integrated_medicine_management.html")
# print("• 醫生工作台: http://localhost:/doctor.html")
# print("• 處方籤管理: http://localhost:/Prescription.html")
# print("• 適配器狀態: http://localhost:/api/order/status")
# print("\n 測試方法:")
        print("python test_order_flow.py basic")
# print("\n 您的節點會收到的 YAML 格式:")
        print("order_id: \"\"")
        print("prescription_id: ")
# print("patient_name: \"張三\"")
        print("medicine:")
# print("  - name: 阿斯匹靈")
        print("    amount: ")
        print("    locate: [, ]")
        print("    prompt: tablet")
# print("\n 停止系統: Ctrl+C")
        print("="  )
    
    def shutdown(self):
# """關閉系統"""
# print(" 關閉整合系統...")
# 停止所有子進程
        for process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=10)
# print(" Web 服務器已關閉")
            except subprocess.TimeoutExpired:
                process.kill()
# print(" 強制關閉 Web 服務器")
            except Exception as e:
# print(f" 關閉進程時發生錯誤: {e}")
# print(" 整合系統已完全關閉")
    
    def run(self):
# """運行整合系統"""
# 設置信號處理器
        signal.signal(signal.SIGINT, lambda s, f: self.shutdown())
        signal.signal(signal.SIGTERM, lambda s, f: self.shutdown())
# print(" 啟動 ROS 節點整合系統")
        print("="  )
# 啟動主要 Web 服務器
        if not self.start_web_server():
# print(" 主要 Web 服務器啟動失敗")
            return False
# 啟動適配器服務器
        if not self.start_adapter_server():
# print(" 適配器服務器啟動失敗")
            return False
# 顯示整合信息
        self.print_integration_info()
# 保持運行
        try:
            while not self.shutdown_event.is_set():
                time.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()
        
        return True


def main():
# """主函數"""
    integration = YourNodeIntegration()
    
    try:
        success = integration.run()
        sys.exit( if success else )
    except Exception as e:
# print(f" 整合系統啟動失敗: {e}")
        sys.exit()


if __name__ == "__main__":
    main()
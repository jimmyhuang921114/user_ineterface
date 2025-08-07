#!/usr/bin/env python3
"""
訂單流程測試腳本
展示如何創建測試處方籤並監控狀態變化
"""

import requests
import time
import json
import sys
from typing import Optional

class OrderFlowTester:
    """訂單流程測試器"""
    
    def __init__(self, base_url: str = "http://localhost:8001"):
        self.base_url = base_url
        self.session = requests.Session()
        
    def check_server_status(self) -> bool:
        """檢查服務器狀態"""
        try:
            response = self.session.get(f"{self.base_url}/api/system/status", timeout=5)
            if response.status_code == 200:
                status = response.json()
                print(f"✅ 服務器運行中 - 模式: {status.get('ros2_mode', 'unknown')}")
                return True
            else:
                print(f"❌ 服務器狀態異常: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ 無法連接服務器: {e}")
            return False
    
    def create_test_medicine(self, name: str, description: str = "測試用藥物", stock: int = 100) -> bool:
        """創建測試藥物"""
        try:
            data = {
                "name": name,
                "description": description,
                "stock_quantity": stock
            }
            
            response = self.session.post(f"{self.base_url}/api/medicine/unified", json=data)
            if response.status_code == 200:
                print(f"✅ 創建測試藥物成功: {name}")
                return True
            else:
                print(f"⚠️ 創建藥物失敗 ({response.status_code}): {name} - 可能已存在")
                return True  # 藥物已存在也算成功
        except Exception as e:
            print(f"❌ 創建藥物時發生錯誤: {e}")
            return False
    
    def create_test_prescription(self, patient_name: str, medicine_name: str, amount: int = 5) -> Optional[int]:
        """創建測試處方籤"""
        try:
            data = {
                "patient_name": patient_name,
                "medicines": [
                    {
                        "name": medicine_name,
                        "amount": amount
                    }
                ]
            }
            
            print(f"📋 創建處方籤: {patient_name} - {medicine_name} x{amount}")
            response = self.session.post(f"{self.base_url}/api/prescription/", json=data)
            
            if response.status_code == 200:
                result = response.json()
                prescription_id = result.get('id')
                print(f"✅ 處方籤創建成功: ID {prescription_id}")
                return prescription_id
            else:
                print(f"❌ 創建處方籤失敗: {response.status_code}")
                try:
                    error_detail = response.json()
                    print(f"   錯誤詳情: {error_detail}")
                except:
                    print(f"   回應內容: {response.text}")
                return None
                
        except Exception as e:
            print(f"❌ 創建處方籤時發生錯誤: {e}")
            return None
    
    def check_prescription_status(self, prescription_id: int) -> Optional[str]:
        """檢查處方籤狀態"""
        try:
            response = self.session.get(f"{self.base_url}/api/prescription/{prescription_id}")
            if response.status_code == 200:
                data = response.json()
                status = data.get('status', 'unknown')
                patient_name = data.get('patient_name', 'N/A')
                return status
            else:
                print(f"❌ 查詢狀態失敗: {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ 查詢狀態時發生錯誤: {e}")
            return None
    
    def get_all_prescriptions(self) -> list:
        """獲取所有處方籤"""
        try:
            response = self.session.get(f"{self.base_url}/api/prescription/")
            if response.status_code == 200:
                return response.json()
            else:
                print(f"❌ 獲取處方籤列表失敗: {response.status_code}")
                return []
        except Exception as e:
            print(f"❌ 獲取處方籤列表時發生錯誤: {e}")
            return []
    
    def manual_complete_order(self, prescription_id: int) -> bool:
        """手動完成訂單"""
        try:
            data = {"status": "completed"}
            response = self.session.put(
                f"{self.base_url}/api/prescription/{prescription_id}/status", 
                json=data
            )
            
            if response.status_code == 200:
                print(f"✅ 手動完成訂單: {prescription_id}")
                return True
            else:
                print(f"❌ 手動完成失敗: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ 手動完成時發生錯誤: {e}")
            return False
    
    def reset_prescription_status(self, prescription_id: int, status: str = "pending") -> bool:
        """重置處方籤狀態"""
        try:
            data = {"status": status}
            response = self.session.put(
                f"{self.base_url}/api/prescription/{prescription_id}/status", 
                json=data
            )
            
            if response.status_code == 200:
                print(f"✅ 重置處方籤 {prescription_id} 狀態為: {status}")
                return True
            else:
                print(f"❌ 重置狀態失敗: {response.status_code}")
                return False
        except Exception as e:
            print(f"❌ 重置狀態時發生錯誤: {e}")
            return False
    
    def monitor_prescription_status(self, prescription_id: int, timeout: int = 60) -> str:
        """監控處方籤狀態變化"""
        print(f"🔍 監控處方籤 {prescription_id} 狀態變化...")
        
        start_time = time.time()
        last_status = None
        
        while (time.time() - start_time) < timeout:
            current_status = self.check_prescription_status(prescription_id)
            
            if current_status != last_status:
                print(f"📋 處方籤 {prescription_id} 狀態: {current_status}")
                last_status = current_status
                
                if current_status == 'completed':
                    print("✅ 訂單已完成!")
                    return current_status
                elif current_status == 'processing':
                    print("🔄 訂單處理中...")
            
            time.sleep(2)
        
        print(f"⏰ 監控超時 ({timeout}秒)")
        return last_status or 'timeout'
    
    def run_basic_test(self):
        """運行基本測試"""
        print("🧪 開始基本訂單流程測試")
        print("=" * 50)
        
        # 1. 檢查服務器狀態
        if not self.check_server_status():
            print("❌ 服務器未運行，請先啟動系統")
            return False
        
        # 2. 創建測試藥物
        medicine_name = "自動測試藥物"
        if not self.create_test_medicine(medicine_name):
            print("❌ 無法創建測試藥物")
            return False
        
        # 3. 創建測試處方籤
        patient_name = "自動測試病患"
        prescription_id = self.create_test_prescription(patient_name, medicine_name, 3)
        
        if not prescription_id:
            print("❌ 無法創建測試處方籤")
            return False
        
        # 4. 監控狀態變化
        final_status = self.monitor_prescription_status(prescription_id, timeout=30)
        
        if final_status == 'completed':
            print("✅ 測試完成 - 訂單自動處理成功")
            return True
        elif final_status in ['pending', 'processing']:
            print("⚠️ 訂單未自動完成，請檢查您的處理系統")
            
            # 詢問是否手動完成
            user_input = input("是否手動完成此訂單? (y/n): ").strip().lower()
            if user_input == 'y':
                return self.manual_complete_order(prescription_id)
            else:
                print("ℹ️ 訂單保持當前狀態")
                return False
        else:
            print("❌ 測試失敗")
            return False
    
    def run_batch_test(self, count: int = 3):
        """運行批量測試"""
        print(f"🧪 開始批量測試 ({count} 個訂單)")
        print("=" * 50)
        
        if not self.check_server_status():
            return False
        
        medicine_name = "批量測試藥物"
        self.create_test_medicine(medicine_name)
        
        prescription_ids = []
        
        # 創建多個處方籤
        for i in range(count):
            patient_name = f"批量測試病患{i+1}"
            prescription_id = self.create_test_prescription(patient_name, medicine_name, 2)
            if prescription_id:
                prescription_ids.append(prescription_id)
            time.sleep(1)  # 避免創建過快
        
        print(f"📋 已創建 {len(prescription_ids)} 個處方籤")
        
        # 監控所有處方籤
        completed_count = 0
        for prescription_id in prescription_ids:
            print(f"\n🔍 監控處方籤 {prescription_id}...")
            status = self.monitor_prescription_status(prescription_id, timeout=20)
            if status == 'completed':
                completed_count += 1
        
        print(f"\n📊 批量測試結果: {completed_count}/{len(prescription_ids)} 完成")
        return completed_count == len(prescription_ids)
    
    def show_current_prescriptions(self):
        """顯示當前所有處方籤"""
        print("📋 當前處方籤狀態:")
        print("=" * 60)
        
        prescriptions = self.get_all_prescriptions()
        if not prescriptions:
            print("📭 沒有找到處方籤")
            return
        
        for prescription in prescriptions:
            prescription_id = prescription.get('id', 'N/A')
            patient_name = prescription.get('patient_name', 'N/A')
            status = prescription.get('status', 'N/A')
            medicines = prescription.get('medicines', [])
            
            print(f"ID: {prescription_id} | 病患: {patient_name} | 狀態: {status}")
            for med in medicines:
                med_name = med.get('name', 'N/A')
                amount = med.get('amount', 'N/A')
                print(f"  - {med_name} x{amount}")
            print("-" * 40)


def main():
    """主函數"""
    tester = OrderFlowTester()
    
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        
        if command == 'basic':
            # 基本測試
            success = tester.run_basic_test()
            sys.exit(0 if success else 1)
            
        elif command == 'batch':
            # 批量測試
            count = int(sys.argv[2]) if len(sys.argv) > 2 else 3
            success = tester.run_batch_test(count)
            sys.exit(0 if success else 1)
            
        elif command == 'list':
            # 顯示當前處方籤
            tester.show_current_prescriptions()
            
        elif command == 'complete':
            # 手動完成指定處方籤
            if len(sys.argv) > 2:
                prescription_id = int(sys.argv[2])
                success = tester.manual_complete_order(prescription_id)
                sys.exit(0 if success else 1)
            else:
                print("❌ 請提供處方籤 ID")
                sys.exit(1)
                
        elif command == 'reset':
            # 重置指定處方籤狀態
            if len(sys.argv) > 2:
                prescription_id = int(sys.argv[2])
                status = sys.argv[3] if len(sys.argv) > 3 else "pending"
                success = tester.reset_prescription_status(prescription_id, status)
                sys.exit(0 if success else 1)
            else:
                print("❌ 請提供處方籤 ID")
                sys.exit(1)
        else:
            print(f"❌ 未知指令: {command}")
            sys.exit(1)
    else:
        # 互動模式
        print("🧪 訂單流程測試工具")
        print("=" * 50)
        print("可用指令:")
        print("  python3 test_order_flow.py basic          - 基本測試")
        print("  python3 test_order_flow.py batch [count]  - 批量測試")
        print("  python3 test_order_flow.py list           - 顯示所有處方籤")
        print("  python3 test_order_flow.py complete <id>  - 手動完成處方籤")
        print("  python3 test_order_flow.py reset <id> [status] - 重置處方籤狀態")
        print("")
        print("📋 Web 界面:")
        print("  • 藥物管理: http://localhost:8001/integrated_medicine_management.html")
        print("  • 醫生工作台: http://localhost:8001/doctor.html")
        print("  • 處方籤管理: http://localhost:8001/Prescription.html")
        print("")
        
        while True:
            try:
                choice = input("請選擇操作 (basic/batch/list/完成 ID/重置 ID/q): ").strip()
                
                if choice.lower() == 'q':
                    break
                elif choice.lower() == 'basic':
                    tester.run_basic_test()
                elif choice.lower() == 'batch':
                    count = input("請輸入測試數量 (預設 3): ").strip()
                    count = int(count) if count.isdigit() else 3
                    tester.run_batch_test(count)
                elif choice.lower() == 'list':
                    tester.show_current_prescriptions()
                elif choice.startswith('完成 '):
                    try:
                        prescription_id = int(choice.split()[1])
                        tester.manual_complete_order(prescription_id)
                    except (IndexError, ValueError):
                        print("❌ 請提供有效的處方籤 ID")
                elif choice.startswith('重置 '):
                    try:
                        parts = choice.split()
                        prescription_id = int(parts[1])
                        status = parts[2] if len(parts) > 2 else "pending"
                        tester.reset_prescription_status(prescription_id, status)
                    except (IndexError, ValueError):
                        print("❌ 請提供有效的處方籤 ID")
                else:
                    print("❌ 無效指令")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ 發生錯誤: {e}")
        
        print("\n👋 測試結束")


if __name__ == "__main__":
    main()
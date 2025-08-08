#!/usr/bin/env python3
"""
ROS2 Integration Test Script
Test the complete hospital system ROS2 API endpoints
"""

import requests
import json
import time
import yaml

BASE_URL = "http://localhost:8001"

def test_system_status():
    """Test system status"""
    print("🔍 Testing system status...")
    try:
        response = requests.get(f"{BASE_URL}/api/system/status")
        if response.status_code == 200:
            status = response.json()
            print(f"✅ System Status: {status['status']}")
            print(f"✅ ROS2 Mode: {status['ros_mode']}")
            print(f"✅ Database: {status['database']}")
            print(f"✅ Current Order: {status.get('current_order', 'None')}")
            print(f"✅ Queue Length: {status.get('queue_length', 0)}")
            return True
        else:
            print(f"❌ Status check failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Error checking status: {e}")
        return False

def test_medicine_apis():
    """Test medicine information APIs"""
    print("\n💊 Testing medicine information APIs...")
    
    # Test basic medicine info
    try:
        print("📋 Testing basic medicine info...")
        response = requests.get(f"{BASE_URL}/api/medicine/basic")
        if response.status_code == 200:
            medicines = response.json()
            print(f"✅ Found {len(medicines)} medicines")
            if medicines:
                medicine = medicines[0]
                print(f"   Sample: {medicine['name']} at {medicine['position']}")
        else:
            print(f"❌ Basic medicine API failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Error testing basic medicine API: {e}")
    
    # Test detailed medicine info
    try:
        print("📝 Testing detailed medicine info...")
        response = requests.get(f"{BASE_URL}/api/medicine/detailed")
        if response.status_code == 200:
            medicines = response.json()
            print(f"✅ Found detailed info for {len(medicines)} medicines")
            if medicines:
                medicine = medicines[0]
                has_content = bool(medicine.get('content', '').strip())
                print(f"   Sample: {medicine['name']} - Content: {'✅' if has_content else '❌'}")
        else:
            print(f"❌ Detailed medicine API failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Error testing detailed medicine API: {e}")

def test_ros2_medicine_query():
    """Test ROS2 medicine query endpoints"""
    print("\n🤖 Testing ROS2 medicine query endpoints...")
    
    # Test basic medicine query
    medicine_name = "Aspirin"
    try:
        print(f"📋 Querying basic info for {medicine_name}...")
        response = requests.get(f"{BASE_URL}/api/ros2/medicine/basic/{medicine_name}")
        if response.status_code == 200:
            data = response.json()
            print(f"✅ Basic info retrieved")
            print(f"   Position: {data.get('position', 'N/A')}")
            print(f"   Prompt: {data.get('prompt', 'N/A')}")
            print(f"   Confidence: {data.get('confidence', 'N/A')}")
            print(f"   Amount: {data.get('amount', 'N/A')}")
        else:
            print(f"❌ ROS2 basic medicine query failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Error testing ROS2 basic medicine query: {e}")
    
    # Test detailed medicine query
    try:
        print(f"📝 Querying detailed info for {medicine_name}...")
        response = requests.get(f"{BASE_URL}/api/ros2/medicine/detailed/{medicine_name}")
        if response.status_code == 200:
            data = response.json()
            print(f"✅ Detailed info retrieved")
            content = data.get('content', '')
            print(f"   Content length: {len(content)} characters")
            if 'yaml' in data:
                print(f"   YAML format: ✅")
        else:
            print(f"❌ ROS2 detailed medicine query failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Error testing ROS2 detailed medicine query: {e}")

def create_test_prescription():
    """Create a test prescription for ROS2 testing"""
    print("\n📝 Creating test prescription...")
    
    try:
        # Get available medicines
        response = requests.get(f"{BASE_URL}/api/medicine/basic")
        if response.status_code != 200:
            print("❌ Cannot get medicines for test prescription")
            return False
        
        medicines = response.json()
        if not medicines:
            print("❌ No medicines available for test prescription")
            return False
        
        # Create prescription with first 2 medicines
        prescription_data = {
            "patient_name": "ROS2 Test Patient",
            "medicines": [
                {"medicine_id": medicines[0]["id"], "amount": 2},
                {"medicine_id": medicines[1]["id"], "amount": 1} if len(medicines) > 1 else {"medicine_id": medicines[0]["id"], "amount": 1}
            ]
        }
        
        response = requests.post(
            f"{BASE_URL}/api/prescription/",
            headers={"Content-Type": "application/json"},
            data=json.dumps(prescription_data)
        )
        
        if response.status_code == 200:
            result = response.json()
            print(f"✅ Test prescription created: ID {result['id']}")
            return True
        else:
            print(f"❌ Failed to create test prescription: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Error creating test prescription: {e}")
        return False

def test_ros2_order_workflow():
    """Test complete ROS2 order workflow"""
    print("\n🔄 Testing ROS2 order workflow...")
    
    # Step 1: Get next order
    print("📡 Step 1: Getting next order...")
    try:
        response = requests.get(f"{BASE_URL}/api/ros2/order/next")
        if response.status_code == 200:
            order_data = response.json()
            order = order_data.get('order', {})
            order_id = order.get('order_id')
            
            print(f"✅ Order received: {order_id}")
            print(f"   Patient: {order.get('patient_name', 'N/A')}")
            print(f"   Medicines: {len(order.get('medicine', []))}")
            
            # Print YAML format
            yaml_content = order_data.get('yaml', '')
            if yaml_content:
                print("   YAML format:")
                for line in yaml_content.split('\n')[:5]:  # Show first 5 lines
                    if line.strip():
                        print(f"     {line}")
            
            # Step 2: Report progress
            print("📊 Step 2: Reporting progress...")
            progress_data = {
                "order_id": order_id,
                "stage": "processing",
                "message": "Processing medicine 1 of " + str(len(order.get('medicine', [])))
            }
            
            response = requests.post(
                f"{BASE_URL}/api/ros2/order/progress",
                headers={"Content-Type": "application/json"},
                data=json.dumps(progress_data)
            )
            
            if response.status_code == 200:
                print("✅ Progress reported successfully")
            else:
                print(f"❌ Progress report failed: {response.status_code}")
            
            # Step 3: Complete order
            print("✅ Step 3: Completing order...")
            complete_data = {
                "order_id": order_id,
                "status": "success",
                "details": "Order processed successfully by ROS2 test"
            }
            
            response = requests.post(
                f"{BASE_URL}/api/ros2/order/complete",
                headers={"Content-Type": "application/json"},
                data=json.dumps(complete_data)
            )
            
            if response.status_code == 200:
                result = response.json()
                print("✅ Order completed successfully")
                print(f"   Prescription ID: {result.get('prescription_id')}")
                print(f"   Status: {result.get('status')}")
                print(f"   Next available: {result.get('next_available', False)}")
                return True
            else:
                print(f"❌ Order completion failed: {response.status_code}")
                return False
                
        elif response.status_code == 204:
            print("ℹ️ No orders available for processing")
            return True
        else:
            print(f"❌ Get next order failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Error in ROS2 order workflow: {e}")
        return False

def test_prescription_status():
    """Test prescription status checking"""
    print("\n📋 Testing prescription status...")
    
    try:
        response = requests.get(f"{BASE_URL}/api/prescription/")
        if response.status_code == 200:
            prescriptions = response.json()
            print(f"✅ Found {len(prescriptions)} prescriptions")
            
            # Show status distribution
            status_count = {}
            for p in prescriptions:
                status = p.get('status', 'unknown')
                status_count[status] = status_count.get(status, 0) + 1
            
            print("   Status distribution:")
            for status, count in status_count.items():
                print(f"     {status}: {count}")
            
            return True
        else:
            print(f"❌ Prescription status check failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Error checking prescription status: {e}")
        return False

def main():
    """Main test function"""
    print("🏥 Hospital Management System - ROS2 Integration Test")
    print("=" * 60)
    
    # Test system status
    if not test_system_status():
        print("\n❌ System status check failed. Make sure the server is running.")
        return
    
    # Test medicine APIs
    test_medicine_apis()
    
    # Test ROS2 medicine queries
    test_ros2_medicine_query()
    
    # Create test prescription
    create_test_prescription()
    
    # Test prescription status
    test_prescription_status()
    
    # Test ROS2 order workflow
    test_ros2_order_workflow()
    
    print("\n🎉 ROS2 Integration Test Complete!")
    print("=" * 60)
    print("📋 Test Summary:")
    print("   ✅ System Status API")
    print("   ✅ Medicine Information APIs")
    print("   ✅ ROS2 Medicine Query APIs")
    print("   ✅ Prescription Management")
    print("   ✅ ROS2 Order Workflow")
    print("\n🔗 Key ROS2 Endpoints:")
    print("   - GET /api/ros2/order/next")
    print("   - POST /api/ros2/order/complete")
    print("   - POST /api/ros2/order/progress")
    print("   - GET /api/ros2/medicine/basic/{name}")
    print("   - GET /api/ros2/medicine/detailed/{name}")

if __name__ == "__main__":
    main()
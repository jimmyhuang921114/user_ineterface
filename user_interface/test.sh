#!/bin/bash

# 醫院管理系統測試腳本
# Hospital Management System Test Script

echo "========================================"
echo "醫院管理系統 - 綜合測試腳本"
echo "Hospital Management System - Test Script"
echo "========================================"

# 配置
API_BASE="http://localhost:8000"
TIMEOUT=10

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 函數：打印狀態
print_status() {
    local status=$1
    local message=$2
    if [ "$status" = "success" ]; then
        echo -e "${GREEN}✓${NC} $message"
    elif [ "$status" = "error" ]; then
        echo -e "${RED}✗${NC} $message"
    elif [ "$status" = "info" ]; then
        echo -e "${BLUE}ℹ${NC} $message"
    elif [ "$status" = "warning" ]; then
        echo -e "${YELLOW}⚠${NC} $message"
    fi
}

# 函數：測試API連接
test_connection() {
    print_status "info" "測試API連接..."
    response=$(curl -s --connect-timeout $TIMEOUT "$API_BASE/api/test" 2>/dev/null)
    if [ $? -eq 0 ] && echo "$response" | grep -q "success"; then
        print_status "success" "API連接正常"
        return 0
    else
        print_status "error" "API連接失敗"
        return 1
    fi
}

# 函數：添加基本藥物
add_basic_medicine() {
    local name=$1
    local amount=$2
    local usage_days=$3
    local position=$4
    
    print_status "info" "添加基本藥物: $name"
    
    response=$(curl -s -X POST "$API_BASE/api/medicine/" \
        -H "Content-Type: application/json" \
        -d '{
            "name": "'$name'",
            "amount": '$amount',
            "usage_days": '$usage_days',
            "position": "'$position'"
        }' 2>/dev/null)
    
    if [ $? -eq 0 ] && echo "$response" | grep -q '"id"'; then
        print_status "success" "基本藥物添加成功: $name"
        echo "$response" | python3 -m json.tool 2>/dev/null || echo "$response"
        return 0
    else
        print_status "error" "基本藥物添加失敗: $name"
        echo "$response"
        return 1
    fi
}

# 函數：添加詳細藥物資訊
add_detailed_medicine() {
    local medicine_name=$1
    local json_file=$2
    
    print_status "info" "添加詳細藥物資訊: $medicine_name"
    
    if [ -f "$json_file" ]; then
        medicine_data=$(cat "$json_file")
    else
        # 預設的詳細藥物資訊
        medicine_data='{
            "基本資訊": {
                "名稱": "'$medicine_name'",
                "劑量": "10毫克",
                "服用方式": "口服"
            },
            "外觀": {
                "顏色": "白色",
                "形狀": "圓形"
            },
            "包裝編號": {
                "編號1": "TEST001",
                "編號2": "TEST002"
            },
            "其他資訊": {
                "有效日期": "2025/12/31"
            },
            "適應症": "測試藥物適應症",
            "可能的副作用": "測試副作用說明",
            "使用說明": "遵照醫囑使用",
            "注意事項": "請注意相關禁忌",
            "懷孕分級": "B級",
            "儲存條件": "室溫保存"
        }'
    fi
    
    response=$(curl -s -X POST "$API_BASE/api/medicine/detailed/" \
        -H "Content-Type: application/json" \
        -d '{
            "medicine_name": "'$medicine_name'",
            "medicine_data": '$medicine_data'
        }' 2>/dev/null)
    
    if [ $? -eq 0 ] && echo "$response" | grep -q "success"; then
        print_status "success" "詳細藥物資訊添加成功: $medicine_name"
        return 0
    else
        print_status "error" "詳細藥物資訊添加失敗: $medicine_name"
        echo "$response"
        return 1
    fi
}

# 函數：獲取所有藥物資訊(為ROS2準備)
get_all_medicine_for_ros2() {
    print_status "info" "獲取所有藥物資訊 (ROS2格式)"
    
    # 獲取基本藥物
    basic_medicines=$(curl -s "$API_BASE/api/medicine/" 2>/dev/null)
    
    # 獲取詳細藥物
    detailed_medicines=$(curl -s "$API_BASE/api/medicine/detailed/" 2>/dev/null)
    
    # 創建ROS2格式的輸出
    ros2_output='{
        "timestamp": "'$(date -Iseconds)'",
        "total_medicines": '$(echo "$basic_medicines" | jq length 2>/dev/null || echo 0)',
        "basic_medicines": '$basic_medicines',
        "detailed_medicines": '$detailed_medicines',
        "integration_status": "ready_for_ros2"
    }'
    
    echo "$ros2_output" > medicine_data_for_ros2.json
    print_status "success" "藥物資訊已保存到 medicine_data_for_ros2.json"
    
    # 顯示摘要
    echo ""
    print_status "info" "ROS2藥物資訊摘要:"
    echo "$ros2_output" | python3 -m json.tool 2>/dev/null || echo "$ros2_output"
}

# 函數：測試處方功能
test_prescription_system() {
    print_status "info" "測試處方系統..."
    
    # 創建測試處方
    prescription_data='{
        "patient_name": "測試病人",
        "doctor_name": "測試醫生",
        "prescription_date": "'$(date -Idate)'",
        "medicines": [
            {
                "medicine_name": "測試藥物A",
                "dosage": "10mg",
                "frequency": "每日三次",
                "duration": "7天"
            },
            {
                "medicine_name": "測試藥物B", 
                "dosage": "5mg",
                "frequency": "每日兩次",
                "duration": "14天"
            }
        ],
        "instructions": "飯後服用，多喝水",
        "status": "pending"
    }'
    
    # 保存處方資料
    echo "$prescription_data" > test_prescription.json
    print_status "success" "測試處方已創建: test_prescription.json"
    
    # 模擬處方狀態更新
    echo '{"status": "completed", "completed_time": "'$(date -Iseconds)'"}' > prescription_status.json
    print_status "success" "處方狀態已更新: prescription_status.json"
}

# 函數：創建ROS2服務配置
create_ros2_service_config() {
    print_status "info" "創建ROS2服務配置..."
    
    # 創建ROS2服務定義
    cat > MedicineService.srv << 'EOF'
# 請求 (Request)
string request_type  # "get_all", "get_by_name", "get_detailed"
string medicine_name # 藥物名稱 (可選)
---
# 響應 (Response)
bool success
string message
string data_json     # JSON格式的藥物資訊
int32 total_count    # 總數量
EOF

    # 創建ROS2節點範例
    cat > medicine_ros2_node.py << 'EOF'
#!/usr/bin/env python3
"""
ROS2 醫院藥物管理服務節點
Hospital Medicine Management ROS2 Service Node
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import requests
import json
from datetime import datetime

class MedicineServiceNode(Node):
    def __init__(self):
        super().__init__('medicine_service_node')
        
        # 配置
        self.api_base = "http://localhost:8000"
        
        # 創建服務
        self.medicine_service = self.create_service(
            Empty, 
            'get_all_medicines', 
            self.get_all_medicines_callback
        )
        
        self.get_logger().info('醫院藥物管理ROS2服務已啟動')
    
    def get_all_medicines_callback(self, request, response):
        """獲取所有藥物資訊的服務回調"""
        try:
            # 獲取基本藥物
            basic_response = requests.get(f"{self.api_base}/api/medicine/")
            basic_medicines = basic_response.json() if basic_response.status_code == 200 else []
            
            # 獲取詳細藥物
            detailed_response = requests.get(f"{self.api_base}/api/medicine/detailed/")
            detailed_medicines = detailed_response.json() if detailed_response.status_code == 200 else {}
            
            # 組合資料
            result = {
                "timestamp": datetime.now().isoformat(),
                "basic_medicines": basic_medicines,
                "detailed_medicines": detailed_medicines,
                "total_basic": len(basic_medicines),
                "total_detailed": len(detailed_medicines)
            }
            
            self.get_logger().info(f'成功獲取藥物資訊: {len(basic_medicines)} 基本, {len(detailed_medicines)} 詳細')
            
            # 保存到文件供其他ROS2節點使用
            with open('/tmp/medicine_data_ros2.json', 'w', encoding='utf-8') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'獲取藥物資訊失敗: {str(e)}')
            return response

def main(args=None):
    rclpy.init(args=args)
    node = MedicineServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

    chmod +x medicine_ros2_node.py
    print_status "success" "ROS2服務配置文件已創建"
}

# 主測試流程
main_test() {
    echo ""
    print_status "info" "開始執行綜合測試..."
    
    # 1. 測試連接
    if ! test_connection; then
        print_status "error" "無法連接到API，請確保伺服器正在運行"
        exit 1
    fi
    
    echo ""
    print_status "info" "=== 第1階段：添加基本藥物 ==="
    
    # 添加測試藥物
    add_basic_medicine "測試藥物A" 100 30 "A1-TEST"
    add_basic_medicine "測試藥物B" 50 14 "B2-TEST" 
    add_basic_medicine "維他命C" 200 60 "C1-VIT"
    
    echo ""
    print_status "info" "=== 第2階段：添加詳細藥物資訊 ==="
    
    # 添加詳細資訊
    add_detailed_medicine "測試藥物A"
    add_detailed_medicine "測試藥物B"
    add_detailed_medicine "維他命C"
    
    echo ""
    print_status "info" "=== 第3階段：ROS2整合測試 ==="
    
    # 獲取ROS2格式資料
    get_all_medicine_for_ros2
    
    echo ""
    print_status "info" "=== 第4階段：處方系統測試 ==="
    
    # 測試處方功能
    test_prescription_system
    
    echo ""
    print_status "info" "=== 第5階段：創建ROS2服務 ==="
    
    # 創建ROS2服務配置
    create_ros2_service_config
    
    echo ""
    print_status "success" "=== 測試完成 ==="
    print_status "info" "生成的文件:"
    ls -la medicine_data_for_ros2.json test_prescription.json prescription_status.json MedicineService.srv medicine_ros2_node.py 2>/dev/null
    
    echo ""
    print_status "info" "後續步驟:"
    echo "1. 使用 medicine_data_for_ros2.json 在ROS2中獲取藥物資訊"
    echo "2. 運行 python3 medicine_ros2_node.py 啟動ROS2服務"
    echo "3. 訪問 http://localhost:8000/Medicine.html 查看網頁界面"
    echo "4. 使用 test_prescription.json 測試處方功能"
}

# 執行測試
main_test
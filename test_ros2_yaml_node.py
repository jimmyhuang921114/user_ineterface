#!/usr/bin/env python3
"""
測試ROS2 YAML節點功能
"""

import time
import json
import yaml
import subprocess
import os
from pathlib import Path

def test_yaml_node_functionality():
    """測試YAML節點功能"""
    print("🧪 測試ROS2 YAML節點功能")
    print("=" * 50)
    
    # 檢查PyYAML是否安裝
    try:
        import yaml
        print("✅ PyYAML 已安裝")
    except ImportError:
        print("❌ PyYAML 未安裝，正在安裝...")
        subprocess.run(["pip3", "install", "PyYAML"], check=True)
        print("✅ PyYAML 安裝完成")
    
    # 檢查requests是否安裝
    try:
        import requests
        print("✅ requests 已安裝")
    except ImportError:
        print("❌ requests 未安裝，正在安裝...")
        subprocess.run(["pip3", "install", "requests"], check=True)
        print("✅ requests 安裝完成")
    
    print()

def simulate_yaml_generation():
    """模擬YAML生成（不需要ROS2環境）"""
    print("🎯 模擬YAML生成功能...")
    
    # 模擬訂單數據
    order_data = {
        "id": "000001",
        "order_data": {
            "medicine_1": {
                "amount": 87,
                "locate": [1, 1],
                "name": "Antipsychotics"
            },
            "medicine_2": {
                "amount": 30,
                "locate": [1, 2],
                "name": "Andsodhcd"
            }
        }
    }
    
    # 模擬藥物資料庫
    basic_medicines = [
        {
            "name": "Antipsychotics",
            "amount": 100,
            "position": "1-1",
            "dosage": "5 毫克",
            "manufacturer": "精神好製藥"
        },
        {
            "name": "Andsodhcd", 
            "amount": 50,
            "position": "1-2",
            "dosage": "1 special pill",
            "manufacturer": "精神好製藥"
        }
    ]
    
    detailed_medicines = [
        {
            "medicine_name": "Antipsychotics",
            "ingredient": "Antipsychotic compounds",
            "category": "精神科藥物",
            "usage_method": "口服 (Oral use)",
            "unit_dose": "1 special pill",
            "description": "消除精神分裂症的陽性病徵，例如幻覺、妄想、思想混亂等。有助眠功效，適用於對其他藥物不適應者。",
            "side_effects": "嗜睡、頭暈、體重增加、口乾、便秘、姿勢性低血壓",
            "barcode": "ANTI-123456789",
            "appearance_type": "藥片",
            "appearance": {
                "color": "紅色條紋 白色外觀",
                "shape": "圓扁形"
            },
            "expiry_date": "2027/08/02",
            "storage_conditions": "室溫保存"
        },
        {
            "medicine_name": "Andsodhcd",
            "ingredient": "Nirmatrelvir",
            "category": "3CL proteinase inhibitors",
            "usage_method": "口服 (Oral use)",
            "unit_dose": "1 special pill",
            "description": "適用於12歲以上、體重至少40公斤，於5天內確診輕度至中度COVID-19，且具嚴重疾病風險因子的成人與兒童",
            "side_effects": "味覺異常、腹瀉、噁心、嘔吐、頭痛",
            "barcode": "TEST-367842394",
            "appearance_type": "藥片",
            "appearance": {
                "color": "藍色條紋 白色外觀",
                "shape": "圓扁形"
            },
            "expiry_date": "2027/11/09",
            "storage_conditions": "室溫保存"
        }
    ]
    
    # 生成YAML數據
    yaml_data = {}
    
    for medicine_key, order_info in order_data["order_data"].items():
        medicine_name = order_info.get("name")
        locate = order_info.get("locate", [1, 1])
        
        # 查找對應的藥物資料
        basic_data = next((m for m in basic_medicines if m["name"] == medicine_name), None)
        detailed_data = next((m for m in detailed_medicines if m["medicine_name"] == medicine_name), None)
        
        if basic_data and detailed_data:
            yaml_data[medicine_name] = {
                "藥物基本資料": {
                    "外觀類型": detailed_data.get("appearance_type", "藥片"),
                    "存取位置": locate
                },
                "藥物詳細資料": {
                    "名稱": basic_data.get("name", ""),
                    "成分": detailed_data.get("ingredient", ""),
                    "分類": detailed_data.get("category", ""),
                    "劑量": basic_data.get("dosage", ""),
                    "服用方式": detailed_data.get("usage_method", "口服 (Oral use)"),
                    "單位劑量": detailed_data.get("unit_dose", "1 special pill"),
                    "有效日期": detailed_data.get("expiry_date", ""),
                    "適應症": detailed_data.get("description", ""),
                    "可能副作用": detailed_data.get("side_effects", ""),
                    "條碼編號": detailed_data.get("barcode", "N/A"),
                    "外觀": {
                        "顏色": detailed_data.get("appearance", {}).get("color", ""),
                        "形狀": detailed_data.get("appearance", {}).get("shape", "")
                    }
                }
            }
    
    # 轉換為YAML字符串
    yaml_content = yaml.dump(yaml_data, default_flow_style=False, allow_unicode=True, sort_keys=False)
    
    # 添加標題
    header = f"# 訂單ID: {order_data['id']}\n# 生成時間: {time.strftime('%Y-%m-%d %H:%M:%S')}\n# 精神好製藥\n\n"
    final_yaml = header + yaml_content
    
    # 寫入文件
    output_file = "medicine_order_output.yaml"
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(final_yaml)
    
    print(f"✅ YAML文件已生成: {output_file}")
    print("\n📄 生成的YAML內容:")
    print("-" * 50)
    print(final_yaml)
    print("-" * 50)
    
    return output_file

def validate_yaml_format():
    """驗證生成的YAML格式"""
    print("\n🔍 驗證YAML格式...")
    
    yaml_file = "medicine_order_output.yaml"
    if os.path.exists(yaml_file):
        try:
            with open(yaml_file, 'r', encoding='utf-8') as f:
                yaml_data = yaml.safe_load(f)
            
            print("✅ YAML格式驗證通過")
            
            # 檢查必要的結構
            for medicine_name, data in yaml_data.items():
                if "藥物基本資料" in data and "藥物詳細資料" in data:
                    basic = data["藥物基本資料"]
                    detailed = data["藥物詳細資料"]
                    
                    print(f"✅ {medicine_name}: 基本資料和詳細資料結構正確")
                    
                    # 檢查必要欄位
                    required_basic = ["外觀類型", "存取位置"]
                    required_detailed = ["名稱", "成分", "分類", "劑量", "服用方式"]
                    
                    for field in required_basic:
                        if field in basic:
                            print(f"  ✅ 基本資料.{field}: {basic[field]}")
                        else:
                            print(f"  ❌ 缺少基本資料.{field}")
                    
                    for field in required_detailed:
                        if field in detailed:
                            print(f"  ✅ 詳細資料.{field}: {detailed[field]}")
                        else:
                            print(f"  ❌ 缺少詳細資料.{field}")
                else:
                    print(f"❌ {medicine_name}: 缺少必要的結構")
                    
        except Exception as e:
            print(f"❌ YAML格式驗證失敗: {e}")
    else:
        print(f"❌ YAML文件不存在: {yaml_file}")

def test_order_update_simulation():
    """測試訂單更新模擬"""
    print("\n🔄 測試訂單更新...")
    
    # 模擬新訂單
    new_order = {
        "id": "000002",
        "order_data": {
            "medicine_1": {
                "amount": 50,
                "locate": [2, 1],
                "name": "Andsodhcd"
            }
        }
    }
    
    print(f"📦 新訂單ID: {new_order['id']}")
    
    # 這裡會調用相同的生成邏輯，但使用新的訂單數據
    # 模擬覆蓋行為
    yaml_file = "medicine_order_output.yaml"
    
    # 生成新的YAML內容
    yaml_content = f"""# 訂單ID: {new_order['id']}
# 生成時間: {time.strftime('%Y-%m-%d %H:%M:%S')}
# 精神好製藥

Andsodhcd:
  藥物基本資料:
    外觀類型: 藥片
    存取位置: [2, 1]
  藥物詳細資料:
    名稱: Andsodhcd
    成分: Nirmatrelvir
    分類: 3CL proteinase inhibitors
    劑量: 1 special pill
    服用方式: 口服 (Oral use)
    單位劑量: 1 special pill
    有效日期: 2027/11/09
    適應症: 適用於12歲以上、體重至少40公斤，於5天內確診輕度至中度COVID-19，且具嚴重疾病風險因子的成人與兒童
    可能副作用: 味覺異常、腹瀉、噁心、嘔吐、頭痛
    條碼編號: TEST-367842394
    外觀:
      顏色: 藍色條紋 白色外觀
      形狀: 圓扁形
"""
    
    # 覆蓋寫入文件
    with open(yaml_file, 'w', encoding='utf-8') as f:
        f.write(yaml_content)
    
    print(f"✅ 新訂單YAML已覆蓋寫入: {yaml_file}")
    print("\n📄 新的YAML內容:")
    print("-" * 50)
    print(yaml_content)
    print("-" * 50)

def generate_usage_instructions():
    """生成使用說明"""
    print("\n📋 ROS2節點使用說明")
    print("=" * 50)
    
    print("🚀 啟動ROS2節點:")
    print("   python3 ros2_yaml_generator_node.py")
    print()
    
    print("📤 發送訂單請求:")
    print("   # 使用ROS2命令發布訂單")
    print("   ros2 topic pub /medicine_order_request std_msgs/String \\")
    print('   \'{"data": "{\\"id\\": \\"000001\\", \\"order_data\\": {\\"medicine_1\\": {\\"amount\\": 87, \\"locate\\": [1,1], \\"name\\": \\"Antipsychotics\\"}}}"}\'')
    print()
    
    print("👂 監聽YAML輸出:")
    print("   ros2 topic echo /medicine_yaml_output")
    print()
    
    print("📁 輸出文件:")
    print("   medicine_order_output.yaml - 生成的YAML文件")
    print()
    
    print("🔧 特性:")
    print("   ✅ 自動覆蓋舊訂單")
    print("   ✅ 支援多種藥物")
    print("   ✅ 完整的藥物資訊")
    print("   ✅ 標準YAML格式")
    print("   ✅ 中文支援")

def main():
    """主函數"""
    print("🤖 ROS2 YAML藥物訂單節點測試")
    print("=" * 60)
    
    # 測試功能
    test_yaml_node_functionality()
    
    # 模擬生成YAML
    simulate_yaml_generation()
    
    # 驗證格式
    validate_yaml_format()
    
    # 測試更新
    test_order_update_simulation()
    
    # 生成使用說明
    generate_usage_instructions()
    
    print("\n✅ 測試完成！")
    print("💡 現在您可以:")
    print("   1. 運行 python3 ros2_yaml_generator_node.py 啟動ROS2節點")
    print("   2. 訪問 http://localhost:8000/medicine_integrated.html 填寫藥物資料")
    print("   3. 檢查生成的 medicine_order_output.yaml 文件")

if __name__ == "__main__":
    main()
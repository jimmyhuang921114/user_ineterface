#!/usr/bin/env python3
"""
醫院藥物管理系統清理腳本
移除不必要的檔案，保留核心功能
"""

import os
import shutil
import sys

def safe_remove(path):
    """安全移除檔案或目錄"""
    try:
        if os.path.isfile(path):
            os.remove(path)
            print(f"✓ 已移除檔案: {path}")
        elif os.path.isdir(path):
            shutil.rmtree(path)
            print(f"✓ 已移除目錄: {path}")
        else:
            print(f"- 檔案不存在: {path}")
    except Exception as e:
        print(f"✗ 移除失敗 {path}: {e}")

def main():
    print("醫院藥物管理系統清理工具")
    print("========================================")
    
    # 確認當前目錄
    if not os.path.exists("simple_server_clean.py"):
        print("錯誤: 請在 user_interface 目錄下執行此腳本")
        sys.exit(1)
    
    # 移除快取檔案
    print("\n1. 清理快取檔案...")
    safe_remove("__pycache__")
    
    # 移除重複的文檔檔案
    print("\n2. 清理重複文檔...")
    redundant_docs = [
        "README.md",
        "README_CLEAN.md", 
        "FINAL_FILE_LIST.md",
        "FINAL_SYSTEM_GUIDE.md"
    ]
    
    for doc in redundant_docs:
        safe_remove(doc)
    
    # 移除舊版啟動器
    print("\n3. 清理舊版啟動器...")
    safe_remove("start_clean_server.py")
    safe_remove("run_complete_system.py")
    
    # 清理 ROS2 套件目錄中的重複套件
    print("\n4. 清理 ROS2 套件...")
    if os.path.exists("../ros2_packages"):
        redundant_packages = [
            "../ros2_packages/medicine_basic_provider",
            "../ros2_packages/medicine_client", 
            "../ros2_packages/medicine_detailed_provider",
            "../ros2_packages/medicine_info_provider",
            "../ros2_packages/medicine_interfaces",
            "../ros2_packages/medicine_management_client",
            "../ros2_packages/medicine_order_processor",
            "../ros2_packages/medicine_order_service"
        ]
        
        for package in redundant_packages:
            safe_remove(package)
    
    # 清理本地 ros2_services 目錄 (如果與 ros2_packages 重複)
    print("\n5. 清理本地 ROS2 服務...")
    if os.path.exists("ros2_services") and os.path.exists("../ros2_packages"):
        safe_remove("ros2_services")
    
    # 清理測試資料目錄 (保留重要資料)
    print("\n6. 檢查資料目錄...")
    if os.path.exists("data"):
        print("注意: data/ 目錄存在，請手動檢查是否包含重要資料")
        print("如確認無重要資料，可手動執行: rm -rf data/")
    
    # 檢查最終檔案結構
    print("\n7. 檢查核心檔案...")
    essential_files = [
        "simple_server_clean.py",
        "simple_server_ros2_real.py", 
        "simple_server_no_ros2.py",
        "database_clean.py",
        "ros2_mock_clean.py",
        "start_clean_system.py",
        "start_ros2_real_server.py",
        "start_no_ros2_server.py",
        "clean_test_system.py",
        "test_complete_system.py",
        "test_ros2_services.py",
        "README_NO_EMOJI.md",
        "SYSTEM_OVERVIEW.md"
    ]
    
    missing_files = []
    for file in essential_files:
        if os.path.exists(file):
            print(f"✓ {file}")
        else:
            print(f"✗ {file} (缺失)")
            missing_files.append(file)
    
    # 檢查前端檔案
    print("\n8. 檢查前端檔案...")
    if os.path.exists("static"):
        print("✓ static/ 目錄存在")
        frontend_files = [
            "static/integrated_medicine_management.html",
            "static/Prescription.html", 
            "static/doctor.html"
        ]
        
        for file in frontend_files:
            if os.path.exists(file):
                print(f"  ✓ {file}")
            else:
                print(f"  ✗ {file} (缺失)")
                missing_files.append(file)
    else:
        print("✗ static/ 目錄不存在")
        missing_files.append("static/")
    
    # 總結
    print("\n========================================")
    print("清理完成!")
    
    if missing_files:
        print(f"\n警告: 發現 {len(missing_files)} 個缺失的核心檔案:")
        for file in missing_files:
            print(f"  - {file}")
        print("\n請確保這些檔案存在以保證系統正常運作。")
    else:
        print("\n✓ 所有核心檔案都存在，系統應該可以正常運作。")
    
    print("\n建議執行測試驗證:")
    print("python3 clean_test_system.py")

if __name__ == "__main__":
    main()
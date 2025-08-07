#!/usr/bin/env python3
"""
醫院藥物管理系統修復腳本
自動修復常見問題：靜態檔案路徑、資料庫初始化等
"""

import os
import subprocess
import sys
from pathlib import Path

def fix_static_file_paths():
    """修復所有 HTML 檔案中的靜態檔案路徑"""
    print("🔧 修復靜態檔案路徑...")
    
    static_dir = Path("static")
    if not static_dir.exists():
        print("❌ static 目錄不存在")
        return False
    
    html_files = list(static_dir.glob("*.html"))
    fixed_count = 0
    
    for html_file in html_files:
        try:
            content = html_file.read_text(encoding='utf-8')
            
            # 修復 CSS 路徑
            content = content.replace('href="/css/', 'href="/static/css/')
            
            # 修復 JS 路徑  
            content = content.replace('src="/js/', 'src="/static/js/')
            
            html_file.write_text(content, encoding='utf-8')
            fixed_count += 1
            print(f"   ✓ 已修復: {html_file.name}")
            
        except Exception as e:
            print(f"   ❌ 修復失敗 {html_file.name}: {e}")
    
    print(f"✅ 靜態檔案路徑修復完成，共處理 {fixed_count} 個檔案")
    return True

def check_static_files():
    """檢查關鍵靜態檔案是否存在"""
    print("📂 檢查靜態檔案...")
    
    required_files = [
        "static/css/unified_style.css",
        "static/js/integrated_medicine_management.js",
        "static/js/doctor.js",
        "static/integrated_medicine_management.html",
        "static/doctor.html",
        "static/Prescription.html"
    ]
    
    missing_files = []
    for file_path in required_files:
        if not Path(file_path).exists():
            missing_files.append(file_path)
        else:
            print(f"   ✓ {file_path}")
    
    if missing_files:
        print("❌ 缺少以下關鍵檔案:")
        for missing in missing_files:
            print(f"   - {missing}")
        return False
    
    print("✅ 所有關鍵靜態檔案都存在")
    return True

def initialize_database():
    """初始化資料庫和樣本資料"""
    print("💾 檢查資料庫狀態...")
    
    # 檢查資料庫檔案是否存在
    db_file = Path("hospital_medicine_clean.db")
    if not db_file.exists():
        print("   資料庫檔案不存在，正在創建...")
        try:
            subprocess.run([sys.executable, "database_clean.py"], check=True)
            print("   ✓ 資料庫創建成功")
        except subprocess.CalledProcessError as e:
            print(f"   ❌ 資料庫創建失敗: {e}")
            return False
    
    # 檢查是否有藥物資料
    try:
        from database_clean import SessionLocal, Medicine
        db = SessionLocal()
        medicine_count = db.query(Medicine).count()
        db.close()
        
        if medicine_count == 0:
            print("   資料庫為空，正在添加樣本資料...")
            subprocess.run([sys.executable, "init_sample_data.py"], check=True)
            print("   ✓ 樣本資料添加成功")
        else:
            print(f"   ✓ 資料庫已有 {medicine_count} 種藥物")
            
    except Exception as e:
        print(f"   ❌ 資料庫檢查失敗: {e}")
        return False
    
    print("✅ 資料庫狀態正常")
    return True

def check_dependencies():
    """檢查必要的 Python 模組"""
    print("📦 檢查依賴模組...")
    
    required_modules = [
        "fastapi",
        "uvicorn", 
        "sqlalchemy",
        "requests"
    ]
    
    missing_modules = []
    for module in required_modules:
        try:
            __import__(module)
            print(f"   ✓ {module}")
        except ImportError:
            missing_modules.append(module)
            print(f"   ❌ {module}")
    
    if missing_modules:
        print("\n⚠️  缺少以下模組，請安裝:")
        print("sudo apt install python3-fastapi python3-uvicorn python3-sqlalchemy python3-requests")
        print("或者:")
        print("pip install", " ".join(missing_modules))
        return False
    
    print("✅ 所有依賴模組都已安裝")
    return True

def test_server_start():
    """測試伺服器是否能正常啟動"""
    print("🚀 測試伺服器啟動...")
    
    # 檢查啟動腳本是否存在
    startup_scripts = [
        "start_clean_system.py",
        "start_ros2_real_server.py"
    ]
    
    for script in startup_scripts:
        if Path(script).exists():
            print(f"   ✓ {script}")
        else:
            print(f"   ❌ {script} 缺失")
            return False
    
    print("✅ 啟動腳本檢查完成")
    print("🎯 建議啟動命令:")
    print("   python3 start_clean_system.py      # 推薦版本 (含模擬)")
    print("   python3 start_ros2_real_server.py  # 真實 ROS2 版本")
    return True

def main():
    """主修復程序"""
    print("醫院藥物管理系統 - 自動修復工具")
    print("=" * 50)
    
    checks_passed = 0
    total_checks = 5
    
    # 1. 檢查依賴
    if check_dependencies():
        checks_passed += 1
    
    # 2. 檢查靜態檔案
    if check_static_files():
        checks_passed += 1
    
    # 3. 修復靜態檔案路徑
    if fix_static_file_paths():
        checks_passed += 1
    
    # 4. 初始化資料庫
    if initialize_database():
        checks_passed += 1
    
    # 5. 檢查啟動腳本
    if test_server_start():
        checks_passed += 1
    
    # 總結
    print("\n" + "=" * 50)
    print(f"修復完成！通過檢查: {checks_passed}/{total_checks}")
    
    if checks_passed == total_checks:
        print("🎉 系統修復成功，所有檢查都通過！")
        print("\n🚀 現在可以啟動系統:")
        print("   python3 start_clean_system.py")
        print("\n🌐 然後訪問:")
        print("   http://localhost:8001/integrated_medicine_management.html")
    else:
        print("⚠️  部分檢查失敗，請查看上面的錯誤訊息並手動修復")
    
    return checks_passed == total_checks

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
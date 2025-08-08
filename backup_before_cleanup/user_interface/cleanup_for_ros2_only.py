#!/usr/bin/env python3
"""
ROS2 專用清理腳本
只保留必要的 ROS2 檔案，刪除所有多餘的檔案和文檔
"""

import os
import shutil
import sys
from pathlib import Path

class ROS2PackageCleaner:
    def __init__(self):
        self.workspace_root = Path("/workspace")
        self.user_interface_dir = self.workspace_root / "user_interface"
        
        # 必要的 ROS2 核心檔案
        self.essential_files = {
            # 資料庫和伺服器
            "database_final.py",
            "simple_server_final.py", 
            "hospital_medicine_final.db",
            
            # ROS2 整合檔案
            "integration_with_your_node.py",
            "medicine_detail_service_node.py",
            
            # 測試工具
            "test_order_flow.py",
            
            # Web 界面
            "static/",  # 整個目錄
        }
        
        # 要刪除的檔案類型
        self.files_to_delete = {
            # 所有 README 和文檔
            "README.md",
            "INTEGRATION_GUIDE.md",
            "ROS2_SETUP_INSTRUCTIONS.md", 
            "YOUR_ROS2_GUIDE.md",
            "ESSENTIAL_FILES_FOR_ROS2.md",
            "QUICK_REFERENCE.md",
            "ORDER_FLOW_GUIDE.md",
            "MEDICINE_DETAIL_SERVICE_GUIDE.md",
            "ROS2_SERVICES_GUIDE.md",
            "WEB_ROS2_ARCHITECTURE.md",
            "FINAL_SYSTEM_GUIDE.md",
            "ESSENTIAL_FILES_ANALYSIS.md",
            
            # 舊的 ROS2 檔案
            "ros2_services_interface.py",
            "ros2_client_example.py", 
            "ros2_medicine_detail_service.py",
            "medicine_client_example.py",
            "ros2_order_pusher.py",
            "ros2_interface_final.py",
            
            # 舊的啟動檔案
            "start_system_modes.py",
            "start_complete_system.py",
            "start_final_server.py",
            "start_your_system.py",
            "your_ros2_node.py",
            
            # 範例和清理檔案
            "integration_example.py",
            "cleanup_unnecessary_files.py",
            
            # Python 快取
            "__pycache__/",
        }
        
        # 根目錄要刪除的檔案
        self.root_files_to_delete = {
            "README.md",
            "start_system.sh",
            "start_system_simple.sh", 
            "requirements.txt",
            "requirements_simple.txt",
            "requirements_sql.txt",
        }

    def create_backup(self):
        """創建備份"""
        backup_dir = self.workspace_root / "backup_before_cleanup"
        if backup_dir.exists():
            shutil.rmtree(backup_dir)
        
        print(f"🗄️ 創建備份到: {backup_dir}")
        shutil.copytree(self.user_interface_dir, backup_dir / "user_interface")
        
        # 備份根目錄的檔案
        for file_name in self.root_files_to_delete:
            file_path = self.workspace_root / file_name
            if file_path.exists():
                shutil.copy2(file_path, backup_dir / file_name)
        
        print("✅ 備份完成")

    def clean_user_interface(self):
        """清理 user_interface 目錄"""
        print("🧹 清理 user_interface 目錄...")
        
        deleted_count = 0
        for file_name in self.files_to_delete:
            file_path = self.user_interface_dir / file_name
            
            if file_path.exists():
                if file_path.is_file():
                    file_path.unlink()
                    print(f"🗑️ 刪除檔案: {file_name}")
                elif file_path.is_dir():
                    shutil.rmtree(file_path)
                    print(f"🗑️ 刪除目錄: {file_name}")
                deleted_count += 1
        
        print(f"✅ user_interface 清理完成，刪除了 {deleted_count} 個項目")

    def clean_root_directory(self):
        """清理根目錄"""
        print("🧹 清理根目錄...")
        
        deleted_count = 0
        for file_name in self.root_files_to_delete:
            file_path = self.workspace_root / file_name
            
            if file_path.exists():
                if file_path.is_file():
                    file_path.unlink()
                    print(f"🗑️ 刪除根目錄檔案: {file_name}")
                elif file_path.is_dir():
                    shutil.rmtree(file_path)
                    print(f"🗑️ 刪除根目錄: {file_name}")
                deleted_count += 1
        
        print(f"✅ 根目錄清理完成，刪除了 {deleted_count} 個項目")

    def create_new_readme(self):
        """創建新的簡潔 README"""
        readme_content = """# 🤖 ROS2 醫院藥物管理系統

## 🚀 快速啟動

### 1️⃣ 啟動 Web 系統和適配器
```bash
cd user_interface
python3 integration_with_your_node.py
```

### 2️⃣ 啟動您的 ROS2 節點
```bash
# 新終端機
source /opt/ros/humble/setup.bash
export ORDER_BASE_URL='http://127.0.0.1:8002'
python3 your_order_handler_node.py
```

### 3️⃣ (可選) 啟動藥物查詢服務
```bash
# 新終端機  
source /opt/ros/humble/setup.bash
python3 user_interface/medicine_detail_service_node.py
```

## 📋 YAML 訂單格式

您的 ROS2 節點會收到：
```yaml
order_id: "000001"
prescription_id: 1
patient_name: "張三"
medicine:
  - name: 阿斯匹靈
    amount: 10
    locate: [2, 3]
    prompt: tablet
```

## 🌐 Web 界面

- 藥物管理: http://localhost:8001/integrated_medicine_management.html
- 醫生工作台: http://localhost:8001/doctor.html  
- 處方籤管理: http://localhost:8001/Prescription.html

## 💊 ROS2 藥物查詢服務

```bash
ros2 service call /hospital/get_medicine_detail tm_robot_if/srv/MedicineDetail "{name: '阿斯匹靈'}"
ros2 service call /hospital/get_all_medicines tm_robot_if/srv/MedicineDetail "{name: ''}"
ros2 service call /hospital/search_medicines tm_robot_if/srv/MedicineDetail "{name: '感冒'}"
```

## 📦 必要檔案

- `user_interface/database_final.py` - 資料庫模型
- `user_interface/simple_server_final.py` - Web 伺服器
- `user_interface/integration_with_your_node.py` - ROS2 適配器
- `user_interface/medicine_detail_service_node.py` - 藥物查詢服務
- `user_interface/test_order_flow.py` - 測試工具
- `user_interface/static/` - Web 界面檔案
- `user_interface/hospital_medicine_final.db` - 資料庫

## 🔧 在您的節點中實現

```python
def _process_medicine(self, order_id: str, med: Dict[str, Any], idx: int, total: int):
    name = med.get('name')
    amount = med.get('amount') 
    locate = med.get('locate')  # [row, col]
    prompt = med.get('prompt')  # tablet/capsule/white_circle_box
    
    # 在這裡實現您的機器人邏輯
    # 1. 移動到 locate 位置
    # 2. 根據 prompt 抓取 amount 數量的 name 藥物
    # 3. 放置到配藥區
```

## 🧪 測試

```bash
cd user_interface
python3 test_order_flow.py basic
```
"""
        
        readme_path = self.workspace_root / "README.md"
        with open(readme_path, 'w', encoding='utf-8') as f:
            f.write(readme_content)
        
        print(f"📝 創建新的 README.md")

    def create_requirements(self):
        """創建簡化的 requirements.txt"""
        requirements_content = """# ROS2 醫院藥物管理系統 - 必要套件
fastapi==0.104.1
uvicorn[standard]==0.24.0
sqlalchemy==2.0.23
pydantic==2.5.0
requests==2.31.0
pyyaml==6.0.1
"""
        
        requirements_path = self.workspace_root / "requirements.txt"
        with open(requirements_path, 'w', encoding='utf-8') as f:
            f.write(requirements_content)
        
        print(f"📝 創建新的 requirements.txt")

    def show_final_structure(self):
        """顯示最終的檔案結構"""
        print("\n" + "="*60)
        print("🎉 ROS2 專用 Package 清理完成！")
        print("="*60)
        
        print("\n📦 保留的檔案結構:")
        print("workspace/")
        print("├── README.md                           # 新的簡潔說明")
        print("├── requirements.txt                    # 必要套件")
        print("└── user_interface/")
        print("    ├── database_final.py               # 資料庫模型")
        print("    ├── simple_server_final.py          # Web 伺服器")
        print("    ├── integration_with_your_node.py   # ROS2 適配器")
        print("    ├── medicine_detail_service_node.py # 藥物查詢服務")
        print("    ├── test_order_flow.py              # 測試工具")
        print("    ├── hospital_medicine_final.db      # 資料庫檔案")
        print("    └── static/                         # Web 界面")
        
        remaining_files = list(self.user_interface_dir.glob("*"))
        remaining_count = len([f for f in remaining_files if f.is_file() and not f.name.startswith('.')])
        
        print(f"\n📊 統計:")
        print(f"   保留檔案: {remaining_count + 2} 個 (含 README.md, requirements.txt)")
        print(f"   備份位置: /workspace/backup_before_cleanup/")
        
        print(f"\n🚀 快速啟動:")
        print(f"   cd user_interface")
        print(f"   python3 integration_with_your_node.py")

    def run(self, create_backup=True):
        """執行清理"""
        print("🤖 ROS2 專用 Package 清理工具")
        print("="*50)
        
        if create_backup:
            self.create_backup()
        
        self.clean_user_interface()
        self.clean_root_directory()
        self.create_new_readme()
        self.create_requirements()
        self.show_final_structure()


def main():
    cleaner = ROS2PackageCleaner()
    
    print("⚠️ 這將刪除所有舊的 README 和不必要的檔案")
    print("⚠️ 只保留 ROS2 必要的核心檔案")
    
    response = input("確定要繼續嗎？ (y/N): ").strip().lower()
    if response == 'y':
        cleaner.run(create_backup=True)
        print("\n✅ 清理完成！您現在有一個簡潔的 ROS2 專用 package。")
    else:
        print("❌ 取消清理")
        sys.exit(0)


if __name__ == "__main__":
    main()
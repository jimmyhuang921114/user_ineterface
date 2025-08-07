# 醫院藥物管理系統 - 核心必要檔案清單

## 核心伺服器檔案 (必須保留)

### 主要伺服器
- `simple_server_clean.py` - 主伺服器 (含 ROS2 模擬，推薦)
- `simple_server_ros2_real.py` - 真實 ROS2 版本
- `simple_server_no_ros2.py` - 純 API 版本

### 資料庫與核心組件
- `database_clean.py` - 資料庫模型定義
- `ros2_mock_clean.py` - ROS2 模擬器
- `hospital_medicine_clean.db` - SQLite 資料庫檔案

## 啟動腳本 (必須保留)

- `start_clean_system.py` - 乾淨版啟動器 (推薦)
- `start_ros2_real_server.py` - 真實 ROS2 啟動器
- `start_no_ros2_server.py` - 純 API 啟動器

## 測試檔案 (建議保留)

- `clean_test_system.py` - 乾淨版測試 (推薦)
- `test_complete_system.py` - 完整系統測試
- `test_ros2_services.py` - ROS2 服務測試

## 前端檔案 (必須保留)

### static/ 目錄下所有檔案
- `integrated_medicine_management.html` - 主管理界面
- `Prescription.html` - 處方籤管理
- `doctor.html` - 醫生工作站
- `css/` - 樣式檔案
- `js/` - JavaScript 檔案

## 文檔檔案 (建議保留)

- `README_NO_EMOJI.md` - 使用說明 (推薦)
- `SYSTEM_OVERVIEW.md` - 系統總覽
- `ROS2_NODE_SUMMARY.md` - ROS2 節點說明
- `ROS2_PACKAGE_GUIDE.md` - ROS2 套件指南

## 可移除的檔案

### 重複的文檔檔案
- `README.md` - 與 README_NO_EMOJI.md 重複
- `README_CLEAN.md` - 舊版說明
- `FINAL_FILE_LIST.md` - 臨時檔案
- `FINAL_SYSTEM_GUIDE.md` - 臨時檔案

### 舊版啟動器
- `start_clean_server.py` - 被 start_clean_system.py 取代

### 測試版腳本
- `run_complete_system.py` - 臨時測試檔案

## 可移除的目錄

### ros2_packages/ 下的多餘套件
保留核心套件，移除重複或測試用套件：

#### 建議保留
- `hospital_medicine_msgs/` - 核心訊息定義
- `hospital_medicine_ros2/` - 核心 ROS2 節點

#### 可移除
- `medicine_basic_provider/` - 重複功能
- `medicine_client/` - 測試用
- `medicine_detailed_provider/` - 重複功能
- `medicine_info_provider/` - 重複功能
- `medicine_interfaces/` - 重複功能
- `medicine_management_client/` - 測試用
- `medicine_order_processor/` - 重複功能
- `medicine_order_service/` - 重複功能

### 其他可移除
- `__pycache__/` - Python 快取檔案
- `data/` - 測試資料目錄 (如果沒有重要資料)
- `ros2_services/` - 如果與 ros2_packages 重複

## 清理建議

1. **立即移除**: `__pycache__/` 目錄
2. **合併文檔**: 保留 `README_NO_EMOJI.md`，移除其他 README
3. **簡化 ROS2**: 只保留核心 ROS2 套件
4. **測試驗證**: 清理後執行完整測試

## 最終核心檔案結構

```
user_interface/
├── simple_server_clean.py          # 主伺服器
├── simple_server_ros2_real.py      # 真實 ROS2 版本
├── simple_server_no_ros2.py        # 純 API 版本
├── database_clean.py               # 資料庫模型
├── ros2_mock_clean.py              # ROS2 模擬器
├── start_clean_system.py           # 主要啟動器
├── start_ros2_real_server.py       # ROS2 啟動器
├── start_no_ros2_server.py         # API 啟動器
├── clean_test_system.py            # 主要測試
├── test_complete_system.py         # 完整測試
├── test_ros2_services.py           # ROS2 測試
├── hospital_medicine_clean.db      # 資料庫
├── static/                         # 前端檔案
├── ros2_packages/                  # ROS2 套件 (精簡)
├── README_NO_EMOJI.md             # 使用說明
└── SYSTEM_OVERVIEW.md             # 系統總覽
```

這個結構保持功能完整性，同時移除冗余檔案，確保系統穩定且易於維護。
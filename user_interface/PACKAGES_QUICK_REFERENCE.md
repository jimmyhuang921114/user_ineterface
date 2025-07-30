# 包功能快速參考表
Quick Reference for Package Functionality

## 目錄說明
| 目錄/包 | 主要用途 | 包含文件數 | 核心功能 |
|---------|----------|------------|----------|
| `api/` | API路由模組 | 3 | 藥物和處方的REST API端點 |
| `data/` | 資料儲存 | 4+ | JSON持久化儲存和備份 |
| `model/` | 資料模型 | 2 | SQLModel/Pydantic資料定義 |
| `route/` | 路由處理 | 2 | HTTP請求路由邏輯 |
| `schemas/` | 資料驗證 | 2 | 輸入輸出格式驗證 |
| `services/` | 業務邏輯 | 2 | CRUD操作實現 |
| `static/` | 前端資源 | 11 | HTML/CSS/JS網頁文件 |

## 核心文件功能
| 文件名 | 類型 | 主要功能 | 執行方式 |
|--------|------|----------|----------|
| `modular_server.py` | 主伺服器 | FastAPI應用程式，整合所有模組 | `python3 modular_server.py` |
| `data_persistence.py` | 資料持久化 | JSON文件讀寫、備份管理 | 模組導入 |
| `start_modular.py` | 啟動腳本 | 依賴檢查、伺服器啟動 | `python3 start_modular.py` |
| `doctor.html` | 前端介面 | 醫生工作台，藥物和處方管理 | 瀏覽器訪問 |
| `Medicine.html` | 前端介面 | 藥物庫存管理和查詢 | 瀏覽器訪問 |
| `Prescription.html` | 前端介面 | 處方管理和狀態追蹤 | 瀏覽器訪問 |

## API模組詳細功能
| 模組 | 端點數量 | 主要端點 | 功能描述 |
|------|----------|----------|----------|
| `medicine_api.py` | 9 | `/api/medicine/` | 基本藥物CRUD |
| | | `/api/medicine/detailed/` | 詳細藥物資訊管理 |
| | | `/api/medicine/search/code/{code}` | 按編號搜索 |
| | | `/api/medicine/integrated/{name}` | 整合查詢 |
| `prescription_api.py` | 7 | `/api/prescription/` | 處方CRUD |
| | | `/api/prescription/status/{status}` | 狀態篩選 |
| | | `/api/prescription/{id}/status` | 狀態更新 |

## ROS2整合模組
| 文件名 | 功能 | ROS2類型 | 通信方式 |
|--------|------|----------|----------|
| `ros2_prescription_service.py` | 處方服務節點 | Publisher/Subscriber | 話題通信 |
| `test_ros2_client.py` | 處方處理客戶端 | Subscriber/Publisher | 話題通信 |
| `ros2_medicine_client.py` | 藥物資料客戶端 | Service Client | 服務調用 |
| `medicine_ros2_node.py` | 藥物服務節點 | Service Server | 服務提供 |

## 測試文件功能
| 文件名 | 測試類型 | 主要驗證項目 |
|--------|----------|--------------|
| `test_data_persistence.py` | 資料持久化 | 儲存、載入、備份功能 |
| `test_patient_creation.py` | 病患記錄 | 處方創建、狀態流程 |
| `quick_test_system.py` | 系統健康 | API端點、基本功能 |
| `test.sh` | 綜合測試 | 完整系統流程 |

## 資料流向圖
```
前端網頁 (HTML/CSS/JS)
    ↓ HTTP請求
API路由 (medicine_api.py, prescription_api.py)
    ↓ 資料處理
記憶體資料庫 (medicines_db, prescriptions_db)
    ↓ 持久化
JSON文件 (data/*.json)
    ↓ 備份
備份目錄 (data/backups/)

ROS2節點 ←→ API後端 (話題/服務通信)
```

## 快速命令參考
| 操作 | 命令 | 說明 |
|------|------|------|
| 啟動系統 | `python3 start_modular.py` | 完整啟動流程 |
| 直接啟動 | `python3 modular_server.py` | 僅啟動伺服器 |
| 測試持久化 | `python3 test_data_persistence.py` | 測試資料儲存 |
| 移除emoji | `python3 remove_all_emojis.py` | 清理文件emoji |
| 快速測試 | `python3 quick_test_system.py` | 基本功能檢查 |
| 綜合測試 | `./test.sh` | 完整功能測試 |

## 訪問地址
| 介面 | URL | 用途 |
|------|-----|------|
| 醫生工作台 | http://localhost:8000/doctor.html | 藥物輸入、處方開立 |
| 藥物管理 | http://localhost:8000/Medicine.html | 庫存管理、資料查詢 |
| 處方管理 | http://localhost:8000/Prescription.html | 處方狀態、統計資訊 |
| API文檔 | http://localhost:8000/docs | Swagger API文檔 |
| 系統狀態 | http://localhost:8000/api/system/status | 系統健康檢查 |

## 關鍵特色
- **零emoji干擾**: 所有文件已清除emoji字符
- **模組化架構**: 功能分離，便於維護
- **資料持久化**: 重啟不丟失資料
- **ROS2整合**: 支援機器人系統
- **統一風格**: 繁體中文介面
- **完整測試**: 全面驗證功能
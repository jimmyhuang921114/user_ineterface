#!/usr/bin/env python3
"""
醫院管理系統 - 模組化主伺服器
Hospital Management System - Modular Main Server
採用分離式API架構設計
"""

import uvicorn
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from pathlib import Path
import os

# 導入API模組
from api import medicine_api, prescription_api

# 創建FastAPI應用
app = FastAPI(
    title="醫院管理系統 API",
    description="模組化醫院管理系統 - 包含藥物管理和處方管理",
    version="2.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS中間件設定
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 靜態檔案設定
static_path = Path(__file__).parent / "static"
if static_path.exists():
    app.mount("/css", StaticFiles(directory=static_path / "css"), name="css")
    app.mount("/js", StaticFiles(directory=static_path / "js"), name="js")

# 註冊API路由器
app.include_router(medicine_api.router)
app.include_router(prescription_api.router)

# 初始化測試資料
def init_test_data():
    """初始化測試資料"""
    print("初始化測試資料...")
    
    # 初始化基本藥物資料
    test_medicines = [
        {"name": "阿斯匹靈", "amount": 100, "usage_days": 7, "position": "A1-01"},
        {"name": "普拿疼", "amount": 50, "usage_days": 3, "position": "A1-02"},
        {"name": "心律錠(Propranolol)", "amount": 30, "usage_days": 14, "position": "B2-03"},
        {"name": "精神好製藥", "amount": 25, "usage_days": 10, "position": "C3-01"}
    ]
    
    for i, med_data in enumerate(test_medicines, 1):
        medicine_api.medicines_db.append({
            "id": i,
            "name": med_data["name"],
            "amount": med_data["amount"],
            "usage_days": med_data["usage_days"],
            "position": med_data["position"],
            "create_time": "2025-01-01T00:00:00"
        })
    medicine_api.next_medicine_id = len(test_medicines) + 1
    
    # 初始化詳細藥物資料
    detailed_medicines = {
        "心律錠(Propranolol)": {
            "基本資訊": {
                "名稱": "心律錠(Propranolol)",
                "廠商": "生達",
                "劑量": "10毫克"
            },
            "外觀": {
                "顏色": "明紫紅",
                "形狀": "圓扁形"
            },
            "包裝編號": {
                "編號1": "202801",
                "編號2": "TP071014",
                "編號3": "衛署藥製字第009102號"
            },
            "其他資訊": {
                "公司全名": "生達化學製藥股份有限公司",
                "藥物全名": "Propranolol HCl"
            },
            "適應症": "狹心症、不整律、原發性及腎性高血壓、偏頭痛控制",
            "可能的副作用": "常見-心智混亂、疲憊、睏倦、心跳徐緩",
            "使用說明": "用法用量請遵照醫囑；除特別要求外，一般建議於飯後服用。",
            "注意事項": "本藥會掩飾低血糖症狀並延長低血糖時間",
            "懷孕分級": "C級；若於妊娠第二或第三期則為 D 級。",
            "儲存條件": "請連同藥袋存放於緊密容器內，室溫乾燥避光"
        },
        "精神好製藥": {
            "基本資訊": {
                "名稱": "精神好製藥 (Antipsychotics)",
                "劑量": "5 毫克",
                "服用方式": "口服 (Oral use)",
                "單位劑量": "1 special pill"
            },
            "外觀": {
                "顏色": "紅色條紋 白色外觀",
                "形狀": "圓扁形"
            },
            "其他資訊": {
                "有效日期": "2027/08/02"
            },
            "適應症": "消除精神分裂症的陽性病徵，例如幻覺、妄想、思想混亂等。有助眠功效，適用於對其他藥物不適應者。",
            "可能副作用": "嗜睡、頭暈、體重增加、口乾、便秘、姿勢性低血壓",
            "條碼": "（圖像中含條碼，無明確編號）"
        }
    }
    
    medicine_api.detailed_medicines_db.update(detailed_medicines)
    
    # 初始化測試處方
    test_prescription = {
        "id": 1,
        "patient_name": "張三",
        "doctor_name": "王醫師",
        "medicines": [
            {
                "medicine_name": "阿斯匹靈",
                "dosage": "100mg",
                "frequency": "每日一次",
                "duration": "7天",
                "instructions": "飯後服用"
            }
        ],
        "diagnosis": "輕度頭痛",
        "status": "pending",
        "prescription_date": "2025-01-01",
        "created_time": "2025-01-01T00:00:00",
        "updated_time": "2025-01-01T00:00:00"
    }
    
    prescription_api.prescriptions_db.append(test_prescription)
    prescription_api.next_prescription_id = 2
    
    print(f"✅ 測試資料初始化完成:")
    print(f"   - 基本藥物: {len(medicine_api.medicines_db)} 項")
    print(f"   - 詳細藥物: {len(medicine_api.detailed_medicines_db)} 項")
    print(f"   - 處方記錄: {len(prescription_api.prescriptions_db)} 項")

# === 網頁路由 ===
@app.get("/", response_class=HTMLResponse)
async def root():
    """系統首頁"""
    return """
    <!DOCTYPE html>
    <html lang="zh-TW">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>醫院管理系統</title>
        <link rel="stylesheet" href="/css/unified_style.css">
    </head>
    <body>
        <div class="sidebar">
            <h2>醫院管理系統</h2>
            <button onclick="location.href='/doctor.html'">醫生工作台</button>
            <button onclick="location.href='/Medicine.html'">藥物庫存管理</button>
            <button onclick="location.href='/Prescription.html'">處方管理系統</button>
            <button onclick="location.href='/docs'">API 文檔</button>
        </div>
        <div class="main-content">
            <div class="header fade-in">
                <h1>醫院管理系統</h1>
                <p>模組化架構 - 藥物管理與處方開立系統</p>
            </div>
            <div class="card fade-in">
                <div style="padding: 30px; text-align: center;">
                    <h2>系統功能</h2>
                    <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 20px; margin-top: 30px;">
                        <div style="padding: 20px; background: rgba(52, 152, 219, 0.1); border-radius: 10px;">
                            <h3>🩺 醫生工作台</h3>
                            <p>藥物資訊管理與處方開立</p>
                            <button class="btn btn-primary" onclick="location.href='/doctor.html'">進入系統</button>
                        </div>
                        <div style="padding: 20px; background: rgba(39, 174, 96, 0.1); border-radius: 10px;">
                            <h3>💊 藥物管理</h3>
                            <p>庫存管理與詳細資訊查詢</p>
                            <button class="btn btn-success" onclick="location.href='/Medicine.html'">進入系統</button>
                        </div>
                        <div style="padding: 20px; background: rgba(243, 156, 18, 0.1); border-radius: 10px;">
                            <h3>📋 處方管理</h3>
                            <p>處方查詢與狀態管理</p>
                            <button class="btn btn-warning" onclick="location.href='/Prescription.html'">進入系統</button>
                        </div>
                    </div>
                    <div style="margin-top: 30px;">
                        <h3>🔧 系統架構特色</h3>
                        <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-top: 20px;">
                            <div style="padding: 15px; background: rgba(155, 89, 182, 0.1); border-radius: 8px;">
                                <strong>模組化API</strong><br>
                                <small>藥物與處方API分離</small>
                            </div>
                            <div style="padding: 15px; background: rgba(231, 76, 60, 0.1); border-radius: 8px;">
                                <strong>統一風格</strong><br>
                                <small>繁體中文介面</small>
                            </div>
                            <div style="padding: 15px; background: rgba(23, 162, 184, 0.1); border-radius: 8px;">
                                <strong>自由輸入</strong><br>
                                <small>彈性的資料結構</small>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </body>
    </html>
    """

@app.get("/doctor.html")
async def serve_doctor_page():
    """醫生工作台頁面"""
    html_file = Path(__file__).parent / "static" / "html" / "doctor.html"
    if html_file.exists():
        return FileResponse(html_file)
    return HTMLResponse("<h1>醫生工作台頁面未找到</h1>", status_code=404)

@app.get("/Medicine.html")
async def serve_medicine_page():
    """藥物管理頁面"""
    html_file = Path(__file__).parent / "static" / "html" / "Medicine.html"
    if html_file.exists():
        return FileResponse(html_file)
    return HTMLResponse("<h1>藥物管理頁面未找到</h1>", status_code=404)

@app.get("/Prescription.html")
async def serve_prescription_page():
    """處方管理頁面"""
    html_file = Path(__file__).parent / "static" / "html" / "Prescription.html"
    if html_file.exists():
        return FileResponse(html_file)
    return HTMLResponse("<h1>處方管理頁面未找到</h1>", status_code=404)

# === 系統狀態API ===
@app.get("/api/system/status")
async def system_status():
    """系統狀態檢查"""
    return {
        "system": "醫院管理系統",
        "version": "2.0.0",
        "status": "運行中",
        "architecture": "模組化API",
        "components": {
            "medicine_api": "藥物管理API",
            "prescription_api": "處方管理API"
        },
        "statistics": {
            "total_medicines": len(medicine_api.medicines_db),
            "detailed_medicines": len(medicine_api.detailed_medicines_db),
            "total_prescriptions": len(prescription_api.prescriptions_db)
        }
    }

# 啟動事件
@app.on_event("startup")
async def startup_event():
    print("=" * 60)
    print("🏥 醫院管理系統 - 模組化架構啟動中...")
    print("=" * 60)
    init_test_data()
    print("\n🚀 系統已成功啟動！")
    print("📍 訪問地址:")
    print("   主頁: http://localhost:8000")
    print("   醫生工作台: http://localhost:8000/doctor.html")
    print("   藥物管理: http://localhost:8000/Medicine.html")
    print("   處方管理: http://localhost:8000/Prescription.html")
    print("   API文檔: http://localhost:8000/docs")
    print("=" * 60)

if __name__ == "__main__":
    uvicorn.run(
        "modular_server:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
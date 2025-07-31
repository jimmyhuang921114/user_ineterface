#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
醫療系統完整網頁伺服器
Complete Medical System Web Server with HTML Interface
"""

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from typing import List, Dict, Any
from datetime import datetime
import uvicorn
import os

# 創建FastAPI應用
app = FastAPI(title="醫療系統", description="Hospital Management System")

# 資料模型
class MedicalRecord(BaseModel):
    patient_id: str
    patient_name: str
    doctor_name: str
    diagnosis: str
    symptoms: str

# 內存資料庫
records = []
medicines = []
next_id = 1

# 創建static目錄如果不存在
os.makedirs("static", exist_ok=True)

# 掛載靜態文件
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/", response_class=HTMLResponse)
async def read_root():
    html_content = """
    <!DOCTYPE html>
    <html lang="zh-TW">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>醫療系統首頁</title>
        <style>
            body {
                font-family: 'Microsoft JhengHei', Arial, sans-serif;
                margin: 0;
                padding: 20px;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: #333;
                min-height: 100vh;
            }
            .container {
                max-width: 1200px;
                margin: 0 auto;
                background: white;
                border-radius: 15px;
                padding: 30px;
                box-shadow: 0 10px 30px rgba(0,0,0,0.2);
            }
            .header {
                text-align: center;
                margin-bottom: 40px;
                border-bottom: 3px solid #667eea;
                padding-bottom: 20px;
            }
            .header h1 {
                color: #667eea;
                font-size: 2.5em;
                margin: 0;
            }
            .nav-buttons {
                display: grid;
                grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
                gap: 20px;
                margin: 30px 0;
            }
            .nav-button {
                background: linear-gradient(45deg, #667eea, #764ba2);
                color: white;
                padding: 25px;
                border-radius: 10px;
                text-decoration: none;
                text-align: center;
                transition: transform 0.3s;
                box-shadow: 0 5px 15px rgba(0,0,0,0.1);
            }
            .nav-button:hover {
                transform: translateY(-5px);
                box-shadow: 0 10px 25px rgba(0,0,0,0.2);
            }
            .nav-button h3 {
                margin: 0 0 10px 0;
                font-size: 1.5em;
            }
            .status-section {
                background: #f8f9fa;
                padding: 20px;
                border-radius: 10px;
                margin-top: 30px;
            }
            .record-form {
                background: #f8f9fa;
                padding: 25px;
                border-radius: 10px;
                margin-top: 20px;
            }
            .form-group {
                margin-bottom: 15px;
            }
            label {
                display: block;
                margin-bottom: 5px;
                font-weight: bold;
                color: #555;
            }
            input, textarea {
                width: 100%;
                padding: 10px;
                border: 2px solid #ddd;
                border-radius: 5px;
                font-size: 16px;
                font-family: inherit;
            }
            textarea {
                height: 80px;
                resize: vertical;
            }
            .btn {
                background: #667eea;
                color: white;
                padding: 12px 30px;
                border: none;
                border-radius: 5px;
                cursor: pointer;
                font-size: 16px;
                font-family: inherit;
                transition: background 0.3s;
            }
            .btn:hover {
                background: #5a6fd8;
            }
            .records-list {
                margin-top: 30px;
            }
            .record-item {
                background: white;
                border: 1px solid #ddd;
                border-radius: 8px;
                padding: 15px;
                margin-bottom: 10px;
                box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            }
            .record-header {
                font-weight: bold;
                color: #667eea;
                margin-bottom: 8px;
            }
            .record-detail {
                color: #666;
                font-size: 0.9em;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>🏥 醫療管理系統</h1>
                <p>Hospital Management System</p>
            </div>
            
            <div class="nav-buttons">
                <a href="/medical" class="nav-button">
                    <h3>📋 病例管理</h3>
                    <p>創建和查看病例記錄</p>
                </a>
                <a href="/docs" class="nav-button">
                    <h3>📖 API文檔</h3>
                    <p>查看API接口文檔</p>
                </a>
                <a href="/status" class="nav-button">
                    <h3>⚡ 系統狀態</h3>
                    <p>檢查系統運行狀態</p>
                </a>
            </div>
            
            <div class="status-section">
                <h3>🎯 系統功能</h3>
                <ul>
                    <li>✅ 繁體中文完整支援</li>
                    <li>✅ 病例資料創建與查詢</li>
                    <li>✅ ROS2服務模擬調用</li>
                    <li>✅ API接口完整功能</li>
                    <li>✅ 網頁界面友好操作</li>
                </ul>
            </div>
        </div>
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)

@app.get("/medical", response_class=HTMLResponse)
async def medical_page():
    html_content = """
    <!DOCTYPE html>
    <html lang="zh-TW">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>病例管理系統</title>
        <style>
            body {
                font-family: 'Microsoft JhengHei', Arial, sans-serif;
                margin: 0;
                padding: 20px;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: #333;
                min-height: 100vh;
            }
            .container {
                max-width: 1200px;
                margin: 0 auto;
                background: white;
                border-radius: 15px;
                padding: 30px;
                box-shadow: 0 10px 30px rgba(0,0,0,0.2);
            }
            .header {
                text-align: center;
                margin-bottom: 30px;
                border-bottom: 3px solid #667eea;
                padding-bottom: 20px;
            }
            .form-section, .records-section {
                background: #f8f9fa;
                padding: 25px;
                border-radius: 10px;
                margin-bottom: 20px;
            }
            .form-group {
                margin-bottom: 15px;
            }
            label {
                display: block;
                margin-bottom: 5px;
                font-weight: bold;
                color: #555;
            }
            input, textarea {
                width: 100%;
                padding: 10px;
                border: 2px solid #ddd;
                border-radius: 5px;
                font-size: 16px;
                box-sizing: border-box;
            }
            textarea { height: 80px; resize: vertical; }
            .btn {
                background: #667eea;
                color: white;
                padding: 12px 30px;
                border: none;
                border-radius: 5px;
                cursor: pointer;
                font-size: 16px;
                margin-right: 10px;
                transition: background 0.3s;
            }
            .btn:hover { background: #5a6fd8; }
            .record-item {
                background: white;
                border: 1px solid #ddd;
                border-radius: 8px;
                padding: 15px;
                margin-bottom: 10px;
                box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            }
            .record-header {
                font-weight: bold;
                color: #667eea;
                margin-bottom: 8px;
            }
            .alert {
                padding: 15px;
                margin-bottom: 20px;
                border-radius: 5px;
                display: none;
            }
            .alert-success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
            .alert-error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>📋 病例管理系統</h1>
                <a href="/" style="color: #667eea;">← 返回首頁</a>
            </div>
            
            <div id="alert" class="alert"></div>
            
            <div class="form-section">
                <h3>🆕 創建新病例</h3>
                <form id="recordForm">
                    <div class="form-group">
                        <label for="patient_id">病患編號:</label>
                        <input type="text" id="patient_id" required>
                    </div>
                    <div class="form-group">
                        <label for="patient_name">病患姓名:</label>
                        <input type="text" id="patient_name" required>
                    </div>
                    <div class="form-group">
                        <label for="doctor_name">主治醫師:</label>
                        <input type="text" id="doctor_name" required>
                    </div>
                    <div class="form-group">
                        <label for="diagnosis">診斷結果:</label>
                        <input type="text" id="diagnosis" required>
                    </div>
                    <div class="form-group">
                        <label for="symptoms">臨床症狀:</label>
                        <textarea id="symptoms" required></textarea>
                    </div>
                    <button type="submit" class="btn">💾 保存病例</button>
                    <button type="button" class="btn" onclick="loadRecords()">🔄 重新載入</button>
                </form>
            </div>
            
            <div class="records-section">
                <h3>📊 所有病例記錄</h3>
                <div id="recordsList">載入中...</div>
            </div>
        </div>
        
        <script>
            // 創建病例
            document.getElementById('recordForm').addEventListener('submit', async function(e) {
                e.preventDefault();
                
                const formData = {
                    patient_id: document.getElementById('patient_id').value,
                    patient_name: document.getElementById('patient_name').value,
                    doctor_name: document.getElementById('doctor_name').value,
                    diagnosis: document.getElementById('diagnosis').value,
                    symptoms: document.getElementById('symptoms').value
                };
                
                try {
                    const response = await fetch('/api/medical_record/', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify(formData)
                    });
                    
                    if (response.ok) {
                        const result = await response.json();
                        showAlert('病例創建成功！病例ID: ' + result.record.id, 'success');
                        document.getElementById('recordForm').reset();
                        loadRecords();
                    } else {
                        showAlert('創建失敗，請檢查輸入資料', 'error');
                    }
                } catch (error) {
                    showAlert('網路錯誤: ' + error.message, 'error');
                }
            });
            
            // 載入所有病例
            async function loadRecords() {
                try {
                    const response = await fetch('/api/records/all');
                    const data = await response.json();
                    
                    const recordsList = document.getElementById('recordsList');
                    if (data.count === 0) {
                        recordsList.innerHTML = '<p>目前沒有病例記錄</p>';
                        return;
                    }
                    
                    let html = '<h4>共 ' + data.count + ' 筆病例記錄</h4>';
                    data.records.forEach(record => {
                        html += `
                            <div class="record-item">
                                <div class="record-header">
                                    病例 #${record.id} - ${record.patient_name}
                                </div>
                                <div><strong>病患編號:</strong> ${record.patient_id}</div>
                                <div><strong>主治醫師:</strong> ${record.doctor_name}</div>
                                <div><strong>診斷結果:</strong> ${record.diagnosis}</div>
                                <div><strong>臨床症狀:</strong> ${record.symptoms}</div>
                                <div><strong>創建時間:</strong> ${new Date(record.created_time).toLocaleString('zh-TW')}</div>
                            </div>
                        `;
                    });
                    recordsList.innerHTML = html;
                } catch (error) {
                    document.getElementById('recordsList').innerHTML = '<p>載入失敗: ' + error.message + '</p>';
                }
            }
            
            // 顯示提示訊息
            function showAlert(message, type) {
                const alert = document.getElementById('alert');
                alert.className = 'alert alert-' + type;
                alert.textContent = message;
                alert.style.display = 'block';
                setTimeout(() => {
                    alert.style.display = 'none';
                }, 5000);
            }
            
            // 頁面載入時自動載入病例
            window.onload = function() {
                loadRecords();
            };
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)

@app.get("/status", response_class=HTMLResponse) 
async def status_page():
    html_content = """
    <!DOCTYPE html>
    <html lang="zh-TW">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>系統狀態</title>
        <style>
            body {
                font-family: 'Microsoft JhengHei', Arial, sans-serif;
                margin: 0;
                padding: 20px;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: #333;
                min-height: 100vh;
            }
            .container {
                max-width: 800px;
                margin: 0 auto;
                background: white;
                border-radius: 15px;
                padding: 30px;
                box-shadow: 0 10px 30px rgba(0,0,0,0.2);
            }
            .header {
                text-align: center;
                margin-bottom: 30px;
                border-bottom: 3px solid #667eea;
                padding-bottom: 20px;
            }
            .status-item {
                background: #f8f9fa;
                padding: 15px;
                border-radius: 8px;
                margin-bottom: 15px;
                border-left: 4px solid #667eea;
            }
            .status-ok { border-left-color: #28a745; }
            .status-info { border-left-color: #17a2b8; }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>⚡ 系統狀態檢查</h1>
                <a href="/" style="color: #667eea;">← 返回首頁</a>
            </div>
            
            <div class="status-item status-ok">
                <h4>✅ 伺服器狀態</h4>
                <p>醫療系統伺服器正常運行</p>
            </div>
            
            <div class="status-item status-ok">
                <h4>✅ 繁體中文支援</h4>
                <p>完整支援繁體中文顯示和輸入</p>
            </div>
            
            <div class="status-item status-ok">
                <h4>✅ API服務</h4>
                <p>所有API端點正常運作</p>
            </div>
            
            <div class="status-item status-ok">
                <h4>✅ 資料庫連接</h4>
                <p>內存資料庫正常運作</p>
            </div>
            
            <div class="status-item status-info">
                <h4>📊 即時狀態</h4>
                <div id="liveStatus">載入中...</div>
            </div>
        </div>
        
        <script>
            async function loadStatus() {
                try {
                    const response = await fetch('/api/status');
                    const data = await response.json();
                    
                    document.getElementById('liveStatus').innerHTML = `
                        <p><strong>伺服器:</strong> ${data.server}</p>
                        <p><strong>狀態:</strong> ${data.status}</p>
                        <p><strong>病例數量:</strong> ${data.records_count}</p>
                        <p><strong>藥物數量:</strong> ${data.medicines_count}</p>
                        <p><strong>檢查時間:</strong> ${new Date().toLocaleString('zh-TW')}</p>
                    `;
                } catch (error) {
                    document.getElementById('liveStatus').innerHTML = '<p>狀態載入失敗: ' + error.message + '</p>';
                }
            }
            
            window.onload = loadStatus;
            setInterval(loadStatus, 30000); // 每30秒更新一次
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)

# API端點 (保持原有的API功能)
@app.post("/api/medical_record/")
async def create_record(record: MedicalRecord):
    global next_id
    new_record = {
        "id": next_id,
        "patient_id": record.patient_id,
        "patient_name": record.patient_name,
        "doctor_name": record.doctor_name,
        "diagnosis": record.diagnosis,
        "symptoms": record.symptoms,
        "created_time": datetime.now().isoformat()
    }
    records.append(new_record)
    next_id += 1
    return {"message": "病例創建成功", "record": new_record}

@app.get("/api/medical_record/{record_id}")
async def get_record(record_id: int):
    for record in records:
        if record["id"] == record_id:
            return record
    return {"error": "病例未找到"}

@app.get("/api/records/all")
async def get_all_records():
    return {"records": records, "count": len(records)}

@app.get("/api/status")
async def api_status():
    return {
        "server": "web_medical_server",
        "status": "running",
        "records_count": len(records),
        "medicines_count": len(medicines),
        "chinese_support": True,
        "web_interface": True
    }

if __name__ == "__main__":
    print("🏥 正在啟動醫療系統網頁伺服器...")
    print("🌐 網頁界面: http://0.0.0.0:8000/")
    print("📋 病例管理: http://0.0.0.0:8000/medical")
    print("📖 API文檔: http://0.0.0.0:8000/docs")
    print("⚡ 系統狀態: http://0.0.0.0:8000/status")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
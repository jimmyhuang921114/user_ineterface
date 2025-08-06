#!/bin/bash

echo "🏥 啟動簡化醫院藥物管理系統"
echo "================================"

# 檢查Python是否安裝
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 未安裝，請先安裝Python3"
    exit 1
fi

# 檢查是否在正確的目錄
if [ ! -f "user_interface/main.py" ]; then
    echo "❌ 請在專案根目錄執行此腳本"
    exit 1
fi

# 安裝依賴
echo "📦 安裝依賴..."
pip3 install --break-system-packages -r requirements_simple.txt

# 初始化資料庫
echo "🗄️ 初始化資料庫..."
cd user_interface
python3 database.py

# 啟動系統
echo "🚀 啟動系統..."
echo "📖 API文檔: http://localhost:8000/docs"
echo "🌐 藥物管理: http://localhost:8000/Medicine.html"
echo "📋 處方籤管理: http://localhost:8000/Prescription.html"
echo "👨‍⚕️ 醫生工作站: http://localhost:8000/doctor.html"
echo "🤖 ROS2整合: 模擬模式（如需真實ROS2請安裝rclpy）"
echo "=" * 50

python3 main.py
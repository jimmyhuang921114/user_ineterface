#!/bin/bash

echo "🏥 啟動醫院藥物管理系統"
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
pip3 install -r requirements_simple.txt

# 初始化資料庫
echo "🗄️ 初始化資料庫..."
cd user_interface
python3 database.py

# 啟動系統
echo "🚀 啟動系統..."
python3 main.py
#!/bin/bash

echo "🏥 醫院藥物管理系統啟動腳本"
echo "================================"

# 殺死現有進程
echo "🔄 清理現有進程..."
pkill -f uvicorn 2>/dev/null || true
pkill -f "python.*server" 2>/dev/null || true
sleep 2

# 設置Python路徑
export PATH=$PATH:/home/ubuntu/.local/bin

# 啟動伺服器
echo "🚀 啟動伺服器..."
cd /workspace/user_interface

echo "使用最終版本伺服器..."
python3 final_server.py &
SERVER_PID=$!

echo "伺服器PID: $SERVER_PID"
echo "等待伺服器啟動..."
sleep 5

# 測試連接
echo "🧪 測試API連接..."
curl -s http://localhost:8000/api/test || echo "❌ API連接失敗"

echo "✅ 伺服器啟動完成"
echo "🌐 訪問: http://localhost:8000"
echo "📱 API文檔: http://localhost:8000/docs"
echo "💊 藥物管理: http://localhost:8000/Medicine.html"

# 保持腳本運行
wait $SERVER_PID
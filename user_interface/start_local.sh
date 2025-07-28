#!/bin/bash

echo "醫院藥物管理系統啟動腳本"
echo "================================"

# 獲取當前腳本所在目錄
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "當前目錄: $SCRIPT_DIR"

# 殺死現有進程
echo "清理現有進程..."
pkill -f uvicorn 2>/dev/null || true
pkill -f "python.*server" 2>/dev/null || true
sleep 2

# 檢查Python
if ! command -v python3 &> /dev/null; then
    echo "錯誤: 找不到 python3，請先安裝 Python"
    exit 1
fi

echo "Python版本: $(python3 --version)"

# 檢查依賴
echo "檢查依賴..."
python3 -c "import fastapi, uvicorn, pydantic" 2>/dev/null || {
    echo "安裝缺失的依賴..."
    pip3 install fastapi uvicorn pydantic requests
}

# 進入腳本目錄
cd "$SCRIPT_DIR"

# 檢查伺服器檔案
if [[ -f "enhanced_server.py" ]]; then
    SERVER_FILE="enhanced_server.py"
    echo "使用增強版伺服器..."
elif [[ -f "final_server.py" ]]; then
    SERVER_FILE="final_server.py"
    echo "使用最終版本伺服器..."
elif [[ -f "hospital_server.py" ]]; then
    SERVER_FILE="hospital_server.py"
    echo "使用醫院伺服器..."
else
    echo "錯誤: 找不到伺服器檔案"
    exit 1
fi

# 啟動伺服器
echo "啟動伺服器..."
python3 "$SERVER_FILE" &
SERVER_PID=$!

echo "伺服器PID: $SERVER_PID"
echo "等待伺服器啟動..."
sleep 5

# 測試連接
echo "測試API連接..."
if curl -s http://localhost:8000/api/test > /dev/null 2>&1; then
    echo "成功: API連接正常"
else
    echo "警告: API連接測試失敗，但伺服器可能仍在啟動中"
fi

echo "================================"
echo "成功: 伺服器啟動完成"
echo "訪問地址："
echo "  主頁: http://localhost:8000"
echo "  API文檔: http://localhost:8000/docs"
echo "  藥物管理: http://localhost:8000/Medicine.html"
echo "  病人管理: http://localhost:8000/Patients.html"
echo "  病例記錄: http://localhost:8000/Records.html"
echo "================================"
echo "按 Ctrl+C 停止伺服器"

# 保持腳本運行
wait $SERVER_PID
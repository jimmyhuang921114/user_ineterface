# 快速啟動指南 - 修復路徑問題

## 問題解決

你遇到的問題是路徑設定錯誤。以下是修復方法：

### 方法1：使用新的本地啟動腳本（推薦）

```bash
# 使用Python版本（最可靠）
python3 start_local.py

# 或使用Shell版本
chmod +x start_local.sh
./start_local.sh
```

### 方法2：直接啟動伺服器

```bash
# 在你的 user_interface 目錄中執行
python3 enhanced_server.py

# 或者
python3 final_server.py
```

### 方法3：修復原始的start.sh

編輯 `start.sh` 檔案，將第17行：
```bash
cd /workspace/user_interface
```
改為：
```bash
cd "$(dirname "$0")"
```

## 確認你的目錄結構

你應該在這個目錄中：
```
~/user_ineterface/user_interface/
├── enhanced_server.py
├── final_server.py
├── start_local.py      # 新的啟動腳本
├── start_local.sh      # 新的Shell腳本
└── 其他檔案...
```

## 快速測試

1. **檢查檔案是否存在**：
   ```bash
   ls -la *.py
   ```

2. **檢查Python**：
   ```bash
   python3 --version
   ```

3. **安裝依賴**：
   ```bash
   pip3 install fastapi uvicorn pydantic requests
   ```

4. **直接啟動**：
   ```bash
   python3 enhanced_server.py
   ```

## 成功後訪問

打開瀏覽器訪問：
- http://localhost:8000
- http://localhost:8000/Medicine.html

## 如果還有問題

1. **檢查端口**：
   ```bash
   lsof -i :8000
   ```

2. **殺死佔用的進程**：
   ```bash
   pkill -f uvicorn
   pkill -f python.*server
   ```

3. **檢查錯誤訊息**：
   ```bash
   python3 enhanced_server.py
   # 查看詳細錯誤訊息
   ```
#!/usr/bin/env python3
"""
簡單的伺服器測試腳本
"""

import requests
import time
import subprocess

def test_connection():
    """測試各種連接方式"""
    print("測試伺服器連接...")
    
    tests = [
        ("主頁", "http://localhost:8000/"),
        ("API測試", "http://localhost:8000/api/test"),
        ("藥物API", "http://localhost:8000/api/medicine/"),
        ("文檔", "http://localhost:8000/docs"),
    ]
    
    for name, url in tests:
        try:
            response = requests.get(url, timeout=5)
            print(f"  {name}: {response.status_code} - {'成功' if response.status_code == 200 else '失敗'}")
            if response.status_code == 200:
                print(f"    內容長度: {len(response.text)} 字符")
        except requests.exceptions.RequestException as e:
            print(f"  {name}: 連接失敗 - {e}")

def check_processes():
    """檢查相關進程"""
    print("\n檢查運行中的進程...")
    try:
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
        lines = result.stdout.split('\n')
        
        server_processes = []
        for line in lines:
            if any(keyword in line for keyword in ['python', 'uvicorn', 'server.py']):
                if any(server in line for server in ['enhanced_server', 'final_server', 'hospital_server']):
                    server_processes.append(line.strip())
        
        if server_processes:
            print("  找到伺服器進程:")
            for proc in server_processes:
                print(f"    {proc}")
        else:
            print("  沒有找到伺服器進程")
            
    except Exception as e:
        print(f"  檢查進程失敗: {e}")

def check_port():
    """檢查端口"""
    print("\n檢查端口狀態...")
    try:
        result = subprocess.run(['lsof', '-i', ':8000'], capture_output=True, text=True)
        if result.stdout.strip():
            print("  端口8000被以下進程佔用:")
            print(f"    {result.stdout.strip()}")
        else:
            print("  端口8000沒有被佔用")
    except Exception as e:
        print(f"  檢查端口失敗: {e}")

def main():
    """主函數"""
    print("醫院藥物管理系統 - 簡單測試")
    print("=" * 40)
    
    check_processes()
    check_port()
    test_connection()
    
    print("\n建議:")
    print("1. 如果沒有進程在運行，執行: python3 enhanced_server.py")
    print("2. 如果端口被佔用，執行: pkill -f uvicorn")
    print("3. 如果連接失敗，等待10-15秒後重試")

if __name__ == "__main__":
    main()
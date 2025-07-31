#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import json
import os

print("=" * 50)
print("繁體中文顯示測試")
print("Traditional Chinese Display Test")
print("=" * 50)

# 檢查編碼
print(f"\n1. 系統編碼檢查:")
print(f"   Python stdout encoding: {sys.stdout.encoding}")
print(f"   系統預設編碼: {sys.getdefaultencoding()}")
print(f"   檔案系統編碼: {sys.getfilesystemencoding()}")

# 測試各種中文字符
print(f"\n2. 繁體中文字符測試:")
test_strings = [
    "醫院管理系統",
    "病患資料",
    "藥物資訊", 
    "處方箋",
    "診斷報告",
    "測試成功！",
    "繁體中文顯示正常"
]

for i, text in enumerate(test_strings, 1):
    print(f"   {i}. {text}")

# JSON 中文測試
print(f"\n3. JSON 繁體中文測試:")
test_data = {
    "病患姓名": "陳小明",
    "醫師": "李醫師", 
    "診斷": "高血壓",
    "症狀": "頭痛、眩暈",
    "藥物": "心律錠",
    "劑量": "10毫克",
    "注意事項": "飯後服用"
}

print(json.dumps(test_data, ensure_ascii=False, indent=2))

# 環境變數檢查
print(f"\n4. 環境變數檢查:")
lang_vars = ['LANG', 'LC_ALL', 'LC_CTYPE', 'LANGUAGE']
for var in lang_vars:
    value = os.environ.get(var, '未設定')
    print(f"   {var}: {value}")

# 設定繁體中文環境（如果需要）
print(f"\n5. 繁體中文環境設定建議:")
print("   如果顯示有問題，可以嘗試:")
print("   export LANG=zh_TW.UTF-8")
print("   export LC_ALL=zh_TW.UTF-8")

print(f"\n6. 醫療系統專用詞彙測試:")
medical_terms = [
    "藥物管理", "病例記錄", "處方開立", 
    "診療紀錄", "病患資訊", "醫療資料",
    "藥品查詢", "處方狀態", "完成回報"
]

for term in medical_terms:
    print(f"   ✓ {term}")

print("\n" + "=" * 50)
print("測試完成 - Test Completed")
print("如果您能看到上面所有的繁體中文，代表顯示正常")
print("=" * 50)
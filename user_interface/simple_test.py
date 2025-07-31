#!/usr/bin/env python3

print("Hello, 這是一個簡單的測試！")
print("Current directory test working!")

import sys
print(f"Python version: {sys.version}")

try:
    import requests
    print("requests module available")
except ImportError:
    print("requests module not available")

try:
    import fastapi
    print("fastapi module available")
except ImportError:
    print("fastapi module not available")

print("Test completed!")
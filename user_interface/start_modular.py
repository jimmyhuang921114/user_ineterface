#!/usr/bin/env python3
"""
 -
Hospital Management System - Modular Architecture Startup Script
"""

import subprocess
import sys
import os
import time
import signal
from pathlib import Path

def check_dependencies():
    """"""
    print(" ...")

    required_packages = ['fastapi', 'uvicorn', 'pydantic']
    missing_packages = []

    for package in required_packages:
        try:
            __import__(package)
            print(f"    {package}")
        except ImportError:
            missing_packages.append(package)
            print(f"    {package} ()")

    if missing_packages:
        print(f"\n  : {', '.join(missing_packages)}")
        print(": pip install fastapi uvicorn pydantic")
        return False

    print(" ")
    return True

def cleanup_old_processes():
    """"""
    print(" ...")
    try:
        #  uvicorn
        subprocess.run(['pkill', '-f', 'uvicorn'], capture_output=True)
        subprocess.run(['pkill', '-f', 'modular_server'], capture_output=True)
        time.sleep(2)
        print(" ")
    except Exception as e:
        print(f"  : {e}")

def start_server():
    """"""
    print(" ...")

    #
    script_dir = Path(__file__).parent
    os.chdir(script_dir)

    try:
        #
        process = subprocess.Popen([
            sys.executable, '-m', 'uvicorn',
            'modular_server:app',
            '--host', '0.0.0.0',
            '--port', '8000',
            '--reload'
        ])

        print(f" ... (PID: {process.pid})")
        return process

    except Exception as e:
        print(f" : {e}")
        return None

def test_api_connection():
    """API"""
    print(" API...")

    import requests
    import time

    api_base = "http://localhost:8000"
    max_attempts = 10

    for attempt in range(1, max_attempts + 1):
        try:
            response = requests.get(f"{api_base}/api/system/status", timeout=5)
            if response.status_code == 200:
                data = response.json()
                print(f" API!")
                print(f"   : {data.get('system', 'Unknown')}")
                print(f"   : {data.get('version', 'Unknown')}")
                print(f"   : {data.get('architecture', 'Unknown')}")
                return True
            else:
                print(f"    {attempt}/{max_attempts}: HTTP {response.status_code}")
        except requests.exceptions.RequestException as e:
            print(f"    {attempt}/{max_attempts}: ...")

        if attempt < max_attempts:
            time.sleep(3)

    print("  API")
    return False

def print_access_info():
    """"""
    print("\n" + "=" * 60)
    print("  - ")
    print("=" * 60)
    print(" :")
    print("    :         http://localhost:8000")
    print("    :   http://localhost:8000/doctor.html")
    print("    :     http://localhost:8000/Medicine.html")
    print("    :     http://localhost:8000/Prescription.html")
    print("    API:      http://localhost:8000/docs")
    print("\n :")
    print("    API - API")
    print("    ")
    print("    ")
    print("    ")
    print("=" * 60)
    print(" :")
    print("   1. ")
    print("   2. ")
    print("   3. ")
    print("   4.  Ctrl+C ")
    print("=" * 60)

def main():
    """"""
    print("  - ")
    print("=" * 50)

    #
    if not check_dependencies():
        sys.exit(1)

    #
    cleanup_old_processes()

    #
    server_process = start_server()
    if not server_process:
        sys.exit(1)

    #
    time.sleep(5)

    #
    test_api_connection()

    #
    print_access_info()

    #
    try:
        print("\n⌨   Ctrl+C ...")
        server_process.wait()
    except KeyboardInterrupt:
        print("\n ...")
        server_process.terminate()
        try:
            server_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server_process.kill()
        print(" ")

if __name__ == "__main__":
    main()
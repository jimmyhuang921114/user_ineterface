#!/usr/bin/env python3
"""
 -
Hospital Medicine Management System - Local Startup Script
"""

import os
import sys
import subprocess
import time
import signal
import requests
from pathlib import Path

def check_dependencies():
    """"""
    print("...")

    required_packages = ['fastapi', 'uvicorn', 'pydantic']
    missing_packages = []

    for package in required_packages:
        try:
            __import__(package)
            print(f"  : {package}")
        except ImportError:
            print(f"  : {package}")
            missing_packages.append(package)

    if missing_packages:
        print(f"\n: {', '.join(missing_packages)}")
        try:
            subprocess.check_call([
                sys.executable, '-m', 'pip', 'install',
                '--user', *missing_packages, 'requests'
            ])
            print(": ")
        except subprocess.CalledProcessError:
            print(": ")
            print(f"pip3 install {' '.join(missing_packages)} requests")
            return False

    return True

def find_server_file():
    """"""
    current_dir = Path(__file__).parent

    server_files = [
        ('enhanced_server.py', ''),
        ('hospital_server.py', ''),
        ('final_server.py', ''),
        ('working_server.py', ''),
        ('main.py', '')
    ]

    for filename, description in server_files:
        file_path = current_dir / filename
        if file_path.exists():
            print(f": {description}")
            return file_path

    print(": ")
    return None

def kill_existing_servers():
    """"""
    print("...")
    try:
        # uvicorn
        subprocess.run(['pkill', '-f', 'uvicorn'],
                      capture_output=True, check=False)
        subprocess.run(['pkill', '-f', 'python.*server'],
                      capture_output=True, check=False)
        time.sleep(2)
    except Exception:
        pass

def test_server():
    """"""
    max_attempts = 10
    for attempt in range(max_attempts):
        try:
            response = requests.get('http://localhost:8000/api/test', timeout=2)
            if response.status_code == 200:
                return True
        except requests.exceptions.RequestException:
            pass

        if attempt < max_attempts - 1:
            print(f"   {attempt + 1}/{max_attempts}...")
            time.sleep(1)

    return False

def main():
    """"""
    print(" - ")
    print("=" * 50)

    #
    current_dir = Path(__file__).parent
    print(f": {current_dir}")

    #
    os.chdir(current_dir)

    #
    if not check_dependencies():
        return False

    #
    kill_existing_servers()

    #
    server_file = find_server_file()
    if not server_file:
        return False

    #
    print(f"\n: {server_file.name}")
    try:
        #
        process = subprocess.Popen([
            sys.executable, str(server_file)
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        print(f"PID: {process.pid}")
        print("...")

        #
        time.sleep(3)

        #
        print("API...")
        if test_server():
            print(": API")
        else:
            print(": API")

        #
        print("\n" + "=" * 50)
        print(": ")
        print("\n:")
        print("  : http://localhost:8000")
        print("  API: http://localhost:8000/docs")
        print("  : http://localhost:8000/Medicine.html")
        print("  : http://localhost:8000/Patients.html")
        print("  : http://localhost:8000/Records.html")
        print("=" * 50)
        print(" Ctrl+C ")

        #
        try:
            process.wait()
        except KeyboardInterrupt:
            print("\n...")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
            print("")

        return True

    except Exception as e:
        print(f": : {e}")
        return False

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n")
        sys.exit(0)
    except Exception as e:
        print(f": {e}")
        sys.exit(1)
#!/usr/bin/env python3
"""

Quick Test Hospital Management System
"""

import requests
import json
from datetime import datetime

def test_system():
    """"""
    API_BASE = "http://localhost:8000"

    print("  - ")
    print("=" * 50)

    # 1: API
    print("1. API...")
    try:
        response = requests.get(f"{API_BASE}/api/test", timeout=5)
        if response.status_code == 200:
            print(" API")
            print(f"   : {response.json()}")
        else:
            print(f" API: {response.status_code}")
    except Exception as e:
        print(f" API: {e}")
        print(": python3 fixed_server.py")
        return

    # 2:
    print("\n2. ...")
    medicine_data = {
        "name": "_Doctor",
        "amount": 50,
        "usage_days": 14,
        "position": "DOC-01"
    }

    try:
        response = requests.post(f"{API_BASE}/api/medicine/",
                               json=medicine_data, timeout=5)
        if response.status_code == 200:
            result = response.json()
            print(f" : ID={result['id']}")
            medicine_id = result['id']
        else:
            print(f" : {response.status_code}")
            medicine_id = None
    except Exception as e:
        print(f" : {e}")
        medicine_id = None

    # 3:
    print("\n3. ...")
    detailed_data = {
        "medicine_name": "_Doctor",
        "medicine_data": {
            "": {
                "": "_Doctor",
                "": "",
                "": "10",
                "": ""
            },
            "": {
                "": "",
                "": ""
            },
            "": {
                "1": "DOC001",
                "2": "TEST002"
            },
            "": "",
            "": "",
            "": "",
            "": ""
        }
    }

    try:
        response = requests.post(f"{API_BASE}/api/medicine/detailed/",
                               json=detailed_data, timeout=5)
        if response.status_code == 200:
            print(" ")
        else:
            print(f" : {response.status_code}")
    except Exception as e:
        print(f" : {e}")

    # 4:
    print("\n4. ...")
    prescription_data = {
        "patient_name": "",
        "doctor_name": "",
        "diagnosis": "",
        "medicines": [{
            "medicine_name": "_Doctor",
            "dosage": "10mg",
            "frequency": "",
            "duration": "7",
            "instructions": ""
        }]
    }

    try:
        response = requests.post(f"{API_BASE}/api/prescription/",
                               json=prescription_data, timeout=5)
        if response.status_code == 200:
            result = response.json()
            print(f" : ID={result['id']}")
        else:
            print(f" : {response.status_code}")
    except Exception as e:
        print(f" : {e}")

    # 5:
    print("\n5. ...")
    try:
        #
        response = requests.get(f"{API_BASE}/api/medicine/", timeout=5)
        if response.status_code == 200:
            medicines = response.json()
            print(f" : {len(medicines)}")

        #
        response = requests.get(f"{API_BASE}/api/medicine/detailed/", timeout=5)
        if response.status_code == 200:
            detailed = response.json()
            print(f" : {len(detailed)}")

        #
        response = requests.get(f"{API_BASE}/api/prescription/", timeout=5)
        if response.status_code == 200:
            prescriptions = response.json()
            print(f" : {len(prescriptions)}")

    except Exception as e:
        print(f" : {e}")

    print("\n" + "=" * 50)
    print(" ")
    print("\n :")
    print("1. : http://localhost:8000/doctor.html")
    print("2. : http://localhost:8000/Medicine.html")
    print("3. : http://localhost:8000/Prescription.html")
    print("\n Doctor:")
    print("- : ")
    print("- : ")
    print("- : ")
    print("\n :")
    print("-  Medicine ")
    print("-  Medicine ")
    print("-  Prescription ")

if __name__ == "__main__":
    test_system()
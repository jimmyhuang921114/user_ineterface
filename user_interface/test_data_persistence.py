#!/usr/bin/env python3
"""

Data Persistence Test Script
"""

import requests
import json
import time
from datetime import datetime

API_BASE = "http://localhost:8000"

def test_data_persistence():
    """"""
    print("  - ")
    print("=" * 60)

    # 1.
    print(" ...")
    try:
        response = requests.get(f"{API_BASE}/api/system/status")
        if response.status_code == 200:
            status = response.json()
            print(f"   : {status.get('system')}")
            print(f"   : {status.get('version')}")
            print(f"   : {status.get('persistence')}")

            stats = status.get('statistics', {})
            print(f"   :  {stats.get('total_medicines', 0)},  {stats.get('total_prescriptions', 0)}")
        else:
            print(f" : {response.status_code}")
            return
    except Exception as e:
        print(f" : {e}")
        return

    # 2.
    print("\n ...")
    test_medicine = {
        "name": "_",
        "amount": 100,
        "usage_days": 7,
        "position": "TEST-01"
    }

    try:
        response = requests.post(f"{API_BASE}/api/medicine/", json=test_medicine)
        if response.status_code == 200:
            medicine_result = response.json()
            medicine_id = medicine_result['id']
            print(f"    ID: {medicine_id}")
        else:
            print(f"    : {response.status_code}")
            return
    except Exception as e:
        print(f"    : {e}")
        return

    # 3.
    print("\n ...")
    detailed_data = {
        "medicine_name": "_",
        "medicine_data": {
            "": {
                "": "_",
                "": "",
                "": ""
            },
            "": {
                "": "",
                "": ""
            },
            "": "",
            "": datetime.now().isoformat(),
            "": "DATA_PERSISTENCE_TEST"
        }
    }

    try:
        response = requests.post(f"{API_BASE}/api/medicine/detailed/", json=detailed_data)
        if response.status_code == 200:
            print("    ")
        else:
            print(f"    : {response.status_code}")
    except Exception as e:
        print(f"    : {e}")

    # 4.
    print("\n ...")
    test_prescription = {
        "patient_name": "_",
        "doctor_name": "_",
        "diagnosis": "",
        "medicines": [
            {
                "medicine_name": "_",
                "dosage": "",
                "frequency": "",
                "duration": "",
                "instructions": ""
            }
        ],
        "prescription_date": datetime.now().date().isoformat()
    }

    try:
        response = requests.post(f"{API_BASE}/api/prescription/", json=test_prescription)
        if response.status_code == 200:
            prescription_result = response.json()
            prescription_id = prescription_result['id']
            print(f"    ID: {prescription_id}")
        else:
            print(f"    : {response.status_code}")
    except Exception as e:
        print(f"    : {e}")

    # 5.
    print("\n ...")
    try:
        response = requests.post(f"{API_BASE}/api/system/save")
        if response.status_code == 200:
            save_result = response.json()
            if save_result.get('success'):
                print("    ")
                print(f"   : {save_result.get('timestamp')}")
            else:
                print(f"    : {save_result.get('message')}")
        else:
            print(f"    : {response.status_code}")
    except Exception as e:
        print(f"    : {e}")

    # 6.
    print("\n ...")
    try:
        response = requests.post(f"{API_BASE}/api/system/backup")
        if response.status_code == 200:
            backup_result = response.json()
            if backup_result.get('success'):
                print("    ")
                print(f"   : {backup_result.get('backup_path')}")
            else:
                print(f"    : {backup_result.get('message')}")
        else:
            print(f"    : {response.status_code}")
    except Exception as e:
        print(f"    : {e}")

    # 7.
    print("\n ...")
    try:
        response = requests.get(f"{API_BASE}/api/system/backups")
        if response.status_code == 200:
            backups_result = response.json()
            backups = backups_result.get('backups', [])
            print(f"     {len(backups)} :")

            for backup in backups[:3]:  # 3
                print(f"     - {backup['name']} ({backup['files']} )")
                print(f"       : {backup['created_time']}")
        else:
            print(f"    : {response.status_code}")
    except Exception as e:
        print(f"    : {e}")

    # 8.
    print("\n ...")
    try:
        response = requests.get(f"{API_BASE}/api/system/status")
        if response.status_code == 200:
            final_status = response.json()
            final_stats = final_status.get('statistics', {})

            print(f"   :")
            print(f"     - : {final_stats.get('total_medicines', 0)}")
            print(f"     - : {final_stats.get('detailed_medicines', 0)}")
            print(f"     - : {final_stats.get('total_prescriptions', 0)}")

            #
            data_files = final_status.get('data_files', {}).get('files', {})
            print(f"   :")
            for file_name, file_info in data_files.items():
                if file_info.get('exists'):
                    size_kb = file_info.get('size_bytes', 0) / 1024
                    print(f"     - {file_name}: {size_kb:.1f} KB (: {file_info.get('modified_time', 'Unknown')[:19]})")
                else:
                    print(f"     - {file_name}: ")
        else:
            print(f"    : {response.status_code}")
    except Exception as e:
        print(f"    : {e}")

    print("\n" + "=" * 60)
    print(" ")
    print("\n :")
    print("    ")
    print("    ")
    print("    ")
    print("    ")
    print("    JSON")
    print("\n :")
    print("   1. ")
    print("   2. ")
    print("   3. ")

if __name__ == "__main__":
    test_data_persistence()
#!/usr/bin/env python3
"""

Test Script for Patient Creation in Prescription System

"""

import requests
import json
import time
from datetime import datetime

API_BASE = "http://localhost:8000"

def print_section(title):
    """"""
    print("\n" + "=" * 60)
    print(f" {title}")
    print("=" * 60)

def test_prescription_creation_with_patient():
    """"""
    print_section("")

    #
    test_patients = [
        {
            "patient_name": "",
            "doctor_name": "",
            "diagnosis": "",
            "medicines": [
                {
                    "medicine_name": "",
                    "dosage": "500mg",
                    "frequency": "8",
                    "duration": "3",
                    "instructions": ""
                }
            ]
        },
        {
            "patient_name": "",
            "doctor_name": "",
            "diagnosis": "",
            "medicines": [
                {
                    "medicine_name": "",
                    "dosage": "10mg",
                    "frequency": "",
                    "duration": "7",
                    "instructions": ""
                },
                {
                    "medicine_name": "",
                    "dosage": "1",
                    "frequency": "",
                    "duration": "5",
                    "instructions": ""
                }
            ]
        },
        {
            "patient_name": "",
            "doctor_name": "",
            "diagnosis": "",
            "medicines": [
                {
                    "medicine_name": "",
                    "dosage": "5mg",
                    "frequency": "",
                    "duration": "30",
                    "instructions": ""
                }
            ]
        }
    ]

    created_prescriptions = []

    # 1.
    print("\n ...")
    try:
        response = requests.get(f"{API_BASE}/api/prescription/")
        if response.status_code == 200:
            before_count = len(response.json())
            print(f"   : {before_count}")
        else:
            print("   ")
            before_count = 0
    except Exception as e:
        print(f"   : {e}")
        before_count = 0

    # 2.
    print("\n ...")
    for i, patient_data in enumerate(test_patients, 1):
        print(f"\n   {i}.  {patient_data['patient_name']} ...")
        print(f"      : {patient_data['doctor_name']}")
        print(f"      : {patient_data['diagnosis']}")
        print(f"      : {len(patient_data['medicines'])}")

        try:
            #
            patient_data['prescription_date'] = datetime.now().date().isoformat()

            response = requests.post(
                f"{API_BASE}/api/prescription/",
                json=patient_data,
                headers={"Content-Type": "application/json"}
            )

            if response.status_code == 200:
                result = response.json()
                created_prescriptions.append(result)
                print(f"       ! : {result['id']}")
                print(f"       : {result.get('created_time', 'Unknown')}")
            else:
                print(f"       : {response.status_code}")
                print(f"      : {response.text}")

        except Exception as e:
            print(f"       : {e}")

        #
        time.sleep(1)

    # 3.
    print("\n ...")
    try:
        response = requests.get(f"{API_BASE}/api/prescription/")
        if response.status_code == 200:
            after_prescriptions = response.json()
            after_count = len(after_prescriptions)
            print(f"   : {after_count}")
            print(f"   : {after_count - before_count}")

            #
            if after_prescriptions:
                print("\n :")
                latest_prescriptions = sorted(after_prescriptions,
                                            key=lambda x: x.get('created_time', ''),
                                            reverse=True)[:3]

                for prescription in latest_prescriptions:
                    print(f"   #{prescription['id']}: {prescription['patient_name']} - {prescription['doctor_name']}")
                    print(f"     : {prescription.get('diagnosis', '')}")
                    print(f"     : {prescription.get('status', 'unknown')}")
                    print(f"     : {len(prescription.get('medicines', []))}")
        else:
            print("   ")

    except Exception as e:
        print(f"   : {e}")

    # 4.  ()
    print("\n ...")
    try:
        # API
        patient_endpoints = [
            "/api/patients/",
            "/api/patient/",
            "/api/records/",
            "/api/medical_records/"
        ]

        found_patient_system = False
        for endpoint in patient_endpoints:
            try:
                response = requests.get(f"{API_BASE}{endpoint}")
                if response.status_code == 200:
                    patients = response.json()
                    print(f"    : {endpoint}")
                    print(f"    : {len(patients) if isinstance(patients, list) else ''}")
                    found_patient_system = True

                    #
                    if isinstance(patients, list):
                        print("\n    :")
                        for patient in patients[-5:]:  # 5
                            if isinstance(patient, dict):
                                name = patient.get('name', patient.get('patient_name', ''))
                                print(f"     - {name}")
                    break

            except Exception:
                continue

        if not found_patient_system:
            print("     ")
            print("    ")

    except Exception as e:
        print(f"   : {e}")

    return created_prescriptions

def test_prescription_status_flow():
    """"""
    print_section("")

    try:
        #
        response = requests.get(f"{API_BASE}/api/prescription/")
        if response.status_code != 200:
            print(" ")
            return

        prescriptions = response.json()
        if not prescriptions:
            print("  ")
            return

        #
        test_prescription = prescriptions[0]
        prescription_id = test_prescription['id']

        print(f" : #{prescription_id} - {test_prescription['patient_name']}")
        print(f"   : {test_prescription.get('status', 'unknown')}")

        #
        status_flow = [
            ("pending", ""),
            ("processing", ""),
            ("completed", "")
        ]

        for status, status_zh in status_flow:
            print(f"\n : {status_zh} ({status})")

            update_data = {
                "status": status,
                "updated_by": "",
                "notes": f": {status_zh}"
            }

            try:
                response = requests.put(
                    f"{API_BASE}/api/prescription/{prescription_id}/status",
                    json=update_data,
                    headers={"Content-Type": "application/json"}
                )

                if response.status_code == 200:
                    result = response.json()
                    print(f"    : {result.get('new_status', status)}")
                else:
                    print(f"    : {response.status_code}")
                    print(f"   : {response.text}")

                time.sleep(1)

            except Exception as e:
                print(f"    : {e}")

        #
        try:
            response = requests.get(f"{API_BASE}/api/prescription/{prescription_id}")
            if response.status_code == 200:
                final_data = response.json()
                prescription = final_data.get('prescription', final_data)
                print(f"\n : {prescription.get('status', 'unknown')}")

                #
                status_history = final_data.get('status_history', [])
                if status_history:
                    print("\n :")
                    for record in status_history[-3:]:  # 3
                        print(f"   {record.get('updated_time', 'Unknown')}: {record.get('status', 'unknown')} by {record.get('updated_by', 'Unknown')}")

        except Exception as e:
            print(f" : {e}")

    except Exception as e:
        print(f" : {e}")

def generate_test_report():
    """"""
    print_section("")

    try:
        #
        response = requests.get(f"{API_BASE}/api/system/status")
        if response.status_code == 200:
            system_status = response.json()
            print(" :")
            print(f"   : {system_status.get('system', 'Unknown')}")
            print(f"   : {system_status.get('version', 'Unknown')}")
            print(f"   : {system_status.get('architecture', 'Unknown')}")

            stats = system_status.get('statistics', {})
            print(f"\n :")
            print(f"   : {stats.get('total_medicines', 0)}")
            print(f"   : {stats.get('detailed_medicines', 0)}")
            print(f"   : {stats.get('total_prescriptions', 0)}")

        #
        response = requests.get(f"{API_BASE}/api/prescription/")
        if response.status_code == 200:
            prescriptions = response.json()

            status_counts = {}
            doctor_counts = {}

            for prescription in prescriptions:
                status = prescription.get('status', 'unknown')
                doctor = prescription.get('doctor_name', 'Unknown')

                status_counts[status] = status_counts.get(status, 0) + 1
                doctor_counts[doctor] = doctor_counts.get(doctor, 0) + 1

            print(f"\n :")
            for status, count in status_counts.items():
                print(f"   {status}: {count}")

            print(f"\n‍ :")
            for doctor, count in doctor_counts.items():
                print(f"   {doctor}: {count}")

    except Exception as e:
        print(f" : {e}")

def main():
    """"""
    print("  - ")
    print("=" * 60)
    print("")

    # API
    try:
        response = requests.get(f"{API_BASE}/api/system/status", timeout=5)
        if response.status_code == 200:
            print(" API")
        else:
            print(f"  API: {response.status_code}")
    except Exception as e:
        print(f" API: {e}")
        print("")
        return

    #
    created_prescriptions = test_prescription_creation_with_patient()
    time.sleep(2)
    test_prescription_status_flow()
    time.sleep(1)
    generate_test_report()

    print("\n" + "=" * 60)
    print(" !")
    print(f"  {len(created_prescriptions)} ")
    print("\n :")
    print("   - ")
    print("   - ")
    print("   - ")
    print("   - ")

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""

Data Persistence Module
JSON
"""

import json
import os
from pathlib import Path
from datetime import datetime
import shutil

class DataPersistence:
    def __init__(self, data_dir="data"):
        """"""
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(exist_ok=True)

        #
        self.medicines_file = self.data_dir / "medicines.json"
        self.detailed_medicines_file = self.data_dir / "detailed_medicines.json"
        self.prescriptions_file = self.data_dir / "prescriptions.json"
        self.prescription_status_file = self.data_dir / "prescription_status.json"
        self.counters_file = self.data_dir / "counters.json"

        #
        self.backup_dir = self.data_dir / "backups"
        self.backup_dir.mkdir(exist_ok=True)

    def save_medicines(self, medicines_db, next_medicine_id):
        """"""
        try:
            data = {
                "medicines": medicines_db,
                "next_id": next_medicine_id,
                "last_updated": datetime.now().isoformat()
            }

            with open(self.medicines_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)

            return True
        except Exception as e:
            print(f" : {e}")
            return False

    def load_medicines(self):
        """"""
        try:
            if self.medicines_file.exists():
                with open(self.medicines_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)

                medicines_db = data.get("medicines", [])
                next_medicine_id = data.get("next_id", 1)

                print(f" : {len(medicines_db)} ")
                return medicines_db, next_medicine_id
            else:
                print(" ")
                return [], 1

        except Exception as e:
            print(f" : {e}")
            return [], 1

    def save_detailed_medicines(self, detailed_medicines_db):
        """"""
        try:
            data = {
                "detailed_medicines": detailed_medicines_db,
                "last_updated": datetime.now().isoformat()
            }

            with open(self.detailed_medicines_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)

            return True
        except Exception as e:
            print(f" : {e}")
            return False

    def load_detailed_medicines(self):
        """"""
        try:
            if self.detailed_medicines_file.exists():
                with open(self.detailed_medicines_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)

                detailed_medicines_db = data.get("detailed_medicines", {})

                print(f" : {len(detailed_medicines_db)} ")
                return detailed_medicines_db
            else:
                print(" ")
                return {}

        except Exception as e:
            print(f" : {e}")
            return {}

    def save_prescriptions(self, prescriptions_db, prescription_status_db, next_prescription_id):
        """"""
        try:
            #
            prescriptions_data = {
                "prescriptions": prescriptions_db,
                "next_id": next_prescription_id,
                "last_updated": datetime.now().isoformat()
            }

            with open(self.prescriptions_file, 'w', encoding='utf-8') as f:
                json.dump(prescriptions_data, f, ensure_ascii=False, indent=2)

            #
            status_data = {
                "prescription_status": prescription_status_db,
                "last_updated": datetime.now().isoformat()
            }

            with open(self.prescription_status_file, 'w', encoding='utf-8') as f:
                json.dump(status_data, f, ensure_ascii=False, indent=2)

            return True
        except Exception as e:
            print(f" : {e}")
            return False

    def load_prescriptions(self):
        """"""
        try:
            prescriptions_db = []
            prescription_status_db = []
            next_prescription_id = 1

            #
            if self.prescriptions_file.exists():
                with open(self.prescriptions_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)

                prescriptions_db = data.get("prescriptions", [])
                next_prescription_id = data.get("next_id", 1)

            #
            if self.prescription_status_file.exists():
                with open(self.prescription_status_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)

                prescription_status_db = data.get("prescription_status", [])

            print(f" : {len(prescriptions_db)} , {len(prescription_status_db)} ")
            return prescriptions_db, prescription_status_db, next_prescription_id

        except Exception as e:
            print(f" : {e}")
            return [], [], 1

    def save_all_data(self, medicines_db, next_medicine_id, detailed_medicines_db,
                      prescriptions_db, prescription_status_db, next_prescription_id):
        """"""
        print(" ...")

        success = True
        success &= self.save_medicines(medicines_db, next_medicine_id)
        success &= self.save_detailed_medicines(detailed_medicines_db)
        success &= self.save_prescriptions(prescriptions_db, prescription_status_db, next_prescription_id)

        if success:
            print(" ")
        else:
            print(" ")

        return success

    def load_all_data(self):
        """"""
        print(" ...")

        medicines_db, next_medicine_id = self.load_medicines()
        detailed_medicines_db = self.load_detailed_medicines()
        prescriptions_db, prescription_status_db, next_prescription_id = self.load_prescriptions()

        return {
            'medicines_db': medicines_db,
            'next_medicine_id': next_medicine_id,
            'detailed_medicines_db': detailed_medicines_db,
            'prescriptions_db': prescriptions_db,
            'prescription_status_db': prescription_status_db,
            'next_prescription_id': next_prescription_id
        }

    def create_backup(self, backup_name=None):
        """"""
        try:
            if backup_name is None:
                backup_name = f"backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

            backup_path = self.backup_dir / backup_name
            backup_path.mkdir(exist_ok=True)

            #
            data_files = [
                self.medicines_file,
                self.detailed_medicines_file,
                self.prescriptions_file,
                self.prescription_status_file
            ]

            copied_files = 0
            for data_file in data_files:
                if data_file.exists():
                    shutil.copy2(data_file, backup_path)
                    copied_files += 1

            if copied_files > 0:
                print(f" : {backup_path} ({copied_files} )")
                return str(backup_path)
            else:
                print("  ")
                return None

        except Exception as e:
            print(f" : {e}")
            return None

    def restore_backup(self, backup_name):
        """"""
        try:
            backup_path = self.backup_dir / backup_name

            if not backup_path.exists():
                print(f" : {backup_name}")
                return False

            #
            data_files = [
                "medicines.json",
                "detailed_medicines.json",
                "prescriptions.json",
                "prescription_status.json"
            ]

            restored_files = 0
            for filename in data_files:
                backup_file = backup_path / filename
                if backup_file.exists():
                    shutil.copy2(backup_file, self.data_dir)
                    restored_files += 1

            if restored_files > 0:
                print(f" : {restored_files} ")
                return True
            else:
                print("  ")
                return False

        except Exception as e:
            print(f" : {e}")
            return False

    def list_backups(self):
        """"""
        try:
            backups = []
            if self.backup_dir.exists():
                for backup_path in self.backup_dir.iterdir():
                    if backup_path.is_dir():
                        #
                        backup_info = {
                            'name': backup_path.name,
                            'path': str(backup_path),
                            'created_time': datetime.fromtimestamp(backup_path.stat().st_mtime).isoformat(),
                            'files': len(list(backup_path.glob('*.json')))
                        }
                        backups.append(backup_info)

            #
            backups.sort(key=lambda x: x['created_time'], reverse=True)
            return backups

        except Exception as e:
            print(f" : {e}")
            return []

    def get_data_info(self):
        """"""
        try:
            info = {
                'data_directory': str(self.data_dir),
                'files': {}
            }

            data_files = {
                'medicines': self.medicines_file,
                'detailed_medicines': self.detailed_medicines_file,
                'prescriptions': self.prescriptions_file,
                'prescription_status': self.prescription_status_file
            }

            for name, file_path in data_files.items():
                if file_path.exists():
                    stat = file_path.stat()
                    info['files'][name] = {
                        'exists': True,
                        'size_bytes': stat.st_size,
                        'modified_time': datetime.fromtimestamp(stat.st_mtime).isoformat()
                    }
                else:
                    info['files'][name] = {'exists': False}

            return info

        except Exception as e:
            print(f" : {e}")
            return {}

#
data_persistence = DataPersistence()
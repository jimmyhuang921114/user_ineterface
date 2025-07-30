#!/usr/bin/env python3
"""
完整醫療系統測試範例
Complete Medical System Test Example

測試流程：
1. 寫入病例
2. 透過ROS2調取病例和藥物資料
3. 回報完成狀態
"""

import requests
import json
import time
from datetime import datetime
from ros2_medical_data_service import simulate_medical_data_query, simulate_completion_report

API_BASE = "http://localhost:8000"

def test_complete_medical_workflow():
    """完整醫療工作流程測試"""
    print("=" * 80)
    print("完整醫療系統工作流程測試")
    print("=" * 80)
    
    # === 步驟1: 創建測試藥物 ===
    print("\n步驟1: 創建測試藥物")
    print("-" * 40)
    
    # 創建基本藥物
    medicine_data = {
        "name": "心律錠",
        "amount": 100,
        "usage_days": 30,
        "position": "A1-01"
    }
    
    try:
        response = requests.post(f"{API_BASE}/api/medicine/", json=medicine_data)
        if response.status_code == 200:
            medicine_result = response.json()
            print(f"✓ 基本藥物創建成功: {medicine_result['medicine']['name']}")
        else:
            print(f"✗ 基本藥物創建失敗: {response.status_code}")
    except Exception as e:
        print(f"✗ 基本藥物創建錯誤: {e}")
    
    # 創建詳細藥物資訊
    detailed_medicine_data = {
        "medicine_name": "心律錠",
        "medicine_data": {
            "基本資訊": {
                "名稱": "心律錠(Propranolol)",
                "廠商": "生達",
                "劑量": "10毫克"
            },
            "外觀": {
                "顏色": "明紫紅",
                "形狀": "圓扁形"
            },
            "包裝編號": {
                "編號1": "202801",
                "編號2": "TP071014",
                "編號3": "衛署藥製字第009102號"
            },
            "適應症": "狹心症、不整律、原發性及腎性高血壓",
            "可能的副作用": "常見-心智混亂、疲憊、睏倦、心跳徐緩",
            "使用說明": "用法用量請遵照醫囑；除特別要求外，一般建議於飯後服用",
            "注意事項": "本藥會掩飾低血糖症狀並延長低血糖時間",
            "懷孕分級": "C級；若於妊娠第二或第三期則為 D 級",
            "儲存條件": "請連同藥袋存放於緊密容器內，室溫乾燥避光"
        }
    }
    
    try:
        response = requests.post(f"{API_BASE}/api/medicine/detailed/", json=detailed_medicine_data)
        if response.status_code == 200:
            print(f"✓ 詳細藥物資訊創建成功")
        else:
            print(f"✗ 詳細藥物資訊創建失敗: {response.status_code}")
    except Exception as e:
        print(f"✗ 詳細藥物資訊創建錯誤: {e}")
    
    # === 步驟2: 創建病例 ===
    print("\n步驟2: 創建病例記錄")
    print("-" * 40)
    
    medical_record_data = {
        "patient_id": "P001",
        "patient_name": "張小明",
        "doctor_name": "李醫師",
        "diagnosis": "高血壓",
        "symptoms": "頭痛、眩暈、心悸",
        "treatment_plan": "藥物治療配合生活調整",
        "prescribed_medicines": [
            {
                "medicine_name": "心律錠",
                "dosage": "10mg",
                "frequency": "每日兩次",
                "duration": "30天",
                "instructions": "飯後服用"
            }
        ],
        "vital_signs": {
            "血壓": "150/95 mmHg",
            "心率": "85 bpm",
            "體溫": "36.5°C",
            "體重": "70 kg"
        },
        "medical_history": "無特殊病史",
        "notes": "初次診斷高血壓，需定期追蹤",
        "visit_date": datetime.now().date().isoformat()
    }
    
    record_id = None
    try:
        response = requests.post(f"{API_BASE}/api/medical_record/", json=medical_record_data)
        if response.status_code == 200:
            record_result = response.json()
            record_id = record_result['record']['id']
            print(f"✓ 病例創建成功: ID {record_id}")
            print(f"  病患: {record_result['record']['patient_name']}")
            print(f"  診斷: {record_result['record']['diagnosis']}")
        else:
            print(f"✗ 病例創建失敗: {response.status_code}")
            return
    except Exception as e:
        print(f"✗ 病例創建錯誤: {e}")
        return
    
    # === 步驟3: 透過ROS2調取病例資料 ===
    print("\n步驟3: 透過ROS2調取醫療資料")
    print("-" * 40)
    
    # 等待一下確保資料已保存
    time.sleep(1)
    
    # 調取病例資料
    print("3.1 調取病例資料...")
    medical_record_response = simulate_medical_data_query(
        query_type="medical_record",
        query_id=str(record_id),
        patient_id="",
        additional_params=""
    )
    
    if medical_record_response.success:
        print("✓ 病例資料調取成功")
        record_data = json.loads(medical_record_response.data)
        print(f"  病患姓名: {record_data.get('patient_name')}")
        print(f"  診斷: {record_data.get('diagnosis')}")
        print(f"  處方藥物數量: {len(record_data.get('prescribed_medicines', []))}")
    else:
        print(f"✗ 病例資料調取失敗: {medical_record_response.message}")
    
    # 調取藥物資料
    print("\n3.2 調取藥物資料...")
    medicine_response = simulate_medical_data_query(
        query_type="medicine",
        query_id="心律錠",
        patient_id="",
        additional_params=""
    )
    
    if medicine_response.success:
        print("✓ 藥物資料調取成功")
        medicine_data = json.loads(medicine_response.data)
        print(f"  藥物名稱: {medicine_data.get('medicine_name')}")
        print(f"  有基本資訊: {'是' if medicine_data.get('basic_info') else '否'}")
        print(f"  有詳細資訊: {'是' if medicine_data.get('detailed_info') else '否'}")
        print(f"  資訊完整: {'是' if medicine_data.get('has_complete_info') else '否'}")
    else:
        print(f"✗ 藥物資料調取失敗: {medicine_response.message}")
    
    # === 步驟4: 模擬處理過程 ===
    print("\n步驟4: 模擬處理過程")
    print("-" * 40)
    
    # 模擬處理時間
    print("正在處理醫療資料...")
    time.sleep(2)
    
    # 處理結果數據
    processing_result = {
        "processed_record_id": record_id,
        "medicine_dispensed": "心律錠",
        "dispensed_amount": "30錠",
        "patient_counseled": True,
        "next_appointment": (datetime.now().date()).isoformat(),
        "processing_time": datetime.now().isoformat()
    }
    
    print("✓ 處理完成")
    print(f"  處理的病例ID: {processing_result['processed_record_id']}")
    print(f"  配發藥物: {processing_result['medicine_dispensed']}")
    print(f"  配發數量: {processing_result['dispensed_amount']}")
    
    # === 步驟5: 回報完成狀態 ===
    print("\n步驟5: 回報完成狀態")
    print("-" * 40)
    
    # 回報病例處理完成
    completion_response = simulate_completion_report(
        task_id=f"MED_RECORD_{record_id}",
        task_type="medical_record_processing",
        status="completed",
        notes="病例處理完成，藥物已配發，病患已接受用藥指導",
        processed_data=json.dumps(processing_result, ensure_ascii=False)
    )
    
    if completion_response.success:
        print("✓ 完成報告提交成功")
        print(f"  報告ID: {completion_response.report_id}")
        print(f"  下一步動作: {completion_response.next_action}")
    else:
        print(f"✗ 完成報告提交失敗: {completion_response.message}")
    
    # === 步驟6: 驗證整個流程 ===
    print("\n步驟6: 驗證整個流程")
    print("-" * 40)
    
    # 再次查詢病例確認資料完整性
    try:
        response = requests.get(f"{API_BASE}/api/medical_record/{record_id}")
        if response.status_code == 200:
            final_record = response.json()
            print("✓ 病例資料驗證成功")
            print(f"  最後更新時間: {final_record.get('updated_time')}")
            print(f"  狀態: {final_record.get('status')}")
        else:
            print(f"✗ 病例資料驗證失敗: {response.status_code}")
    except Exception as e:
        print(f"✗ 病例資料驗證錯誤: {e}")
    
    # 查詢系統統計
    try:
        response = requests.get(f"{API_BASE}/api/medical_record/stats/summary")
        if response.status_code == 200:
            stats = response.json()
            print("✓ 系統統計查詢成功")
            print(f"  總病例數: {stats.get('total_records')}")
            print(f"  醫師統計: {stats.get('doctor_statistics')}")
        else:
            print(f"✗ 系統統計查詢失敗: {response.status_code}")
    except Exception as e:
        print(f"✗ 系統統計查詢錯誤: {e}")
    
    print("\n" + "=" * 80)
    print("完整醫療系統工作流程測試完成！")
    print("=" * 80)
    
    return {
        "record_id": record_id,
        "medicine_name": "心律錠",
        "completion_report": completion_response.report_id if completion_response.success else None
    }

def test_patient_multiple_records():
    """測試單一病患多筆病例"""
    print("\n" + "=" * 80)
    print("測試單一病患多筆病例功能")
    print("=" * 80)
    
    patient_id = "P002"
    patient_name = "王小華"
    
    # 創建多筆病例
    records = [
        {
            "diagnosis": "感冒",
            "symptoms": "發燒、咳嗽、流鼻涕",
            "treatment_plan": "症狀治療",
            "prescribed_medicines": [
                {"medicine_name": "普拿疼", "dosage": "500mg", "frequency": "每4-6小時", "duration": "3天"}
            ]
        },
        {
            "diagnosis": "高血壓追蹤",
            "symptoms": "血壓偏高",
            "treatment_plan": "調整藥物劑量",
            "prescribed_medicines": [
                {"medicine_name": "心律錠", "dosage": "20mg", "frequency": "每日兩次", "duration": "30天"}
            ]
        }
    ]
    
    created_records = []
    
    for i, record_template in enumerate(records, 1):
        record_data = {
            "patient_id": patient_id,
            "patient_name": patient_name,
            "doctor_name": "陳醫師",
            "visit_date": datetime.now().date().isoformat(),
            **record_template,
            "vital_signs": {"血壓": "140/90 mmHg"},
            "notes": f"第{i}次就診記錄"
        }
        
        try:
            response = requests.post(f"{API_BASE}/api/medical_record/", json=record_data)
            if response.status_code == 200:
                result = response.json()
                created_records.append(result['record']['id'])
                print(f"✓ 病例{i}創建成功: {record_template['diagnosis']}")
            else:
                print(f"✗ 病例{i}創建失敗: {response.status_code}")
        except Exception as e:
            print(f"✗ 病例{i}創建錯誤: {e}")
    
    # 測試按病患ID查詢
    print(f"\n按病患ID查詢: {patient_id}")
    patient_records_response = simulate_medical_data_query(
        query_type="medical_record",
        query_id="",
        patient_id=patient_id
    )
    
    if patient_records_response.success:
        patient_records = json.loads(patient_records_response.data)
        print(f"✓ 病患記錄查詢成功，共 {len(patient_records)} 筆")
        for record in patient_records:
            print(f"  - ID {record['id']}: {record['diagnosis']} ({record['visit_date']})")
    else:
        print(f"✗ 病患記錄查詢失敗: {patient_records_response.message}")
    
    return created_records

def main():
    """主測試函數"""
    print("醫療系統完整測試開始")
    print("請確保API伺服器已啟動 (http://localhost:8000)")
    
    # 檢查API連接
    try:
        response = requests.get(f"{API_BASE}/api/system/status")
        if response.status_code == 200:
            print("✓ API伺服器連接成功")
        else:
            print("✗ API伺服器連接失敗")
            return
    except Exception as e:
        print(f"✗ 無法連接API伺服器: {e}")
        print("請先啟動伺服器: python3 start_modular.py")
        return
    
    # 執行主要測試
    main_result = test_complete_medical_workflow()
    
    # 執行額外測試
    additional_records = test_patient_multiple_records()
    
    # 最終報告
    print("\n" + "=" * 80)
    print("測試總結")
    print("=" * 80)
    print(f"✓ 主要測試完成，病例ID: {main_result.get('record_id')}")
    print(f"✓ 額外測試完成，創建了 {len(additional_records)} 筆額外病例")
    print(f"✓ 完成報告ID: {main_result.get('completion_report')}")
    
    print("\n建議的下一步測試：")
    print("1. 啟動ROS2節點: python3 ros2_medical_data_service.py")
    print("2. 在ROS2環境中測試服務調用")
    print("3. 測試資料持久化: python3 test_data_persistence.py")

if __name__ == "__main__":
    main()
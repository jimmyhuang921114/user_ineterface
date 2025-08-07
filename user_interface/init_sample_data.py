#!/usr/bin/env python3
"""
初始化樣本資料腳本
為醫院藥物管理系統添加基本的藥物資料
"""

from sqlalchemy.orm import Session
from database_clean import SessionLocal, Medicine, MedicineDetail
from datetime import datetime

def add_sample_medicines():
    """添加樣本藥物資料"""
    db = SessionLocal()
    
    try:
        # 檢查是否已有資料
        existing_count = db.query(Medicine).count()
        if existing_count > 0:
            print(f"資料庫已有 {existing_count} 種藥物，跳過初始化")
            return
        
        print("正在添加樣本藥物資料...")
        
        # 基本藥物資料
        medicines_data = [
            {
                "basic": {
                    "name": "阿司匹林",
                    "amount": 100,
                    "position": "A-001",
                    "manufacturer": "拜耳藥廠",
                    "dosage": "100mg"
                },
                "detail": {
                    "description": "解熱鎮痛抗炎藥",
                    "ingredient": "乙醯水楊酸",
                    "category": "解熱鎮痛藥",
                    "usage_method": "口服",
                    "unit_dose": 100.0,
                    "side_effects": "可能引起胃腸道不適",
                    "storage_conditions": "室溫保存，避光防潮",
                    "expiry_date": "2025-12-31",
                    "barcode": "4711001234567",
                    "appearance_type": "白色圓形錠劑",
                    "notes": "餐後服用以減少胃腸道刺激"
                }
            },
            {
                "basic": {
                    "name": "布洛芬",
                    "amount": 80,
                    "position": "A-002",
                    "manufacturer": "強生藥廠",
                    "dosage": "200mg"
                },
                "detail": {
                    "description": "非類固醇抗炎藥",
                    "ingredient": "布洛芬",
                    "category": "解熱鎮痛藥",
                    "usage_method": "口服",
                    "unit_dose": 200.0,
                    "side_effects": "可能引起頭痛、胃腸道不適",
                    "storage_conditions": "室溫保存，避免高溫",
                    "expiry_date": "2025-10-15",
                    "barcode": "4711001234568",
                    "appearance_type": "橙色膠囊",
                    "notes": "不建議空腹服用"
                }
            },
            {
                "basic": {
                    "name": "維他命C",
                    "amount": 150,
                    "position": "B-001",
                    "manufacturer": "羅氏藥廠",
                    "dosage": "500mg"
                },
                "detail": {
                    "description": "維生素補充劑",
                    "ingredient": "抗壞血酸",
                    "category": "維生素",
                    "usage_method": "口服",
                    "unit_dose": 500.0,
                    "side_effects": "大劑量可能引起腹瀉",
                    "storage_conditions": "陰涼乾燥處保存",
                    "expiry_date": "2026-03-20",
                    "barcode": "4711001234569",
                    "appearance_type": "黃色錠劑",
                    "notes": "可隨餐或餐後服用"
                }
            },
            {
                "basic": {
                    "name": "感冒糖漿",
                    "amount": 50,
                    "position": "C-001",
                    "manufacturer": "諾華藥廠",
                    "dosage": "15ml"
                },
                "detail": {
                    "description": "複方感冒藥",
                    "ingredient": "對乙醯氨基酚、右美沙芬",
                    "category": "感冒藥",
                    "usage_method": "口服",
                    "unit_dose": 15.0,
                    "side_effects": "可能引起嗜睡",
                    "storage_conditions": "室溫保存，開封後冷藏",
                    "expiry_date": "2025-08-30",
                    "barcode": "4711001234570",
                    "appearance_type": "紅色糖漿",
                    "notes": "兒童劑量請諮詢醫師"
                }
            },
            {
                "basic": {
                    "name": "胃藥",
                    "amount": 120,
                    "position": "D-001",
                    "manufacturer": "輝瑞藥廠",
                    "dosage": "20mg"
                },
                "detail": {
                    "description": "質子泵抑制劑",
                    "ingredient": "奧美拉唑",
                    "category": "消化系統藥物",
                    "usage_method": "口服",
                    "unit_dose": 20.0,
                    "side_effects": "可能引起頭痛、腹瀉",
                    "storage_conditions": "室溫保存，避光",
                    "expiry_date": "2025-11-15",
                    "barcode": "4711001234571",
                    "appearance_type": "紫色膠囊",
                    "notes": "餐前30分鐘服用效果最佳"
                }
            }
        ]
        
        # 添加藥物資料
        for med_data in medicines_data:
            # 創建基本藥物資料
            basic_medicine = Medicine(**med_data["basic"])
            db.add(basic_medicine)
            db.flush()  # 獲取 ID
            
            # 創建詳細藥物資料
            detail_data = med_data["detail"]
            detail_data["medicine_id"] = basic_medicine.id
            detailed_medicine = MedicineDetail(**detail_data)
            db.add(detailed_medicine)
            
            print(f"✓ 已添加: {med_data['basic']['name']}")
        
        db.commit()
        
        # 顯示結果
        total_count = db.query(Medicine).count()
        print(f"\n✅ 樣本資料初始化完成！")
        print(f"📊 總共添加了 {total_count} 種藥物")
        print(f"💊 藥物列表:")
        
        medicines = db.query(Medicine).all()
        for med in medicines:
            print(f"   - {med.name} (庫存: {med.amount}, 位置: {med.position})")
        
    except Exception as e:
        db.rollback()
        print(f"❌ 添加樣本資料失敗: {e}")
        
    finally:
        db.close()

def clear_all_data():
    """清空所有資料 (慎用)"""
    db = SessionLocal()
    
    try:
        print("⚠️  警告: 即將清空所有資料...")
        confirm = input("確定要清空嗎？ (輸入 'YES' 確認): ")
        
        if confirm == "YES":
            # 刪除詳細資料
            detail_count = db.query(MedicineDetail).count()
            db.query(MedicineDetail).delete()
            
            # 刪除基本資料
            basic_count = db.query(Medicine).count()
            db.query(Medicine).delete()
            
            db.commit()
            print(f"✅ 已清空 {basic_count} 種基本藥物和 {detail_count} 條詳細資料")
        else:
            print("❌ 取消清空操作")
            
    except Exception as e:
        db.rollback()
        print(f"❌ 清空資料失敗: {e}")
        
    finally:
        db.close()

def main():
    """主函數"""
    print("醫院藥物管理系統 - 樣本資料初始化")
    print("=" * 50)
    
    if len(__import__('sys').argv) > 1 and __import__('sys').argv[1] == "--clear":
        clear_all_data()
    else:
        add_sample_medicines()

if __name__ == "__main__":
    main()
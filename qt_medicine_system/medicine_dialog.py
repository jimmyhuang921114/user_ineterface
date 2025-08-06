#!/usr/bin/env python3
"""
💊 藥物對話框模組
用於新增和編輯藥物資訊
"""

import sys
from datetime import datetime, timedelta
from typing import Dict, Optional

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLineEdit,
    QComboBox, QSpinBox, QDoubleSpinBox, QTextEdit, QPushButton,
    QDialogButtonBox, QLabel, QDateEdit, QGroupBox, QCheckBox
)
from PyQt6.QtCore import Qt, QDate
from PyQt6.QtGui import QFont


class MedicineDialog(QDialog):
    """藥物新增/編輯對話框"""
    
    def __init__(self, parent=None, medicine_data: Optional[Dict] = None):
        super().__init__(parent)
        
        self.medicine_data = medicine_data or {}
        self.is_edit_mode = medicine_data is not None
        
        self.init_ui()
        self.load_data()
        
    def init_ui(self):
        """初始化使用者介面"""
        self.setWindowTitle("💊 藥物資訊" if not self.is_edit_mode else "✏️ 編輯藥物")
        self.setModal(True)
        self.setMinimumWidth(500)
        
        # 創建主佈局
        layout = QVBoxLayout(self)
        
        # 基本資訊群組
        basic_group = QGroupBox("📋 基本資訊")
        basic_layout = QFormLayout(basic_group)
        
        # 藥物ID
        self.id_edit = QLineEdit()
        self.id_edit.setPlaceholderText("自動生成")
        if self.is_edit_mode:
            self.id_edit.setReadOnly(True)
        basic_layout.addRow("藥物ID:", self.id_edit)
        
        # 藥物名稱
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("輸入藥物名稱")
        basic_layout.addRow("藥物名稱:", self.name_edit)
        
        # 英文名稱
        self.english_name_edit = QLineEdit()
        self.english_name_edit.setPlaceholderText("輸入英文名稱")
        basic_layout.addRow("英文名稱:", self.english_name_edit)
        
        # 藥物分類
        self.category_combo = QComboBox()
        self.category_combo.setEditable(True)
        self.category_combo.addItems([
            "解熱鎮痛藥", "維生素", "礦物質", "抗生素", "心血管藥物",
            "消化系統藥物", "呼吸系統藥物", "皮膚用藥", "眼科用藥", "其他"
        ])
        basic_layout.addRow("藥物分類:", self.category_combo)
        
        # 劑型
        self.dosage_form_combo = QComboBox()
        self.dosage_form_combo.setEditable(True)
        self.dosage_form_combo.addItems([
            "錠劑", "膠囊", "液劑", "注射劑", "軟膏", "眼藥水", "吸入劑", "貼片"
        ])
        basic_layout.addRow("劑型:", self.dosage_form_combo)
        
        # 劑量強度
        self.strength_edit = QLineEdit()
        self.strength_edit.setPlaceholderText("例如: 100mg")
        basic_layout.addRow("劑量強度:", self.strength_edit)
        
        layout.addWidget(basic_group)
        
        # 庫存資訊群組
        inventory_group = QGroupBox("📦 庫存資訊")
        inventory_layout = QFormLayout(inventory_group)
        
        # 庫存數量
        self.stock_quantity_spin = QSpinBox()
        self.stock_quantity_spin.setRange(0, 999999)
        self.stock_quantity_spin.setSuffix(" 個")
        inventory_layout.addRow("庫存數量:", self.stock_quantity_spin)
        
        # 單位
        self.unit_edit = QLineEdit()
        self.unit_edit.setPlaceholderText("例如: 錠、顆、瓶")
        inventory_layout.addRow("單位:", self.unit_edit)
        
        # 單價
        self.price_spin = QDoubleSpinBox()
        self.price_spin.setRange(0.0, 999999.99)
        self.price_spin.setPrefix("$ ")
        self.price_spin.setDecimals(2)
        inventory_layout.addRow("單價:", self.price_spin)
        
        # 過期日期
        self.expiry_date_edit = QDateEdit()
        self.expiry_date_edit.setCalendarPopup(True)
        self.expiry_date_edit.setDate(QDate.currentDate().addYears(1))
        inventory_layout.addRow("過期日期:", self.expiry_date_edit)
        
        layout.addWidget(inventory_group)
        
        # 詳細資訊群組
        details_group = QGroupBox("📄 詳細資訊")
        details_layout = QFormLayout(details_group)
        
        # 製造商
        self.manufacturer_edit = QLineEdit()
        self.manufacturer_edit.setPlaceholderText("輸入製造商名稱")
        details_layout.addRow("製造商:", self.manufacturer_edit)
        
        # 描述
        self.description_edit = QTextEdit()
        self.description_edit.setMaximumHeight(60)
        self.description_edit.setPlaceholderText("輸入藥物描述")
        details_layout.addRow("描述:", self.description_edit)
        
        # 副作用
        self.side_effects_edit = QTextEdit()
        self.side_effects_edit.setMaximumHeight(60)
        self.side_effects_edit.setPlaceholderText("輸入可能的副作用")
        details_layout.addRow("副作用:", self.side_effects_edit)
        
        # 儲存條件
        self.storage_conditions_edit = QLineEdit()
        self.storage_conditions_edit.setPlaceholderText("例如: 室溫保存、避光保存")
        details_layout.addRow("儲存條件:", self.storage_conditions_edit)
        
        layout.addWidget(details_group)
        
        # 按鈕
        button_layout = QHBoxLayout()
        
        # 預設值按鈕
        if not self.is_edit_mode:
            default_btn = QPushButton("📝 填入預設值")
            default_btn.clicked.connect(self.fill_default_values)
            button_layout.addWidget(default_btn)
            
        button_layout.addStretch()
        
        # 標準按鈕
        button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        button_layout.addWidget(button_box)
        
        layout.addLayout(button_layout)
        
        # 設置驗證
        self.setup_validation()
        
    def setup_validation(self):
        """設置表單驗證"""
        # 連接信號以進行即時驗證
        self.name_edit.textChanged.connect(self.validate_form)
        self.category_combo.currentTextChanged.connect(self.validate_form)
        self.stock_quantity_spin.valueChanged.connect(self.validate_form)
        
    def validate_form(self):
        """驗證表單"""
        is_valid = True
        
        # 檢查必填欄位
        if not self.name_edit.text().strip():
            is_valid = False
            self.name_edit.setStyleSheet("border: 1px solid red;")
        else:
            self.name_edit.setStyleSheet("")
            
        if not self.category_combo.currentText().strip():
            is_valid = False
            self.category_combo.setStyleSheet("border: 1px solid red;")
        else:
            self.category_combo.setStyleSheet("")
            
        # 檢查庫存數量
        if self.stock_quantity_spin.value() < 0:
            is_valid = False
            self.stock_quantity_spin.setStyleSheet("border: 1px solid red;")
        else:
            self.stock_quantity_spin.setStyleSheet("")
            
        # 啟用/禁用確定按鈕
        button_box = self.findChild(QDialogButtonBox)
        if button_box:
            ok_button = button_box.button(QDialogButtonBox.StandardButton.Ok)
            if ok_button:
                ok_button.setEnabled(is_valid)
                
    def load_data(self):
        """載入藥物資料"""
        if not self.medicine_data:
            return
            
        # 填入現有資料
        self.id_edit.setText(self.medicine_data.get('id', ''))
        self.name_edit.setText(self.medicine_data.get('name', ''))
        self.english_name_edit.setText(self.medicine_data.get('english_name', ''))
        
        # 設置分類
        category = self.medicine_data.get('category', '')
        index = self.category_combo.findText(category)
        if index >= 0:
            self.category_combo.setCurrentIndex(index)
        else:
            self.category_combo.setCurrentText(category)
            
        # 設置劑型
        dosage_form = self.medicine_data.get('dosage_form', '')
        index = self.dosage_form_combo.findText(dosage_form)
        if index >= 0:
            self.dosage_form_combo.setCurrentIndex(index)
        else:
            self.dosage_form_combo.setCurrentText(dosage_form)
            
        self.strength_edit.setText(self.medicine_data.get('strength', ''))
        self.stock_quantity_spin.setValue(self.medicine_data.get('stock_quantity', 0))
        self.unit_edit.setText(self.medicine_data.get('unit', ''))
        self.price_spin.setValue(self.medicine_data.get('price', 0.0))
        
        # 設置過期日期
        expiry_date = self.medicine_data.get('expiry_date', '')
        if expiry_date:
            try:
                date = QDate.fromString(expiry_date, 'yyyy-MM-dd')
                if date.isValid():
                    self.expiry_date_edit.setDate(date)
            except:
                pass
                
        self.manufacturer_edit.setText(self.medicine_data.get('manufacturer', ''))
        self.description_edit.setPlainText(self.medicine_data.get('description', ''))
        self.side_effects_edit.setPlainText(self.medicine_data.get('side_effects', ''))
        self.storage_conditions_edit.setText(self.medicine_data.get('storage_conditions', ''))
        
    def fill_default_values(self):
        """填入預設值"""
        self.name_edit.setText("新藥物")
        self.english_name_edit.setText("New Medicine")
        self.category_combo.setCurrentText("其他")
        self.dosage_form_combo.setCurrentText("錠劑")
        self.strength_edit.setText("100mg")
        self.stock_quantity_spin.setValue(100)
        self.unit_edit.setText("錠")
        self.price_spin.setValue(10.0)
        self.manufacturer_edit.setText("製造商")
        self.description_edit.setPlainText("藥物描述")
        self.side_effects_edit.setPlainText("可能的副作用")
        self.storage_conditions_edit.setText("室溫保存")
        
    def get_medicine_data(self) -> Dict:
        """獲取藥物資料"""
        data = {
            'id': self.id_edit.text().strip(),
            'name': self.name_edit.text().strip(),
            'english_name': self.english_name_edit.text().strip(),
            'category': self.category_combo.currentText().strip(),
            'dosage_form': self.dosage_form_combo.currentText().strip(),
            'strength': self.strength_edit.text().strip(),
            'stock_quantity': self.stock_quantity_spin.value(),
            'unit': self.unit_edit.text().strip(),
            'price': self.price_spin.value(),
            'expiry_date': self.expiry_date_edit.date().toString('yyyy-MM-dd'),
            'manufacturer': self.manufacturer_edit.text().strip(),
            'description': self.description_edit.toPlainText().strip(),
            'side_effects': self.side_effects_edit.toPlainText().strip(),
            'storage_conditions': self.storage_conditions_edit.text().strip()
        }
        
        # 移除空值
        return {k: v for k, v in data.items() if v}
        
    def accept(self):
        """確認對話框"""
        # 驗證必填欄位
        if not self.name_edit.text().strip():
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "警告", "請輸入藥物名稱")
            self.name_edit.setFocus()
            return
            
        if not self.category_combo.currentText().strip():
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "警告", "請選擇藥物分類")
            self.category_combo.setFocus()
            return
            
        super().accept()
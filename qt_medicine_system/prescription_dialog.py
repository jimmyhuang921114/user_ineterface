#!/usr/bin/env python3
"""
📋 處方籤對話框模組
用於新增處方籤資訊
"""

import sys
from datetime import datetime
from typing import Dict, List, Optional

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLineEdit,
    QComboBox, QSpinBox, QTextEdit, QPushButton, QDialogButtonBox,
    QLabel, QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView,
    QMessageBox, QDateEdit, QTimeEdit
)
from PyQt6.QtCore import Qt, QDate, QTime
from PyQt6.QtGui import QFont


class PrescriptionDialog(QDialog):
    """處方籤新增對話框"""
    
    def __init__(self, parent=None, data_manager=None):
        super().__init__(parent)
        
        self.data_manager = data_manager
        self.medicines = []
        self.selected_medicines = []
        
        self.init_ui()
        self.load_medicines()
        
    def init_ui(self):
        """初始化使用者介面"""
        self.setWindowTitle("📋 新增處方籤")
        self.setModal(True)
        self.setMinimumWidth(700)
        self.setMinimumHeight(600)
        
        # 創建主佈局
        layout = QVBoxLayout(self)
        
        # 病人資訊群組
        patient_group = QGroupBox("👤 病人資訊")
        patient_layout = QFormLayout(patient_group)
        
        # 病人姓名
        self.patient_name_edit = QLineEdit()
        self.patient_name_edit.setPlaceholderText("輸入病人姓名")
        patient_layout.addRow("病人姓名:", self.patient_name_edit)
        
        # 病人ID
        self.patient_id_edit = QLineEdit()
        self.patient_id_edit.setPlaceholderText("輸入病人ID")
        patient_layout.addRow("病人ID:", self.patient_id_edit)
        
        # 醫師姓名
        self.doctor_name_edit = QLineEdit()
        self.doctor_name_edit.setPlaceholderText("輸入醫師姓名")
        patient_layout.addRow("醫師姓名:", self.doctor_name_edit)
        
        # 處方日期
        self.prescription_date_edit = QDateEdit()
        self.prescription_date_edit.setCalendarPopup(True)
        self.prescription_date_edit.setDate(QDate.currentDate())
        patient_layout.addRow("處方日期:", self.prescription_date_edit)
        
        # 處方時間
        self.prescription_time_edit = QTimeEdit()
        self.prescription_time_edit.setTime(QTime.currentTime())
        patient_layout.addRow("處方時間:", self.prescription_time_edit)
        
        layout.addWidget(patient_group)
        
        # 藥物選擇群組
        medicine_group = QGroupBox("💊 藥物選擇")
        medicine_layout = QVBoxLayout(medicine_group)
        
        # 藥物搜尋
        search_layout = QHBoxLayout()
        self.medicine_search = QLineEdit()
        self.medicine_search.setPlaceholderText("🔍 搜尋藥物...")
        self.medicine_search.textChanged.connect(self.filter_medicines)
        search_layout.addWidget(self.medicine_search)
        
        # 分類篩選
        self.category_filter = QComboBox()
        self.category_filter.addItem("全部分類")
        self.category_filter.currentTextChanged.connect(self.filter_medicines)
        search_layout.addWidget(QLabel("分類:"))
        search_layout.addWidget(self.category_filter)
        
        medicine_layout.addLayout(search_layout)
        
        # 藥物表格
        self.medicine_table = QTableWidget()
        self.medicine_table.setColumnCount(6)
        self.medicine_table.setHorizontalHeaderLabels([
            "選擇", "藥物名稱", "分類", "劑型", "庫存", "單價"
        ])
        
        header = self.medicine_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        
        self.medicine_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.medicine_table.setAlternatingRowColors(True)
        self.medicine_table.itemClicked.connect(self.on_medicine_selected)
        
        medicine_layout.addWidget(self.medicine_table)
        
        layout.addWidget(medicine_group)
        
        # 已選藥物群組
        selected_group = QGroupBox("📋 已選藥物")
        selected_layout = QVBoxLayout(selected_group)
        
        # 已選藥物表格
        self.selected_table = QTableWidget()
        self.selected_table.setColumnCount(7)
        self.selected_table.setHorizontalHeaderLabels([
            "藥物名稱", "劑量", "頻率", "數量", "備註", "小計", "操作"
        ])
        
        header = self.selected_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        
        self.selected_table.setAlternatingRowColors(True)
        
        selected_layout.addWidget(self.selected_table)
        
        # 總價顯示
        total_layout = QHBoxLayout()
        total_layout.addStretch()
        self.total_price_label = QLabel("總價: $0.00")
        self.total_price_label.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        total_layout.addWidget(self.total_price_label)
        
        selected_layout.addLayout(total_layout)
        
        layout.addWidget(selected_group)
        
        # 備註群組
        notes_group = QGroupBox("📝 備註")
        notes_layout = QVBoxLayout(notes_group)
        
        self.notes_edit = QTextEdit()
        self.notes_edit.setMaximumHeight(80)
        self.notes_edit.setPlaceholderText("輸入處方籤備註...")
        notes_layout.addWidget(self.notes_edit)
        
        layout.addWidget(notes_group)
        
        # 按鈕
        button_layout = QHBoxLayout()
        
        # 清空按鈕
        clear_btn = QPushButton("🗑️ 清空選擇")
        clear_btn.clicked.connect(self.clear_selection)
        button_layout.addWidget(clear_btn)
        
        button_layout.addStretch()
        
        # 標準按鈕
        button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        button_layout.addWidget(button_box)
        
        layout.addLayout(button_layout)
        
    def load_medicines(self):
        """載入藥物列表"""
        if self.data_manager:
            self.medicines = self.data_manager.get_medicines()
            
            # 更新分類篩選
            categories = self.data_manager.get_categories()
            self.category_filter.clear()
            self.category_filter.addItem("全部分類")
            self.category_filter.addItems(categories)
            
        self.refresh_medicine_table()
        
    def refresh_medicine_table(self):
        """重新整理藥物表格"""
        self.medicine_table.setRowCount(len(self.medicines))
        
        for row, medicine in enumerate(self.medicines):
            # 選擇框
            checkbox = QTableWidgetItem()
            checkbox.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
            checkbox.setCheckState(Qt.CheckState.Unchecked)
            self.medicine_table.setItem(row, 0, checkbox)
            
            # 藥物資訊
            self.medicine_table.setItem(row, 1, QTableWidgetItem(medicine['name']))
            self.medicine_table.setItem(row, 2, QTableWidgetItem(medicine['category']))
            self.medicine_table.setItem(row, 3, QTableWidgetItem(medicine['dosage_form']))
            self.medicine_table.setItem(row, 4, QTableWidgetItem(str(medicine['stock_quantity'])))
            self.medicine_table.setItem(row, 5, QTableWidgetItem(f"${medicine['price']:.2f}"))
            
    def filter_medicines(self):
        """篩選藥物"""
        search_text = self.medicine_search.text().lower()
        category_filter = self.category_filter.currentText()
        
        for row in range(self.medicine_table.rowCount()):
            name = self.medicine_table.item(row, 1).text().lower()
            category = self.medicine_table.item(row, 2).text()
            
            name_match = search_text in name
            category_match = category_filter == "全部分類" or category == category_filter
            
            self.medicine_table.setRowHidden(row, not (name_match and category_match))
            
    def on_medicine_selected(self, item):
        """藥物選擇事件"""
        row = item.row()
        if item.column() == 0:  # 選擇框
            checkbox = self.medicine_table.item(row, 0)
            if checkbox.checkState() == Qt.CheckState.Checked:
                self.add_medicine_to_selection(row)
            else:
                self.remove_medicine_from_selection(row)
                
    def add_medicine_to_selection(self, row):
        """添加藥物到選擇列表"""
        medicine = self.medicines[row]
        
        # 檢查是否已經選擇
        for selected in self.selected_medicines:
            if selected['medicine_id'] == medicine['id']:
                QMessageBox.information(self, "提示", f"藥物 '{medicine['name']}' 已經在選擇列表中")
                return
                
        # 創建藥物選擇對話框
        dialog = MedicineSelectionDialog(self, medicine)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            medicine_data = dialog.get_medicine_data()
            self.selected_medicines.append(medicine_data)
            self.refresh_selected_table()
            self.update_total_price()
            
    def remove_medicine_from_selection(self, row):
        """從選擇列表中移除藥物"""
        medicine = self.medicines[row]
        
        for i, selected in enumerate(self.selected_medicines):
            if selected['medicine_id'] == medicine['id']:
                del self.selected_medicines[i]
                self.refresh_selected_table()
                self.update_total_price()
                break
                
    def refresh_selected_table(self):
        """重新整理已選藥物表格"""
        self.selected_table.setRowCount(len(self.selected_medicines))
        
        for row, medicine in enumerate(self.selected_medicines):
            self.selected_table.setItem(row, 0, QTableWidgetItem(medicine['medicine_name']))
            self.selected_table.setItem(row, 1, QTableWidgetItem(medicine['dosage']))
            self.selected_table.setItem(row, 2, QTableWidgetItem(medicine['frequency']))
            self.selected_table.setItem(row, 3, QTableWidgetItem(str(medicine['quantity'])))
            self.selected_table.setItem(row, 4, QTableWidgetItem(medicine['notes']))
            
            # 計算小計
            medicine_info = self.data_manager.get_medicine_by_id(medicine['medicine_id'])
            if medicine_info:
                subtotal = medicine_info['price'] * medicine['quantity']
                self.selected_table.setItem(row, 5, QTableWidgetItem(f"${subtotal:.2f}"))
            
            # 移除按鈕
            remove_btn = QPushButton("🗑️")
            remove_btn.clicked.connect(lambda checked, r=row: self.remove_selected_medicine(r))
            self.selected_table.setCellWidget(row, 6, remove_btn)
            
    def remove_selected_medicine(self, row):
        """移除已選藥物"""
        if 0 <= row < len(self.selected_medicines):
            del self.selected_medicines[row]
            self.refresh_selected_table()
            self.update_total_price()
            
    def update_total_price(self):
        """更新總價"""
        total_price = 0.0
        
        for medicine in self.selected_medicines:
            medicine_info = self.data_manager.get_medicine_by_id(medicine['medicine_id'])
            if medicine_info:
                total_price += medicine_info['price'] * medicine['quantity']
                
        self.total_price_label.setText(f"總價: ${total_price:.2f}")
        
    def clear_selection(self):
        """清空選擇"""
        self.selected_medicines.clear()
        self.refresh_selected_table()
        self.update_total_price()
        
        # 清空選擇框
        for row in range(self.medicine_table.rowCount()):
            checkbox = self.medicine_table.item(row, 0)
            checkbox.setCheckState(Qt.CheckState.Unchecked)
            
    def get_prescription_data(self) -> Dict:
        """獲取處方籤資料"""
        # 組合日期和時間
        date = self.prescription_date_edit.date()
        time = self.prescription_time_edit.time()
        timestamp = f"{date.toString('yyyy-MM-dd')}T{time.toString('HH:mm:ss')}"
        
        data = {
            'patient_name': self.patient_name_edit.text().strip(),
            'patient_id': self.patient_id_edit.text().strip(),
            'doctor_name': self.doctor_name_edit.text().strip(),
            'timestamp': timestamp,
            'medicines': self.selected_medicines,
            'notes': self.notes_edit.toPlainText().strip()
        }
        
        return data
        
    def accept(self):
        """確認對話框"""
        # 驗證必填欄位
        if not self.patient_name_edit.text().strip():
            QMessageBox.warning(self, "警告", "請輸入病人姓名")
            self.patient_name_edit.setFocus()
            return
            
        if not self.doctor_name_edit.text().strip():
            QMessageBox.warning(self, "警告", "請輸入醫師姓名")
            self.doctor_name_edit.setFocus()
            return
            
        if not self.selected_medicines:
            QMessageBox.warning(self, "警告", "請至少選擇一種藥物")
            return
            
        super().accept()


class MedicineSelectionDialog(QDialog):
    """藥物選擇詳細對話框"""
    
    def __init__(self, parent=None, medicine_data=None):
        super().__init__(parent)
        
        self.medicine_data = medicine_data or {}
        
        self.init_ui()
        
    def init_ui(self):
        """初始化使用者介面"""
        self.setWindowTitle(f"💊 選擇藥物 - {self.medicine_data.get('name', '')}")
        self.setModal(True)
        self.setMinimumWidth(400)
        
        # 創建主佈局
        layout = QVBoxLayout(self)
        
        # 藥物資訊顯示
        info_group = QGroupBox("📋 藥物資訊")
        info_layout = QFormLayout(info_group)
        
        info_layout.addRow("藥物名稱:", QLabel(self.medicine_data.get('name', '')))
        info_layout.addRow("分類:", QLabel(self.medicine_data.get('category', '')))
        info_layout.addRow("劑型:", QLabel(self.medicine_data.get('dosage_form', '')))
        info_layout.addRow("劑量強度:", QLabel(self.medicine_data.get('strength', '')))
        info_layout.addRow("庫存:", QLabel(str(self.medicine_data.get('stock_quantity', 0))))
        info_layout.addRow("單價:", QLabel(f"${self.medicine_data.get('price', 0.0):.2f}"))
        
        layout.addWidget(info_group)
        
        # 處方資訊
        prescription_group = QGroupBox("📝 處方資訊")
        prescription_layout = QFormLayout(prescription_group)
        
        # 劑量
        self.dosage_edit = QLineEdit()
        self.dosage_edit.setText(self.medicine_data.get('strength', ''))
        prescription_layout.addRow("劑量:", self.dosage_edit)
        
        # 頻率
        self.frequency_combo = QComboBox()
        self.frequency_combo.setEditable(True)
        self.frequency_combo.addItems([
            "每日一次", "每日兩次", "每日三次", "每6小時一次", "每8小時一次",
            "每12小時一次", "依醫囑", "需要時服用"
        ])
        prescription_layout.addRow("頻率:", self.frequency_combo)
        
        # 數量
        self.quantity_spin = QSpinBox()
        self.quantity_spin.setRange(1, self.medicine_data.get('stock_quantity', 999))
        self.quantity_spin.setValue(30)
        prescription_layout.addRow("數量:", self.quantity_spin)
        
        # 備註
        self.notes_edit = QTextEdit()
        self.notes_edit.setMaximumHeight(60)
        self.notes_edit.setPlaceholderText("輸入服用說明或備註...")
        prescription_layout.addRow("備註:", self.notes_edit)
        
        layout.addWidget(prescription_group)
        
        # 按鈕
        button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
        
    def get_medicine_data(self) -> Dict:
        """獲取藥物選擇資料"""
        return {
            'medicine_id': self.medicine_data.get('id', ''),
            'medicine_name': self.medicine_data.get('name', ''),
            'dosage': self.dosage_edit.text().strip(),
            'frequency': self.frequency_combo.currentText().strip(),
            'quantity': self.quantity_spin.value(),
            'notes': self.notes_edit.toPlainText().strip()
        }
        
    def accept(self):
        """確認對話框"""
        # 驗證必填欄位
        if not self.dosage_edit.text().strip():
            QMessageBox.warning(self, "警告", "請輸入劑量")
            self.dosage_edit.setFocus()
            return
            
        if not self.frequency_combo.currentText().strip():
            QMessageBox.warning(self, "警告", "請選擇頻率")
            self.frequency_combo.setFocus()
            return
            
        if self.quantity_spin.value() <= 0:
            QMessageBox.warning(self, "警告", "請輸入有效的數量")
            self.quantity_spin.setFocus()
            return
            
        super().accept()
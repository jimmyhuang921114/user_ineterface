#!/usr/bin/env python3
"""
🏥 Qt藥物管理系統 - 主程序
整合Qt GUI、YAML資料儲存和ROS2通訊
"""

import sys
import os
import yaml
import json
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QTableWidget, QTableWidgetItem, QPushButton,
    QLabel, QLineEdit, QComboBox, QSpinBox, QTextEdit, QMessageBox,
    QGroupBox, QFormLayout, QSplitter, QHeaderView, QStatusBar,
    QProgressBar, QDialog, QDialogButtonBox, QListWidget, QCheckBox
)
from PyQt6.QtCore import (
    Qt, QTimer, QThread, pyqtSignal, QSettings, QSize
)
from PyQt6.QtGui import (
    QFont, QIcon, QPalette, QColor, QPixmap, QAction
)

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# 自定義模組
from data_manager import DataManager
from ros2_interface import ROS2Interface
from medicine_dialog import MedicineDialog
from prescription_dialog import PrescriptionDialog
from settings_dialog import SettingsDialog


class QtMedicineSystem(QMainWindow):
    """Qt藥物管理系統主視窗"""
    
    def __init__(self):
        super().__init__()
        self.data_manager = DataManager()
        self.ros2_interface = None
        
        # 初始化ROS2
        self.init_ros2()
        
        # 初始化UI
        self.init_ui()
        
        # 載入資料
        self.load_data()
        
        # 設置定時器
        self.setup_timers()
        
        # 設置狀態欄
        self.setup_status_bar()
        
    def init_ros2(self):
        """初始化ROS2"""
        try:
            rclpy.init()
            self.ros2_interface = ROS2Interface()
            
            # 添加狀態回調
            self.ros2_interface.add_status_callback(self.on_ros2_status_update)
            
            print("✅ ROS2初始化成功")
        except Exception as e:
            print(f"❌ ROS2初始化失敗: {e}")
            self.ros2_interface = None
    
    def init_ui(self):
        """初始化使用者介面"""
        self.setWindowTitle("🏥 Qt藥物管理系統 v1.0")
        self.setGeometry(100, 100, 1400, 900)
        
        # 設置應用程序圖標
        self.setWindowIcon(self.create_icon())
        
        # 創建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 創建主佈局
        main_layout = QVBoxLayout(central_widget)
        
        # 創建標題
        title_label = QLabel("🏥 Qt藥物管理系統")
        title_label.setFont(QFont("Arial", 24, QFont.Weight.Bold))
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setStyleSheet("color: #2c3e50; margin: 10px;")
        main_layout.addWidget(title_label)
        
        # 創建標籤頁
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)
        
        # 添加各個標籤頁
        self.setup_medicine_tab()
        self.setup_prescription_tab()
        self.setup_inventory_tab()
        self.setup_ros2_tab()
        self.setup_settings_tab()
        
        # 創建工具列
        self.setup_toolbar()
        
        # 創建選單列
        self.setup_menu_bar()
        
    def setup_medicine_tab(self):
        """設置藥物管理標籤頁"""
        medicine_widget = QWidget()
        layout = QVBoxLayout(medicine_widget)
        
        # 控制按鈕
        button_layout = QHBoxLayout()
        
        self.add_medicine_btn = QPushButton("➕ 新增藥物")
        self.add_medicine_btn.clicked.connect(self.add_medicine)
        button_layout.addWidget(self.add_medicine_btn)
        
        self.edit_medicine_btn = QPushButton("✏️ 編輯藥物")
        self.edit_medicine_btn.clicked.connect(self.edit_medicine)
        button_layout.addWidget(self.edit_medicine_btn)
        
        self.delete_medicine_btn = QPushButton("🗑️ 刪除藥物")
        self.delete_medicine_btn.clicked.connect(self.delete_medicine)
        button_layout.addWidget(self.delete_medicine_btn)
        
        self.refresh_medicine_btn = QPushButton("🔄 重新整理")
        self.refresh_medicine_btn.clicked.connect(self.refresh_medicine_table)
        button_layout.addWidget(self.refresh_medicine_btn)
        
        button_layout.addStretch()
        
        # 搜尋框
        self.medicine_search = QLineEdit()
        self.medicine_search.setPlaceholderText("🔍 搜尋藥物...")
        self.medicine_search.textChanged.connect(self.filter_medicines)
        button_layout.addWidget(self.medicine_search)
        
        layout.addLayout(button_layout)
        
        # 藥物表格
        self.medicine_table = QTableWidget()
        self.medicine_table.setColumnCount(8)
        self.medicine_table.setHorizontalHeaderLabels([
            "ID", "藥物名稱", "英文名稱", "分類", "劑型", "庫存", "單價", "狀態"
        ])
        
        # 設置表格屬性
        header = self.medicine_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        
        self.medicine_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.medicine_table.setAlternatingRowColors(True)
        
        layout.addWidget(self.medicine_table)
        
        self.tab_widget.addTab(medicine_widget, "💊 藥物管理")
        
    def setup_prescription_tab(self):
        """設置處方籤管理標籤頁"""
        prescription_widget = QWidget()
        layout = QVBoxLayout(prescription_widget)
        
        # 控制按鈕
        button_layout = QHBoxLayout()
        
        self.add_prescription_btn = QPushButton("➕ 新增處方籤")
        self.add_prescription_btn.clicked.connect(self.add_prescription)
        button_layout.addWidget(self.add_prescription_btn)
        
        self.process_prescription_btn = QPushButton("⚙️ 處理處方籤")
        self.process_prescription_btn.clicked.connect(self.process_prescription)
        button_layout.addWidget(self.process_prescription_btn)
        
        self.view_prescription_btn = QPushButton("👁️ 查看詳情")
        self.view_prescription_btn.clicked.connect(self.view_prescription)
        button_layout.addWidget(self.view_prescription_btn)
        
        button_layout.addStretch()
        
        # 狀態篩選
        self.status_filter = QComboBox()
        self.status_filter.addItems(["全部狀態", "pending", "processing", "completed", "cancelled"])
        self.status_filter.currentTextChanged.connect(self.filter_prescriptions)
        button_layout.addWidget(QLabel("狀態:"))
        button_layout.addWidget(self.status_filter)
        
        layout.addLayout(button_layout)
        
        # 處方籤表格
        self.prescription_table = QTableWidget()
        self.prescription_table.setColumnCount(7)
        self.prescription_table.setHorizontalHeaderLabels([
            "處方籤ID", "病人姓名", "醫師", "時間", "狀態", "藥物數量", "總價"
        ])
        
        header = self.prescription_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        
        self.prescription_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.prescription_table.setAlternatingRowColors(True)
        
        layout.addWidget(self.prescription_table)
        
        self.tab_widget.addTab(prescription_widget, "📋 處方籤管理")
        
    def setup_inventory_tab(self):
        """設置庫存管理標籤頁"""
        inventory_widget = QWidget()
        layout = QVBoxLayout(inventory_widget)
        
        # 庫存概覽
        overview_group = QGroupBox("📊 庫存概覽")
        overview_layout = QHBoxLayout(overview_group)
        
        self.total_medicines_label = QLabel("總藥物種類: 0")
        self.low_stock_label = QLabel("低庫存警告: 0")
        self.expiring_soon_label = QLabel("即將過期: 0")
        self.total_value_label = QLabel("總庫存價值: $0")
        
        overview_layout.addWidget(self.total_medicines_label)
        overview_layout.addWidget(self.low_stock_label)
        overview_layout.addWidget(self.expiring_soon_label)
        overview_layout.addWidget(self.total_value_label)
        
        layout.addWidget(overview_group)
        
        # 庫存表格
        self.inventory_table = QTableWidget()
        self.inventory_table.setColumnCount(6)
        self.inventory_table.setHorizontalHeaderLabels([
            "藥物名稱", "當前庫存", "最低庫存", "狀態", "過期日期", "建議補貨"
        ])
        
        header = self.inventory_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        
        self.inventory_table.setAlternatingRowColors(True)
        
        layout.addWidget(self.inventory_table)
        
        self.tab_widget.addTab(inventory_widget, "📦 庫存管理")
        
    def setup_ros2_tab(self):
        """設置ROS2通訊標籤頁"""
        ros2_widget = QWidget()
        layout = QVBoxLayout(ros2_widget)
        
        # ROS2狀態
        status_group = QGroupBox("🤖 ROS2系統狀態")
        status_layout = QFormLayout(status_group)
        
        self.ros2_status_label = QLabel("未連接")
        self.ros2_status_label.setStyleSheet("color: red; font-weight: bold;")
        status_layout.addRow("連接狀態:", self.ros2_status_label)
        
        self.ros2_node_label = QLabel("無")
        status_layout.addRow("節點數量:", self.ros2_node_label)
        
        self.ros2_topic_label = QLabel("無")
        status_layout.addRow("活躍主題:", self.ros2_topic_label)
        
        layout.addWidget(status_group)
        
        # ROS2控制
        control_group = QGroupBox("🎮 ROS2控制")
        control_layout = QHBoxLayout(control_group)
        
        self.connect_ros2_btn = QPushButton("🔗 連接ROS2")
        self.connect_ros2_btn.clicked.connect(self.connect_ros2)
        control_layout.addWidget(self.connect_ros2_btn)
        
        self.disconnect_ros2_btn = QPushButton("🔌 斷開連接")
        self.disconnect_ros2_btn.clicked.connect(self.disconnect_ros2)
        control_layout.addWidget(self.disconnect_ros2_btn)
        
        self.send_test_msg_btn = QPushButton("📤 發送測試訊息")
        self.send_test_msg_btn.clicked.connect(self.send_test_message)
        control_layout.addWidget(self.send_test_msg_btn)
        
        layout.addWidget(control_group)
        
        # 訊息日誌
        log_group = QGroupBox("📝 訊息日誌")
        log_layout = QVBoxLayout(log_group)
        
        self.ros2_log = QTextEdit()
        self.ros2_log.setMaximumHeight(200)
        self.ros2_log.setReadOnly(True)
        log_layout.addWidget(self.ros2_log)
        
        # 清除日誌按鈕
        clear_log_btn = QPushButton("🗑️ 清除日誌")
        clear_log_btn.clicked.connect(self.ros2_log.clear)
        log_layout.addWidget(clear_log_btn)
        
        layout.addWidget(log_group)
        
        self.tab_widget.addTab(ros2_widget, "🤖 ROS2通訊")
        
    def setup_settings_tab(self):
        """設置系統設定標籤頁"""
        settings_widget = QWidget()
        layout = QVBoxLayout(settings_widget)
        
        # 一般設定
        general_group = QGroupBox("⚙️ 一般設定")
        general_layout = QFormLayout(general_group)
        
        self.auto_save_checkbox = QCheckBox("啟用自動儲存")
        self.auto_save_checkbox.setChecked(True)
        general_layout.addRow("自動儲存:", self.auto_save_checkbox)
        
        self.backup_checkbox = QCheckBox("啟用自動備份")
        self.backup_checkbox.setChecked(True)
        general_layout.addRow("自動備份:", self.backup_checkbox)
        
        self.low_stock_threshold = QSpinBox()
        self.low_stock_threshold.setRange(1, 1000)
        self.low_stock_threshold.setValue(50)
        general_layout.addRow("低庫存警告閾值:", self.low_stock_threshold)
        
        layout.addWidget(general_group)
        
        # 資料管理
        data_group = QGroupBox("💾 資料管理")
        data_layout = QHBoxLayout(data_group)
        
        self.export_data_btn = QPushButton("📤 匯出資料")
        self.export_data_btn.clicked.connect(self.export_data)
        data_layout.addWidget(self.export_data_btn)
        
        self.import_data_btn = QPushButton("📥 匯入資料")
        self.import_data_btn.clicked.connect(self.import_data)
        data_layout.addWidget(self.import_data_btn)
        
        self.backup_data_btn = QPushButton("💾 建立備份")
        self.backup_data_btn.clicked.connect(self.backup_data)
        data_layout.addWidget(self.backup_data_btn)
        
        layout.addWidget(data_group)
        
        # 關於
        about_group = QGroupBox("ℹ️ 關於")
        about_layout = QVBoxLayout(about_group)
        
        about_text = QTextEdit()
        about_text.setPlainText("""
🏥 Qt藥物管理系統 v1.0

功能特色:
• Qt圖形化使用者介面
• YAML格式資料儲存
• ROS2整合通訊
• 藥物庫存管理
• 處方籤處理
• 自動備份功能

開發者: AI Assistant
版本: 1.0.0
更新日期: 2024-12-02
        """)
        about_text.setReadOnly(True)
        about_text.setMaximumHeight(150)
        about_layout.addWidget(about_text)
        
        layout.addWidget(about_group)
        
        layout.addStretch()
        
        self.tab_widget.addTab(settings_widget, "⚙️ 系統設定")
        
    def setup_toolbar(self):
        """設置工具列"""
        toolbar = self.addToolBar("主要工具")
        toolbar.setMovable(False)
        
        # 新增藥物動作
        add_medicine_action = QAction("➕ 新增藥物", self)
        add_medicine_action.triggered.connect(self.add_medicine)
        toolbar.addAction(add_medicine_action)
        
        # 新增處方籤動作
        add_prescription_action = QAction("📋 新增處方籤", self)
        add_prescription_action.triggered.connect(self.add_prescription)
        toolbar.addAction(add_prescription_action)
        
        toolbar.addSeparator()
        
        # 重新整理動作
        refresh_action = QAction("🔄 重新整理", self)
        refresh_action.triggered.connect(self.refresh_all)
        toolbar.addAction(refresh_action)
        
        # 設定動作
        settings_action = QAction("⚙️ 設定", self)
        settings_action.triggered.connect(self.show_settings)
        toolbar.addAction(settings_action)
        
    def setup_menu_bar(self):
        """設置選單列"""
        menubar = self.menuBar()
        
        # 檔案選單
        file_menu = menubar.addMenu("📁 檔案")
        
        new_medicine_action = QAction("新增藥物", self)
        new_medicine_action.triggered.connect(self.add_medicine)
        file_menu.addAction(new_medicine_action)
        
        new_prescription_action = QAction("新增處方籤", self)
        new_prescription_action.triggered.connect(self.add_prescription)
        file_menu.addAction(new_prescription_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("離開", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # 編輯選單
        edit_menu = menubar.addMenu("✏️ 編輯")
        
        edit_medicine_action = QAction("編輯藥物", self)
        edit_medicine_action.triggered.connect(self.edit_medicine)
        edit_menu.addAction(edit_medicine_action)
        
        # 工具選單
        tools_menu = menubar.addMenu("🔧 工具")
        
        settings_action = QAction("設定", self)
        settings_action.triggered.connect(self.show_settings)
        tools_menu.addAction(settings_action)
        
        # 說明選單
        help_menu = menubar.addMenu("❓ 說明")
        
        about_action = QAction("關於", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
        
    def setup_status_bar(self):
        """設置狀態欄"""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
        # 狀態標籤
        self.status_label = QLabel("就緒")
        self.status_bar.addWidget(self.status_label)
        
        # 進度條
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.status_bar.addPermanentWidget(self.progress_bar)
        
    def setup_timers(self):
        """設置定時器"""
        # 自動儲存定時器
        self.auto_save_timer = QTimer()
        self.auto_save_timer.timeout.connect(self.auto_save)
        self.auto_save_timer.start(300000)  # 5分鐘
        
        # 狀態更新定時器
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)  # 1秒
        
    def create_icon(self):
        """創建應用程序圖標"""
        # 這裡可以載入自定義圖標
        return QIcon()
        
    def load_data(self):
        """載入資料"""
        try:
            self.data_manager.load_data()
            self.refresh_all()
            self.status_label.setText("資料載入完成")
        except Exception as e:
            QMessageBox.critical(self, "錯誤", f"載入資料失敗: {e}")
            
    def refresh_all(self):
        """重新整理所有資料"""
        self.refresh_medicine_table()
        self.refresh_prescription_table()
        self.refresh_inventory_table()
        self.update_status()
        
    def refresh_medicine_table(self):
        """重新整理藥物表格"""
        medicines = self.data_manager.get_medicines()
        self.medicine_table.setRowCount(len(medicines))
        
        for row, medicine in enumerate(medicines):
            self.medicine_table.setItem(row, 0, QTableWidgetItem(medicine['id']))
            self.medicine_table.setItem(row, 1, QTableWidgetItem(medicine['name']))
            self.medicine_table.setItem(row, 2, QTableWidgetItem(medicine['english_name']))
            self.medicine_table.setItem(row, 3, QTableWidgetItem(medicine['category']))
            self.medicine_table.setItem(row, 4, QTableWidgetItem(medicine['dosage_form']))
            self.medicine_table.setItem(row, 5, QTableWidgetItem(str(medicine['stock_quantity'])))
            self.medicine_table.setItem(row, 6, QTableWidgetItem(f"${medicine['price']:.2f}"))
            
            # 設置狀態顏色
            status_item = QTableWidgetItem("正常" if medicine['stock_quantity'] > 50 else "低庫存")
            if medicine['stock_quantity'] <= 50:
                status_item.setBackground(QColor(255, 200, 200))
            self.medicine_table.setItem(row, 7, status_item)
            
    def refresh_prescription_table(self):
        """重新整理處方籤表格"""
        prescriptions = self.data_manager.get_prescriptions()
        self.prescription_table.setRowCount(len(prescriptions))
        
        for row, prescription in enumerate(prescriptions):
            self.prescription_table.setItem(row, 0, QTableWidgetItem(prescription['id']))
            self.prescription_table.setItem(row, 1, QTableWidgetItem(prescription['patient_name']))
            self.prescription_table.setItem(row, 2, QTableWidgetItem(prescription['doctor_name']))
            self.prescription_table.setItem(row, 3, QTableWidgetItem(prescription['timestamp']))
            
            # 設置狀態顏色
            status_item = QTableWidgetItem(prescription['status'])
            if prescription['status'] == 'completed':
                status_item.setBackground(QColor(200, 255, 200))
            elif prescription['status'] == 'processing':
                status_item.setBackground(QColor(255, 255, 200))
            elif prescription['status'] == 'pending':
                status_item.setBackground(QColor(255, 200, 200))
            self.prescription_table.setItem(row, 4, status_item)
            
            self.prescription_table.setItem(row, 5, QTableWidgetItem(str(len(prescription['medicines']))))
            self.prescription_table.setItem(row, 6, QTableWidgetItem(f"${prescription['total_price']:.2f}"))
            
    def refresh_inventory_table(self):
        """重新整理庫存表格"""
        medicines = self.data_manager.get_medicines()
        self.inventory_table.setRowCount(len(medicines))
        
        total_value = 0
        low_stock_count = 0
        expiring_count = 0
        
        for row, medicine in enumerate(medicines):
            self.inventory_table.setItem(row, 0, QTableWidgetItem(medicine['name']))
            self.inventory_table.setItem(row, 1, QTableWidgetItem(str(medicine['stock_quantity'])))
            self.inventory_table.setItem(row, 2, QTableWidgetItem("50"))
            
            # 計算總價值
            value = medicine['stock_quantity'] * medicine['price']
            total_value += value
            
            # 檢查低庫存
            if medicine['stock_quantity'] <= 50:
                low_stock_count += 1
                status_item = QTableWidgetItem("低庫存")
                status_item.setBackground(QColor(255, 200, 200))
            else:
                status_item = QTableWidgetItem("正常")
                status_item.setBackground(QColor(200, 255, 200))
            self.inventory_table.setItem(row, 3, status_item)
            
            self.inventory_table.setItem(row, 4, QTableWidgetItem(medicine['expiry_date']))
            
            # 建議補貨
            if medicine['stock_quantity'] <= 50:
                self.inventory_table.setItem(row, 5, QTableWidgetItem("建議補貨"))
            else:
                self.inventory_table.setItem(row, 5, QTableWidgetItem("庫存充足"))
                
        # 更新概覽標籤
        self.total_medicines_label.setText(f"總藥物種類: {len(medicines)}")
        self.low_stock_label.setText(f"低庫存警告: {low_stock_count}")
        self.expiring_soon_label.setText(f"即將過期: {expiring_count}")
        self.total_value_label.setText(f"總庫存價值: ${total_value:.2f}")
        
    def filter_medicines(self):
        """篩選藥物"""
        search_text = self.medicine_search.text().lower()
        for row in range(self.medicine_table.rowCount()):
            name = self.medicine_table.item(row, 1).text().lower()
            english_name = self.medicine_table.item(row, 2).text().lower()
            category = self.medicine_table.item(row, 3).text().lower()
            
            if (search_text in name or search_text in english_name or 
                search_text in category):
                self.medicine_table.setRowHidden(row, False)
            else:
                self.medicine_table.setRowHidden(row, True)
                
    def filter_prescriptions(self):
        """篩選處方籤"""
        status_filter = self.status_filter.currentText()
        for row in range(self.prescription_table.rowCount()):
            status = self.prescription_table.item(row, 4).text()
            if status_filter == "全部狀態" or status == status_filter:
                self.prescription_table.setRowHidden(row, False)
            else:
                self.prescription_table.setRowHidden(row, True)
                
    def add_medicine(self):
        """新增藥物"""
        dialog = MedicineDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            medicine_data = dialog.get_medicine_data()
            
            # 先儲存到本地
            self.data_manager.add_medicine(medicine_data)
            
            # 通過ROS2服務同步到其他系統
            if self.ros2_interface:
                try:
                    result = self.ros2_interface.create_medicine_via_service(medicine_data)
                    if result:
                        self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 藥物已同步到ROS2系統: {medicine_data.get('name', '')}")
                    else:
                        self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 藥物同步失敗: {medicine_data.get('name', '')}")
                except Exception as e:
                    self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 藥物同步錯誤: {e}")
            
            self.refresh_medicine_table()
            self.refresh_inventory_table()
            self.status_label.setText("藥物新增成功")
            
    def edit_medicine(self):
        """編輯藥物"""
        current_row = self.medicine_table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "警告", "請選擇要編輯的藥物")
            return
            
        medicine_id = self.medicine_table.item(current_row, 0).text()
        medicine = self.data_manager.get_medicine_by_id(medicine_id)
        
        dialog = MedicineDialog(self, medicine)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            medicine_data = dialog.get_medicine_data()
            
            # 先更新本地資料
            self.data_manager.update_medicine(medicine_id, medicine_data)
            
            # 通過ROS2服務同步到其他系統
            if self.ros2_interface:
                try:
                    result = self.ros2_interface.update_medicine_via_service(medicine_id, medicine_data)
                    if result:
                        self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 藥物已同步更新到ROS2系統: {medicine_data.get('name', '')}")
                    else:
                        self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 藥物同步更新失敗: {medicine_data.get('name', '')}")
                except Exception as e:
                    self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 藥物同步更新錯誤: {e}")
            
            self.refresh_medicine_table()
            self.refresh_inventory_table()
            self.status_label.setText("藥物更新成功")
            
    def delete_medicine(self):
        """刪除藥物"""
        current_row = self.medicine_table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "警告", "請選擇要刪除的藥物")
            return
            
        medicine_id = self.medicine_table.item(current_row, 0).text()
        medicine_name = self.medicine_table.item(current_row, 1).text()
        
        reply = QMessageBox.question(
            self, "確認刪除", 
            f"確定要刪除藥物 '{medicine_name}' 嗎？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            # 先從本地刪除
            self.data_manager.delete_medicine(medicine_id)
            
            # 通過ROS2服務同步到其他系統
            if self.ros2_interface:
                try:
                    result = self.ros2_interface.delete_medicine_via_service(medicine_id)
                    if result:
                        self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 藥物已同步刪除到ROS2系統: {medicine_name}")
                    else:
                        self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 藥物同步刪除失敗: {medicine_name}")
                except Exception as e:
                    self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 藥物同步刪除錯誤: {e}")
            
            self.refresh_medicine_table()
            self.refresh_inventory_table()
            self.status_label.setText("藥物刪除成功")
            
    def add_prescription(self):
        """新增處方籤"""
        dialog = PrescriptionDialog(self, self.data_manager)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            prescription_data = dialog.get_prescription_data()
            self.data_manager.add_prescription(prescription_data)
            self.refresh_prescription_table()
            self.status_label.setText("處方籤新增成功")
            
    def process_prescription(self):
        """處理處方籤（串行處理）"""
        current_row = self.prescription_table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "警告", "請選擇要處理的處方籤")
            return
            
        prescription_id = self.prescription_table.item(current_row, 0).text()
        status = self.prescription_table.item(current_row, 4).text()
        
        if status == "completed":
            QMessageBox.information(self, "資訊", "此處方籤已完成處理")
            return
            
        # 檢查是否已在處理中
        if self.ros2_interface:
            order_status = self.ros2_interface.get_order_status()
            if order_status['processing_order']:
                QMessageBox.information(self, "資訊", f"當前正在處理訂單: {order_status['current_order']['id']}\n請等待完成後再處理下一筆")
                return
                
            # 檢查是否已在待處理列表中
            pending_ids = [order['id'] for order in order_status['pending_orders']]
            if prescription_id in pending_ids:
                QMessageBox.information(self, "資訊", "此處方籤已在待處理列表中")
                return
        
        # 更新狀態為處理中
        self.data_manager.update_prescription_status(prescription_id, "processing")
        
        # 發送ROS2訊息（串行處理）
        if self.ros2_interface:
            self.ros2_interface.send_prescription(prescription_id)
            self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 添加處方籤到處理佇列: {prescription_id}")
            
        self.refresh_prescription_table()
        self.status_label.setText("處方籤已加入處理佇列...")
        
    def view_prescription(self):
        """查看處方籤詳情"""
        current_row = self.prescription_table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "警告", "請選擇要查看的處方籤")
            return
            
        prescription_id = self.prescription_table.item(current_row, 0).text()
        prescription = self.data_manager.get_prescription_by_id(prescription_id)
        
        if prescription:
            details = f"""
處方籤詳情:
ID: {prescription['id']}
病人姓名: {prescription['patient_name']}
病人ID: {prescription['patient_id']}
醫師: {prescription['doctor_name']}
時間: {prescription['timestamp']}
狀態: {prescription['status']}
總價: ${prescription['total_price']:.2f}
備註: {prescription['notes']}

藥物清單:
"""
            for medicine in prescription['medicines']:
                details += f"• {medicine['medicine_name']} - {medicine['dosage']} - {medicine['frequency']} - {medicine['quantity']}個\n"
                
            QMessageBox.information(self, "處方籤詳情", details)
            
    def connect_ros2(self):
        """連接ROS2"""
        if self.ros2_interface:
            try:
                self.ros2_interface.connect()
                self.ros2_status_label.setText("已連接")
                self.ros2_status_label.setStyleSheet("color: green; font-weight: bold;")
                self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] ROS2連接成功")
            except Exception as e:
                self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] ROS2連接失敗: {e}")
                
    def disconnect_ros2(self):
        """斷開ROS2連接"""
        if self.ros2_interface:
            try:
                self.ros2_interface.disconnect()
                self.ros2_status_label.setText("未連接")
                self.ros2_status_label.setStyleSheet("color: red; font-weight: bold;")
                self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] ROS2連接已斷開")
            except Exception as e:
                self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 斷開連接失敗: {e}")
                
    def send_test_message(self):
        """發送測試訊息"""
        if self.ros2_interface:
            try:
                self.ros2_interface.send_test_message("Hello from Qt Medicine System!")
                self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 測試訊息已發送")
            except Exception as e:
                self.ros2_log.append(f"[{datetime.now().strftime('%H:%M:%S')}] 發送測試訊息失敗: {e}")
                
    def export_data(self):
        """匯出資料"""
        try:
            self.data_manager.export_data()
            QMessageBox.information(self, "成功", "資料匯出成功")
        except Exception as e:
            QMessageBox.critical(self, "錯誤", f"資料匯出失敗: {e}")
            
    def import_data(self):
        """匯入資料"""
        try:
            self.data_manager.import_data()
            self.refresh_all()
            QMessageBox.information(self, "成功", "資料匯入成功")
        except Exception as e:
            QMessageBox.critical(self, "錯誤", f"資料匯入失敗: {e}")
            
    def backup_data(self):
        """建立備份"""
        try:
            self.data_manager.backup_data()
            QMessageBox.information(self, "成功", "備份建立成功")
        except Exception as e:
            QMessageBox.critical(self, "錯誤", f"備份建立失敗: {e}")
            
    def show_settings(self):
        """顯示設定對話框"""
        dialog = SettingsDialog(self)
        dialog.exec()
        
    def show_about(self):
        """顯示關於對話框"""
        QMessageBox.about(self, "關於", 
                         "🏥 Qt藥物管理系統 v1.0\n\n"
                         "整合Qt GUI、YAML資料儲存和ROS2通訊的完整藥物管理解決方案。")
        
    def auto_save(self):
        """自動儲存"""
        if self.auto_save_checkbox.isChecked():
            try:
                self.data_manager.save_data()
                self.status_label.setText("自動儲存完成")
            except Exception as e:
                self.status_label.setText(f"自動儲存失敗: {e}")
                
    def update_status(self):
        """更新狀態"""
        # 更新ROS2狀態
        if self.ros2_interface:
            try:
                node_count = self.ros2_interface.get_node_count()
                topic_count = self.ros2_interface.get_topic_count()
                self.ros2_node_label.setText(str(node_count))
                self.ros2_topic_label.setText(str(topic_count))
                
                # 更新訂單處理狀態
                processing_status = self.ros2_interface.get_processing_status()
                self.status_label.setText(f"ROS2: {processing_status}")
                
            except:
                pass
                
    def on_ros2_status_update(self, event_type: str, data):
        """ROS2狀態更新回調"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        
        if event_type == 'order_processing_started':
            self.ros2_log.append(f"[{timestamp}] 🚀 開始處理訂單: {data['id']}")
            self.status_label.setText(f"處理中: {data['id']}")
            
        elif event_type == 'order_completed':
            self.ros2_log.append(f"[{timestamp}] ✅ 訂單處理完成: {data['id']} - {data.get('result', 'Unknown')}")
            self.status_label.setText(f"完成: {data['id']}")
            
            # 更新處方籤狀態
            if data.get('result') == 'success':
                self.data_manager.update_prescription_status(data['id'], 'completed')
            else:
                self.data_manager.update_prescription_status(data['id'], 'failed')
                
            self.refresh_prescription_table()
            
        elif event_type == 'prescription_response':
            self.ros2_log.append(f"[{timestamp}] 📋 處方籤回應: {data}")
            
        elif event_type == 'status_update':
            self.ros2_log.append(f"[{timestamp}] 📊 狀態更新: {data}")
            
        elif event_type == 'medicine_info_response':
            self.ros2_log.append(f"[{timestamp}] 💊 藥物資訊回應: {data}")
            
        elif event_type == 'error':
            self.ros2_log.append(f"[{timestamp}] ❌ 錯誤: {data}")
                
    def closeEvent(self, event):
        """關閉事件"""
        try:
            self.data_manager.save_data()
            if self.ros2_interface:
                self.ros2_interface.cleanup()
            rclpy.shutdown()
        except:
            pass
        event.accept()


def main():
    """主函數"""
    app = QApplication(sys.argv)
    
    # 設置應用程序樣式
    app.setStyle('Fusion')
    
    # 創建主視窗
    window = QtMedicineSystem()
    window.show()
    
    # 執行應用程序
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
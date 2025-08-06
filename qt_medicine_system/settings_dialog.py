#!/usr/bin/env python3
"""
⚙️ 設定對話框模組
用於系統設定管理
"""

import sys
from typing import Dict, Any

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLineEdit,
    QSpinBox, QCheckBox, QPushButton, QDialogButtonBox, QLabel,
    QGroupBox, QTabWidget, QTextEdit, QComboBox, QSlider
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


class SettingsDialog(QDialog):
    """設定對話框"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.settings = {}
        self.init_ui()
        self.load_settings()
        
    def init_ui(self):
        """初始化使用者介面"""
        self.setWindowTitle("⚙️ 系統設定")
        self.setModal(True)
        self.setMinimumWidth(600)
        self.setMinimumHeight(500)
        
        # 創建主佈局
        layout = QVBoxLayout(self)
        
        # 創建標籤頁
        self.tab_widget = QTabWidget()
        layout.addWidget(self.tab_widget)
        
        # 添加各個標籤頁
        self.setup_general_tab()
        self.setup_ros2_tab()
        self.setup_data_tab()
        self.setup_advanced_tab()
        
        # 按鈕
        button_layout = QHBoxLayout()
        
        # 重置按鈕
        reset_btn = QPushButton("🔄 重置為預設值")
        reset_btn.clicked.connect(self.reset_to_defaults)
        button_layout.addWidget(reset_btn)
        
        button_layout.addStretch()
        
        # 標準按鈕
        button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        button_layout.addWidget(button_box)
        
        layout.addLayout(button_layout)
        
    def setup_general_tab(self):
        """設置一般設定標籤頁"""
        general_widget = QWidget()
        layout = QVBoxLayout(general_widget)
        
        # 自動儲存設定
        auto_save_group = QGroupBox("💾 自動儲存設定")
        auto_save_layout = QFormLayout(auto_save_group)
        
        self.auto_save_checkbox = QCheckBox("啟用自動儲存")
        auto_save_layout.addRow("自動儲存:", self.auto_save_checkbox)
        
        self.auto_save_interval = QSpinBox()
        self.auto_save_interval.setRange(1, 60)
        self.auto_save_interval.setSuffix(" 分鐘")
        auto_save_layout.addRow("儲存間隔:", self.auto_save_interval)
        
        layout.addWidget(auto_save_group)
        
        # 備份設定
        backup_group = QGroupBox("💾 備份設定")
        backup_layout = QFormLayout(backup_group)
        
        self.backup_checkbox = QCheckBox("啟用自動備份")
        backup_layout.addRow("自動備份:", self.backup_checkbox)
        
        self.backup_interval = QSpinBox()
        self.backup_interval.setRange(1, 30)
        self.backup_interval.setSuffix(" 天")
        backup_layout.addRow("備份間隔:", self.backup_interval)
        
        layout.addWidget(backup_group)
        
        # 庫存警告設定
        inventory_group = QGroupBox("📦 庫存警告設定")
        inventory_layout = QFormLayout(inventory_group)
        
        self.low_stock_threshold = QSpinBox()
        self.low_stock_threshold.setRange(1, 1000)
        self.low_stock_threshold.setSuffix(" 個")
        inventory_layout.addRow("低庫存警告閾值:", self.low_stock_threshold)
        
        self.expiry_warning_days = QSpinBox()
        self.expiry_warning_days.setRange(1, 365)
        self.expiry_warning_days.setSuffix(" 天")
        inventory_layout.addRow("過期警告天數:", self.expiry_warning_days)
        
        layout.addWidget(inventory_group)
        
        layout.addStretch()
        
        self.tab_widget.addTab(general_widget, "⚙️ 一般設定")
        
    def setup_ros2_tab(self):
        """設置ROS2設定標籤頁"""
        ros2_widget = QWidget()
        layout = QVBoxLayout(ros2_widget)
        
        # ROS2連接設定
        connection_group = QGroupBox("🔗 ROS2連接設定")
        connection_layout = QFormLayout(connection_group)
        
        self.ros2_domain_id = QSpinBox()
        self.ros2_domain_id.setRange(0, 255)
        self.ros2_domain_id.setValue(0)
        connection_layout.addRow("Domain ID:", self.ros2_domain_id)
        
        self.ros2_node_name = QLineEdit()
        self.ros2_node_name.setText("qt_medicine_system")
        connection_layout.addRow("節點名稱:", self.ros2_node_name)
        
        self.auto_connect_ros2 = QCheckBox("啟動時自動連接ROS2")
        connection_layout.addRow("自動連接:", self.auto_connect_ros2)
        
        layout.addWidget(connection_group)
        
        # ROS2服務設定
        service_group = QGroupBox("🤖 ROS2服務設定")
        service_layout = QFormLayout(service_group)
        
        self.prescription_service_name = QLineEdit()
        self.prescription_service_name.setText("/medicine/prescription_service")
        service_layout.addRow("處方籤服務:", self.prescription_service_name)
        
        self.medicine_info_service_name = QLineEdit()
        self.medicine_info_service_name.setText("/medicine/info_service")
        service_layout.addRow("藥物資訊服務:", self.medicine_info_service_name)
        
        self.order_processor_service_name = QLineEdit()
        self.order_processor_service_name.setText("/medicine/order_processor")
        service_layout.addRow("訂單處理服務:", self.order_processor_service_name)
        
        layout.addWidget(service_group)
        
        # ROS2主題設定
        topic_group = QGroupBox("📡 ROS2主題設定")
        topic_layout = QFormLayout(topic_group)
        
        self.prescription_topic = QLineEdit()
        self.prescription_topic.setText("/medicine/prescription_request")
        topic_layout.addRow("處方籤請求主題:", self.prescription_topic)
        
        self.prescription_response_topic = QLineEdit()
        self.prescription_response_topic.setText("/medicine/prescription_response")
        topic_layout.addRow("處方籤回應主題:", self.prescription_response_topic)
        
        self.status_topic = QLineEdit()
        self.status_topic.setText("/medicine/system_status")
        topic_layout.addRow("系統狀態主題:", self.status_topic)
        
        layout.addWidget(topic_group)
        
        layout.addStretch()
        
        self.tab_widget.addTab(ros2_widget, "🤖 ROS2設定")
        
    def setup_data_tab(self):
        """設置資料設定標籤頁"""
        data_widget = QWidget()
        layout = QVBoxLayout(data_widget)
        
        # 資料儲存設定
        storage_group = QGroupBox("💾 資料儲存設定")
        storage_layout = QFormLayout(storage_group)
        
        self.data_directory = QLineEdit()
        self.data_directory.setText("data")
        storage_layout.addRow("資料目錄:", self.data_directory)
        
        self.backup_directory = QLineEdit()
        self.backup_directory.setText("backups")
        storage_layout.addRow("備份目錄:", self.backup_directory)
        
        self.export_directory = QLineEdit()
        self.export_directory.setText("exports")
        storage_layout.addRow("匯出目錄:", self.export_directory)
        
        layout.addWidget(storage_group)
        
        # 資料清理設定
        cleanup_group = QGroupBox("🧹 資料清理設定")
        cleanup_layout = QFormLayout(cleanup_group)
        
        self.max_prescription_history = QSpinBox()
        self.max_prescription_history.setRange(100, 10000)
        self.max_prescription_history.setSuffix(" 筆")
        cleanup_layout.addRow("最大處方籤歷史:", self.max_prescription_history)
        
        self.cleanup_old_data = QCheckBox("自動清理舊資料")
        cleanup_layout.addRow("自動清理:", self.cleanup_old_data)
        
        self.cleanup_days = QSpinBox()
        self.cleanup_days.setRange(30, 3650)
        self.cleanup_days.setSuffix(" 天")
        cleanup_layout.addRow("清理天數:", self.cleanup_days)
        
        layout.addWidget(cleanup_group)
        
        # 資料格式設定
        format_group = QGroupBox("📄 資料格式設定")
        format_layout = QFormLayout(format_group)
        
        self.default_export_format = QComboBox()
        self.default_export_format.addItems(["YAML", "JSON", "CSV"])
        format_layout.addRow("預設匯出格式:", self.default_export_format)
        
        self.include_timestamp = QCheckBox("匯出時包含時間戳")
        format_layout.addRow("包含時間戳:", self.include_timestamp)
        
        layout.addWidget(format_group)
        
        layout.addStretch()
        
        self.tab_widget.addTab(data_widget, "💾 資料設定")
        
    def setup_advanced_tab(self):
        """設置進階設定標籤頁"""
        advanced_widget = QWidget()
        layout = QVBoxLayout(advanced_widget)
        
        # 效能設定
        performance_group = QGroupBox("⚡ 效能設定")
        performance_layout = QFormLayout(performance_group)
        
        self.refresh_interval = QSpinBox()
        self.refresh_interval.setRange(1, 60)
        self.refresh_interval.setSuffix(" 秒")
        performance_layout.addRow("重新整理間隔:", self.refresh_interval)
        
        self.max_table_rows = QSpinBox()
        self.max_table_rows.setRange(100, 10000)
        self.max_table_rows.setSuffix(" 行")
        performance_layout.addRow("最大表格行數:", self.max_table_rows)
        
        self.enable_animations = QCheckBox("啟用動畫效果")
        performance_layout.addRow("動畫效果:", self.enable_animations)
        
        layout.addWidget(performance_group)
        
        # 日誌設定
        logging_group = QGroupBox("📝 日誌設定")
        logging_layout = QFormLayout(logging_group)
        
        self.log_level = QComboBox()
        self.log_level.addItems(["DEBUG", "INFO", "WARNING", "ERROR"])
        logging_layout.addRow("日誌等級:", self.log_level)
        
        self.enable_file_logging = QCheckBox("啟用檔案日誌")
        logging_layout.addRow("檔案日誌:", self.enable_file_logging)
        
        self.max_log_files = QSpinBox()
        self.max_log_files.setRange(1, 100)
        self.max_log_files.setSuffix(" 個")
        logging_layout.addRow("最大日誌檔案:", self.max_log_files)
        
        layout.addWidget(logging_group)
        
        # 除錯設定
        debug_group = QGroupBox("🐛 除錯設定")
        debug_layout = QFormLayout(debug_group)
        
        self.enable_debug_mode = QCheckBox("啟用除錯模式")
        debug_layout.addRow("除錯模式:", self.enable_debug_mode)
        
        self.show_ros2_debug = QCheckBox("顯示ROS2除錯資訊")
        debug_layout.addRow("ROS2除錯:", self.show_ros2_debug)
        
        self.enable_performance_monitoring = QCheckBox("啟用效能監控")
        debug_layout.addRow("效能監控:", self.enable_performance_monitoring)
        
        layout.addWidget(debug_group)
        
        layout.addStretch()
        
        self.tab_widget.addTab(advanced_widget, "🔧 進階設定")
        
    def load_settings(self):
        """載入設定"""
        # 這裡應該從設定檔案或資料庫載入設定
        # 暫時使用預設值
        self.auto_save_checkbox.setChecked(True)
        self.auto_save_interval.setValue(5)
        self.backup_checkbox.setChecked(True)
        self.backup_interval.setValue(7)
        self.low_stock_threshold.setValue(50)
        self.expiry_warning_days.setValue(30)
        
        self.auto_connect_ros2.setChecked(True)
        self.max_prescription_history.setValue(1000)
        self.cleanup_old_data.setChecked(True)
        self.cleanup_days.setValue(365)
        
        self.refresh_interval.setValue(5)
        self.max_table_rows.setValue(1000)
        self.enable_animations.setChecked(True)
        
        self.log_level.setCurrentText("INFO")
        self.enable_file_logging.setChecked(True)
        self.max_log_files.setValue(10)
        
    def get_settings(self) -> Dict[str, Any]:
        """獲取設定"""
        return {
            # 一般設定
            'auto_save_enabled': self.auto_save_checkbox.isChecked(),
            'auto_save_interval': self.auto_save_interval.value(),
            'backup_enabled': self.backup_checkbox.isChecked(),
            'backup_interval': self.backup_interval.value(),
            'low_stock_threshold': self.low_stock_threshold.value(),
            'expiry_warning_days': self.expiry_warning_days.value(),
            
            # ROS2設定
            'ros2_domain_id': self.ros2_domain_id.value(),
            'ros2_node_name': self.ros2_node_name.text(),
            'auto_connect_ros2': self.auto_connect_ros2.isChecked(),
            'prescription_service_name': self.prescription_service_name.text(),
            'medicine_info_service_name': self.medicine_info_service_name.text(),
            'order_processor_service_name': self.order_processor_service_name.text(),
            'prescription_topic': self.prescription_topic.text(),
            'prescription_response_topic': self.prescription_response_topic.text(),
            'status_topic': self.status_topic.text(),
            
            # 資料設定
            'data_directory': self.data_directory.text(),
            'backup_directory': self.backup_directory.text(),
            'export_directory': self.export_directory.text(),
            'max_prescription_history': self.max_prescription_history.value(),
            'cleanup_old_data': self.cleanup_old_data.isChecked(),
            'cleanup_days': self.cleanup_days.value(),
            'default_export_format': self.default_export_format.currentText(),
            'include_timestamp': self.include_timestamp.isChecked(),
            
            # 進階設定
            'refresh_interval': self.refresh_interval.value(),
            'max_table_rows': self.max_table_rows.value(),
            'enable_animations': self.enable_animations.isChecked(),
            'log_level': self.log_level.currentText(),
            'enable_file_logging': self.enable_file_logging.isChecked(),
            'max_log_files': self.max_log_files.value(),
            'enable_debug_mode': self.enable_debug_mode.isChecked(),
            'show_ros2_debug': self.show_ros2_debug.isChecked(),
            'enable_performance_monitoring': self.enable_performance_monitoring.isChecked()
        }
        
    def reset_to_defaults(self):
        """重置為預設值"""
        from PyQt6.QtWidgets import QMessageBox
        
        reply = QMessageBox.question(
            self, "確認重置", 
            "確定要重置所有設定為預設值嗎？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            self.load_settings()
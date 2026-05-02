#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
现代化交通图编辑器
采用Material Design风格设计
"""

import sys
import os
import signal
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QFrame, QMessageBox, 
                             QInputDialog, QSplitter, QGraphicsDropShadowEffect,
                             QScrollArea, QGridLayout, QSpacerItem, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, QSize
from PyQt5.QtGui import QFont, QColor, QPalette, QLinearGradient, QPainter
from geometry_msgs.msg import Pose
from fleet_msgs.srv import LoadTrafficMap, SaveTrafficMap, SubmitTask

try:
    from qt_material import apply_stylesheet
    HAS_MATERIAL = True
except ImportError:
    HAS_MATERIAL = False

try:
    from .traffic_map_widget import TrafficMapWidget
    from .ui_sections import create_title_section as build_title_section
    from .ui_sections import create_waypoint_card as build_waypoint_card
    from .editor_actions import (
        clear_waypoints as action_clear_waypoints,
        load_map_yaml as action_load_map_yaml,
        load_traffic_map as action_load_traffic_map,
        save_traffic_map as action_save_traffic_map,
        submit_task as action_submit_task,
    )
except ImportError:
    if __package__ in (None, ""):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        if current_dir not in sys.path:
            sys.path.insert(0, current_dir)
        from traffic_map_widget import TrafficMapWidget
        from ui_sections import create_title_section as build_title_section
        from ui_sections import create_waypoint_card as build_waypoint_card
        from editor_actions import (
            clear_waypoints as action_clear_waypoints,
            load_map_yaml as action_load_map_yaml,
            load_traffic_map as action_load_traffic_map,
            save_traffic_map as action_save_traffic_map,
            submit_task as action_submit_task,
        )
    else:
        raise

# 现代化样式表
try:
    from .ui_style import MODERN_STYLE
except ImportError:
    from ui_style import MODERN_STYLE


class TrafficEditorNode(Node):
    def __init__(self):
        super().__init__('traffic_editor')
        self.load_map_client = self.create_client(LoadTrafficMap, '/fleet_manager/load_traffic_map')
        self.save_map_client = self.create_client(SaveTrafficMap, '/fleet_manager/save_traffic_map')
        self.submit_task_client = self.create_client(SubmitTask, '/fleet_manager/submit_task')


class ModernButton(QPushButton):
    """现代化按钮，带有阴影效果"""
    def __init__(self, text, icon_text="", parent=None):
        super().__init__(text, parent)
        self.setCursor(Qt.PointingHandCursor)
        
        # 添加阴影效果
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(10)
        shadow.setColor(QColor(14, 165, 233, 60))
        shadow.setOffset(0, 2)
        self.setGraphicsEffect(shadow)
        
        # 设置最小尺寸
        self.setMinimumHeight(45)


class ModernCard(QFrame):
    """现代化卡片容器"""
    def __init__(self, title="", parent=None):
        super().__init__(parent)
        self.setObjectName("cardFrame")
        
        # 添加阴影效果
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(14)
        shadow.setColor(QColor(2, 132, 199, 30))
        shadow.setOffset(0, 3)
        self.setGraphicsEffect(shadow)
        
        # 布局
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 10, 16, 14)
        layout.setSpacing(10)
        
        if title:
            title_label = QLabel(title)
            title_label.setObjectName("sectionTitle")
            title_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            layout.addWidget(title_label)


class TrafficEditorMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FleetOS 车队交通图编辑器")
        self.setGeometry(100, 100, 1400, 900)
        
        self.ros_node = None
        self.current_file = None
        self.map_yaml_path = None
        
        self.init_ui()
        self.init_ros()
        self.init_timer()
        self.apply_modern_style()
        
    def apply_modern_style(self):
        """应用现代化样式 - 使用自定义亮色主题"""
        self.setStyleSheet(MODERN_STYLE)

    def set_interaction_mode(self, mode: str):
        """切换地图交互模式（供右侧按钮调用）"""
        self.map_widget.set_interaction_mode(mode)
        label_map = {
            "pan": "移动地图",
            "add": "添加航点",
            "connect": "连接航点",
            "edit": "编辑航点属性",
        }
        if hasattr(self, "mode_label"):
            self.mode_label.setText(f"当前模式: {label_map.get(mode, mode)}")
        status_map = {
            "pan": "左键拖拽移动地图",
            "add": "添加航点：点击地图空白处添加（自动编号）",
            "connect": "连接航线：先点起点，再点终点建立连接",
            "edit": "编辑属性：点击航点修改名称/停车/充电",
        }
        self.statusBar().showMessage(status_map.get(mode, "mode switched"))
    
    def init_ui(self):
        # 中央部件
        central_frame = QFrame()
        central_frame.setObjectName("centralFrame")
        self.setCentralWidget(central_frame)
        
        # 主布局
        main_layout = QVBoxLayout(central_frame)
        main_layout.setContentsMargins(30, 30, 30, 30)
        main_layout.setSpacing(20)
        
        # 标题区域
        self.create_title_section(main_layout)
        
        # 内容区域
        content_layout = QHBoxLayout()
        content_layout.setSpacing(20)
        
        # 左侧：地图编辑区域
        left_frame = ModernCard("交通图编辑区")
        left_layout = left_frame.layout()
        
        # 地图控件
        self.map_widget = TrafficMapWidget()
        self.map_widget.setMinimumHeight(600)
        left_layout.addWidget(self.map_widget)
        
        # 地图控制按钮
        map_buttons_layout = QHBoxLayout()
        map_buttons_layout.setSpacing(10)
        
        btn_load_map = ModernButton("加载地图")
        btn_load_map.clicked.connect(self.load_map_yaml)
        map_buttons_layout.addWidget(btn_load_map)
        
        btn_load_traffic = ModernButton("加载交通图")
        btn_load_traffic.setObjectName("secondaryButton")
        btn_load_traffic.clicked.connect(self.load_traffic_map)
        map_buttons_layout.addWidget(btn_load_traffic)
        
        btn_save_traffic = ModernButton("保存交通图")
        btn_save_traffic.clicked.connect(self.save_traffic_map)
        map_buttons_layout.addWidget(btn_save_traffic)
        
        left_layout.addLayout(map_buttons_layout)
        content_layout.addWidget(left_frame, stretch=3)
        
        # 右侧：控制面板
        right_frame = QFrame()
        right_frame.setMaximumWidth(350)
        right_frame.setMinimumWidth(300)
        right_layout = QVBoxLayout(right_frame)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(15)
        
        # 航点管理卡片
        self.create_waypoint_card(right_layout)
        
        # 任务提交/系统状态/操作提示卡片默认不展示（生产简约风）
        
        right_layout.addStretch()
        content_layout.addWidget(right_frame, stretch=1)
        
        main_layout.addLayout(content_layout)
        
        # 状态栏
        self.statusBar().showMessage("Ready - select interaction mode on the right, then left-click to operate")
        
        # 连接信号
        self.map_widget.waypoint_added.connect(self.on_waypoint_added)
        self.map_widget.waypoint_removed.connect(self.on_waypoint_removed)
        self.map_widget.connection_added.connect(self.on_connection_added)
        self.map_widget.connection_removed.connect(self.on_connection_removed)
        self.map_widget.map_loaded.connect(self.on_map_loaded)
        
    def create_title_section(self, layout):
        build_title_section(self, layout)
        
    def create_help_card(self, layout):
        """创建操作提示卡片"""
        card = ModernCard("操作提示")
        card_layout = card.layout()
        
        tips = [
            "默认：左键拖拽移动地图",
            "模式：右侧选择添加/连接/编辑后，再用左键操作地图",
            "滚轮：缩放地图",
            "中键拖拽：移动地图（备用）",
            "删除航点：右键删除",
        ]
        
        for tip in tips:
            tip_label = QLabel(tip)
            tip_label.setStyleSheet("color: #666; padding: 3px;")
            card_layout.addWidget(tip_label)
        
        layout.addWidget(card)
        
    def create_waypoint_card(self, layout):
        build_waypoint_card(self, layout, ModernCard, ModernButton)

        # 默认模式
        try:
            self.map_widget.set_interaction_mode("pan")
        except Exception:
            pass
        
    def create_task_card(self, layout):
        """创建任务提交卡片"""
        card = ModernCard("任务提交")
        card_layout = card.layout()
        
        # 说明
        info_label = QLabel("选择目标航点后提交任务")
        info_label.setStyleSheet("color: #666; margin-bottom: 10px;")
        info_label.setWordWrap(True)
        card_layout.addWidget(info_label)
        
        # 提交按钮
        self.btn_submit_task = ModernButton("提交导航任务")
        self.btn_submit_task.clicked.connect(self.submit_task)
        card_layout.addWidget(self.btn_submit_task)
        
        layout.addWidget(card)
        
    def create_stats_card(self, layout):
        """创建统计信息卡片"""
        card = ModernCard("系统状态")
        card_layout = card.layout()
        
        # 统计项
        stats_items = [
            ("监控机器人", "0"),
            ("在线机器人", "0"),
            ("待处理任务", "0"),
            ("执行中任务", "0"),
        ]
        
        self.stats_labels = {}
        for name, value in stats_items:
            stat_layout = QHBoxLayout()
            
            name_label = QLabel(f"{name}:")
            name_label.setStyleSheet("color: #666;")
            stat_layout.addWidget(name_label)
            
            stat_layout.addStretch()
            
            value_label = QLabel(value)
            value_label.setStyleSheet("color: #667eea; font-weight: bold; font-size: 16px;")
            stat_layout.addWidget(value_label)
            
            self.stats_labels[name] = value_label
            
            card_layout.addLayout(stat_layout)
        
        layout.addWidget(card)
        
    def init_ros(self):
        try:
            rclpy.init(args=None)
            self.ros_node = TrafficEditorNode()
            self.status_indicator.setText("● Online")
            self.status_indicator.setStyleSheet("color: #4caf50; font-weight: bold; font-size: 14px;")
        except Exception as e:
            self.status_indicator.setText("● Offline")
            self.status_indicator.setStyleSheet("color: #f44336; font-weight: bold; font-size: 14px;")
            QMessageBox.warning(self, "ROS警告", f"ROS初始化失败: {str(e)}\n部分功能将不可用。")
    
    def init_timer(self):
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)
        
        # 统计更新定时器
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self.update_stats)
        self.stats_timer.start(1000)
    
    def spin_ros(self):
        # 退出/重启阶段时，定时器可能仍会触发；这里要避免在 rclpy.shutdown 后继续 spin_once
        if not self.ros_node:
            return
        if not rclpy.ok():
            return
        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0)
        except Exception:
            # 一旦 WaitSet 初始化失败，停止定时器，避免日志刷屏/卡死
            try:
                self.ros_timer.stop()
            except Exception:
                pass
            self.ros_node = None
    
    def update_stats(self):
        """更新统计信息"""
        # 如果你把统计卡片删除了，labels 可能不存在
        if not hasattr(self, "waypoint_count_label") or not hasattr(self, "connection_count_label"):
            return
        waypoint_count = len(self.map_widget.waypoints)
        connection_count = len(self.map_widget.connections)
        
        self.waypoint_count_label.setText(f"航点数量: {waypoint_count}")
        self.connection_count_label.setText(f"航线数量: {connection_count}")
    
    def load_map_yaml(self):
        action_load_map_yaml(self)
    
    def load_traffic_map(self):
        action_load_traffic_map(self)
    
    def save_traffic_map(self):
        action_save_traffic_map(self)
    
    def submit_task(self):
        action_submit_task(self)
    
    def clear_waypoints(self):
        action_clear_waypoints(self)
    
    def on_waypoint_added(self, waypoint_id, pose):
        self.statusBar().showMessage(f"Added waypoint: {waypoint_id}")
        self.update_stats()
    
    def on_waypoint_removed(self, waypoint_id):
        self.statusBar().showMessage(f"Deleted waypoint: {waypoint_id}")
        self.update_stats()
    
    def on_connection_added(self, from_id, to_id):
        self.statusBar().showMessage(f"Added connection: {from_id} <-> {to_id}")
        self.update_stats()
    
    def on_connection_removed(self, from_id, to_id):
        self.statusBar().showMessage(f"Removed connection: {from_id} <-> {to_id}")
        self.update_stats()
    
    def on_map_loaded(self, map_path):
        self.statusBar().showMessage(f"Loaded map: {os.path.basename(map_path)}")
    
    def closeEvent(self, event):
        # 先停定时器，避免 shutdown 后 timer 继续 spin_once
        try:
            if hasattr(self, "ros_timer") and self.ros_timer:
                self.ros_timer.stop()
        except Exception:
            pass
        try:
            if hasattr(self, "stats_timer") and self.stats_timer:
                self.stats_timer.stop()
        except Exception:
            pass

        # 再销毁 node / shutdown（只在 rclpy 已初始化时进行）
        try:
            if self.ros_node and rclpy.ok():
                self.ros_node.destroy_node()
        except Exception:
            pass
        self.ros_node = None

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        event.accept()


def main(args=None):
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    
    # 设置应用信息
    app.setApplicationName("车队交通图编辑器")
    app.setOrganizationName("Fleet Management System")
    
    # 设置字体
    font = QFont("Microsoft YaHei", 10)
    app.setFont(font)
    
    window = TrafficEditorMainWindow()
    window.show()

    # 让 Ctrl+C 触发正常退出（Qt 主循环退出）
    def _sigint_handler(signum, frame):
        try:
            app.quit()
        except Exception:
            pass

    try:
        signal.signal(signal.SIGINT, _sigint_handler)
    except Exception:
        pass
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FleetOS 交通图编辑器主窗口模块。

实现基于 PyQt5 的现代化交通图编辑器主界面，采用 Material Design 风格的
亮色主题，提供地图加载、航点编辑、航线连接、任务提交等功能的完整交互界面。
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

# -----------------------------------------------------------------------
# 样式表导入
# -----------------------------------------------------------------------
try:
    from .ui_style import MODERN_STYLE
except ImportError:
    from ui_style import MODERN_STYLE


class TrafficEditorNode(Node):
    """ROS 2 通信节点。

    管理与后端 fleet_manager 节点之间的 ROS 服务客户端，
    提供交通图加载、保存以及任务提交等服务的异步调用能力。
    """

    def __init__(self):
        super().__init__('traffic_editor')
        self.load_map_client = self.create_client(LoadTrafficMap, '/fleet_manager/load_traffic_map')
        self.save_map_client = self.create_client(SaveTrafficMap, '/fleet_manager/save_traffic_map')
        self.submit_task_client = self.create_client(SubmitTask, '/fleet_manager/submit_task')


class ModernButton(QPushButton):
    """现代化样式按钮组件。

    继承自 QPushButton，内置阴影效果和手型光标样式，
    提供一致性的 Material Design 风格按钮外观。
    """

    def __init__(self, text, icon_text="", parent=None):
        super().__init__(text, parent)
        self.setCursor(Qt.PointingHandCursor)

        # 添加按钮阴影效果
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(10)
        shadow.setColor(QColor(14, 165, 233, 60))
        shadow.setOffset(0, 2)
        self.setGraphicsEffect(shadow)

        # 设置按钮最小高度
        self.setMinimumHeight(45)


class ModernCard(QFrame):
    """现代化卡片容器组件。

    继承自 QFrame，内置圆角边框和阴影效果，
    用于组织和管理界面中的分组内容区域。
    """

    def __init__(self, title="", parent=None):
        super().__init__(parent)
        self.setObjectName("cardFrame")

        # 添加卡片阴影效果
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(14)
        shadow.setColor(QColor(2, 132, 199, 30))
        shadow.setOffset(0, 3)
        self.setGraphicsEffect(shadow)

        # 设置卡片内部布局
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 10, 16, 14)
        layout.setSpacing(10)

        if title:
            title_label = QLabel(title)
            title_label.setObjectName("sectionTitle")
            title_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            layout.addWidget(title_label)


class TrafficEditorMainWindow(QMainWindow):
    """交通图编辑器主窗口。

    集成地图可视化、航点编辑、航线管理、任务提交等核心功能，
    提供完整的图形化车队交通图编辑界面。
    """

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
        """应用现代化亮色主题样式。

        使用自定义的 Material Design 风格 QSS 样式表覆盖窗口默认外观。
        """
        self.setStyleSheet(MODERN_STYLE)

    def set_interaction_mode(self, mode: str):
        """切换地图交互模式。

        根据传入的模式名称切换地图控件的鼠标交互行为，
        并同步更新模式标签与状态栏提示信息。

        Args:
            mode: 交互模式名称，取值范围为 "pan"、"add"、"connect"、"edit"。
        """
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
        """初始化用户界面布局。

        构建主窗口的完整 UI 结构，包括标题栏、地图编辑区、
        右侧控制面板及状态栏，并连接地图控件的相关信号。
        """
        # -----------------------------------------------------------------------
        # 中央容器
        # -----------------------------------------------------------------------
        central_frame = QFrame()
        central_frame.setObjectName("centralFrame")
        self.setCentralWidget(central_frame)

        # -----------------------------------------------------------------------
        # 主布局
        # -----------------------------------------------------------------------
        main_layout = QVBoxLayout(central_frame)
        main_layout.setContentsMargins(30, 30, 30, 30)
        main_layout.setSpacing(20)

        # -----------------------------------------------------------------------
        # 标题区域
        # -----------------------------------------------------------------------
        self.create_title_section(main_layout)

        # -----------------------------------------------------------------------
        # 内容区域（左右分栏）
        # -----------------------------------------------------------------------
        content_layout = QHBoxLayout()
        content_layout.setSpacing(20)

        # 左侧：地图编辑区
        left_frame = ModernCard("交通图编辑区")
        left_layout = left_frame.layout()

        # 地图可视化控件
        self.map_widget = TrafficMapWidget()
        self.map_widget.setMinimumHeight(600)
        left_layout.addWidget(self.map_widget)

        # 地图操作按钮栏
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

        # 航点管理与交互模式卡片
        self.create_waypoint_card(right_layout)

        # 右侧面板底部留白
        right_layout.addStretch()
        content_layout.addWidget(right_frame, stretch=1)

        main_layout.addLayout(content_layout)

        # -----------------------------------------------------------------------
        # 状态栏
        # -----------------------------------------------------------------------
        self.statusBar().showMessage("Ready - select interaction mode on the right, then left-click to operate")

        # -----------------------------------------------------------------------
        # 连接地图控件信号
        # -----------------------------------------------------------------------
        self.map_widget.waypoint_added.connect(self.on_waypoint_added)
        self.map_widget.waypoint_removed.connect(self.on_waypoint_removed)
        self.map_widget.connection_added.connect(self.on_connection_added)
        self.map_widget.connection_removed.connect(self.on_connection_removed)
        self.map_widget.map_loaded.connect(self.on_map_loaded)

    def create_title_section(self, layout):
        """构建标题栏区域。

        委托至 ui_sections 模块构建包含应用标题与在线状态指示器的顶部区域。

        Args:
            layout: 父级布局，标题栏将被添加至此布局中。
        """
        build_title_section(self, layout)

    def create_help_card(self, layout):
        """创建操作提示卡片。

        在指定布局中添加包含鼠标快捷键等操作提示信息的卡片组件。

        Args:
            layout: 父级布局，提示卡片将被添加至此布局中。
        """
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
        """构建航点管理卡片。

        委托至 ui_sections 模块创建包含模式切换按钮、统计信息与清空操作的
        航点管理卡片。

        Args:
            layout: 父级布局，航点管理卡片将被添加至此布局中。
        """
        build_waypoint_card(self, layout, ModernCard, ModernButton)

        # 默认设置为平移模式
        try:
            self.map_widget.set_interaction_mode("pan")
        except Exception:
            pass

    def create_task_card(self, layout):
        """创建任务提交卡片。

        在指定布局中添加用于提交导航任务的操作卡片组件。

        Args:
            layout: 父级布局，任务提交卡片将被添加至此布局中。
        """
        card = ModernCard("任务提交")
        card_layout = card.layout()

        # 操作说明
        info_label = QLabel("选择目标航点后提交任务")
        info_label.setStyleSheet("color: #666; margin-bottom: 10px;")
        info_label.setWordWrap(True)
        card_layout.addWidget(info_label)

        # 提交任务按钮
        self.btn_submit_task = ModernButton("提交导航任务")
        self.btn_submit_task.clicked.connect(self.submit_task)
        card_layout.addWidget(self.btn_submit_task)

        layout.addWidget(card)

    def create_stats_card(self, layout):
        """创建系统状态统计卡片。

        在指定布局中添加包含机器人数量、任务状态等统计信息的卡片组件。

        Args:
            layout: 父级布局，统计信息卡片将被添加至此布局中。
        """
        card = ModernCard("系统状态")
        card_layout = card.layout()

        # 统计项定义
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
        """初始化 ROS 2 通信。

        启动 rclpy 并创建 ROS 节点实例，
        根据初始化结果更新在线状态指示器。
        """
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
        """初始化定时器。

        创建 ROS 事件循环定时器和统计信息更新定时器，
        分别用于驱动 ROS 回调处理和周期性界面刷新。
        """
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)

        # 统计信息更新定时器（每秒刷新）
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self.update_stats)
        self.stats_timer.start(1000)

    def spin_ros(self):
        """执行 ROS 事件循环单次轮询。

        由定时器驱动，在 Qt 主循环中穿插执行 rclpy.spin_once，
        以处理 ROS 异步服务响应。退出或重启阶段需避免在 shutdown
        后继续调用。
        """
        if not self.ros_node:
            return
        if not rclpy.ok():
            return
        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0)
        except Exception:
            # WaitSet 初始化失败时停止定时器，防止日志刷屏或界面卡死
            try:
                self.ros_timer.stop()
            except Exception:
                pass
            self.ros_node = None

    def update_stats(self):
        """更新航点与航线统计信息。

        从地图控件获取当前航点数和连接数，
        并刷新右侧面板中对应的统计标签显示。
        """
        if not hasattr(self, "waypoint_count_label") or not hasattr(self, "connection_count_label"):
            return
        waypoint_count = len(self.map_widget.waypoints)
        connection_count = len(self.map_widget.connections)

        self.waypoint_count_label.setText(f"航点数量: {waypoint_count}")
        self.connection_count_label.setText(f"航线数量: {connection_count}")

    def load_map_yaml(self):
        """加载地图 YAML 文件。"""
        action_load_map_yaml(self)

    def load_traffic_map(self):
        """加载交通图文件。"""
        action_load_traffic_map(self)

    def save_traffic_map(self):
        """保存交通图文件。"""
        action_save_traffic_map(self)

    def submit_task(self):
        """提交导航任务。"""
        action_submit_task(self)

    def clear_waypoints(self):
        """清空所有航点。"""
        action_clear_waypoints(self)

    def on_waypoint_added(self, waypoint_id, pose):
        """航点添加事件回调。

        Args:
            waypoint_id: 被添加航点的 ID。
            pose: 被添加航点的位姿信息。
        """
        self.statusBar().showMessage(f"Added waypoint: {waypoint_id}")
        self.update_stats()

    def on_waypoint_removed(self, waypoint_id):
        """航点删除事件回调。

        Args:
            waypoint_id: 被删除航点的 ID。
        """
        self.statusBar().showMessage(f"Deleted waypoint: {waypoint_id}")
        self.update_stats()

    def on_connection_added(self, from_id, to_id):
        """连接建立事件回调。

        Args:
            from_id: 连接起点的航点 ID。
            to_id: 连接终点的航点 ID。
        """
        self.statusBar().showMessage(f"Added connection: {from_id} <-> {to_id}")
        self.update_stats()

    def on_connection_removed(self, from_id, to_id):
        """连接删除事件回调。

        Args:
            from_id: 被删除连接的起点航点 ID。
            to_id: 被删除连接的终点航点 ID。
        """
        self.statusBar().showMessage(f"Removed connection: {from_id} <-> {to_id}")
        self.update_stats()

    def on_map_loaded(self, map_path):
        """地图加载完成事件回调。

        Args:
            map_path: 加载的地图文件路径。
        """
        self.statusBar().showMessage(f"Loaded map: {os.path.basename(map_path)}")

    def closeEvent(self, event):
        """窗口关闭事件处理。

        按顺序停止所有定时器、销毁 ROS 节点并执行 rclpy shutdown，
        确保资源正确释放，避免在 shutdown 后定时器继续触发 spin_once。

        Args:
            event: 关闭事件对象。
        """
        # 步骤一：停止定时器，防止 shutdown 后 timer 继续触发 spin_once
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

        # 步骤二：销毁 ROS 节点
        try:
            if self.ros_node and rclpy.ok():
                self.ros_node.destroy_node()
        except Exception:
            pass
        self.ros_node = None

        # 步骤三：关闭 rclpy
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        event.accept()


def main(args=None):
    """应用程序入口函数。

    创建 QApplication 实例，初始化主窗口并启动 Qt 事件循环。
    同时注册 SIGINT 信号处理器以支持 Ctrl+C 优雅退出。
    """
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)

    # 设置应用元信息
    app.setApplicationName("车队交通图编辑器")
    app.setOrganizationName("Fleet Management System")

    # 设置默认字体
    font = QFont("Microsoft YaHei", 10)
    app.setFont(font)

    window = TrafficEditorMainWindow()
    window.show()

    # 注册 SIGINT 信号处理器，使 Ctrl+C 可正常退出 Qt 主循环
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UI 分区组件模块。

提供主界面中各 UI 分区的构建函数，包括标题栏区域和航点管理卡片区域，
用于组合主窗口的顶部标题栏与右侧控制面板。
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QLabel, QHBoxLayout


def create_title_section(window, layout) -> None:
    """构建顶部标题栏区域。

    创建包含应用标题和 ROS 在线状态指示器的标题栏，
    并在下方添加渐变分隔线。

    Args:
        window: 主窗口实例，用于绑定 status_indicator 属性。
        layout: 父级布局，标题栏将被添加至此布局中。
    """
    title_frame = QFrame()
    title_layout = QHBoxLayout(title_frame)
    title_layout.setContentsMargins(0, 0, 0, 10)

    title_label = QLabel("FleetOS 车队交通图编辑器")
    title_label.setObjectName("titleLabel")
    title_layout.addWidget(title_label)
    title_layout.addStretch()

    window.status_indicator = QLabel("● 在线")
    window.status_indicator.setStyleSheet("color: #4caf50; font-weight: bold; font-size: 14px;")
    title_layout.addWidget(window.status_indicator)
    layout.addWidget(title_frame)

    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet(
        "background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #667eea, stop:1 #764ba2); max-height: 2px;"
    )
    layout.addWidget(line)


def create_waypoint_card(window, layout, modern_card_cls, modern_button_cls) -> None:
    """构建航点管理卡片。

    创建包含航点/航线统计、交互模式切换按钮和清空操作按钮的
    航点管理卡片，用于右侧控制面板。

    Args:
        window: 主窗口实例，用于绑定统计标签和模式标签属性。
        layout: 父级布局，航点管理卡片将被添加至此布局中。
        modern_card_cls: 现代化卡片组件类。
        modern_button_cls: 现代化按钮组件类。
    """
    card = modern_card_cls("航点管理")
    card_layout = card.layout()

    window.waypoint_count_label = QLabel("航点数量: 0")
    window.waypoint_count_label.setStyleSheet("color: #667eea; font-weight: bold;")
    card_layout.addWidget(window.waypoint_count_label)

    window.connection_count_label = QLabel("航线数量: 0")
    window.connection_count_label.setStyleSheet("color: #764ba2; font-weight: bold;")
    card_layout.addWidget(window.connection_count_label)

    mode_title = QLabel("交互模式")
    mode_title.setStyleSheet("color: #00f5ff; font-weight: bold;")
    card_layout.addWidget(mode_title)

    window.mode_label = QLabel("当前模式: 添加航点")
    window.mode_label.setStyleSheet("color: #94a3b8; font-weight: bold;")
    card_layout.addWidget(window.mode_label)

    btn_add_mode = modern_button_cls("添加航点")
    btn_add_mode.clicked.connect(lambda: window.set_interaction_mode("add"))
    card_layout.addWidget(btn_add_mode)

    btn_connect_mode = modern_button_cls("连接航点")
    btn_connect_mode.setObjectName("secondaryButton")
    btn_connect_mode.clicked.connect(lambda: window.set_interaction_mode("connect"))
    card_layout.addWidget(btn_connect_mode)

    btn_edit_mode = modern_button_cls("编辑航点属性")
    btn_edit_mode.setObjectName("secondaryButton")
    btn_edit_mode.clicked.connect(lambda: window.set_interaction_mode("edit"))
    card_layout.addWidget(btn_edit_mode)

    btn_clear = modern_button_cls("清空所有航点")
    btn_clear.setObjectName("dangerButton")
    btn_clear.clicked.connect(window.clear_waypoints)
    card_layout.addWidget(btn_clear)

    layout.addWidget(card)

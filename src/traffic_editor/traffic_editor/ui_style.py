#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UI 样式表模块。

定义交通图编辑器的全局 Qt 样式表（QSS），
实现 Material Design 风格的亮色主题外观，
涵盖全局样式、卡片、按钮、工具栏和状态栏等组件。
"""

MODERN_STYLE = """
/* 全局样式 */
QMainWindow {
    background: #f8fafc;
}

QWidget {
    font-family: "Microsoft YaHei", "PingFang SC", "Hiragino Sans GB", sans-serif;
    font-size: 13px;
}

/* 中央部件 */
QFrame#centralFrame {
    background: #ffffff;
    border-radius: 16px;
}

/* 卡片样式 */
QFrame#cardFrame {
    background: #ffffff;
    border-radius: 14px;
    border: 1px solid #e5e7eb;
}

/* 标题标签 */
QLabel#titleLabel {
    font-size: 22px;
    font-weight: bold;
    color: #0ea5e9;
    padding: 6px 8px;
}

QLabel#sectionTitle {
    font-size: 14px;
    font-weight: bold;
    color: #0f172a;
    padding: 0px;
}

/* 按钮样式 */
QPushButton {
    background: #0ea5e9;
    color: white;
    border: 1px solid #0ea5e9;
    padding: 10px 18px;
    border-radius: 10px;
    font-size: 14px;
    font-weight: 600;
    min-width: 110px;
}

QPushButton:hover {
    background: #0284c7;
}

QPushButton:pressed {
    background: #0369a1;
}

QPushButton#secondaryButton {
    background: #ffffff;
    color: #0ea5e9;
    border: 1px solid #0ea5e9;
}

QPushButton#secondaryButton:hover {
    background: rgba(14, 165, 233, 0.08);
}

QPushButton#dangerButton {
    background: #ef4444;
    border: 1px solid #ef4444;
}

QPushButton#dangerButton:hover {
    background: #dc2626;
}

/* 滚动区域 */
QScrollArea {
    border: none;
    background: transparent;
}

QScrollArea > QWidget > QWidget {
    background: transparent;
}

/* 状态栏 */
QStatusBar {
    background: white;
    color: #475569;
    border-top: 1px solid #e2e8f0;
    padding: 5px;
    font-size: 12px;
}

/* 工具栏 */
QToolBar {
    background: white;
    border-bottom: 1px solid #e2e8f0;
    padding: 5px;
    spacing: 5px;
}

QToolBar QPushButton {
    background: white;
    color: #00d4ff;
    border: 2px solid #e2e8f0;
    padding: 8px 16px;
    border-radius: 8px;
    min-width: 100px;
}

QToolBar QPushButton:hover {
    background: rgba(0, 212, 255, 0.12);
    color: white;
    border-color: #00d4ff;
}

/* 分割器 */
QSplitter::handle {
    background: #e2e8f0;
    width: 2px;
}

QSplitter::handle:hover {
    background: #00d4ff;
}
"""

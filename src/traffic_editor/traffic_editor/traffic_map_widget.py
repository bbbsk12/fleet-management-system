#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
交通地图可视化组件
支持地图显示、航点编辑和航线连接
"""

import yaml
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                             QLabel, QSlider, QFileDialog, QInputDialog, QMessageBox,
                             QApplication, QDialog, QCheckBox, QLineEdit, QFormLayout,
                             QDialogButtonBox)
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QWheelEvent, QMouseEvent, QPixmap, QFont
from geometry_msgs.msg import Pose
import os
try:
    from .map_io import dump_traffic_map, load_traffic_map as parse_traffic_map
except ImportError:
    from map_io import dump_traffic_map, load_traffic_map as parse_traffic_map


class TrafficMapWidget(QWidget):
    """交通地图编辑控件"""
    
    # 信号定义
    waypoint_added = pyqtSignal(str, Pose)
    waypoint_removed = pyqtSignal(str)
    waypoint_moved = pyqtSignal(str, Pose)
    connection_added = pyqtSignal(str, str)
    connection_removed = pyqtSignal(str, str)
    map_loaded = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        
        # 数据
        self.waypoints = {}
        self.connections = []
        self.map_image = None
        self.map_yaml = None
        
        # 视图参数
        self.scale = 1.0
        self.offset = QPointF(0, 0)
        self.selected_waypoint = None
        self.connecting_from = None
        self.hovered_waypoint = None
        self.waypoint_radius = 12
        
        # 拖拽参数
        self.dragging = False
        self.last_mouse_pos = None
        
        # 设置
        self.setMouseTracking(True)
        self.setMinimumSize(800, 600)
        
        # 设置字体
        self.setFont(QFont("Microsoft YaHei", 10))
        
        # 航点计数器
        self._waypoint_counter = 0

        # 交互模式（由右侧按钮切换，不再依赖键盘组合键）
        # - "pan": 左键拖拽移动地图（默认）
        # - "add": 左键点击空白添加航点
        # - "connect": 左键选择起点，再左键选择终点建立连接（默认双向）
        # - "edit": 左键点击航点编辑属性
        self.interaction_mode = "pan"
    
    def _generate_waypoint_id(self):
        """生成自动航点ID"""
        self._waypoint_counter += 1
        return f"wp_{self._waypoint_counter:03d}"

    def set_interaction_mode(self, mode: str):
        """切换地图交互模式（只影响鼠标左键行为）"""
        if mode not in ("pan", "add", "connect", "edit"):
            return
        self.interaction_mode = mode
        # 切模式时取消连接构建，避免误连
        self.connecting_from = None
        self.update()

    def _set_parent_status(self, msg: str):
        """兼容 QWidget 没有 statusBar 的情况"""
        try:
            win = self.window()
            if win is not None and hasattr(win, "statusBar"):
                win.statusBar().showMessage(msg)
        except Exception:
            pass

    def clear_all(self):
        """清空编辑内容并重置自动编号"""
        self.waypoints.clear()
        self.connections.clear()
        self.selected_waypoint = None
        self.connecting_from = None
        self.hovered_waypoint = None
        self.scale = 1.0
        self.offset = QPointF(0, 0)
        self._waypoint_counter = 0
        self.update()
    
    def _show_waypoint_properties_dialog(self, waypoint_id=None):
        """显示航点属性对话框"""
        dialog = QDialog(self)
        dialog.setWindowTitle("航点属性" if waypoint_id else "添加航点")
        dialog.setMinimumWidth(300)
        
        layout = QFormLayout(dialog)
        
        # 航点ID
        id_edit = QLineEdit()
        if waypoint_id:
            id_edit.setText(waypoint_id)
            id_edit.setReadOnly(True)
        else:
            id_edit.setText(self._generate_waypoint_id())
            # 添加模式下不允许手动改 ID，避免重复编号导致连接/航点打乱
            id_edit.setReadOnly(True)
        layout.addRow("航点ID:", id_edit)
        
        # 航点名称
        name_edit = QLineEdit()
        name_edit.setPlaceholderText("可选，用于显示")
        if waypoint_id and waypoint_id in self.waypoints:
            name_edit.setText(self.waypoints[waypoint_id].get('name', waypoint_id))
        layout.addRow("名称:", name_edit)
        
        # 是否停车位
        is_parking_cb = QCheckBox("设为停车位")
        if waypoint_id and waypoint_id in self.waypoints:
            is_parking_cb.setChecked(self.waypoints[waypoint_id].get('is_parking_spot', False))
        layout.addRow(is_parking_cb)
        
        # 是否充电站
        is_charging_cb = QCheckBox("设为充电站")
        if waypoint_id and waypoint_id in self.waypoints:
            is_charging_cb.setChecked(self.waypoints[waypoint_id].get('is_charging_station', False))
        layout.addRow(is_charging_cb)
        
        # 按钮
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)
        
        if dialog.exec_() == QDialog.Accepted:
            return {
                'id': id_edit.text(),
                'name': name_edit.text() or id_edit.text(),
                'is_parking_spot': is_parking_cb.isChecked(),
                'is_charging_station': is_charging_cb.isChecked()
            }
        return None
        
    def load_map(self, yaml_path):
        """加载地图YAML文件"""
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                self.map_yaml = yaml.safe_load(f)
            
            image_path = os.path.join(os.path.dirname(yaml_path), self.map_yaml['image'])
            self.map_image = QPixmap(image_path)
            
            if self.map_image.isNull():
                QMessageBox.critical(self, "Error", f"Unable to load map image: {image_path}")
                return False
            
            # 内部保存绝对路径，落盘时再转换为相对当前交通图文件的路径。
            self.map_yaml_file = os.path.abspath(yaml_path)
            # 存储地图图片文件名
            self.map_image_file = self.map_yaml.get('image', '')
            
            # 调整视图以适应地图
            self.setMinimumSize(self.map_image.width(), self.map_image.height())
            self.map_loaded.emit(yaml_path)
            self.update()
            return True
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load map: {str(e)}")
            return False
    
    def add_waypoint(self, waypoint_id, pose, name=None, is_parking_spot=False, is_charging_station=False):
        """添加航点"""
        self.waypoints[waypoint_id] = {
            'pose': pose,
            'name': name or waypoint_id,
            'connections': [],
            'is_parking_spot': is_parking_spot,
            'is_charging_station': is_charging_station,
            'radius': 0.5
        }
        self.update()
    
    def remove_waypoint(self, waypoint_id):
        """删除航点"""
        if waypoint_id in self.waypoints:
            del self.waypoints[waypoint_id]
            # 删除相关连接
            self.connections = [(f, t) for f, t in self.connections 
                               if f != waypoint_id and t != waypoint_id]
            # 同步清理每个航点内部保存的 connections，避免保存后仍带旧数据
            for wp_id, wp_data in self.waypoints.items():
                wp_data["connections"] = []
            for f, t in self.connections:
                if f in self.waypoints:
                    self.waypoints[f]["connections"].append(t)
            if self.selected_waypoint == waypoint_id:
                self.selected_waypoint = None
            if self.connecting_from == waypoint_id:
                self.connecting_from = None
            self.update()
    
    def add_connection(self, from_id, to_id):
        """添加航线连接"""
        # Editor 里连接的语义对齐地图邻接图：默认把连接当作无向边，自动生成双向连接
        if from_id == to_id:
            return

        def add_one_way(a, b):
            if (a, b) in self.connections:
                return
            self.connections.append((a, b))
            if b not in self.waypoints[a].get('connections', []):
                self.waypoints[a]['connections'].append(b)

        add_one_way(from_id, to_id)
        add_one_way(to_id, from_id)
        self.update()
    
    def remove_connection(self, from_id, to_id):
        """删除航线连接"""
        # 按无向边语义：删除双向连接
        removed = False
        if (from_id, to_id) in self.connections:
            self.connections.remove((from_id, to_id))
            if to_id in self.waypoints[from_id]['connections']:
                self.waypoints[from_id]['connections'].remove(to_id)
            removed = True
        if (to_id, from_id) in self.connections:
            self.connections.remove((to_id, from_id))
            if from_id in self.waypoints[to_id]['connections']:
                self.waypoints[to_id]['connections'].remove(from_id)
            removed = True
        if removed:
            self.update()
    
    def save_traffic_map(self, file_path):
        """保存交通图到文件"""
        try:
            dump_traffic_map(
                file_path=file_path,
                waypoints=self.waypoints,
                map_yaml_path=getattr(self, 'map_yaml_file', ''),
                map_image_file=getattr(self, 'map_image_file', ''),
                map_yaml_meta=self.map_yaml if self.map_yaml else None,
            )
            return True
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save traffic map: {str(e)}")
            return False
    
    def load_traffic_map(self, file_path):
        """从文件加载交通图"""
        try:
            waypoints, connections, max_num = parse_traffic_map(file_path)
            self.waypoints = waypoints
            self.connections = connections
            self.selected_waypoint = None
            self.connecting_from = None
            self.hovered_waypoint = None
            self._waypoint_counter = max_num
            self.update()
            return True
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load traffic map: {str(e)}")
            return False
    
    def get_world_pos(self, screen_pos):
        """屏幕坐标转世界坐标"""
        x = (screen_pos.x() - self.offset.x()) / self.scale
        y = (screen_pos.y() - self.offset.y()) / self.scale
        return x, y
    
    def get_screen_pos(self, world_x, world_y):
        """世界坐标转屏幕坐标"""
        x = world_x * self.scale + self.offset.x()
        y = world_y * self.scale + self.offset.y()
        return x, y
    
    def get_waypoint_at(self, pos):
        """获取指定位置的航点"""
        for wp_id, wp_data in self.waypoints.items():
            wx, wy = self.get_screen_pos(wp_data['pose'].position.x, wp_data['pose'].position.y)
            dist = ((pos.x() - wx) ** 2 + (pos.y() - wy) ** 2) ** 0.5
            if dist <= self.waypoint_radius * 2:
                return wp_id
        return None
    
    def wheelEvent(self, event: QWheelEvent):
        """鼠标滚轮事件 - 缩放"""
        delta = event.angleDelta().y()
        zoom_factor = 1.1 if delta > 0 else 0.9
        
        old_scale = self.scale
        self.scale = max(0.1, min(5.0, self.scale * zoom_factor))
        
        # 以鼠标位置为中心缩放
        mouse_pos = event.pos()
        self.offset = QPointF(mouse_pos) - (QPointF(mouse_pos) - self.offset) * (self.scale / old_scale)
        
        self.update()
    
    def mousePressEvent(self, event: QMouseEvent):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            # 默认/移动模式：左键拖拽平移地图
            if self.interaction_mode == "pan":
                self.dragging = True
                self.last_mouse_pos = event.pos()
                self.setCursor(Qt.ClosedHandCursor)
                return

            wp_id = self.get_waypoint_at(event.pos())
            
            if wp_id:
                # 点击到航点
                # 所有模式下：先选中航点高亮
                self.selected_waypoint = wp_id

                if self.interaction_mode == "connect":
                    # 连接模式：点起点->点终点
                    if self.connecting_from is None:
                        self.connecting_from = wp_id
                        self._set_parent_status("Selected start waypoint. Click target waypoint to create connection.")
                    else:
                        if self.connecting_from != wp_id:
                            self.add_connection(self.connecting_from, wp_id)
                            self.connection_added.emit(self.connecting_from, wp_id)
                            self._set_parent_status(f"Connection created: {self.connecting_from} <-> {wp_id}")
                        self.connecting_from = None

                elif self.interaction_mode == "edit":
                    # 编辑模式：弹窗修改属性
                    props = self._show_waypoint_properties_dialog(wp_id)
                    if props:
                        self.waypoints[wp_id]['name'] = props['name']
                        self.waypoints[wp_id]['is_parking_spot'] = props['is_parking_spot']
                        self.waypoints[wp_id]['is_charging_station'] = props['is_charging_station']
                        self.update()
            else:
                # 点击空白区域
                self.selected_waypoint = None
                if self.interaction_mode == "add":
                    # 添加模式：快速添加航点（自动ID）
                    wp_id = self._generate_waypoint_id()
                    x, y = self.get_world_pos(event.pos())
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    self.add_waypoint(wp_id, pose)
                    self.waypoint_added.emit(wp_id, pose)
                elif self.interaction_mode == "connect":
                    # 连接模式下点击空白：取消当前连接构建
                    self.connecting_from = None
        
        elif event.button() == Qt.RightButton:
            wp_id = self.get_waypoint_at(event.pos())
            if wp_id:
                # 右键删除航点（不需要先选中）
                reply = QMessageBox.question(
                    self, "确认删除", 
                    f"确定要删除航点 '{wp_id}' 吗？",
                    QMessageBox.Yes | QMessageBox.No
                )
                if reply == QMessageBox.Yes:
                    self.remove_waypoint(wp_id)
                    self.waypoint_removed.emit(wp_id)
        
        elif event.button() == Qt.MiddleButton:
            # 中键拖拽
            self.dragging = True
            self.last_mouse_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
        
        self.update()
    
    def mouseReleaseEvent(self, event: QMouseEvent):
        """鼠标释放事件"""
        if event.button() in (Qt.MiddleButton, Qt.LeftButton):
            self.dragging = False
            self.setCursor(Qt.ArrowCursor)
    
    def mouseMoveEvent(self, event: QMouseEvent):
        """鼠标移动事件"""
        self.hovered_waypoint = self.get_waypoint_at(event.pos())
        
        if self.dragging and self.last_mouse_pos:
            # 拖拽地图
            delta = event.pos() - self.last_mouse_pos
            self.offset += QPointF(delta.x(), delta.y())
            self.last_mouse_pos = event.pos()
        
        self.update()
    
    def paintEvent(self, event):
        """绑定绑定事件"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 背景
        painter.fillRect(self.rect(), QColor(245, 247, 250))
        
        # 绘制地图
        if self.map_image:
            painter.save()
            painter.translate(self.offset)
            painter.scale(self.scale, self.scale)
            painter.drawPixmap(0, 0, self.map_image)
            painter.restore()
        
        # 绘制航线
        painter.save()
        painter.translate(self.offset)
        painter.scale(self.scale, self.scale)
        
        pen = QPen()
        pen.setWidth(3)
        pen.setColor(QColor(102, 126, 234, 180))
        
        for from_id, to_id in self.connections:
            if from_id in self.waypoints and to_id in self.waypoints:
                # 双向边会导致重复绘线：这里对同一无向边只绘制一次
                if (to_id, from_id) in self.connections and from_id > to_id:
                    continue
                from_wp = self.waypoints[from_id]
                to_wp = self.waypoints[to_id]
                
                painter.setPen(pen)
                painter.drawLine(
                    int(from_wp['pose'].position.x), int(from_wp['pose'].position.y),
                    int(to_wp['pose'].position.x), int(to_wp['pose'].position.y)
                )
        
        # 绘制航点
        for wp_id, wp_data in self.waypoints.items():
            x = wp_data['pose'].position.x
            y = wp_data['pose'].position.y
            
            # 航点颜色
            if wp_id == self.selected_waypoint:
                color = QColor(244, 67, 54)  # 红色 - 选中
            elif wp_id == self.hovered_waypoint:
                color = QColor(255, 152, 0)  # 橙色 - 悬停
            elif wp_data['is_charging_station']:
                color = QColor(76, 175, 80)  # 绿色 - 充电站
            elif wp_data['is_parking_spot']:
                color = QColor(33, 150, 243)  # 蓝色 - 停车位
            else:
                color = QColor(102, 126, 234)  # 紫色 - 普通航点
            
            # 绘制航点圆圈
            pen = QPen(color.darker(120), 2)
            brush = QBrush(color)
            painter.setPen(pen)
            painter.setBrush(brush)
            
            radius = self.waypoint_radius / self.scale
            # drawEllipse需要整数参数
            painter.drawEllipse(int(x - radius), int(y - radius), int(radius * 2), int(radius * 2))
            
            # 绘制航点ID
            if wp_id == self.selected_waypoint or wp_id == self.hovered_waypoint:
                painter.setPen(QColor(51, 51, 51))
                font = QFont("Microsoft YaHei", 9, QFont.Bold)
                painter.setFont(font)
                painter.drawText(
                    int(x + radius * 1.5),
                    int(y - radius),
                    wp_id
                )
        
        # 绘制连接线预览
        if self.connecting_from and self.connecting_from in self.waypoints:
            pen = QPen(QColor(255, 152, 0), 2, Qt.DashLine)
            painter.setPen(pen)
            from_wp = self.waypoints[self.connecting_from]
            cursor_pos = self.mapFromGlobal(self.cursor().pos())
            world_x, world_y = self.get_world_pos(cursor_pos)
            painter.drawLine(
                int(from_wp['pose'].position.x), int(from_wp['pose'].position.y),
                int(world_x), int(world_y)
            )
        
        painter.restore()

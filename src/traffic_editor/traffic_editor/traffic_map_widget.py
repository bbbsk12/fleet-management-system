#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
交通地图可视化控件模块。

实现交通图编辑器中的核心地图可视化控件。支持地图 YAML 文件的加载与底图渲染、
航点的添加/删除/编辑、航线连接关系的建立与解除，
以及平移、添加、连接、编辑等多种鼠标交互模式。
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
    """交通地图编辑控件。

    提供基于 PyQt5 的地图可视化与航点编辑功能，支持以下交互模式：
        - pan:     左键拖拽平移地图（默认模式）
        - add:     左键点击空白区域添加航点
        - connect: 依次点击两个航点建立双向连接
        - edit:    左键点击航点弹出属性编辑对话框

    Signals:
        waypoint_added: 添加航点时触发，传递航点 ID 和位姿信息。
        waypoint_removed: 删除航点时触发，传递航点 ID。
        waypoint_moved: 移动航点时触发，传递航点 ID 和更新后的位姿。
        connection_added: 建立连接时触发，传递起点和终点 ID。
        connection_removed: 删除连接时触发，传递起点和终点 ID。
        map_loaded: 地图加载完成时触发，传递地图文件路径。
    """

    # -----------------------------------------------------------------------
    # 信号定义
    # -----------------------------------------------------------------------
    waypoint_added = pyqtSignal(str, Pose)
    waypoint_removed = pyqtSignal(str)
    waypoint_moved = pyqtSignal(str, Pose)
    connection_added = pyqtSignal(str, str)
    connection_removed = pyqtSignal(str, str)
    map_loaded = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        # -----------------------------------------------------------------------
        # 数据成员
        # -----------------------------------------------------------------------
        self.waypoints = {}
        self.connections = []
        self.map_image = None
        self.map_yaml = None

        # -----------------------------------------------------------------------
        # 视图参数
        # -----------------------------------------------------------------------
        self.scale = 1.0
        self.offset = QPointF(0, 0)
        self.selected_waypoint = None
        self.connecting_from = None
        self.hovered_waypoint = None
        self.waypoint_radius = 12

        # -----------------------------------------------------------------------
        # 拖拽参数
        # -----------------------------------------------------------------------
        self.dragging = False
        self.last_mouse_pos = None

        # -----------------------------------------------------------------------
        # 控件设置
        # -----------------------------------------------------------------------
        self.setMouseTracking(True)
        self.setMinimumSize(800, 600)

        # 设置默认字体
        self.setFont(QFont("Microsoft YaHei", 10))

        # -----------------------------------------------------------------------
        # 航点自动编号计数器
        # -----------------------------------------------------------------------
        self._waypoint_counter = 0

        # -----------------------------------------------------------------------
        # 交互模式
        # -----------------------------------------------------------------------
        # 由右侧按钮切换，不再依赖键盘组合键。
        # 支持以下模式：
        #   "pan"     - 左键拖拽移动地图（默认）
        #   "add"     - 左键点击空白处添加航点
        #   "connect" - 依次左键选择起点和终点，建立双向连接
        #   "edit"    - 左键点击航点编辑属性
        self.interaction_mode = "pan"

    def _generate_waypoint_id(self):
        """生成自动递增的航点 ID。

        航点 ID 格式为 "wp_XXX"，其中 XXX 为三位数字序号。

        Returns:
            自动生成的航点 ID 字符串。
        """
        self._waypoint_counter += 1
        return f"wp_{self._waypoint_counter:03d}"

    def set_interaction_mode(self, mode: str):
        """切换交互模式。

        切换地图控件的鼠标左键交互行为，切换时自动重置连接构建状态。

        Args:
            mode: 目标模式名称，取值范围为 "pan"、"add"、"connect"、"edit"。
        """
        if mode not in ("pan", "add", "connect", "edit"):
            return
        self.interaction_mode = mode
        # 切换模式时取消正在进行的连接构建，避免误连
        self.connecting_from = None
        self.update()

    def _set_parent_status(self, msg: str):
        """设置父窗口状态栏消息。

        向上查找主窗口实例并调用其状态栏显示消息。
        QWidget 本身没有 statusBar 方法，因此需要安全兼容处理。

        Args:
            msg: 要显示的状态消息。
        """
        try:
            win = self.window()
            if win is not None and hasattr(win, "statusBar"):
                win.statusBar().showMessage(msg)
        except Exception:
            pass

    def clear_all(self):
        """清空所有编辑内容。

        清除所有航点数据、连接关系、视图状态和自动编号计数器，
        将地图控件恢复至初始状态。
        """
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
        """显示航点属性编辑对话框。

        弹出自定义对话框，允许用户查看或编辑航点的名称、
        停车位标记和充电站标记等属性。

        Args:
            waypoint_id: 要编辑的航点 ID；为 None 时表示新建航点。

        Returns:
            包含 'id'、'name'、'is_parking_spot'、'is_charging_station'
            键的字典；若用户取消操作则返回 None。
        """
        dialog = QDialog(self)
        dialog.setWindowTitle("航点属性" if waypoint_id else "添加航点")
        dialog.setMinimumWidth(300)

        layout = QFormLayout(dialog)

        # -----------------------------------------------------------------------
        # 航点 ID 输入
        # -----------------------------------------------------------------------
        id_edit = QLineEdit()
        if waypoint_id:
            id_edit.setText(waypoint_id)
            id_edit.setReadOnly(True)
        else:
            id_edit.setText(self._generate_waypoint_id())
            # 添加模式下禁止手动修改 ID，避免编号冲突导致数据混乱
            id_edit.setReadOnly(True)
        layout.addRow("航点ID:", id_edit)

        # -----------------------------------------------------------------------
        # 航点名称输入
        # -----------------------------------------------------------------------
        name_edit = QLineEdit()
        name_edit.setPlaceholderText("可选，用于显示")
        if waypoint_id and waypoint_id in self.waypoints:
            name_edit.setText(self.waypoints[waypoint_id].get('name', waypoint_id))
        layout.addRow("名称:", name_edit)

        # -----------------------------------------------------------------------
        # 停车位复选框
        # -----------------------------------------------------------------------
        is_parking_cb = QCheckBox("设为停车位")
        if waypoint_id and waypoint_id in self.waypoints:
            is_parking_cb.setChecked(self.waypoints[waypoint_id].get('is_parking_spot', False))
        layout.addRow(is_parking_cb)

        # -----------------------------------------------------------------------
        # 充电站复选框
        # -----------------------------------------------------------------------
        is_charging_cb = QCheckBox("设为充电站")
        if waypoint_id and waypoint_id in self.waypoints:
            is_charging_cb.setChecked(self.waypoints[waypoint_id].get('is_charging_station', False))
        layout.addRow(is_charging_cb)

        # -----------------------------------------------------------------------
        # 确认/取消按钮
        # -----------------------------------------------------------------------
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
        """加载地图 YAML 文件。

        读取地图 YAML 文件并加载对应的底图图像，调整视图以适应底图尺寸。

        Args:
            yaml_path: 地图 YAML 文件路径。

        Returns:
            加载成功返回 True，失败返回 False。
        """
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                self.map_yaml = yaml.safe_load(f)

            image_path = os.path.join(os.path.dirname(yaml_path), self.map_yaml['image'])
            self.map_image = QPixmap(image_path)

            if self.map_image.isNull():
                QMessageBox.critical(self, "Error", f"Unable to load map image: {image_path}")
                return False

            # 内部保存底图 YAML 的绝对路径，落盘时再转换为相对路径
            self.map_yaml_file = os.path.abspath(yaml_path)
            # 存储地图图片文件名
            self.map_image_file = self.map_yaml.get('image', '')

            # 调整视图以适应地图尺寸
            self.setMinimumSize(self.map_image.width(), self.map_image.height())
            self.map_loaded.emit(yaml_path)
            self.update()
            return True

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load map: {str(e)}")
            return False

    def add_waypoint(self, waypoint_id, pose, name=None, is_parking_spot=False, is_charging_station=False):
        """添加航点。

        向编辑器添加一个新的航点，包含位姿、名称和属性标记。

        Args:
            waypoint_id: 航点唯一标识符。
            pose: 航点的位姿（geometry_msgs/Pose）。
            name: 航点显示名称，默认与 ID 相同。
            is_parking_spot: 是否为停车位。
            is_charging_station: 是否为充电站。
        """
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
        """删除指定航点。

        移除指定航点及其相关的所有连接关系，同时清理内部连接缓存。

        Args:
            waypoint_id: 要删除的航点 ID。
        """
        if waypoint_id in self.waypoints:
            del self.waypoints[waypoint_id]
            # 移除与该航点相关的所有连接
            self.connections = [(f, t) for f, t in self.connections
                               if f != waypoint_id and t != waypoint_id]
            # 同步更新每个航点内部维护的连接列表
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
        """添加航线连接。

        在两个航点之间建立双向连接关系，若连接已存在则跳过。

        Args:
            from_id: 起点航点 ID。
            to_id: 终点航点 ID。
        """
        # 编辑器语义：连接视为无向边，自动建立双向连接
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
        """删除航线连接。

        按照无向边语义移除两个航点之间的双向连接关系。

        Args:
            from_id: 起点航点 ID。
            to_id: 终点航点 ID。
        """
        # 按无向边语义：同时移除两个方向的连接
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
        """保存交通图至文件。

        将当前编辑器中的航点、连接及地图关联数据序列化至指定 YAML 文件。

        Args:
            file_path: 目标 YAML 文件路径。

        Returns:
            保存成功返回 True，失败返回 False。
        """
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
        """从文件加载交通图。

        读取指定 YAML 文件中的交通图数据，还原航点和连接关系至编辑器。

        Args:
            file_path: YAML 文件路径。

        Returns:
            加载成功返回 True，失败返回 False。
        """
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
        """将屏幕坐标转换为世界坐标。

        Args:
            screen_pos: 屏幕坐标点。

        Returns:
            世界坐标系下的 (x, y) 坐标元组。
        """
        x = (screen_pos.x() - self.offset.x()) / self.scale
        y = (screen_pos.y() - self.offset.y()) / self.scale
        return x, y

    def get_screen_pos(self, world_x, world_y):
        """将世界坐标转换为屏幕坐标。

        Args:
            world_x: 世界坐标系下的 x 坐标。
            world_y: 世界坐标系下的 y 坐标。

        Returns:
            屏幕坐标系下的 (x, y) 坐标元组。
        """
        x = world_x * self.scale + self.offset.x()
        y = world_y * self.scale + self.offset.y()
        return x, y

    def get_waypoint_at(self, pos):
        """获取指定屏幕位置处的航点。

        检测给定屏幕坐标点是否落在某个航点的点击范围内。

        Args:
            pos: 屏幕坐标点。

        Returns:
            航点 ID；若未命中任何航点则返回 None。
        """
        for wp_id, wp_data in self.waypoints.items():
            wx, wy = self.get_screen_pos(wp_data['pose'].position.x, wp_data['pose'].position.y)
            dist = ((pos.x() - wx) ** 2 + (pos.y() - wy) ** 2) ** 0.5
            if dist <= self.waypoint_radius * 2:
                return wp_id
        return None

    def wheelEvent(self, event: QWheelEvent):
        """鼠标滚轮事件处理 — 视图缩放。

        以鼠标当前位置为中心对地图视图进行缩放，缩放范围限制在 0.1 ~ 5.0 倍。

        Args:
            event: 滚轮事件对象。
        """
        delta = event.angleDelta().y()
        zoom_factor = 1.1 if delta > 0 else 0.9

        old_scale = self.scale
        self.scale = max(0.1, min(5.0, self.scale * zoom_factor))

        # 以鼠标位置为中心进行缩放（保持鼠标下的世界坐标不变）
        mouse_pos = event.pos()
        self.offset = QPointF(mouse_pos) - (QPointF(mouse_pos) - self.offset) * (self.scale / old_scale)

        self.update()

    def mousePressEvent(self, event: QMouseEvent):
        """鼠标按下事件处理。

        根据当前交互模式处理左键点击（平移/添加/连接/编辑），
        以及右键点击删除航点、中键拖拽平移等操作。

        Args:
            event: 鼠标事件对象。
        """
        if event.button() == Qt.LeftButton:
            # -----------------------------------------------------------------------
            # 左键事件 — 根据交互模式分发
            # -----------------------------------------------------------------------

            # 平移模式：左键拖拽平移地图
            if self.interaction_mode == "pan":
                self.dragging = True
                self.last_mouse_pos = event.pos()
                self.setCursor(Qt.ClosedHandCursor)
                return

            wp_id = self.get_waypoint_at(event.pos())

            if wp_id:
                # 点击到航点：选中该航点高亮
                self.selected_waypoint = wp_id

                if self.interaction_mode == "connect":
                    # 连接模式：第一次点击选择起点，第二次点击选择终点创建连接
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
                    # 编辑模式：弹出对话框修改航点属性
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
                    # 添加模式：在点击位置创建新航点，自动生成 ID
                    wp_id = self._generate_waypoint_id()
                    x, y = self.get_world_pos(event.pos())
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    self.add_waypoint(wp_id, pose)
                    self.waypoint_added.emit(wp_id, pose)
                elif self.interaction_mode == "connect":
                    # 连接模式下点击空白处：取消当前连接构建
                    self.connecting_from = None

        elif event.button() == Qt.RightButton:
            # -----------------------------------------------------------------------
            # 右键事件 — 删除航点
            # -----------------------------------------------------------------------
            wp_id = self.get_waypoint_at(event.pos())
            if wp_id:
                reply = QMessageBox.question(
                    self, "确认删除",
                    f"确定要删除航点 '{wp_id}' 吗？",
                    QMessageBox.Yes | QMessageBox.No
                )
                if reply == QMessageBox.Yes:
                    self.remove_waypoint(wp_id)
                    self.waypoint_removed.emit(wp_id)

        elif event.button() == Qt.MiddleButton:
            # -----------------------------------------------------------------------
            # 中键事件 — 拖拽平移地图
            # -----------------------------------------------------------------------
            self.dragging = True
            self.last_mouse_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)

        self.update()

    def mouseReleaseEvent(self, event: QMouseEvent):
        """鼠标释放事件处理。

        结束拖拽操作并将光标恢复为默认样式。

        Args:
            event: 鼠标事件对象。
        """
        if event.button() in (Qt.MiddleButton, Qt.LeftButton):
            self.dragging = False
            self.setCursor(Qt.ArrowCursor)

    def mouseMoveEvent(self, event: QMouseEvent):
        """鼠标移动事件处理。

        更新悬停航点信息，并在拖拽状态下手动平移地图视图。

        Args:
            event: 鼠标事件对象。
        """
        self.hovered_waypoint = self.get_waypoint_at(event.pos())

        if self.dragging and self.last_mouse_pos:
            # 计算拖拽偏移并更新视图偏移量
            delta = event.pos() - self.last_mouse_pos
            self.offset += QPointF(delta.x(), delta.y())
            self.last_mouse_pos = event.pos()

        self.update()

    def paintEvent(self, event):
        """控件绘制事件处理。

        渲染地图底图、航线连接、航点标记以及连接预览线。

        Args:
            event: 绘制事件对象。
        """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # -----------------------------------------------------------------------
        # 绘制背景
        # -----------------------------------------------------------------------
        painter.fillRect(self.rect(), QColor(245, 247, 250))

        # -----------------------------------------------------------------------
        # 绘制地图底图
        # -----------------------------------------------------------------------
        if self.map_image:
            painter.save()
            painter.translate(self.offset)
            painter.scale(self.scale, self.scale)
            painter.drawPixmap(0, 0, self.map_image)
            painter.restore()

        # -----------------------------------------------------------------------
        # 绘制航线连接
        # -----------------------------------------------------------------------
        painter.save()
        painter.translate(self.offset)
        painter.scale(self.scale, self.scale)

        pen = QPen()
        pen.setWidth(3)
        pen.setColor(QColor(102, 126, 234, 180))

        for from_id, to_id in self.connections:
            if from_id in self.waypoints and to_id in self.waypoints:
                # 无向边对应双向连接，为避免重复绘制仅处理单向
                if (to_id, from_id) in self.connections and from_id > to_id:
                    continue
                from_wp = self.waypoints[from_id]
                to_wp = self.waypoints[to_id]

                painter.setPen(pen)
                painter.drawLine(
                    int(from_wp['pose'].position.x), int(from_wp['pose'].position.y),
                    int(to_wp['pose'].position.x), int(to_wp['pose'].position.y)
                )

        # -----------------------------------------------------------------------
        # 绘制航点
        # -----------------------------------------------------------------------
        for wp_id, wp_data in self.waypoints.items():
            x = wp_data['pose'].position.x
            y = wp_data['pose'].position.y

            # 根据航点状态选择颜色
            if wp_id == self.selected_waypoint:
                color = QColor(244, 67, 54)  # 红色：选中状态
            elif wp_id == self.hovered_waypoint:
                color = QColor(255, 152, 0)  # 橙色：悬停状态
            elif wp_data['is_charging_station']:
                color = QColor(76, 175, 80)  # 绿色：充电站
            elif wp_data['is_parking_spot']:
                color = QColor(33, 150, 243)  # 蓝色：停车位
            else:
                color = QColor(102, 126, 234)  # 紫色：普通航点

            # 绘制航点圆形标记
            pen = QPen(color.darker(120), 2)
            brush = QBrush(color)
            painter.setPen(pen)
            painter.setBrush(brush)

            radius = self.waypoint_radius / self.scale
            # drawEllipse 需要整数参数
            painter.drawEllipse(int(x - radius), int(y - radius), int(radius * 2), int(radius * 2))

            # 选中或悬停时显示航点 ID 标签
            if wp_id == self.selected_waypoint or wp_id == self.hovered_waypoint:
                painter.setPen(QColor(51, 51, 51))
                font = QFont("Microsoft YaHei", 9, QFont.Bold)
                painter.setFont(font)
                painter.drawText(
                    int(x + radius * 1.5),
                    int(y - radius),
                    wp_id
                )

        # -----------------------------------------------------------------------
        # 绘制连接预览线
        # -----------------------------------------------------------------------
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

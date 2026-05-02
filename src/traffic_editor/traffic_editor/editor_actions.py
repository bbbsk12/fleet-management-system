#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from PyQt5.QtWidgets import QFileDialog, QMessageBox, QInputDialog

from fleet_msgs.srv import LoadTrafficMap, SubmitTask


def load_map_yaml(window) -> None:
    file_path, _ = QFileDialog.getOpenFileName(
        window, "加载地图文件", "", "YAML文件 (*.yaml *.yml);;所有文件 (*)")
    if file_path and window.map_widget.load_map(file_path):
        window.map_yaml_path = file_path
        window.statusBar().showMessage(f"Loaded map: {os.path.basename(file_path)}")


def load_traffic_map(window) -> None:
    file_path, _ = QFileDialog.getOpenFileName(
        window, "加载交通图", "", "YAML文件 (*.yaml *.yml);;所有文件 (*)")
    if file_path and window.map_widget.load_traffic_map(file_path):
        window.current_file = file_path
        window.statusBar().showMessage(f"Loaded traffic map: {os.path.basename(file_path)}")


def save_traffic_map(window) -> None:
    if not window.current_file:
        file_path, _ = QFileDialog.getSaveFileName(
            window, "保存交通图", "", "YAML文件 (*.yaml *.yml)")
        if not file_path:
            return
        window.current_file = file_path

    if not window.map_widget.save_traffic_map(window.current_file):
        return

    window.statusBar().showMessage(f"Saved traffic map: {os.path.basename(window.current_file)}")
    if window.ros_node and window.ros_node.load_map_client.wait_for_service(timeout_sec=1.0):
        req = LoadTrafficMap.Request()
        req.file_path = window.current_file
        window.ros_node.load_map_client.call_async(req)


def submit_task(window) -> None:
    if not window.map_widget.waypoints:
        QMessageBox.warning(window, "Warning", "请先添加航点！")
        return

    waypoint_ids = list(window.map_widget.waypoints.keys())
    waypoint_id, ok = QInputDialog.getItem(
        window, "提交任务", "选择目标航点:", waypoint_ids, 0, False)
    if not (ok and waypoint_id):
        return

    task_types = ["1: 巡航(CRUISE)", "2: 上料(LOAD)", "3: 放置(UNLOAD)", "4: 地点任务(SITE_SPECIFIC)"]
    type_str, ok2 = QInputDialog.getItem(
        window, "任务类型", "选择任务类型:", task_types, 0, False)
    if not ok2:
        return

    task_type = int(type_str.split(":")[0])
    site_code = 0
    if task_type in (2, 3, 4):
        site_code_str, ok3 = QInputDialog.getText(
            window, "站点代码", "输入 site_code (十六进制，如 0x01):", text="0x01")
        if ok3 and site_code_str:
            try:
                site_code = int(site_code_str, 16)
            except ValueError:
                try:
                    site_code = int(site_code_str)
                except ValueError:
                    site_code = 0

    if window.ros_node and window.ros_node.submit_task_client.wait_for_service(timeout_sec=1.0):
        req = SubmitTask.Request()
        req.waypoint_id = waypoint_id
        req.priority = 0
        req.task_type = task_type
        req.site_code = site_code
        window.ros_node.submit_task_client.call_async(req)
        type_names = {1: "巡航", 2: "上料", 3: "放置", 4: "地点任务"}
        QMessageBox.information(
            window,
            "Success",
            f"Submitted {type_names.get(task_type, '?')} task to waypoint {waypoint_id}",
        )


def clear_waypoints(window) -> None:
    reply = QMessageBox.question(
        window, "确认清空",
        "确定要删除所有航点吗？此操作不可撤销！",
        QMessageBox.Yes | QMessageBox.No,
        QMessageBox.No
    )
    if reply == QMessageBox.Yes:
        window.map_widget.clear_all()
        window.statusBar().showMessage("Cleared all waypoints and connections")
        window.update_stats()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
交通图文件读写模块。

提供交通图数据与 YAML 文件之间的序列化与反序列化功能，
包括航点数据、连接关系以及地图元信息的导出与导入。
"""

import os
from typing import Dict, List, Tuple

import yaml
from geometry_msgs.msg import Pose


def dump_traffic_map(
    file_path: str,
    waypoints: Dict[str, dict],
    map_yaml_path: str = "",
    map_image_file: str = "",
    map_yaml_meta: dict | None = None,
) -> None:
    """将交通图数据导出到 YAML 文件。

    将当前编辑器中的航点数据、连接关系及地图关联信息
    序列化并写入指定路径的 YAML 文件，供后续加载使用。

    Args:
        file_path: 目标 YAML 文件路径。
        waypoints: 航点数据字典，键为航点 ID，值为航点属性字典。
        map_yaml_path: 地图 YAML 文件路径（可选），用于关联底图。
        map_image_file: 地图图片文件名（可选）。
        map_yaml_meta: 地图元信息字典（可选），包含分辨率与原点坐标。
    """
    resolved_map_yaml_path = map_yaml_path
    if resolved_map_yaml_path:
        target_dir = os.path.dirname(os.path.abspath(file_path))
        try:
            resolved_map_yaml_path = os.path.relpath(resolved_map_yaml_path, start=target_dir)
        except ValueError:
            resolved_map_yaml_path = os.path.abspath(resolved_map_yaml_path)

    data = {
        "map_id": os.path.basename(file_path),
        "map_name": "交通图",
        "map_yaml_path": resolved_map_yaml_path,
        "map_image": map_image_file,
        "waypoints": [],
    }
    if map_yaml_meta:
        data["map_resolution"] = map_yaml_meta.get("resolution", 0.05)
        data["map_origin"] = map_yaml_meta.get("origin", [0.0, 0.0, 0.0])

    existing_ids = set(waypoints.keys())
    for wp_id, wp_data in waypoints.items():
        raw_connections = wp_data.get("connections", []) or []
        connections: List[str] = []
        seen = set()
        for cid in raw_connections:
            if cid == wp_id or cid not in existing_ids or cid in seen:
                continue
            seen.add(cid)
            connections.append(cid)

        waypoint = {
            "id": wp_id,
            "name": wp_data["name"],
            "position": {
                "x": wp_data["pose"].position.x,
                "y": wp_data["pose"].position.y,
                "z": wp_data["pose"].position.z,
            },
            "connections": connections,
            "is_parking_spot": wp_data["is_parking_spot"],
            "is_charging_station": wp_data["is_charging_station"],
            "radius": wp_data["radius"],
        }
        data["waypoints"].append(waypoint)

    with open(file_path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)


def load_traffic_map(file_path: str) -> Tuple[Dict[str, dict], List[Tuple[str, str]], int]:
    """从 YAML 文件加载交通图数据。

    从指定 YAML 文件中读取并解析交通图数据，还原航点位置、
    属性及连接关系，并自动确保无向图连接的对称一致性。

    Args:
        file_path: YAML 文件路径。

    Returns:
        包含三个元素的元组：
            - 航点数据字典
            - 连接关系列表（每项为 (起点ID, 终点ID) 二元组）
            - 当前最大航点编号
    """
    with open(file_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    waypoints: Dict[str, dict] = {}
    connections: List[Tuple[str, str]] = []
    max_num = 0

    for wp_data in data.get("waypoints", []):
        pose = Pose()
        pose.position.x = wp_data["position"]["x"]
        pose.position.y = wp_data["position"]["y"]
        pose.position.z = wp_data["position"].get("z", 0.0)

        wp_id_str = str(wp_data.get("id", ""))
        if wp_id_str.startswith("wp_"):
            try:
                max_num = max(max_num, int(wp_id_str.split("_")[-1]))
            except Exception:
                pass

        waypoints[wp_data["id"]] = {
            "pose": pose,
            "name": wp_data.get("name", wp_data["id"]),
            "connections": wp_data.get("connections", []),
            "is_parking_spot": wp_data.get("is_parking_spot", False),
            "is_charging_station": wp_data.get("is_charging_station", False),
            "radius": wp_data.get("radius", 0.5),
        }
        for conn_id in wp_data.get("connections", []):
            edge = (wp_data["id"], conn_id)
            if edge not in connections:
                connections.append(edge)

    # -----------------------------------------------------------------------
    # 无向图连接一致性修正
    # -----------------------------------------------------------------------
    # 若航点 A 的连接列表包含 B 但 B 的连接列表中不包含 A，
    # 则自动将 A 添加至 B 的连接列表，以确保无向边的对称性。
    for a_id, a_data in waypoints.items():
        for b_id in list(a_data.get("connections", []) or []):
            if b_id not in waypoints:
                continue
            b_conns = waypoints[b_id].get("connections", [])
            if a_id not in b_conns:
                b_conns.append(a_id)
                waypoints[b_id]["connections"] = b_conns

    # 根据修正后的航点数据重新生成无重复的连接关系列表
    connections = []
    for a_id, a_data in waypoints.items():
        for b_id in a_data.get("connections", []) or []:
            if a_id != b_id and (a_id, b_id) not in connections:
                connections.append((a_id, b_id))

    return waypoints, connections, max_num

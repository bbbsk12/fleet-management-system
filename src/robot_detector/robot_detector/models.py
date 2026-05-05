#!/usr/bin/env python3
"""
机器人信息数据模型模块。

定义统一的机器人信息数据结构（RobotInfo），
在 ROS2 节点检测器与 CLI 检测器之间共享使用。
"""

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional


@dataclass
class RobotInfo:
    """统一的机器人信息模型，同时适用于 ROS2 节点检测和 CLI 检测。"""

    # ---- 基础标识字段 ----
    namespace: str
    name: str

    # ---- CLI 检测器专用字段 ----
    topics: List[str] = field(default_factory=list)
    actions: List[str] = field(default_factory=list)
    services: List[str] = field(default_factory=list)
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)
    has_scan: bool = False
    has_map: bool = False
    connection_score: int = 0

    # ---- ROS2 节点检测器共享字段 ----
    has_odom: bool = False
    has_battery: bool = False
    has_tf: bool = False
    has_amcl: bool = False
    has_cmd_vel: bool = False
    has_nav2: bool = False
    battery_percent: float = 0.0
    position: Optional[Dict] = None
    last_update: float = 0.0

    def to_dict(self) -> dict:
        """将机器人信息转换为字典格式。"""
        return asdict(self)

    @property
    def is_fully_connected(self) -> bool:
        """判断机器人是否达到完全连接状态（同时具备里程计与坐标变换）。"""
        return self.has_odom and self.has_tf

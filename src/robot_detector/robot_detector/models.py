#!/usr/bin/env python3
"""Shared data models for robot detector tools."""

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional


@dataclass
class RobotInfo:
    """Unified robot info model used by ROS and CLI detectors."""

    namespace: str
    name: str

    # CLI detector fields
    topics: List[str] = field(default_factory=list)
    actions: List[str] = field(default_factory=list)
    services: List[str] = field(default_factory=list)
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)
    has_scan: bool = False
    has_map: bool = False
    connection_score: int = 0

    # Shared/ROS detector fields
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
        return asdict(self)

    @property
    def is_fully_connected(self) -> bool:
        return self.has_odom and self.has_tf

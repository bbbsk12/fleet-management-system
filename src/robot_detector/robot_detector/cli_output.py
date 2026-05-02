#!/usr/bin/env python3
"""Output utilities for robot detector CLI."""

import json
import subprocess
import time
from typing import Dict

try:
    from .models import RobotInfo
except ImportError:
    from models import RobotInfo


def check_zenohd_running() -> bool:
    try:
        result = subprocess.run(
            ["pgrep", "-f", "zenohd"],
            capture_output=True,
            text=True,
        )
        return result.returncode == 0
    except Exception:
        return False


def print_results(robots: Dict[str, RobotInfo], zenoh_router: str) -> None:
    print()
    print("=" * 60)
    print("检测结果")
    print("=" * 60)
    print()

    print(f"Zenoh路由器: {zenoh_router}")
    print("zenohd状态: ✅ 运行中" if check_zenohd_running() else "zenohd状态: ❌ 未运行")
    print()

    if not robots:
        print("❌ 未检测到任何机器人底盘!")
        print()
        print("=" * 60)
        print("底盘端配置建议")
        print("=" * 60)
        print(
            """
# 底盘端 zenoh 配置示例 (~/zenoh_robot.yaml)
plugins:
  ros2dds:
    namespace: "/hefbot1"
    allow:
      publishers: [
        ".*/tf", ".*/odom", ".*/battery_state", ".*/scan",
        ".*/amcl_pose", ".*/particle_cloud", ".*/map", ".*/plan"
      ]
      subscribers: [".*/cmd_vel", ".*/initialpose"]
      action_servers: [".*/navigate_to_pose", ".*/follow_path"]

mode: "client"
connect: { endpoints: ["tcp/<总控IP>:7447"] }
"""
        )
        return

    print(f"✅ 检测到 {len(robots)} 个机器人底盘")
    print("=" * 60)

    for ns, robot in sorted(robots.items(), key=lambda x: -x[1].connection_score):
        print()
        print(f"📦 {robot.name}")
        print("-" * 40)
        print(f"  命名空间: {ns}")
        print(f"  状态: {'✅ 已连接' if robot.is_fully_connected else '⚠️ 部分连接'}")
        print(f"  连接评分: {robot.connection_score}/9")
        print()

        print("  话题状态:")
        print(f"    {'✅' if robot.has_odom else '❌'} /odom (里程计)")
        print(f"    {'✅' if robot.has_tf else '❌'} /tf (坐标变换)")
        print(
            f"    {'✅' if robot.has_cmd_vel else '❌'} /cmd_vel (速度控制) "
            f"{'[订阅]' if 'cmd_vel' in ' '.join(robot.subscribers).lower() else ''}"
        )
        print(f"    {'✅' if robot.has_battery else '❌'} /battery_state (电池)")
        print(f"    {'✅' if robot.has_scan else '❌'} /scan (激光雷达)")
        print(f"    {'✅' if robot.has_amcl else '❌'} /amcl_pose (定位)")
        print(f"    {'✅' if robot.has_map else '❌'} /map (地图)")
        print()

        print("  导航能力:")
        print(f"    {'✅' if robot.has_nav2 else '❌'} Nav2 导航")

        if robot.publishers:
            print()
            print(f"  发布者话题 ({len(robot.publishers)}):")
            for t in sorted(robot.publishers)[:5]:
                print(f"    - {t}")
            if len(robot.publishers) > 5:
                print(f"    ... 还有 {len(robot.publishers) - 5} 个")

        if robot.subscribers:
            print()
            print(f"  订阅者话题 ({len(robot.subscribers)}):")
            for t in sorted(robot.subscribers)[:5]:
                print(f"    - {t}")
            if len(robot.subscribers) > 5:
                print(f"    ... 还有 {len(robot.subscribers) - 5} 个")

        if robot.topics:
            print()
            print(f"  发布话题 ({len(robot.topics)}):")
            for t in sorted(robot.topics)[:5]:
                print(f"    - {t}")
            if len(robot.topics) > 5:
                print(f"    ... 还有 {len(robot.topics) - 5} 个")

        if robot.actions:
            print()
            print(f"  Actions ({len(robot.actions)}):")
            for a in sorted(robot.actions):
                print(f"    - {a}")

    print()
    print("=" * 60)


def to_json_payload(robots: Dict[str, RobotInfo], router: str) -> dict:
    return {
        "timestamp": time.time(),
        "router": router,
        "robot_count": len(robots),
        "robots": [
            {
                "namespace": r.namespace,
                "name": r.name,
                "score": r.connection_score,
                "has_odom": r.has_odom,
                "has_tf": r.has_tf,
                "has_scan": r.has_scan,
                "has_amcl": r.has_amcl,
                "has_nav2": r.has_nav2,
                "topics": r.topics,
                "actions": r.actions,
            }
            for r in robots.values()
        ],
    }


def print_json_payload(robots: Dict[str, RobotInfo], router: str) -> None:
    print(json.dumps(to_json_payload(robots, router), indent=2, ensure_ascii=False))

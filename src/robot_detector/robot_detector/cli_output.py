#!/usr/bin/env python3
"""
CLI 输出工具模块。

提供机器人检测结果的格式化输出功能，
包括控制台表格展示、JSON 序列化及 zenohd 状态检查。
"""

import json
import subprocess
import time
from typing import Dict

try:
    from .models import RobotInfo
except ImportError:
    from models import RobotInfo


def check_zenohd_running() -> bool:
    """检查 zenohd 进程是否正在运行。

    通过 pgrep 命令匹配 zenohd 进程名来判断。

    返回:
        如果 zenohd 正在运行则返回 True，否则返回 False。
    """
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
    """在控制台打印格式化的机器人检测结果。

    参数:
        robots: 命名空间到机器人信息对象的映射字典。
        zenoh_router: 使用的 Zenoh 路由器地址。
    """
    # ---- 检测概要信息 ----
    print()
    print("=" * 60)
    print("检测结果")
    print("=" * 60)
    print()

    print(f"Zenoh路由器: {zenoh_router}")
    print("zenohd状态: ✅ 运行中" if check_zenohd_running() else "zenohd状态: ❌ 未运行")
    print()

    # ---- 未检测到机器人的处理 ----
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

    # ---- 检测到的机器人列表 ----
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

        # 话题状态
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

        # 导航能力
        print("  导航能力:")
        print(f"    {'✅' if robot.has_nav2 else '❌'} Nav2 导航")

        # 发布者话题列表
        if robot.publishers:
            print()
            print(f"  发布者话题 ({len(robot.publishers)}):")
            for t in sorted(robot.publishers)[:5]:
                print(f"    - {t}")
            if len(robot.publishers) > 5:
                print(f"    ... 还有 {len(robot.publishers) - 5} 个")

        # 订阅者话题列表
        if robot.subscribers:
            print()
            print(f"  订阅者话题 ({len(robot.subscribers)}):")
            for t in sorted(robot.subscribers)[:5]:
                print(f"    - {t}")
            if len(robot.subscribers) > 5:
                print(f"    ... 还有 {len(robot.subscribers) - 5} 个")

        # 发布话题列表
        if robot.topics:
            print()
            print(f"  发布话题 ({len(robot.topics)}):")
            for t in sorted(robot.topics)[:5]:
                print(f"    - {t}")
            if len(robot.topics) > 5:
                print(f"    ... 还有 {len(robot.topics) - 5} 个")

        # Action 列表
        if robot.actions:
            print()
            print(f"  Actions ({len(robot.actions)}):")
            for a in sorted(robot.actions):
                print(f"    - {a}")

    print()
    print("=" * 60)


def to_json_payload(robots: Dict[str, RobotInfo], router: str) -> dict:
    """将检测结果转换为 JSON 序列化友好的字典结构。

    参数:
        robots: 命名空间到机器人信息对象的映射字典。
        router: Zenoh 路由器地址。

    返回:
        包含时间戳、路由器地址和机器人列表的字典。
    """
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
    """以 JSON 格式打印检测结果到标准输出。

    参数:
        robots: 命名空间到机器人信息对象的映射字典。
        router: Zenoh 路由器地址。
    """
    print(json.dumps(to_json_payload(robots, router), indent=2, ensure_ascii=False))

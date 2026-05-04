#!/usr/bin/env python3
"""
Zenoh 扫描工具模块。

提供基于 Zenoh admin space 与话题订阅的机器人发现能力，
通过合并两种来源的信息并计算连接评分来评估各机器人状态。
"""

import json
import time
from typing import Dict, Set

try:
    from .models import RobotInfo
except ImportError:
    from models import RobotInfo


def _init_ns_bucket(container: dict, ns: str) -> None:
    """初始化容器中指定命名空间的桶结构。

    如果命名空间尚未在容器中存在，则创建一个包含空集合的条目。

    参数:
        container: 命名空间分发容器字典。
        ns: 目标命名空间。
    """
    if ns not in container:
        container[ns] = {
            "topics": set(),
            "actions": set(),
            "services": set(),
            "publishers": set(),
            "subscribers": set(),
        }


def _merge_and_score(ros2dds_info: dict, discovered: Dict[str, Set[str]]) -> Dict[str, RobotInfo]:
    """合并 admin space 信息与话题发现信息，计算连接评分并生成机器人对象。

    参数:
        ros2dds_info: 从 admin space 查询到的命名空间信息字典。
        discovered: 通过话题订阅发现到的命名空间及其话题集合映射。

    返回:
        符合评分阈值的命名空间到 RobotInfo 对象的映射字典。
    """
    robots: Dict[str, RobotInfo] = {}
    all_namespaces = set(ros2dds_info.keys()) | set(discovered.keys())

    for ns in all_namespaces:
        # ---- 合并两种来源的信息 ----
        topics = set()
        actions = set()
        services = set()
        publishers = set()
        subscribers = set()

        if ns in ros2dds_info:
            topics.update(ros2dds_info[ns]["topics"])
            actions.update(ros2dds_info[ns]["actions"])
            services.update(ros2dds_info[ns]["services"])
            publishers.update(ros2dds_info[ns].get("publishers", set()))
            subscribers.update(ros2dds_info[ns].get("subscribers", set()))

        if ns in discovered:
            topics.update(discovered[ns])

        # ---- 创建机器人信息对象 ----
        robot = RobotInfo(
            namespace=ns,
            name=ns.strip("/"),
            topics=list(topics),
            actions=list(actions),
            services=list(services),
            publishers=list(publishers),
            subscribers=list(subscribers),
        )

        # ---- 根据话题关键字设置功能标志 ----
        all_topics_str = " ".join(topics).lower()
        all_actions_str = " ".join(actions).lower()
        all_publishers_str = " ".join(publishers).lower()
        all_subscribers_str = " ".join(subscribers).lower()

        if "odom" in all_topics_str or "odom" in all_publishers_str:
            robot.has_odom = True
        if "tf" in all_topics_str or "tf" in all_publishers_str:
            robot.has_tf = True
        if "battery" in all_topics_str or "battery" in all_publishers_str:
            robot.has_battery = True
        if "scan" in all_topics_str or "scan" in all_publishers_str:
            robot.has_scan = True
        if "amcl" in all_topics_str or "particle" in all_topics_str:
            robot.has_amcl = True
        if "map" in all_topics_str:
            robot.has_map = True
        if "cmd_vel" in all_subscribers_str or "cmd_vel" in all_topics_str:
            robot.has_cmd_vel = True
        if "navigate" in all_actions_str or "follow_path" in all_actions_str:
            robot.has_nav2 = True

        # ---- 计算连接评分（满分 9 分） ----
        score = 0
        if robot.has_odom:
            score += 2
        if robot.has_tf:
            score += 2
        if robot.has_scan:
            score += 1
        if robot.has_amcl:
            score += 1
        if robot.has_nav2:
            score += 2
        if robot.has_cmd_vel:
            score += 1
        robot.connection_score = score

        # ---- 仅保留评分 >= 2 的机器人 ----
        if score >= 2:
            robots[ns] = robot

    return robots


def detect_robots_via_zenoh(zenoh, zenoh_router: str) -> Dict[str, RobotInfo]:
    """通过 Zenoh admin space 和话题采样进行机器人发现。

    执行两阶段发现：
      1. 查询 Zenoh admin space 获取 ros2dds 插件收集的发布/订阅/动作信息。
      2. 短暂订阅常见 ROS2 话题模式以确认命名空间活跃状态。

    参数:
        zenoh: zenoh 模块引用。
        zenoh_router: Zenoh 路由器地址字符串。

    返回:
        命名空间到 RobotInfo 对象的映射字典。
    """
    robots: Dict[str, RobotInfo] = {}
    print(f"连接到Zenoh路由器: {zenoh_router}")
    print()

    try:
        # ---- 创建 Zenoh 会话 ----
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", json.dumps([zenoh_router]))

        print("正在打开Zenoh会话...")
        session = zenoh.open(conf)
        print("✓ Zenoh会话已建立")
        print()

        # ---- 阶段一：查询 admin space ----
        print("[1/2] 查询 Zenoh admin space...")
        ros2dds_info = {}
        try:
            replies = session.get("@/**", consolidation=zenoh.ConsolidationMode.LATEST)
            for reply in replies:
                try:
                    if not (hasattr(reply, "ok") and reply.ok):
                        continue
                    key = str(reply.ok.key_expr)
                    parts = key.split("/")
                    if "publisher" in parts:
                        idx = parts.index("publisher")
                        if idx + 2 < len(parts):
                            ns = "/" + parts[idx + 1]
                            topic = "/".join(parts[idx + 2:])
                            _init_ns_bucket(ros2dds_info, ns)
                            ros2dds_info[ns]["publishers"].add(topic)
                            ros2dds_info[ns]["topics"].add(topic)
                    elif "subscriber" in parts:
                        idx = parts.index("subscriber")
                        if idx + 2 < len(parts):
                            ns = "/" + parts[idx + 1]
                            topic = "/".join(parts[idx + 2:])
                            _init_ns_bucket(ros2dds_info, ns)
                            ros2dds_info[ns]["subscribers"].add(topic)
                            ros2dds_info[ns]["topics"].add(topic)
                    elif "action" in parts and "srv" in parts:
                        idx = parts.index("action")
                        if idx + 2 < len(parts) and parts[idx + 1] == "srv":
                            ns = "/" + parts[idx + 2]
                            if idx + 3 < len(parts):
                                action = "/".join(parts[idx + 3:])
                                _init_ns_bucket(ros2dds_info, ns)
                                ros2dds_info[ns]["actions"].add(action)
                except Exception:
                    pass
        except Exception as e:
            print(f"  Admin space 查询失败: {e}")

        print(f"  从 admin space 发现 {len(ros2dds_info)} 个命名空间")
        print()

        # ---- 阶段二：话题订阅确认 ----
        print("[2/2] 订阅话题确认活跃状态...")

        discovered: Dict[str, Set[str]] = {}

        def on_sample(sample):
            """话题采样回调：将采样到的 key 归入对应命名空间。"""
            key = str(sample.key_expr)
            parts = key.strip("/").split("/")
            if len(parts) >= 1:
                ns_raw = parts[0]
                ns = ns_raw if ns_raw.startswith("/") else "/" + ns_raw
                if ns not in discovered:
                    discovered[ns] = set()
                discovered[ns].add(key)

        # 常用的机器人话题模式
        topic_patterns = [
            "**/odom",
            "**/tf",
            "**/scan",
            "**/amcl_pose",
            "**/particle_cloud",
            "**/map",
            "**/plan",
            "**/battery",
        ]
        subs = []
        for pattern in topic_patterns:
            try:
                sub = session.declare_subscriber(pattern, on_sample)
                subs.append(sub)
            except Exception:
                pass

        time.sleep(2)
        for sub in subs:
            try:
                session.undeclare_subscriber(sub)
            except Exception:
                pass

        # ---- 合并信息并生成结果 ----
        robots = _merge_and_score(ros2dds_info, discovered)
        session.close()
        print("✓ Zenoh会话已关闭")
    except Exception as e:
        print(f"✗ Zenoh连接失败: {e}")

    return robots

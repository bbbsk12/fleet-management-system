#!/usr/bin/env python3
"""
机器人底盘检测工具 —— CLI 命令行版本。

通过 Zenoh 进行跨 ROS_DOMAIN_ID 的机器人底盘发现，
支持单次检测、JSON 输出与持续监控模式。
"""

import subprocess
import json
import time
import argparse
import sys
from typing import Dict, List, Optional, Set
try:
    from .models import RobotInfo
    from .zenoh_scan import detect_robots_via_zenoh as detect_robots_via_zenoh_impl
    from .cli_output import (
        check_zenohd_running as check_zenohd_running_impl,
        print_results as print_results_impl,
        print_json_payload,
    )
except ImportError:
    from models import RobotInfo
    from zenoh_scan import detect_robots_via_zenoh as detect_robots_via_zenoh_impl
    from cli_output import (
        check_zenohd_running as check_zenohd_running_impl,
        print_results as print_results_impl,
        print_json_payload,
    )

# ---- Zenoh 依赖检查 ----
try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False


def check_zenohd_running() -> bool:
    """检查 zenohd 守护进程是否正在运行。"""
    return check_zenohd_running_impl()


def detect_robots_via_zenoh(zenoh_router: str) -> Dict[str, RobotInfo]:
    """通过 Zenoh 检测机器人（支持跨 ROS_DOMAIN_ID）。

    参数:
        zenoh_router: Zenoh 路由器地址字符串。

    返回:
        命名空间到机器人信息对象的映射字典。
    """
    if not HAS_ZENOH:
        print("错误: 需要安装 zenoh-python")
        print("安装命令: pip install eclipse-zenoh")
        return {}
    return detect_robots_via_zenoh_impl(zenoh, zenoh_router)


def print_results(robots: Dict[str, RobotInfo], zenoh_router: str):
    """打印检测结果到控制台。

    参数:
        robots: 命名空间到机器人信息对象的映射字典。
        zenoh_router: 使用的 Zenoh 路由器地址。
    """
    print_results_impl(robots, zenoh_router)


def main():
    """CLI 入口函数：解析命令行参数并执行检测流程。"""
    parser = argparse.ArgumentParser(description='通过Zenoh检测机器人底盘（支持跨ROS_DOMAIN_ID）')
    parser.add_argument(
        '--router', '-r',
        default="tcp/127.0.0.1:7447",
        help='Zenoh路由器地址 (默认: tcp/127.0.0.1:7447)'
    )
    parser.add_argument(
        '--json', '-j',
        action='store_true',
        help='输出JSON格式'
    )
    parser.add_argument(
        '--watch', '-w',
        action='store_true',
        help='持续监控模式'
    )
    parser.add_argument(
        '--interval', '-i',
        type=float,
        default=5.0,
        help='监控间隔 (默认: 5秒)'
    )

    args = parser.parse_args()

    if not HAS_ZENOH:
        print("错误: 需要安装 zenoh-python")
        print("安装命令: pip install eclipse-zenoh")
        sys.exit(1)

    if args.watch:
        # ---- 持续监控模式 ----
        print("持续监控模式 (Ctrl+C 退出)")
        try:
            while True:
                robots = detect_robots_via_zenoh(args.router)
                print("\033[2J\033[H")
                print_results(robots, args.router)
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n监控已停止")
    else:
        # ---- 单次检测模式 ----
        robots = detect_robots_via_zenoh(args.router)
        if args.json:
            print_json_payload(robots, args.router)
        else:
            print_results(robots, args.router)


if __name__ == '__main__':
    main()

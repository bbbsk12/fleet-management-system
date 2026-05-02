#!/usr/bin/env python3
"""
机器人底盘检测工具 - 通过Zenoh检测
支持跨 ROS_DOMAIN_ID 检测连接的底盘
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

# 尝试导入 zenoh
try:
    import zenoh
    HAS_ZENOH = True
except ImportError:
    HAS_ZENOH = False


def check_zenohd_running() -> bool:
    """检查zenohd是否运行"""
    return check_zenohd_running_impl()


def detect_robots_via_zenoh(zenoh_router: str) -> Dict[str, RobotInfo]:
    """通过Zenoh检测机器人（支持跨DOMAIN_ID）"""
    if not HAS_ZENOH:
        print("错误: 需要安装 zenoh-python")
        print("安装命令: pip install eclipse-zenoh")
        return {}
    return detect_robots_via_zenoh_impl(zenoh, zenoh_router)


def print_results(robots: Dict[str, RobotInfo], zenoh_router: str):
    """打印检测结果"""
    print_results_impl(robots, zenoh_router)


def main():
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
        robots = detect_robots_via_zenoh(args.router)
        if args.json:
            print_json_payload(robots, args.router)
        else:
            print_results(robots, args.router)


if __name__ == '__main__':
    main()

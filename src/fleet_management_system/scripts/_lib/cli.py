"""CLI 参数解析工具模块。

提供命令行接口所需的公共参数解析函数，包括后端地址、轮询参数、
机器人过滤器等参数的标准化添加与解析功能。
"""

from __future__ import annotations

import argparse
import os
from typing import List, Optional


def add_base_url_arg(parser: argparse.ArgumentParser) -> None:
    """添加 Web 后端基础 URL 参数。

    向参数解析器添加 --base 参数，用于指定后端服务的基础 URL。
    默认值优先从环境变量 FLEET_API_BASE 读取，否则回退为 http://127.0.0.1:8080。

    Args:
        parser: 待添加参数的命令行参数解析器实例。
    """
    parser.add_argument(
        "--base",
        default=os.environ.get("FLEET_API_BASE", "http://127.0.0.1:8080"),
        help="Web backend base URL (or env FLEET_API_BASE).",
    )


def add_polling_args(
    parser: argparse.ArgumentParser,
    *,
    default_interval: float,
    allow_duration: bool = True,
    allow_once: bool = True,
) -> None:
    """添加轮询相关参数。

    向参数解析器添加 --once（单次检查）、--interval（轮询间隔）和
    --duration（总运行时长）参数，用于控制脚本的轮询行为。

    Args:
        parser: 待添加参数的命令行参数解析器实例。
        default_interval: 轮询间隔的默认值（秒）。
        allow_duration: 是否允许指定总运行时长参数，默认为 True。
        allow_once: 是否允许单次执行模式参数，默认为 True。
    """
    if allow_once:
        parser.add_argument("--once", action="store_true", help="Run one check and exit.")
    parser.add_argument("--interval", type=float, default=default_interval, help="Polling interval seconds.")
    if allow_duration:
        parser.add_argument("--duration", type=float, default=0.0, help="Total duration seconds; 0 means until Ctrl+C.")


def add_robots_filter_arg(parser: argparse.ArgumentParser) -> None:
    """添加机器人过滤器参数。

    向参数解析器添加 --robots 参数，用于指定要检查的机器人 ID 列表，
    多个 ID 之间以逗号分隔。为空时表示检查所有在线机器人。

    Args:
        parser: 待添加参数的命令行参数解析器实例。
    """
    parser.add_argument(
        "--robots",
        default="",
        help="Comma-separated robot ids to check (empty means all).",
    )


def parse_robots_filter(value: str) -> Optional[List[str]]:
    """解析机器人过滤器字符串。

    将逗号分隔的机器人 ID 字符串解析为列表，去除空白项。
    若解析结果为空列表则返回 None。

    Args:
        value: 逗号分隔的机器人 ID 字符串。

    Returns:
        机器人 ID 列表，若输入为空或全部为空白则返回 None。
    """
    items = [x.strip() for x in (value or "").split(",") if x.strip()]
    return items or None

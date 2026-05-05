"""命令行输出格式工具模块。

提供标准化的时间戳格式化输出函数，用于在终端中以统一格式
显示带时间前缀的状态信息和控制台横幅。
"""

from __future__ import annotations

import time


def ts_hms() -> str:
    """获取当前时间的 HH:MM:SS 格式字符串。

    Returns:
        格式化的当前时间字符串，格式为 %H:%M:%S。
    """
    return time.strftime("%H:%M:%S")


def ok_line(prefix: str, msg: str) -> str:
    """生成带时间戳和前缀的状态行。

    构造格式为 "[HH:MM:SS] 前缀 | 消息" 的单行输出字符串，
    行首使用 \\r 回车符以便覆盖终端当前行。

    Args:
        prefix: 状态前缀标识。
        msg: 要显示的消息内容。

    Returns:
        格式化后的单行字符串。
    """
    return f"\r{ts_hms()} {prefix} | {msg}"


def banner(prefix: str) -> str:
    """生成带时间戳的标题横幅。

    构造格式为 "\\n[HH:MM:SS] 前缀\\n" 的横幅字符串，
    首尾带换行符以便在终端中突出显示。

    Args:
        prefix: 横幅标题标识。

    Returns:
        格式化后的横幅字符串。
    """
    return f"\n{ts_hms()} {prefix}\n"

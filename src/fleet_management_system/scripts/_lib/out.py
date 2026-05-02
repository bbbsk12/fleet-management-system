from __future__ import annotations

import time


def ts_hms() -> str:
    return time.strftime("%H:%M:%S")


def ok_line(prefix: str, msg: str) -> str:
    return f"\r{ts_hms()} {prefix} | {msg}"


def banner(prefix: str) -> str:
    return f"\n{ts_hms()} {prefix}\n"


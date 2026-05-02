from __future__ import annotations

import argparse
import os
from typing import List, Optional


def add_base_url_arg(parser: argparse.ArgumentParser) -> None:
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
    if allow_once:
        parser.add_argument("--once", action="store_true", help="Run one check and exit.")
    parser.add_argument("--interval", type=float, default=default_interval, help="Polling interval seconds.")
    if allow_duration:
        parser.add_argument("--duration", type=float, default=0.0, help="Total duration seconds; 0 means until Ctrl+C.")


def add_robots_filter_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--robots",
        default="",
        help="Comma-separated robot ids to check (empty means all).",
    )


def parse_robots_filter(value: str) -> Optional[List[str]]:
    items = [x.strip() for x in (value or "").split(",") if x.strip()]
    return items or None


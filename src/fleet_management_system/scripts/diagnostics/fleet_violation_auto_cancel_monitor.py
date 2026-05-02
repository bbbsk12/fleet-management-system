#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Violation monitor with auto-cancel.

Polling GET /api/robots and enforcing traffic rules:
  ① 同航点：同一 current_waypoint 上多台车
  ① 同航线：同一无向边 current_segment 上多台车
  ② 点线互斥：航点 A 的车存在时，其相连航段上的车不得同时出现

If any violation is detected:
  1) DELETE /api/tasks/{task_id} for all current tasks
  2) POST /api/command with an error message (API log / websocket)
  3) Write a local error report file
  4) Exit with code 2

This is intended for "pressure test safety guard".
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

SCRIPTS_ROOT = Path(__file__).resolve().parents[1]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from _lib.http_api import http_json
from _lib.cli import add_base_url_arg, add_polling_args, add_robots_filter_arg, parse_robots_filter
from _lib.out import ts_hms
from _lib.traffic_rules import check_traffic_rule_violations


def get_robots(base: str) -> Dict[str, dict]:
    rdata = http_json(f"{base.rstrip('/')}/api/robots")
    return rdata.get("robots") or {}


def get_tasks(base: str) -> List[dict]:
    tdata = http_json(f"{base.rstrip('/')}/api/tasks")
    return tdata.get("tasks") or []


def cancel_all_tasks(base: str) -> Tuple[int, List[str]]:
    tasks = get_tasks(base)
    ids = [t.get("id") for t in tasks if t.get("id")]
    ok = 0
    failed: List[str] = []
    for tid in ids:
        try:
            http_json(f"{base.rstrip('/')}/api/tasks/{tid}", method="DELETE", body=None)
            ok += 1
        except Exception:
            failed.append(tid)
    return ok, failed


def post_error_command(base: str, message: str) -> None:
    # server.py: POST /api/command expects {command: str, payload: dict}
    try:
        http_json(f"{base.rstrip('/')}/api/command", method="POST", body={"command": message, "payload": {}})
    except Exception:
        # Avoid masking the real violation: still write local report.
        pass


def write_local_report(
    report_dir: str,
    message: str,
    violations: List[str],
    robots: Dict[str, dict],
    tasks: List[dict],
) -> str:
    os.makedirs(report_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(report_dir, f"violation_auto_cancel_{ts}.log")
    payload = {
        "time": datetime.now().isoformat(timespec="seconds"),
        "message": message,
        "violations": violations,
        "robots_snapshot": robots,
        "tasks_snapshot": tasks,
    }
    with open(path, "w", encoding="utf-8") as f:
        f.write(payload["time"] + "\n")
        f.write(message + "\n")
        f.write("violations:\n")
        for v in violations:
            f.write(" - " + v + "\n")
        f.write("\nrobots_snapshot:\n")
        f.write(json.dumps(robots, ensure_ascii=False, indent=2) + "\n")
        f.write("\ntasks_snapshot:\n")
        f.write(json.dumps(tasks, ensure_ascii=False, indent=2) + "\n")
    return path


def main() -> int:
    ap = argparse.ArgumentParser(description="Auto-cancel all tasks on fleet traffic rule violation.")
    add_base_url_arg(ap)
    add_polling_args(ap, default_interval=0.25, allow_duration=True, allow_once=True)
    add_robots_filter_arg(ap)
    ap.add_argument("--dry-run", action="store_true", help="Detect violations but do NOT cancel tasks.")
    args = ap.parse_args()

    base = args.base.rstrip("/")
    robots_filter = parse_robots_filter(args.robots)

    t0 = time.time()
    while True:
        try:
            robots = get_robots(base)
        except urllib.error.URLError as e:
            print(f"{ts_hms()} [monitor] API robots fetch failed: {e}", file=sys.stderr)
            return 3

        violations = check_traffic_rule_violations(robots, robots_filter=robots_filter)
        if violations:
            msg = "FLEET TRAFFIC VIOLATION detected -> cancel all tasks"
            # snapshot tasks for report
            try:
                tasks = get_tasks(base)
            except Exception:
                tasks = []

            report_dir = os.path.join(os.getcwd(), "error_reports")
            report_path = write_local_report(
                report_dir=report_dir,
                message=msg,
                violations=violations,
                robots=robots,
                tasks=tasks,
            )

            detail = "\n".join(violations)
            err_msg = msg + "\n" + detail + "\nreport=" + report_path
            print(err_msg, file=sys.stderr, flush=True)
            post_error_command(base, err_msg)

            if args.dry_run:
                print("[monitor] --dry-run enabled: skip cancelling tasks.", file=sys.stderr)
                return 2

            ok, failed = cancel_all_tasks(base)
            print(f"{ts_hms()} [monitor] cancelled tasks ok={ok} failed={failed}", file=sys.stderr, flush=True)
            return 2

        if args.once:
            return 0

        if args.duration > 0.0 and (time.time() - t0) >= args.duration:
            return 0

        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())


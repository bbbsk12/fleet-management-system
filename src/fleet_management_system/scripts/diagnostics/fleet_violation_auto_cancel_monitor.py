#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
违规自动取消监控工具
===================

功能说明：
  轮询 GET /api/robots 并强制执行交通规则：
    ① 同航点冲突：同一 current_waypoint 上出现多台车。
    ② 同航线冲突：同一无向边 current_segment 上出现多台车。
    ③ 点线互斥：某车位于航点 A 时，其相连航段上的其他车不得同时出现。

检测到违规时的处理流程：
  1) 对所有当前进行中的任务执行 DELETE /api/tasks/{task_id}。
  2) 通过 POST /api/command 发送错误消息（供 API 日志 / WebSocket 使用）。
  3) 在本地写入错误报告文件。
  4) 以退出码 2 终止。

设计用途：
  作为"压力测试安全卫士"使用。
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
    """从 API 获取当前所有机器人状态。"""
    rdata = http_json(f"{base.rstrip('/')}/api/robots")
    return rdata.get("robots") or {}


def get_tasks(base: str) -> List[dict]:
    """从 API 获取当前所有任务列表。"""
    tdata = http_json(f"{base.rstrip('/')}/api/tasks")
    return tdata.get("tasks") or []


def cancel_all_tasks(base: str) -> Tuple[int, List[str]]:
    """取消所有当前存在的任务，返回 (成功数, 失败ID列表)。"""
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
    """向 API 发送错误命令消息（server.py: POST /api/command 期望 {command, payload}）。"""
    try:
        http_json(f"{base.rstrip('/')}/api/command",
                  method="POST",
                  body={"command": message, "payload": {}})
    except Exception:
        # 避免掩盖真实的违规信息：即使发送失败也继续生成本地报告
        pass


def write_local_report(
    report_dir: str,
    message: str,
    violations: List[str],
    robots: Dict[str, dict],
    tasks: List[dict],
) -> str:
    """写入本地违规报告文件，包含违规详情及当前状态快照。"""
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
    """主函数：解析参数，启动轮询循环，检测违规时自动取消任务。"""
    ap = argparse.ArgumentParser(
        description="Auto-cancel all tasks on fleet traffic rule violation."
    )
    add_base_url_arg(ap)
    add_polling_args(ap, default_interval=0.25, allow_duration=True, allow_once=True)
    add_robots_filter_arg(ap)
    ap.add_argument("--dry-run", action="store_true",
                    help="Detect violations but do NOT cancel tasks.")
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
            # 快照当前任务以写入报告
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
                print("[monitor] --dry-run enabled: skip cancelling tasks.",
                      file=sys.stderr)
                return 2

            ok, failed = cancel_all_tasks(base)
            print(f"{ts_hms()} [monitor] cancelled tasks ok={ok} failed={failed}",
                  file=sys.stderr, flush=True)
            return 2

        if args.once:
            return 0

        if args.duration > 0.0 and (time.time() - t0) >= args.duration:
            return 0

        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fleet Management System 诊断监控工具
=============================================
通过 Web API 轮询车队状态、提交任务、检查调度规则冲突。
所有输出均写入 logs/diagnostics/ 目录，不输出到终端。

用法:
  # 仅监控（持续轮询 + 规则检查）
  python3 scripts/diagnostics/diagnostic_monitor.py --mode monitor

  # 提交一系列测试任务并监控
  python3 scripts/diagnostics/diagnostic_monitor.py --mode test

  # 提交单个任务
  python3 scripts/diagnostics/diagnostic_monitor.py --mode submit --waypoint wp_006
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime
from collections import defaultdict
from pathlib import Path

SCRIPTS_ROOT = Path(__file__).resolve().parents[1]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from _lib.http_api import http_json
from _lib.traffic_rules import check_traffic_rule_violations

# ======================== 配置 ========================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
LOG_DIR = os.path.join(WORKSPACE_ROOT, "logs", "diagnostics")
BASE_URL = os.environ.get("FLEET_API_BASE", "http://127.0.0.1:8080")
POLL_INTERVAL = 0.5  # 秒


# ======================== 日志 ========================

class FileLogger:
    def __init__(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        now = datetime.now()
        ts = now.strftime("%Y%m%d_%H%M%S")
        ms = f"{now.microsecond // 1000:03d}"
        pid = os.getpid()
        self.path = os.path.join(LOG_DIR, f"diag_{ts}_{ms}_pid{pid}.log")
        self.summary_path = os.path.join(LOG_DIR, f"diag_{ts}_{ms}_pid{pid}_summary.log")
        self._fh = open(self.path, "w", encoding="utf-8")
        self.violations = []
        self.events = []

    def log(self, level: str, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] [{level:5s}] {msg}"
        self._fh.write(line + "\n")
        self._fh.flush()

    def violation(self, msg: str):
        self.log("VIOLN", msg)
        self.violations.append((datetime.now().isoformat(), msg))

    def event(self, msg: str):
        self.log("EVENT", msg)
        self.events.append((datetime.now().isoformat(), msg))

    def write_summary(self):
        with open(self.summary_path, "w", encoding="utf-8") as f:
            f.write("=" * 60 + "\n")
            f.write("Fleet Diagnostic Summary\n")
            f.write(f"Generated: {datetime.now().isoformat()}\n")
            f.write("=" * 60 + "\n\n")
            f.write(f"Total violations: {len(self.violations)}\n")
            f.write(f"Total events: {len(self.events)}\n\n")
            if self.violations:
                f.write("--- Violations ---\n")
                for ts, v in self.violations:
                    f.write(f"  [{ts}] {v}\n")
                f.write("\n")
            if self.events:
                f.write("--- Events ---\n")
                for ts, e in self.events:
                    f.write(f"  [{ts}] {e}\n")
        self.log("INFO", f"Summary written to {self.summary_path}")

    def close(self):
        self.write_summary()
        self._fh.close()


# ======================== HTTP helpers ========================

def api_get(path: str):
    try:
        return http_json(f"{BASE_URL}{path}", method="GET", timeout=5.0)
    except Exception as e:
        return {"_error": str(e)}


def api_post(path: str, data: dict):
    try:
        return http_json(f"{BASE_URL}{path}", method="POST", body=data, timeout=15.0)
    except Exception as e:
        return {"_error": str(e)}


def api_delete(path: str):
    try:
        return http_json(f"{BASE_URL}{path}", method="DELETE", timeout=10.0)
    except Exception as e:
        return {"_error": str(e)}


# ======================== 状态快照 ========================

class FleetSnapshot:
    """一次轮询的快照"""
    def __init__(self, robots: dict, tasks: dict, waypoints: list):
        self.robots = robots          # {id: {...}}
        self.tasks = tasks            # {id: {...}}
        self.waypoints = waypoints    # [{id, ...}]
        self.ts = time.time()

    @staticmethod
    def fetch(logger: FileLogger):
        r = api_get("/api/robots")
        t = api_get("/api/tasks")
        w = api_get("/api/map/waypoints")
        robots = r.get("robots", {}) if "_error" not in r else {}
        tasks = {tk["id"]: tk for tk in t.get("tasks", [])} if "_error" not in t else {}
        waypoints = w.get("waypoints", []) if "_error" not in w else []
        if "_error" in r:
            logger.log("ERROR", f"GET /api/robots failed: {r['_error']}")
        return FleetSnapshot(robots, tasks, waypoints)


# ======================== 规则检查 ========================

def check_rules(snap: FleetSnapshot, logger: FileLogger):
    """对当前快照执行调度规则检查"""
    robots = snap.robots
    if not robots:
        return
    for violation in check_traffic_rule_violations(robots):
        logger.violation(violation)


def log_fleet_state(snap: FleetSnapshot, logger: FileLogger):
    """记录一行精简的全局状态"""
    parts = []
    for rid in sorted(snap.robots.keys()):
        r = snap.robots[rid]
        pos = r.get("position", {})
        wx = pos.get("world_x", 0)
        wy = pos.get("world_y", 0)
        wp = r.get("current_waypoint", "-")
        seg = r.get("current_segment", "-")
        task = r.get("current_task") or "-"
        nav = r.get("nav_status", r.get("status", "?"))
        loc = r.get("location_type", "?")
        if loc == "waypoint":
            loc_str = f"@{wp}"
        elif loc == "segment":
            loc_str = f"on:{seg}"
        else:
            loc_str = "unknown"
        parts.append(f"{rid}({loc_str} nav={nav} task={task} xy=({wx:.2f},{wy:.2f}))")

    logger.log("STATE", " | ".join(parts) if parts else "(no robots)")

    # 任务统计
    task_counts = defaultdict(int)
    for t in snap.tasks.values():
        task_counts[t.get("status", "?")] += 1
    if task_counts:
        logger.log("TASKS", " ".join(f"{k}={v}" for k, v in sorted(task_counts.items())))


# ======================== 模式：监控 ========================

def run_monitor(logger: FileLogger, duration: float = 120.0):
    logger.log("INFO", f"=== Monitor mode (duration={duration}s, poll={POLL_INTERVAL}s) ===")
    start = time.time()
    prev_snap = None
    tick = 0
    while time.time() - start < duration:
        snap = FleetSnapshot.fetch(logger)

        # 每 2 秒记录完整状态
        if tick % max(1, int(2.0 / POLL_INTERVAL)) == 0:
            log_fleet_state(snap, logger)

        check_rules(snap, logger)

        # 检测任务状态变化
        if prev_snap:
            for tid, t in snap.tasks.items():
                old = prev_snap.tasks.get(tid, {})
                if t.get("status") != old.get("status"):
                    logger.event(f"Task {tid}: {old.get('status', 'new')} -> {t['status']}")
            # 检测机器人位置跳变（调试有用）
            for rid in snap.robots:
                r = snap.robots[rid]
                old_r = prev_snap.robots.get(rid, {})
                old_wp = old_r.get("current_waypoint", "")
                new_wp = r.get("current_waypoint", "")
                if old_wp and new_wp and old_wp != new_wp:
                    logger.event(f"Robot {rid} waypoint changed: {old_wp} -> {new_wp}")

        prev_snap = snap
        tick += 1
        time.sleep(POLL_INTERVAL)

    logger.log("INFO", "Monitor finished")


# ======================== 模式：测试 ========================

def run_test(logger: FileLogger):
    """提交一系列任务并持续监控调度行为"""
    logger.log("INFO", "=== Test mode ===")

    # 先获取初始状态
    snap = FleetSnapshot.fetch(logger)
    log_fleet_state(snap, logger)

    robot_ids = sorted(snap.robots.keys())
    if not robot_ids:
        logger.log("ERROR", "No robots online, aborting test")
        return
    logger.log("INFO", f"Online robots: {robot_ids}")

    waypoint_ids = [w["id"] for w in snap.waypoints]
    logger.log("INFO", f"Waypoints: {waypoint_ids}")

    # ----- 测试 1: 给每个机器人一个不同的目标航点 -----
    logger.log("INFO", "--- Test 1: Send each robot to a different destination ---")
    # 选择不在机器人当前位置的航点
    targets = []
    used_wps = set()
    for rid in robot_ids:
        r = snap.robots[rid]
        cur_wp = r.get("current_waypoint", "")
        for wp in waypoint_ids:
            if wp != cur_wp and wp not in used_wps:
                targets.append((rid, wp))
                used_wps.add(wp)
                break

    for rid, wp in targets:
        result = api_post("/api/tasks", {
            "waypoint_id": wp,
            "priority": 0,
            "robot_id": rid
        })
        if "_error" in result:
            logger.log("ERROR", f"Submit failed: {rid}->{wp}: {result['_error']}")
        else:
            logger.event(f"Submitted: {rid}->{wp} task_id={result.get('task_id', '?')}")

    # 监控执行过程
    logger.log("INFO", "Monitoring task execution for 90 seconds...")
    run_monitor(logger, duration=90.0)

    # ----- 测试 2: 同时提交多个任务到同一航点 -----
    logger.log("INFO", "--- Test 2: Submit tasks to same waypoint (auto assign) ---")
    target_wp = "wp_001"  # 选一个中间节点
    for i in range(2):
        result = api_post("/api/tasks", {
            "waypoint_id": target_wp,
            "priority": i
        })
        if "_error" in result:
            logger.log("ERROR", f"Submit failed: auto->{target_wp}: {result['_error']}")
        else:
            logger.event(f"Submitted auto task to {target_wp}: task_id={result.get('task_id', '?')}")

    run_monitor(logger, duration=90.0)

    # ----- 测试 3: 对向通行测试（潜在冲突） -----
    logger.log("INFO", "--- Test 3: Opposite direction tasks (conflict potential) ---")
    snap2 = FleetSnapshot.fetch(logger)
    if len(robot_ids) >= 2:
        r0_wp = snap2.robots.get(robot_ids[0], {}).get("current_waypoint", "")
        r1_wp = snap2.robots.get(robot_ids[1], {}).get("current_waypoint", "")
        # 让他们去对方的位置
        if r0_wp and r1_wp and r0_wp != r1_wp:
            result0 = api_post("/api/tasks", {
                "waypoint_id": r1_wp, "priority": 0, "robot_id": robot_ids[0]
            })
            result1 = api_post("/api/tasks", {
                "waypoint_id": r0_wp, "priority": 0, "robot_id": robot_ids[1]
            })
            logger.event(f"Opposite: {robot_ids[0]}->{r1_wp}, {robot_ids[1]}->{r0_wp}")
        else:
            # 用固定航点
            result0 = api_post("/api/tasks", {
                "waypoint_id": "wp_006", "priority": 0, "robot_id": robot_ids[0]
            })
            result1 = api_post("/api/tasks", {
                "waypoint_id": "wp_010", "priority": 0, "robot_id": robot_ids[1]
            })
            logger.event(f"Opposite fallback: {robot_ids[0]}->wp_006, {robot_ids[1]}->wp_010")

    run_monitor(logger, duration=120.0)

    logger.log("INFO", "=== Test complete ===")


# ======================== 模式：单任务提交 ========================

def run_submit(logger: FileLogger, waypoint: str, robot_id: str = ""):
    payload = {"waypoint_id": waypoint, "priority": 0}
    if robot_id:
        payload["robot_id"] = robot_id
    result = api_post("/api/tasks", payload)
    logger.log("INFO", f"Submit result: {json.dumps(result, ensure_ascii=False)}")
    # 监控 60 秒
    run_monitor(logger, duration=60.0)


# ======================== 入口 ========================

def main():
    global BASE_URL
    parser = argparse.ArgumentParser(description="Fleet Diagnostic Monitor")
    parser.add_argument("--mode", choices=["monitor", "test", "submit"],
                        default="monitor", help="运行模式")
    parser.add_argument("--duration", type=float, default=120.0,
                        help="监控持续时间（秒）")
    parser.add_argument("--waypoint", type=str, default="wp_006",
                        help="submit 模式目标航点")
    parser.add_argument("--robot", type=str, default="",
                        help="submit 模式指定机器人")
    parser.add_argument("--base", type=str, default=BASE_URL,
                        help="Web backend URL (or env FLEET_API_BASE)")
    args = parser.parse_args()

    BASE_URL = args.base.rstrip("/")

    logger = FileLogger()
    logger.log("INFO", f"Diagnostic started: mode={args.mode} url={BASE_URL}")
    logger.log("INFO", f"Log file: {logger.path}")

    try:
        if args.mode == "monitor":
            run_monitor(logger, duration=args.duration)
        elif args.mode == "test":
            run_test(logger)
        elif args.mode == "submit":
            run_submit(logger, args.waypoint, args.robot)
    except KeyboardInterrupt:
        logger.log("INFO", "Interrupted by user")
    except Exception as e:
        logger.log("ERROR", f"Unhandled: {e}")
    finally:
        logger.close()
        # Print summary path to stderr for the user
        print(f"Logs: {logger.path}", file=sys.stderr)
        print(f"Summary: {logger.summary_path}", file=sys.stderr)
        if logger.violations:
            print(f"VIOLATIONS FOUND: {len(logger.violations)}", file=sys.stderr)


if __name__ == "__main__":
    main()

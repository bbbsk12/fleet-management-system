#!/usr/bin/env python3
"""
调度规则违规监控脚本 (Scheduling Rule Violation Watchdog) v2

持续监控 fleet_manager 的机器人状态，检测以下违规：
  【毁灭性】规则①  同一航点上出现两台机器人                  → 零容忍，立即触发
  【毁灭性】规则②  同一航道（无论方向）上出现两台机器人      → 零容忍，立即触发
  【毁灭性】规则③  机器人在航点，另一台在该航点关联的航段上  → 零容忍，立即触发
  【严重】  规则④  两台机器人物理距离过近（安全距离违规）    → 防抖后触发
  【毁灭性】规则⑤  存在挂起任务且 15 秒无任何底盘活动        → 立即触发
  【毁灭性】规则⑥  两台机器人在共享同一航点的不同航段上      → 零容忍，立即触发

航点/航道冲突、调度挂起被定义为「毁灭性故障」：第一次检测到即触发，无需防抖。
"""

import argparse
import datetime
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import urllib.error

SCRIPTS_ROOT = Path(__file__).resolve().parents[1]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from _lib.http_api import http_json
from _lib.traffic_rules import parse_segment

# ─── 地图拓扑（硬编码，与 rmf_map0.yaml 一致） ─────────────────────────
EDGES = [
    ("wp_001", "wp_006"), ("wp_001", "wp_007"),
    ("wp_002", "wp_005"), ("wp_002", "wp_003"), ("wp_002", "wp_007"),
    ("wp_003", "wp_004"), ("wp_003", "wp_010"), ("wp_003", "wp_018"),
    ("wp_007", "wp_016"),
]

# 邻接表（双向）
ADJACENCY: dict[str, set[str]] = {}
for a, b in EDGES:
    ADJACENCY.setdefault(a, set()).add(b)
    ADJACENCY.setdefault(b, set()).add(a)

# 所有合法航段（双向）
VALID_SEGMENTS: set[tuple[str, str]] = set()
for a, b in EDGES:
    VALID_SEGMENTS.add((a, b))
    VALID_SEGMENTS.add((b, a))

# ─── 参数 ─────────────────────────────────────────────────────────────
BASE_URL = os.environ.get("FLEET_API_BASE", "http://127.0.0.1:8080")
SAFE_DISTANCE_M = 0.6           # 物理安全距离（米）
POLL_INTERVAL_S = 0.3           # 轮询间隔（实时监测要更短）
VIOLATION_GRACE_TICKS = 2       # 普通违规连续 N 次检测到才触发（防抖）
STALL_TIMEOUT_S = 15.0          # 挂起任务下，全局无活动超时
ACTIVITY_POS_EPS_M = 0.03       # 认为发生了位置活动的最小位移
ACTIVITY_YAW_EPS_RAD = 0.10     # 认为发生了原地转动的最小角度
ACTIVITY_ROUTE_PREVIEW_LEN = 4  # 活动判定只看前几个规划点，避免日志/比较过大
STALL_WARN_IDLE_S = 2.0         # 挂起监视开始提示前，至少连续静止这么久

TERMINAL_TASK_STATUSES = {"completed", "cancelled", "failed"}

# 毁灭性故障（航点/航道冲突）的严重级别
SEVERITY_CATASTROPHIC = "CATASTROPHIC"  # 零容忍，第1次就触发
SEVERITY_SEVERE = "SEVERE"              # 防抖后触发

# ─── 全局状态 ──────────────────────────────────────────────────────────
running = True
violation_counts: dict[str, int] = {}   # violation_key → consecutive count
log_file = None
cancelled = False


def sig_handler(_sig, _frame):
    global running
    running = False

signal.signal(signal.SIGINT, sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


# ─── 工具函数 ──────────────────────────────────────────────────────────
def ts() -> str:
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

def log(msg: str, level: str = "INFO"):
    line = f"[{ts()}] [{level}] {msg}"
    print(line, flush=True)
    if log_file:
        log_file.write(line + "\n")
        log_file.flush()

def api_get(path: str):
    try:
        return http_json(f"{BASE_URL}{path}", method="GET", timeout=3.0)
    except Exception as e:
        log(f"API GET {path} 失败: {e}", "WARN")
        return None

def api_delete(path: str):
    try:
        return http_json(f"{BASE_URL}{path}", method="DELETE", timeout=3.0)
    except Exception as e:
        log(f"API DELETE {path} 失败: {e}", "WARN")
        return None

def api_post(path: str, body: dict | None = None):
    try:
        return http_json(f"{BASE_URL}{path}", method="POST", body=body, timeout=3.0)
    except Exception as e:
        log(f"API POST {path} 失败: {e}", "WARN")
        return None


def is_non_terminal_task(task: dict) -> bool:
    return task.get("status") not in TERMINAL_TASK_STATUSES


def cancel_all_tasks():
    """取消所有进行中的任务"""
    global cancelled
    if cancelled:
        return
    cancelled = True

    # 方法1: 通过 emergency_stop
    log(">>> 执行紧急停止 + 取消全部任务 <<<", "ACTION")
    api_post("/api/emergency_stop")

    # 方法2: 逐个取消所有非终态任务
    tasks_resp = api_get("/api/tasks")
    if tasks_resp and "tasks" in tasks_resp:
        for t in tasks_resp["tasks"]:
            if is_non_terminal_task(t):
                tid = t["id"]
                api_delete(f"/api/tasks/{tid}")
                log(f"  已取消任务 {tid} status={t.get('status','?')}", "ACTION")

    # 方法3: 逐个 recall 机器人
    robots_resp = api_get("/api/robots")
    if robots_resp and "robots" in robots_resp:
        for rid in robots_resp["robots"]:
            api_post(f"/api/robots/{rid}/recall")
            log(f"  已召回 {rid}", "ACTION")

    # 方法4: 核验是否仍有非终态任务残留
    verify_resp = api_get("/api/tasks")
    if verify_resp and "tasks" in verify_resp:
        remaining = [
            f"{t['id']}({t.get('status','?')})"
            for t in verify_resp["tasks"]
            if is_non_terminal_task(t)
        ]
        if remaining:
            log(f"  警告: 仍有未终止任务残留: {', '.join(remaining)}", "WARN")
        else:
            log("  核验完成: 所有任务已进入终态", "ACTION")


def euclidean(pos_a: dict, pos_b: dict) -> float:
    dx = pos_a.get("world_x", 0) - pos_b.get("world_x", 0)
    dy = pos_a.get("world_y", 0) - pos_b.get("world_y", 0)
    return math.sqrt(dx * dx + dy * dy)


def normalize_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def get_suspended_tasks(tasks_resp: dict | None) -> list[dict]:
    if not tasks_resp or "tasks" not in tasks_resp:
        return []
    suspended = []
    for task in tasks_resp["tasks"]:
        status = task.get("status")
        if not is_non_terminal_task(task):
            continue
        if status == "pending":
            continue
        suspended.append(task)
    return suspended


def robot_activity_snapshot(robot: dict) -> dict:
    pos = robot.get("position", {})
    return {
        "status": robot.get("status", ""),
        "nav_status": robot.get("nav_status", ""),
        "current_task": robot.get("current_task") or "",
        "location_type": robot.get("location_type", "unknown"),
        "current_waypoint": robot.get("current_waypoint", ""),
        "current_segment": robot.get("current_segment", ""),
        "planned_route": tuple((robot.get("planned_route") or [])[:ACTIVITY_ROUTE_PREVIEW_LEN]),
        "world_x": float(pos.get("world_x", 0.0) or 0.0),
        "world_y": float(pos.get("world_y", 0.0) or 0.0),
        "yaw": float(pos.get("yaw", 0.0) or 0.0),
    }


def detect_robot_activity(
    robots: dict,
    activity_anchor_snapshots: dict,
    pos_eps: float = ACTIVITY_POS_EPS_M,
    yaw_eps: float = ACTIVITY_YAW_EPS_RAD,
) -> tuple[dict, list[str]]:
    next_activity_anchors = {}
    reasons: list[str] = []

    for rid, robot in sorted(robots.items()):
        if not robot.get("online"):
            continue
        snap = robot_activity_snapshot(robot)
        anchor = activity_anchor_snapshots.get(rid)
        if not anchor:
            next_activity_anchors[rid] = snap
            continue

        next_activity_anchors[rid] = anchor

        dx = snap["world_x"] - anchor["world_x"]
        dy = snap["world_y"] - anchor["world_y"]
        dist = math.sqrt(dx * dx + dy * dy)
        yaw_delta = abs(normalize_angle_rad(snap["yaw"] - anchor["yaw"]))

        if dist >= pos_eps:
            reasons.append(f"{rid} moved {dist:.2f}m")
            next_activity_anchors[rid] = snap
            continue
        if yaw_delta >= yaw_eps:
            reasons.append(f"{rid} rotated {yaw_delta:.2f}rad")
            next_activity_anchors[rid] = snap
            continue

        if (
            snap["location_type"] != anchor["location_type"] or
            snap["current_waypoint"] != anchor["current_waypoint"] or
            snap["current_segment"] != anchor["current_segment"]
        ):
            before = anchor["current_waypoint"] or anchor["current_segment"] or anchor["location_type"]
            after = snap["current_waypoint"] or snap["current_segment"] or snap["location_type"]
            reasons.append(f"{rid} loc {before}->{after}")
            next_activity_anchors[rid] = snap
            continue

        if snap["nav_status"] != anchor["nav_status"]:
            reasons.append(f"{rid} nav {anchor['nav_status']}->{snap['nav_status']}")
            next_activity_anchors[rid] = snap
            continue

        if snap["status"] != anchor["status"]:
            reasons.append(f"{rid} status {anchor['status']}->{snap['status']}")
            next_activity_anchors[rid] = snap
            continue

        if snap["current_task"] != anchor["current_task"]:
            before_task = anchor["current_task"] or "-"
            after_task = snap["current_task"] or "-"
            reasons.append(f"{rid} task {before_task}->{after_task}")
            next_activity_anchors[rid] = snap
            continue

        if snap["planned_route"] != anchor["planned_route"]:
            reasons.append(f"{rid} route changed")
            next_activity_anchors[rid] = snap

    return next_activity_anchors, reasons


# ─── 违规检测 ──────────────────────────────────────────────────────────
def check_violations(robots: dict, safe_distance: float = SAFE_DISTANCE_M) -> list[dict]:
    """
    返回违规列表，每项:
    {
      "rule": str,
      "severity": "CATASTROPHIC" | "SEVERE",
      "key": 去重键,
      "robots": [id1, id2],
      "detail": 描述
    }
    航点/航道冲突 = CATASTROPHIC（零容忍）
    物理距离过近 = SEVERE（需防抖）
    """
    violations = []
    ids = [rid for rid, r in robots.items() if r.get("online")]
    if len(ids) < 2:
        return violations

    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            ra, rb = robots[ids[i]], robots[ids[j]]
            ra_id, rb_id = ids[i], ids[j]
            ra_loc = ra.get("location_type", "unknown")
            rb_loc = rb.get("location_type", "unknown")
            ra_wp = ra.get("current_waypoint", "")
            rb_wp = rb.get("current_waypoint", "")
            ra_seg_str = ra.get("current_segment", "")
            rb_seg_str = rb.get("current_segment", "")
            ra_u, ra_v = parse_segment(ra_seg_str)
            rb_u, rb_v = parse_segment(rb_seg_str)
            ra_seg = (ra_u, ra_v) if ra_u and ra_v else None
            rb_seg = (rb_u, rb_v) if rb_u and rb_v else None

            # ── 【毁灭性】规则① 同航点 ──
            if ra_loc == "waypoint" and rb_loc == "waypoint" and ra_wp and ra_wp == rb_wp:
                violations.append({
                    "rule": "R1_SAME_WP",
                    "severity": SEVERITY_CATASTROPHIC,
                    "key": f"R1_WP_{ra_wp}",
                    "robots": [ra_id, rb_id],
                    "detail": f"【毁灭性】{ra_id} 和 {rb_id} 同时在航点 {ra_wp}"
                })

            # ── 【毁灭性】规则② 同航道（无论方向） ──
            if ra_loc == "segment" and rb_loc == "segment" and ra_seg and rb_seg:
                # 归一化为同一条边：两方向都算同一航道
                edge_a = tuple(sorted([ra_seg[0], ra_seg[1]]))
                edge_b = tuple(sorted([rb_seg[0], rb_seg[1]]))
                if edge_a == edge_b:
                    violations.append({
                        "rule": "R2_SAME_EDGE",
                        "severity": SEVERITY_CATASTROPHIC,
                        "key": f"R2_EDGE_{edge_a[0]}_{edge_a[1]}",
                        "robots": [ra_id, rb_id],
                        "detail": f"【毁灭性】{ra_id}({ra_seg_str}) 和 {rb_id}({rb_seg_str}) 在同一航道 {edge_a[0]}↔{edge_a[1]}"
                    })

            # ── 【毁灭性】规则③ 航点+关联航段冲突 ──
            # A 在航点 X，B 在经过 X 的航段上（X→Y 或 Y→X）
            if ra_loc == "waypoint" and rb_loc == "segment" and ra_wp and rb_seg:
                if ra_wp in rb_seg:
                    violations.append({
                        "rule": "R3_WP_ADJ_SEG",
                        "severity": SEVERITY_CATASTROPHIC,
                        "key": f"R3_{ra_wp}_{rb_seg_str}",
                        "robots": [ra_id, rb_id],
                        "detail": f"【毁灭性】{ra_id} 在航点 {ra_wp}，{rb_id} 在关联航段 {rb_seg_str}"
                    })
            if rb_loc == "waypoint" and ra_loc == "segment" and rb_wp and ra_seg:
                if rb_wp in ra_seg:
                    violations.append({
                        "rule": "R3_WP_ADJ_SEG",
                        "severity": SEVERITY_CATASTROPHIC,
                        "key": f"R3_{rb_wp}_{ra_seg_str}",
                        "robots": [rb_id, ra_id],
                        "detail": f"【毁灭性】{rb_id} 在航点 {rb_wp}，{ra_id} 在关联航段 {ra_seg_str}"
                    })

            # ── 【严重】规则④ 物理距离过近 ──
            pos_a = ra.get("position", {})
            pos_b = rb.get("position", {})
            if pos_a.get("world_x") is not None and pos_b.get("world_x") is not None:
                dist = euclidean(pos_a, pos_b)
                # 只在双方都在移动或有active goal时检查距离
                both_have_task = (ra.get("current_task") or rb.get("current_task"))
                if dist < safe_distance and both_have_task:
                    violations.append({
                        "rule": "R4_DISTANCE",
                        "severity": SEVERITY_SEVERE,
                        "key": "R4_DIST",
                        "robots": [ra_id, rb_id],
                        "detail": f"【严重】{ra_id} 和 {rb_id} 距离仅 {dist:.2f}m（阈值 {safe_distance}m）"
                    })

            # ── 【毁灭性】规则⑥ 相邻航段冲突（共享航点的不同航段上出现多台机器人） ──
            if ra_loc == "segment" and rb_loc == "segment" and ra_seg and rb_seg:
                edge_a = tuple(sorted([ra_seg[0], ra_seg[1]]))
                edge_b = tuple(sorted([rb_seg[0], rb_seg[1]]))
                if edge_a != edge_b:  # 不同航段（同航段已由 R2 处理）
                    # 检查是否共享一个端点
                    shared = set(edge_a) & set(edge_b)
                    if shared:
                        shared_wp = next(iter(shared))
                        violations.append({
                            "rule": "R6_ADJ_SEGMENTS",
                            "severity": SEVERITY_CATASTROPHIC,
                            "key": f"R6_{shared_wp}_{edge_a[0]}_{edge_a[1]}_{edge_b[0]}_{edge_b[1]}",
                            "robots": [ra_id, rb_id],
                            "detail": (
                                f"【毁灭性】{ra_id}({ra_seg_str}) 和 {rb_id}({rb_seg_str}) "
                                f"在共享航点 {shared_wp} 的不同航段上"
                            )
                        })

    return violations


# ─── 状态快照打印 ─────────────────────────────────────────────────────
def snapshot(robots: dict, tasks_resp: dict | None):
    lines = []
    for rid, r in sorted(robots.items()):
        if not r.get("online"):
            continue
        loc = r.get("location_type", "?")
        wp = r.get("current_waypoint", "")
        seg = r.get("current_segment", "")
        nav = r.get("nav_status", "?")
        task = r.get("current_task", "-")
        route = r.get("planned_route", [])
        pos = r.get("position", {})
        wx = pos.get("world_x", 0)
        wy = pos.get("world_y", 0)
        loc_str = wp if loc == "waypoint" else (seg if loc == "segment" else "?")
        route_str = "→".join(route[:5]) if route else "-"
        lines.append(f"  {rid}: loc={loc_str} nav={nav} task={task or '-'} "
                      f"route=[{route_str}] pos=({wx:.2f},{wy:.2f})")

    # 活跃任务
    active_tasks = []
    if tasks_resp and "tasks" in tasks_resp:
        for t in tasks_resp["tasks"]:
            if is_non_terminal_task(t):
                active_tasks.append(f"{t['id'][-8:]}→{t.get('waypoint_id','?')}({t.get('robot_id','?')},{t.get('status')})")

    log("─── 快照 ───")
    for l in lines:
        log(l)
    if active_tasks:
        log(f"  活跃任务: {', '.join(active_tasks)}")
    else:
        log("  活跃任务: 无")


# ─── 主循环 ────────────────────────────────────────────────────────────
def main():
    global log_file, cancelled, BASE_URL

    parser = argparse.ArgumentParser(description="调度规则违规监控")
    parser.add_argument("--duration", type=int, default=300, help="监控持续秒数（默认300）")
    parser.add_argument("--interval", type=float, default=0.3, help="轮询间隔秒（默认0.3，实时监测）")
    parser.add_argument("--grace", type=int, default=2, help="连续违规检测次数才触发取消")
    parser.add_argument("--snapshot-interval", type=int, default=10, help="快照打印间隔（秒）")
    parser.add_argument("--log-dir", default="test_logs", help="日志目录")
    parser.add_argument("--no-cancel", action="store_true", help="仅报警不取消任务")
    parser.add_argument("--safe-distance", type=float, default=0.6, help="安全距离（米）")
    parser.add_argument("--stall-timeout", type=float, default=15.0, help="存在挂起任务时，全局无活动超时秒数")
    parser.add_argument("--base", type=str, default=BASE_URL, help="Web backend URL (or env FLEET_API_BASE)")
    args = parser.parse_args()
    BASE_URL = args.base.rstrip("/")

    safe_distance = args.safe_distance
    poll_interval = args.interval
    grace_ticks = args.grace
    stall_timeout = args.stall_timeout

    # 创建日志
    os.makedirs(args.log_dir, exist_ok=True)
    now_dt = datetime.datetime.now()
    log_name = (
        f"watchdog_{now_dt.strftime('%Y%m%d_%H%M%S')}_"
        f"{now_dt.microsecond // 1000:03d}_pid{os.getpid()}.log"
    )
    log_path = os.path.join(args.log_dir, log_name)
    log_file = open(log_path, "w")

    log("=" * 60)
    log(f"调度规则违规监控 v2 启动  duration={args.duration}s  interval={args.interval}s  grace={args.grace}")
    log(f"安全距离={safe_distance}m  取消模式={'禁用' if args.no_cancel else '启用'}")
    log(f"航点/航道冲突、相邻航段冲突、调度挂起 = 毁灭性故障（零容忍，第1次即触发）")
    log(f"挂起阈值={stall_timeout:.1f}s（存在 assigned/in_progress/waiting_fleet 等任务且全局无活动）")
    log(f"日志: {log_path}")
    log("=" * 60)

    start_time = time.time()
    last_snapshot = 0
    last_realtime_log = 0
    activity_anchor_snapshots: dict[str, dict] = {}
    last_activity_at: float | None = None
    stall_watch_warned = False
    tick = 0
    total_violations_triggered = 0
    catastrophic_count = 0

    while running and (time.time() - start_time) < args.duration:
        tick += 1

        # 获取状态
        robots_resp = api_get("/api/robots")
        if not robots_resp or "robots" not in robots_resp:
            time.sleep(poll_interval)
            continue

        robots = robots_resp["robots"]
        tasks_resp = api_get("/api/tasks")
        suspended_tasks = get_suspended_tasks(tasks_resp)
        activity_anchor_snapshots, activity_reasons = detect_robot_activity(
            robots,
            activity_anchor_snapshots,
        )

        # 实时位置输出（每2秒一次，确保能追踪位置变化）
        now = time.time()
        if now - last_realtime_log >= 2.0:
            parts = []
            for rid, r in sorted(robots.items()):
                if not r.get("online"):
                    continue
                loc = r.get("location_type", "?")
                wp = r.get("current_waypoint", "")
                seg = r.get("current_segment", "")
                nav = r.get("nav_status", "?")
                loc_str = wp if loc == "waypoint" else (seg if loc == "segment" else "?")
                parts.append(f"{rid}={loc_str}({nav})")
            log(f"[实时] {' | '.join(parts)}")
            last_realtime_log = now

        # 定期快照（详细信息）
        if now - last_snapshot >= args.snapshot_interval:
            snapshot(robots, tasks_resp)
            last_snapshot = now

        # 违规检测
        violations = check_violations(robots, safe_distance)

        # 调度挂起检测：存在挂起任务，但所有机器人持续无活动
        if suspended_tasks:
            if last_activity_at is None:
                last_activity_at = now

            if activity_reasons:
                if stall_watch_warned:
                    log(f"挂起监视复位: 检测到机器人活动 {'; '.join(activity_reasons[:3])}", "INFO")
                last_activity_at = now
                stall_watch_warned = False
            else:
                idle_for = now - last_activity_at
                if idle_for >= STALL_WARN_IDLE_S and not stall_watch_warned:
                    preview = ", ".join(
                        f"{t['id'][-8:]}({t.get('status','?')},{t.get('robot_id') or '-'}->{t.get('waypoint_id') or '-'})"
                        for t in suspended_tasks[:6]
                    )
                    log(
                        f"挂起监视观察中: 存在挂起任务且已连续 {idle_for:.1f}s 无机器人活动 tasks={preview}",
                        "WARN",
                    )
                    stall_watch_warned = True

                if idle_for >= stall_timeout:
                    involved_robots = sorted({
                        t.get("robot_id") for t in suspended_tasks if t.get("robot_id")
                    })
                    if not involved_robots:
                        involved_robots = sorted([rid for rid, r in robots.items() if r.get("online")])
                    violations.append({
                        "rule": "R5_SCHEDULER_STALL",
                        "severity": SEVERITY_CATASTROPHIC,
                        "key": "R5_SCHEDULER_STALL",
                        "robots": involved_robots,
                        "detail": (
                            f"【毁灭性】调度挂起：存在挂起任务且连续 {stall_timeout:.1f}s 无任何底盘活动；"
                            f"tasks={', '.join(f'{t['id'][-8:]}:{t.get('status','?')}:{t.get('robot_id') or '-'}->{t.get('waypoint_id') or '-'}' for t in suspended_tasks[:6])}"
                        )
                    })
        else:
            last_activity_at = None
            stall_watch_warned = False

        # 去重 + 按严重级别处理
        seen_keys = set()
        for v in violations:
            k = v["key"]
            seen_keys.add(k)
            violation_counts[k] = violation_counts.get(k, 0) + 1
            severity = v.get("severity", SEVERITY_SEVERE)

            # 毁灭性故障：第1次检测到就触发（零容忍）
            if severity == SEVERITY_CATASTROPHIC:
                effective_grace = 1
            else:
                effective_grace = grace_ticks

            if violation_counts[k] == effective_grace:
                # 触发！
                total_violations_triggered += 1
                if severity == SEVERITY_CATASTROPHIC:
                    catastrophic_count += 1
                    log("", "")
                    log("╔══════════════════════════════════════════════════════════╗", "CATASTROPHIC")
                    log(f"║  毁灭性故障 #{catastrophic_count}: {v['detail']}", "CATASTROPHIC")
                    log("╚══════════════════════════════════════════════════════════╝", "CATASTROPHIC")
                else:
                    log(f"!!! 违规触发 #{total_violations_triggered}: [{v['rule']}] {v['detail']}", "VIOLATION")

                log(f"    涉及机器人: {v['robots']}", "VIOLATION")

                # 打印当前详细状态
                tasks_resp = api_get("/api/tasks")
                snapshot(robots, tasks_resp)

                # 记录每个机器人的详细位置
                for rid in v["robots"]:
                    r = robots.get(rid, {})
                    pos = r.get("position", {})
                    log(f"    {rid}: world=({pos.get('world_x',0):.3f}, {pos.get('world_y',0):.3f}) "
                        f"yaw={pos.get('yaw',0):.2f} "
                        f"loc_type={r.get('location_type','?')} "
                        f"wp={r.get('current_waypoint','')} "
                        f"seg={r.get('current_segment','')} "
                        f"nav={r.get('nav_status','?')} "
                        f"route={r.get('planned_route',[])}", "VIOLATION")

                if not args.no_cancel:
                    cancel_all_tasks()
                    log("已取消全部任务，等待 5 秒后继续监控...", "ACTION")
                    time.sleep(5)
                    cancelled = False  # 重置，允许后续再次取消
                    violation_counts.clear()
                    break

            elif violation_counts[k] > effective_grace:
                # 毁灭性故障持续中 - 每次都记录
                if severity == SEVERITY_CATASTROPHIC:
                    log(f"  !!! 毁灭性故障持续: {v['detail']} (第{violation_counts[k]}次)", "CATASTROPHIC")
                elif violation_counts[k] % 10 == 0:
                    log(f"  持续违规 [{v['rule']}] {v['detail']} (第{violation_counts[k]}次)", "WARN")

        # 清除不再活跃的违规计数
        stale = [k for k in violation_counts if k not in seen_keys]
        for k in stale:
            if violation_counts[k] >= grace_ticks:
                log(f"  违规消除: {k} (持续了 {violation_counts[k]} 次)", "RESOLVED")
            del violation_counts[k]

        time.sleep(poll_interval)

    # 结束汇总
    elapsed = time.time() - start_time
    log("=" * 60)
    log(f"监控结束  运行 {elapsed:.0f}s  总轮次 {tick}")
    log(f"  触发违规总计: {total_violations_triggered} 次")
    log(f"  其中毁灭性故障: {catastrophic_count} 次")
    if catastrophic_count > 0:
        log("  *** 存在毁灭性故障（航点/航道冲突/调度挂起）！***", "CATASTROPHIC")
    else:
        log("  ✓ 无毁灭性故障")
    log("=" * 60)

    if log_file:
        log_file.close()

    return 0 if total_violations_triggered == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

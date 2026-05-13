#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
车队调度器压力测试（4 机器人）
===============================

功能说明：
  本脚本通过 Web API 驱动，旨在施加以下压力场景：
    - 合并队列到同一航点（多机器人汇聚）
    - 对向行驶/正面冲突（航道争用）
    - 追逐另一机器人当前所在航点（动态阻塞）
    - 通过重复短时爆发突破死胡同/死锁

硬性失败条件：
  - 交通规则违反（同航点、同航道、航点-航段重叠）
  - 任务提交/轮询 API 失败（除非指定 --allow-api-errors）
  - 全局挂起（存在活跃任务但无任何机器人活动）超过阈值

使用示例：
  export FLEET_API_BASE=http://127.0.0.1:8080
  python3 scripts/stress/fleet_scheduler_stress_4bots.py --duration 240 --once

注意事项：
  - 默认需要恰好 4 个在线机器人。
  - 默认使用所有在线机器人；可通过 --robot-ids 覆盖。
"""

from __future__ import annotations

import argparse
import json
import os
import random
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

SCRIPTS_ROOT = Path(__file__).resolve().parents[1]
if str(SCRIPTS_ROOT) not in os.sys.path:
    os.sys.path.insert(0, str(SCRIPTS_ROOT))

from _lib.http_api import http_json_strict  # noqa: E402
from _lib.traffic_rules import check_traffic_rule_violations  # noqa: E402


TERMINAL = {"completed", "cancelled", "failed"}


@dataclass
class Event:
    """事件记录。"""
    ts: str
    kind: str
    detail: str


@dataclass
class Report:
    """测试报告数据结构。"""
    started_at: str
    base_url: str
    seed: int
    robot_ids: List[str]
    duration_sec: float
    submit_ok: int
    submit_total: int
    poll_errors: List[str]
    submit_errors: List[str]
    warnings: List[str]
    violations: List[str]
    events: List[Event]


def _now_iso() -> str:
    """返回 ISO 格式的当前时间字符串。"""
    return datetime.now().isoformat(timespec="seconds")


def _get(base: str, path: str, timeout: float = 5.0) -> dict:
    """发送 GET 请求，严格模式（失败抛出异常）。"""
    return http_json_strict("GET", f"{base}{path}", timeout=timeout)


def _post(base: str, path: str, payload: dict, timeout: float = 8.0) -> dict:
    """发送 POST 请求，严格模式。"""
    return http_json_strict("POST", f"{base}{path}", payload, timeout=timeout)


def _delete(base: str, path: str, timeout: float = 8.0) -> dict:
    """发送 DELETE 请求，严格模式。"""
    return http_json_strict("DELETE", f"{base}{path}", timeout=timeout)


def get_status(base: str) -> dict:
    """获取系统状态。"""
    return _get(base, "/api/status", timeout=5.0)


def get_robots(base: str) -> Dict[str, dict]:
    """获取所有机器人状态，返回机器人 ID 到状态的字典。"""
    data = _get(base, "/api/robots", timeout=5.0)
    robots_obj = data.get("robots") or {}
    if isinstance(robots_obj, dict):
        out: Dict[str, dict] = {}
        for rid, r in robots_obj.items():
            if isinstance(r, dict):
                r = dict(r)
                r.setdefault("id", rid)
                out[rid] = r
        return out
    out = {}
    if isinstance(robots_obj, list):
        for r in robots_obj:
            if isinstance(r, dict) and r.get("id"):
                out[str(r["id"])] = r
    return out


def get_tasks(base: str) -> Dict[str, dict]:
    """获取所有任务，返回任务 ID 到任务信息的字典。"""
    data = _get(base, "/api/tasks", timeout=8.0)
    out: Dict[str, dict] = {}
    for t in data.get("tasks", []) or []:
        if isinstance(t, dict) and t.get("id"):
            out[str(t["id"])] = t
    return out


def get_metrics(base: str) -> dict:
    data = _get(base, "/api/metrics", timeout=5.0)
    metrics = data.get("metrics") or {}
    return metrics if isinstance(metrics, dict) else {}


def get_waypoint_ids(base: str) -> List[str]:
    """获取所有可用航点 ID。"""
    data = _get(base, "/api/map/waypoints", timeout=8.0)
    ids: List[str] = []
    for wp in data.get("waypoints", []) or []:
        if not isinstance(wp, dict):
            continue
        wid = wp.get("waypoint_id") or wp.get("id")
        if wid:
            ids.append(str(wid))
    return sorted(set(ids))


def submit_task(
    base: str,
    waypoint_id: str,
    robot_id: Optional[str],
    priority: int = 0,
) -> str:
    """提交一个任务，返回任务 ID。"""
    body = {"waypoint_id": waypoint_id, "priority": priority}
    if robot_id:
        body["robot_id"] = robot_id
    resp = _post(base, "/api/tasks", body, timeout=10.0)
    tid = resp.get("task_id") or resp.get("id")
    if not tid:
        raise RuntimeError(f"submit missing task_id: {resp}")
    return str(tid)


def recall_robot(base: str, robot_id: str) -> None:
    """召回指定机器人。"""
    _post(base, f"/api/robots/{robot_id}/recall", {}, timeout=8.0)


def clear_live_tasks(base: str) -> None:
    """清除所有非终态任务。"""
    tasks = get_tasks(base)
    for tid, t in tasks.items():
        if t.get("status") in TERMINAL:
            continue
        _delete(base, f"/api/tasks/{tid}", timeout=8.0)


def online_ids(robots: Dict[str, dict]) -> List[str]:
    """从机器人字典中提取所有在线机器人的 ID。"""
    out = []
    for rid, r in robots.items():
        online = r.get("online")
        # server_ros2 可能不包含 online 字段，缺失时视为 True
        if online is False:
            continue
        if r.get("connection_status") == "offline":
            continue
        out.append(rid)
    return sorted(set(out))


def robot_activity_signature(r: dict) -> str:
    """生成机器人活动状态签名，用于检测活动变化。"""
    wp = r.get("current_waypoint") or ""
    seg = r.get("current_segment") or ""
    loc = seg or wp or "-"
    goal = "1" if r.get("goal", False) else "0"
    hold = "1" if r.get("hold", False) else "0"
    task = r.get("task") or r.get("current_task_id") or r.get("current_task") or ""
    nav = r.get("nav_status") or r.get("status") or ""
    pos = r.get("position") if isinstance(r.get("position"), dict) else {}
    x = pos.get("world_x", pos.get("x")) if pos else None
    y = pos.get("world_y", pos.get("y")) if pos else None
    try:
        pose = f"{float(x):.2f},{float(y):.2f}"
    except (TypeError, ValueError):
        pose = "-"
    return f"{loc}|pose={pose}|goal={goal}|hold={hold}|task={task}|nav={nav}"


def task_activity_signature(tasks: Dict[str, dict]) -> str:
    parts = []
    for tid, t in sorted(tasks.items()):
        if t.get("status") in TERMINAL:
            continue
        rid = t.get("assigned_robot_id") or t.get("robot_id") or ""
        wp = t.get("waypoint_id") or t.get("target_waypoint") or t.get("target") or ""
        parts.append(f"{tid}:{t.get('status','')}:{rid}:{wp}")
    return "|".join(parts)


def print_diagnostic_snapshot(
    base: str,
    robots: Dict[str, dict],
    tasks: Dict[str, dict],
    robot_ids: List[str],
) -> None:
    print("\n=== DIAGNOSTIC SNAPSHOT ===", flush=True)
    try:
        metrics = get_metrics(base)
    except Exception as e:  # noqa: BLE001
        metrics = {"_error": str(e)}
    print(
        "metrics "
        f"wait_edges={metrics.get('wait_edges', '-')}"
        f" wait_graph={metrics.get('wait_graph', '-')}"
        f" deadlock_breaks={metrics.get('deadlock_breaks', '-')}",
        flush=True,
    )
    print("robots:", flush=True)
    for rid in robot_ids:
        r = robots.get(rid) or {}
        route = r.get("planned_route") or r.get("route") or []
        if isinstance(route, list):
            route_s = "->".join(str(x) for x in route[:10])
        else:
            route_s = str(route)
        print(
            f"  {rid}: nav={r.get('nav_status') or r.get('status') or '-'}"
            f" task={r.get('current_task_id') or r.get('task') or '-'}"
            f" wp={r.get('current_waypoint') or '-'}"
            f" seg={r.get('current_segment') or '-'}"
            f" route={route_s or '-'}",
            flush=True,
        )
    live = [t for t in tasks.values() if t.get("status") not in TERMINAL]
    print("live_tasks:", flush=True)
    for t in live[:20]:
        tid = t.get("id") or t.get("task_id") or "-"
        rid = t.get("assigned_robot_id") or t.get("robot_id") or "-"
        wp = t.get("waypoint_id") or t.get("target_waypoint") or t.get("target") or "-"
        print(
            f"  {tid}: status={t.get('status', '-')}"
            f" robot={rid} wp={wp}",
            flush=True,
        )


def pick_hot_waypoints(waypoint_ids: List[str]) -> List[str]:
    """
    选择热点航点。
    优先选择已知的热点位置，不足时补充其他可用航点。
    """
    preferred = [
        "wp_001", "wp_002", "wp_003", "wp_004",
        "wp_005", "wp_006", "wp_007", "wp_016"
    ]
    out = [w for w in preferred if w in waypoint_ids]
    if len(out) >= 4:
        return out
    for w in waypoint_ids:
        if w not in out:
            out.append(w)
        if len(out) >= 8:
            break
    return out or waypoint_ids


@dataclass
class TaskPlan:
    scenario: str
    waypoint_id: str
    robot_id: Optional[str]
    priority: int = 0
    detail: str = ""


def _pick_distinct(
    rng: random.Random,
    items: List[str],
    n: int,
) -> List[str]:
    if not items:
        return []
    if len(items) >= n:
        return rng.sample(items, n)
    out = list(items)
    while len(out) < n:
        out.append(rng.choice(items))
    return out


def build_task_plans(
    rng: random.Random,
    robot_ids: List[str],
    waypoint_ids: List[str],
    hot: List[str],
    task_count: int,
) -> List[TaskPlan]:
    plans: List[TaskPlan] = []
    all_wps = waypoint_ids or hot
    hot_wps = hot or all_wps
    preferred_pairs = [
        ("wp_001", "wp_004"),
        ("wp_002", "wp_007"),
        ("wp_003", "wp_016"),
        ("wp_005", "wp_006"),
    ]
    pairs = [
        (a, b) for a, b in preferred_pairs
        if a in all_wps and b in all_wps and a != b
    ]
    if not pairs:
        picked = _pick_distinct(rng, all_wps, 2)
        if len(picked) >= 2:
            pairs = [(picked[0], picked[1])]

    wave = 0
    while len(plans) < task_count:
        scenario = wave % 9

        if scenario == 0:
            target_wp = hot_wps[wave % len(hot_wps)]
            for rid in robot_ids:
                plans.append(TaskPlan(
                    "hotspot_merge",
                    target_wp,
                    rid,
                    detail=f"all_to={target_wp}",
                ))

        elif scenario == 1:
            pair = pairs[wave % len(pairs)] if pairs else (
                rng.choice(all_wps), rng.choice(all_wps)
            )
            a, b = pair
            for i, rid in enumerate(robot_ids):
                plans.append(TaskPlan(
                    "opposite_pair",
                    b if i % 2 == 0 else a,
                    rid,
                    detail=f"pair={a}<->{b}",
                ))

        elif scenario == 2:
            for i, rid in enumerate(robot_ids):
                plans.append(TaskPlan(
                    "chase_current",
                    "",
                    rid,
                    detail=f"chase_index={(i + 1) % len(robot_ids)}",
                ))

        elif scenario == 3:
            ring = _pick_distinct(rng, hot_wps, min(4, len(robot_ids)))
            while len(ring) < len(robot_ids):
                ring.append(rng.choice(hot_wps))
            for i, rid in enumerate(robot_ids):
                plans.append(TaskPlan(
                    "ring_rotation",
                    ring[(i + 1) % len(ring)],
                    rid,
                    detail="cyclic_targets",
                ))

        elif scenario == 4:
            fixed_target = rng.choice(hot_wps)
            for i, rid in enumerate(robot_ids):
                plans.append(TaskPlan(
                    "mixed_auto_fixed",
                    fixed_target if i < 2 else rng.choice(all_wps),
                    rid if i % 2 == 0 else None,
                    priority=1 if i == 0 else 0,
                    detail=f"fixed_target={fixed_target}",
                ))

        elif scenario == 5:
            corridor = pairs[wave % len(pairs)] if pairs else (
                rng.choice(all_wps), rng.choice(all_wps)
            )
            a, b = corridor
            sequence = [a, b, a, b]
            for i, rid in enumerate(robot_ids):
                plans.append(TaskPlan(
                    "corridor_pingpong",
                    sequence[i % len(sequence)],
                    rid,
                    detail=f"corridor={a}<->{b}",
                ))

        elif scenario == 6:
            chosen = _pick_distinct(rng, all_wps, len(robot_ids))
            for i, rid in enumerate(robot_ids):
                plans.append(TaskPlan(
                    "scatter_random",
                    chosen[i % len(chosen)],
                    rid if rng.random() < 0.5 else None,
                    priority=rng.choice([0, 0, 1]),
                    detail="spread_targets",
                ))

        elif scenario == 7:
            queue_wp = rng.choice(hot_wps[: min(4, len(hot_wps))])
            for i in range(max(4, len(robot_ids))):
                rid = robot_ids[i % len(robot_ids)]
                plans.append(TaskPlan(
                    "same_target_queue",
                    queue_wp,
                    rid if i % 3 != 2 else None,
                    detail=f"queue_wp={queue_wp}",
                ))

        else:
            candidates = hot_wps if rng.random() < 0.7 else all_wps
            for i in range(len(robot_ids)):
                rid = rng.choice(robot_ids)
                plans.append(TaskPlan(
                    "random_mixed",
                    rng.choice(candidates),
                    rid if rng.random() < 0.65 else None,
                    priority=rng.choice([0, 0, 0, 1, 2]),
                    detail="randomized",
                ))

        wave += 1

    return plans[:task_count]


def main() -> int:
    """主函数：解析参数，初始化，执行压力测试循环。"""
    ap = argparse.ArgumentParser(
        description="Fleet scheduler stress test for 4 robots"
    )
    ap.add_argument(
        "--base-url",
        default=os.environ.get("FLEET_API_BASE", "http://127.0.0.1:8080")
    )
    ap.add_argument(
        "--robot-ids", default="",
        help="Comma-separated robot IDs (default: all online)"
    )
    ap.add_argument("--duration", type=float, default=3600.0,
                    help="Max test duration seconds")
    ap.add_argument("--task-count", type=int, default=200)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--poll-interval", type=float, default=0.25)
    ap.add_argument("--submit-burst-interval", type=float, default=2.5)
    ap.add_argument("--submit-burst-size", type=int, default=4)
    ap.add_argument("--max-live-tasks", type=int, default=16)
    ap.add_argument("--stall-timeout", type=float, default=18.0,
                    help="Fail if live tasks but no activity")
    ap.add_argument("--allow-api-errors", action="store_true")
    ap.add_argument("--report-json", default="",
                    help="Optional report output path")
    ap.add_argument("--once", action="store_true",
                    help="Run once then exit with status")
    args = ap.parse_args()

    base = args.base_url.rstrip("/")
    rng = random.Random(args.seed)

    rep = Report(
        started_at=_now_iso(),
        base_url=base,
        seed=args.seed,
        robot_ids=[],
        duration_sec=float(args.duration),
        submit_ok=0,
        submit_total=0,
        poll_errors=[],
        submit_errors=[],
        warnings=[],
        violations=[],
        events=[],
    )

    status = get_status(base)
    if not status.get("ros_connected", False):
        print("ERROR: /api/status ros_connected=false", flush=True)
        return 2

    waypoint_ids = get_waypoint_ids(base)
    if not waypoint_ids:
        print("ERROR: no waypoints from /api/map/waypoints", flush=True)
        return 2
    hot = pick_hot_waypoints(waypoint_ids)

    robots0 = get_robots(base)
    online0 = online_ids(robots0)
    requested = [x.strip() for x in args.robot_ids.split(",") if x.strip()]
    target = requested if requested else online0
    target = [rid for rid in target if rid in robots0]
    target = sorted(set(target))
    if len(target) != 4:
        print(f"ERROR: require exactly 4 target robots, got {len(target)}: "
              f"{target}", flush=True)
        return 2
    rep.robot_ids = target

    # 测试前清理
    try:
        clear_live_tasks(base)
        for rid in target:
            recall_robot(base, rid)
    except Exception as e:  # noqa: BLE001
        if not args.allow_api_errors:
            print(f"ERROR: cleanup failed: {e}", flush=True)
            return 2
        rep.poll_errors.append(f"cleanup: {e}")

    plans = build_task_plans(
        rng,
        target,
        waypoint_ids,
        hot,
        max(0, int(args.task_count)),
    )

    print(f"[bootstrap] robots={target} waypoints={len(waypoint_ids)} "
          f"hot={hot[:8]} task_count={len(plans)}", flush=True)

    t0 = time.time()
    next_burst = t0
    last_poll = 0.0

    def record(kind: str, detail: str) -> None:
        """记录一条事件。"""
        rep.events.append(Event(ts=_now_iso(), kind=kind, detail=detail))

    def check_rules(robots: Dict[str, dict]) -> None:
        """检查交通规则违反，若发现则记录并抛出异常。"""
        rule_hits = check_traffic_rule_violations(robots)
        hard_viol = [v for v in rule_hits if v.startswith("[①同航点]")]
        soft_warn = [v for v in rule_hits if not v.startswith("[①同航点]")]
        for warn in soft_warn:
            if warn in rep.warnings:
                continue
            rep.warnings.append(warn)
            record("WARN", warn)
            print(f"\nWARN: {warn}", flush=True)
        if hard_viol:
            rep.violations.extend(hard_viol)
            raise RuntimeError("\n".join(hard_viol))

    def live_task_count(tasks: Dict[str, dict]) -> int:
        """统计非终态活跃任务数量。"""
        return sum(1 for t in tasks.values() if t.get("status") not in TERMINAL)

    plan_index = 0
    last_live_tasks = 0
    stop_requested = False
    timed_out = False
    next_progress_report = 20
    last_activity_sig = ""
    last_activity_time = time.time()
    stall_detected = False
    last_robots: Dict[str, dict] = robots0
    last_tasks: Dict[str, dict] = {}
    while True:
        now = time.time()
        if now - t0 >= args.duration:
            timed_out = True
            record("TIMEOUT", f"duration={args.duration}s")
            break
        if plan_index >= len(plans) and last_live_tasks == 0:
            break

        if now - last_poll >= args.poll_interval:
            last_poll = now
            try:
                robots = get_robots(base)
                tasks = get_tasks(base)
                last_robots = robots
                last_tasks = tasks
                last_live_tasks = live_task_count(tasks)
                check_rules(robots)
                activity_sig = "|".join(
                    f"{rid}:{robot_activity_signature(robots.get(rid) or {})}"
                    for rid in target
                ) + "||tasks=" + task_activity_signature(tasks)
                if last_live_tasks == 0:
                    last_activity_sig = activity_sig
                    last_activity_time = now
                elif activity_sig != last_activity_sig:
                    last_activity_sig = activity_sig
                    last_activity_time = now
                elif now - last_activity_time >= args.stall_timeout:
                    stall_detected = True
                    record(
                        "STALL",
                        f"no activity for {args.stall_timeout}s live={last_live_tasks}",
                    )
                    print(
                        f"\nFAIL: global stall for {args.stall_timeout}s "
                        f"live={last_live_tasks}",
                        flush=True,
                    )
                    print_diagnostic_snapshot(base, robots, tasks, target)
                    break
            except Exception as e:  # noqa: BLE001
                msg = str(e)
                rep.poll_errors.append(msg)
                if not args.allow_api_errors:
                    print(f"\nFAIL: {msg}", flush=True)
                    break

        if plan_index < len(plans) and now >= next_burst:
            next_burst = now + args.submit_burst_interval
            try:
                tasks = get_tasks(base)
                last_live_tasks = live_task_count(tasks)
                if last_live_tasks >= args.max_live_tasks:
                    continue

                robots = get_robots(base)
                slots = max(0, args.max_live_tasks - last_live_tasks)
                burst = min(max(1, args.submit_burst_size), slots,
                            len(plans) - plan_index)
                for _ in range(burst):
                    plan = plans[plan_index]
                    plan_index += 1
                    wp = plan.waypoint_id
                    chase = ""
                    if plan.scenario == "chase_current" and plan.robot_id:
                        try:
                            i = target.index(plan.robot_id)
                        except ValueError:
                            i = 0
                        other = target[(i + 1) % len(target)]
                        r = robots.get(other) or {}
                        wp = (r.get("current_waypoint") or "").strip()
                        if not wp:
                            wp = rng.choice(hot)
                        chase = f" chase={other}"
                    if not wp:
                        wp = rng.choice(hot)

                    rep.submit_total += 1
                    try:
                        tid = submit_task(
                            base,
                            wp,
                            plan.robot_id,
                            plan.priority,
                        )
                        rep.submit_ok += 1
                        last_live_tasks += 1
                        record(
                            "SUBMIT",
                            f"{plan.scenario} "
                            f"{plan.robot_id or 'auto'}->{wp}"
                            f" pri={plan.priority}{chase} tid={tid} "
                            f"{plan.detail}",
                        )
                    except Exception as e:  # noqa: BLE001
                        rep.submit_errors.append(str(e))
                        if not args.allow_api_errors:
                            print(f"\nFAIL: submit error: {e}", flush=True)
                            stop_requested = True
                            break

                while rep.submit_ok >= next_progress_report:
                    print(
                        f"[progress] submitted={rep.submit_ok}/{len(plans)} "
                        f"live={last_live_tasks}",
                        flush=True,
                    )
                    next_progress_report += 20

            except Exception as e:  # noqa: BLE001
                rep.poll_errors.append(f"burst: {e}")
                if not args.allow_api_errors:
                    print(f"\nFAIL: burst error: {e}", flush=True)
                    break
            if stop_requested:
                break

        time.sleep(0.02)

    # ---- 结果输出 ----
    ok = (
        not timed_out and
        not stall_detected and
        rep.submit_ok == len(plans) and
        last_live_tasks == 0 and
        not rep.violations and
        not (rep.poll_errors and not args.allow_api_errors) and
        not (rep.submit_errors and not args.allow_api_errors)
    )
    if rep.violations:
        print("\n=== VIOLATIONS ===")
        for v in rep.violations[:20]:
            print(v)
    if rep.warnings:
        print("\n=== WARNINGS ===")
        for w in rep.warnings[:20]:
            print(w)
    if not ok and not stall_detected:
        print_diagnostic_snapshot(base, last_robots, last_tasks, target)

    print(
        f"\n[summary] ok={ok} submit_ok={rep.submit_ok}/{rep.submit_total} "
        f"planned={len(plans)} live={last_live_tasks} timeout={timed_out} "
        f"viol={len(rep.violations)} warn={len(rep.warnings)} "
        f"poll_err={len(rep.poll_errors)} "
        f"submit_err={len(rep.submit_errors)}",
        flush=True,
    )

    if args.report_json:
        os.makedirs(os.path.dirname(args.report_json) or ".", exist_ok=True)
        with open(args.report_json, "w", encoding="utf-8") as f:
            json.dump(asdict(rep), f, ensure_ascii=False, indent=2)
        print(f"[report] wrote {args.report_json}", flush=True)

    if args.once:
        return 0 if ok else 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

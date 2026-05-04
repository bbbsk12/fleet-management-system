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


def submit_task(base: str, waypoint_id: str, robot_id: Optional[str]) -> str:
    """提交一个任务，返回任务 ID。"""
    body = {"waypoint_id": waypoint_id, "priority": 0}
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
    task = r.get("task") or r.get("current_task_id") or ""
    return f"{loc}|goal={goal}|hold={hold}|task={task}"


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
    ap.add_argument("--duration", type=float, default=240.0,
                    help="Test duration seconds")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--poll-interval", type=float, default=0.25)
    ap.add_argument("--submit-burst-interval", type=float, default=2.5)
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

    print(f"[bootstrap] robots={target} waypoints={len(waypoint_ids)} "
          f"hot={hot[:8]}", flush=True)

    t0 = time.time()
    next_burst = t0
    last_poll = 0.0
    last_activity_ts = t0
    last_activity_sig: Dict[str, str] = {}

    def record(kind: str, detail: str) -> None:
        """记录一条事件。"""
        rep.events.append(Event(ts=_now_iso(), kind=kind, detail=detail))

    def check_rules(robots: Dict[str, dict]) -> None:
        """检查交通规则违反，若发现则记录并抛出异常。"""
        viol = check_traffic_rule_violations(robots)
        if viol:
            rep.violations.extend(viol)
            raise RuntimeError("\n".join(viol))

    def live_task_count(tasks: Dict[str, dict]) -> int:
        """统计非终态活跃任务数量。"""
        return sum(1 for t in tasks.values() if t.get("status") not in TERMINAL)

    # ---- 压力测试主循环 ----
    phase = 0
    while True:
        now = time.time()
        if now - t0 >= args.duration:
            break

        # ---- 状态轮询 ----
        if now - last_poll >= args.poll_interval:
            last_poll = now
            try:
                robots = get_robots(base)
                tasks = get_tasks(base)
                check_rules(robots)
                # 检测活动变化
                activity = False
                for rid in target:
                    r = robots.get(rid) or {}
                    sig = robot_activity_signature(r)
                    if last_activity_sig.get(rid) != sig:
                        activity = True
                        last_activity_sig[rid] = sig
                if activity:
                    last_activity_ts = now
                # 全局挂起检测
                if live_task_count(tasks) > 0 and \
                   (now - last_activity_ts) >= args.stall_timeout:
                    raise RuntimeError(
                        f"stall_timeout: live_tasks={live_task_count(tasks)} "
                        f"no activity for {now - last_activity_ts:.1f}s"
                    )
            except Exception as e:  # noqa: BLE001
                msg = str(e)
                if "stall_timeout" in msg:
                    record("STALL", msg)
                else:
                    rep.poll_errors.append(msg)
                if not args.allow_api_errors:
                    print(f"\nFAIL: {msg}", flush=True)
                    break

        # ---- 任务突发提交（按阶段切换不同压力场景）----
        if now >= next_burst:
            next_burst = now + args.submit_burst_interval
            try:
                tasks = get_tasks(base)
                if live_task_count(tasks) >= args.max_live_tasks:
                    continue

                robots = get_robots(base)

                # 阶段 0：全部到同一航点（合并队列测试）
                if phase % 4 == 0:
                    target_wp = rng.choice(hot[: min(len(hot), 4)])
                    record("PHASE", f"merge_queue_all_to_one wp={target_wp}")
                    for rid in target:
                        rep.submit_total += 1
                        try:
                            tid = submit_task(base, target_wp, rid)
                            rep.submit_ok += 1
                            record("SUBMIT", f"{rid}->{target_wp} tid={tid}")
                        except Exception as e:  # noqa: BLE001
                            rep.submit_errors.append(str(e))

                # 阶段 1：对向交换（两对机器人互换目标）
                elif phase % 4 == 1:
                    a, b, c, d = target
                    wp_a = rng.choice(hot)
                    wp_b = rng.choice([w for w in hot if w != wp_a] or hot)
                    record("PHASE", f"swap_pairs {a}<->{b} {c}<->{d} "
                           f"wps={wp_a},{wp_b}")
                    for rid, wp in [(a, wp_b), (b, wp_a), (c, wp_a), (d, wp_b)]:
                        rep.submit_total += 1
                        try:
                            tid = submit_task(base, wp, rid)
                            rep.submit_ok += 1
                            record("SUBMIT", f"{rid}->{wp} tid={tid}")
                        except Exception as e:  # noqa: BLE001
                            rep.submit_errors.append(str(e))

                # 阶段 2：追逐另一机器人当前所在航点（动态阻塞）
                elif phase % 4 == 2:
                    record("PHASE", "chase_current_waypoints")
                    cur_wp = {}
                    for rid in target:
                        r = robots.get(rid) or {}
                        cur_wp[rid] = (r.get("current_waypoint") or "").strip()
                    for i, rid in enumerate(target):
                        other = target[(i + 1) % 4]
                        wp = cur_wp.get(other) or rng.choice(hot)
                        rep.submit_total += 1
                        try:
                            tid = submit_task(base, wp, rid)
                            rep.submit_ok += 1
                            record("SUBMIT", f"{rid}->{wp} chase={other} "
                                   f"tid={tid}")
                        except Exception as e:  # noqa: BLE001
                            rep.submit_errors.append(str(e))

                # 阶段 3：随机突发（争用 + 死锁暴露）
                else:
                    record("PHASE", "random_burst")
                    for _ in range(4):
                        rid = rng.choice(target)
                        wp = rng.choice(hot)
                        rep.submit_total += 1
                        try:
                            tid = submit_task(
                                base, wp, rid if rng.random() < 0.7 else None
                            )
                            rep.submit_ok += 1
                            record("SUBMIT", f"{rid or 'auto'}->{wp} tid={tid}")
                        except Exception as e:  # noqa: BLE001
                            rep.submit_errors.append(str(e))

                phase += 1

            except Exception as e:  # noqa: BLE001
                rep.poll_errors.append(f"burst: {e}")
                if not args.allow_api_errors:
                    print(f"\nFAIL: burst error: {e}", flush=True)
                    break

        time.sleep(0.02)

    # ---- 结果输出 ----
    ok = not rep.violations and not (rep.poll_errors and not args.allow_api_errors)
    if rep.violations:
        print("\n=== VIOLATIONS ===")
        for v in rep.violations[:20]:
            print(v)

    print(
        f"\n[summary] ok={ok} submit_ok={rep.submit_ok}/{rep.submit_total} "
        f"viol={len(rep.violations)} poll_err={len(rep.poll_errors)} "
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

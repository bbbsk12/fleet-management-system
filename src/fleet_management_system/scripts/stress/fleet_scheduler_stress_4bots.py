#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fleet scheduler stress test (4 robots).

This script is API-driven (Web backend), and is designed to pressure:
- merge queue to same waypoint (multi-robot convergence)
- opposite-direction / head-on contention (lane/edge contention)
- chasing another robot's current waypoint (dynamic blocking)
- dead-end / deadlock breaking via repeated short bursts

Hard fail conditions:
- traffic-rule violations (same waypoint, same segment, node-edge overlap)
- task submission/poll API failures (unless --allow-api-errors)
- global stall (live tasks exist but no robot activity) beyond threshold

Usage:
  export FLEET_API_BASE=http://127.0.0.1:8080
  python3 scripts/stress/fleet_scheduler_stress_4bots.py --duration 240 --once

Notes:
- Default requires exactly 4 online robots.
- By default targets all online robots; override via --robot-ids.
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
    ts: str
    kind: str
    detail: str


@dataclass
class Report:
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
    return datetime.now().isoformat(timespec="seconds")


def _get(base: str, path: str, timeout: float = 5.0) -> dict:
    return http_json_strict("GET", f"{base}{path}", timeout=timeout)


def _post(base: str, path: str, payload: dict, timeout: float = 8.0) -> dict:
    return http_json_strict("POST", f"{base}{path}", payload, timeout=timeout)


def _delete(base: str, path: str, timeout: float = 8.0) -> dict:
    return http_json_strict("DELETE", f"{base}{path}", timeout=timeout)


def get_status(base: str) -> dict:
    return _get(base, "/api/status", timeout=5.0)


def get_robots(base: str) -> Dict[str, dict]:
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
    data = _get(base, "/api/tasks", timeout=8.0)
    out: Dict[str, dict] = {}
    for t in data.get("tasks", []) or []:
        if isinstance(t, dict) and t.get("id"):
            out[str(t["id"])] = t
    return out


def get_waypoint_ids(base: str) -> List[str]:
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
    body = {"waypoint_id": waypoint_id, "priority": 0}
    if robot_id:
        body["robot_id"] = robot_id
    resp = _post(base, "/api/tasks", body, timeout=10.0)
    tid = resp.get("task_id") or resp.get("id")
    if not tid:
        raise RuntimeError(f"submit missing task_id: {resp}")
    return str(tid)


def recall_robot(base: str, robot_id: str) -> None:
    _post(base, f"/api/robots/{robot_id}/recall", {}, timeout=8.0)


def clear_live_tasks(base: str) -> None:
    tasks = get_tasks(base)
    for tid, t in tasks.items():
        if t.get("status") in TERMINAL:
            continue
        _delete(base, f"/api/tasks/{tid}", timeout=8.0)


def online_ids(robots: Dict[str, dict]) -> List[str]:
    out = []
    for rid, r in robots.items():
        online = r.get("online")
        # server_ros2 may not include online; treat missing as True
        if online is False:
            continue
        if r.get("connection_status") == "offline":
            continue
        out.append(rid)
    return sorted(set(out))


def robot_activity_signature(r: dict) -> str:
    # Keep this robust to differing schemas
    wp = r.get("current_waypoint") or ""
    seg = r.get("current_segment") or ""
    loc = seg or wp or "-"
    goal = "1" if r.get("goal", False) else "0"
    hold = "1" if r.get("hold", False) else "0"
    task = r.get("task") or r.get("current_task_id") or ""
    return f"{loc}|goal={goal}|hold={hold}|task={task}"


def pick_hot_waypoints(waypoint_ids: List[str]) -> List[str]:
    # Prefer known hot spots if present, otherwise fallback to available IDs.
    preferred = ["wp_001", "wp_002", "wp_003", "wp_004", "wp_005", "wp_006", "wp_007", "wp_016"]
    out = [w for w in preferred if w in waypoint_ids]
    if len(out) >= 4:
        return out
    # Fill with any others
    for w in waypoint_ids:
        if w not in out:
            out.append(w)
        if len(out) >= 8:
            break
    return out or waypoint_ids


def main() -> int:
    ap = argparse.ArgumentParser(description="Fleet scheduler stress test for 4 robots")
    ap.add_argument("--base-url", default=os.environ.get("FLEET_API_BASE", "http://127.0.0.1:8080"))
    ap.add_argument("--robot-ids", default="", help="Comma-separated robot IDs (default: all online)")
    ap.add_argument("--duration", type=float, default=240.0, help="Test duration seconds")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--poll-interval", type=float, default=0.25)
    ap.add_argument("--submit-burst-interval", type=float, default=2.5)
    ap.add_argument("--max-live-tasks", type=int, default=16)
    ap.add_argument("--stall-timeout", type=float, default=18.0, help="Fail if live tasks but no activity")
    ap.add_argument("--allow-api-errors", action="store_true")
    ap.add_argument("--report-json", default="", help="Optional report output path")
    ap.add_argument("--once", action="store_true", help="Run once then exit with status")
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
        print(f"ERROR: require exactly 4 target robots, got {len(target)}: {target}", flush=True)
        return 2
    rep.robot_ids = target

    # Cleanup before test
    try:
        clear_live_tasks(base)
        for rid in target:
            recall_robot(base, rid)
    except Exception as e:  # noqa: BLE001
        if not args.allow_api_errors:
            print(f"ERROR: cleanup failed: {e}", flush=True)
            return 2
        rep.poll_errors.append(f"cleanup: {e}")

    print(f"[bootstrap] robots={target} waypoints={len(waypoint_ids)} hot={hot[:8]}", flush=True)

    t0 = time.time()
    next_burst = t0
    last_poll = 0.0
    last_activity_ts = t0
    last_activity_sig: Dict[str, str] = {}

    def record(kind: str, detail: str) -> None:
        rep.events.append(Event(ts=_now_iso(), kind=kind, detail=detail))

    def check_rules(robots: Dict[str, dict]) -> None:
        viol = check_traffic_rule_violations(robots)
        if viol:
            rep.violations.extend(viol)
            raise RuntimeError("\n".join(viol))

    def live_task_count(tasks: Dict[str, dict]) -> int:
        return sum(1 for t in tasks.values() if t.get("status") not in TERMINAL)

    # Stress loop
    phase = 0
    while True:
        now = time.time()
        if now - t0 >= args.duration:
            break

        # Poll
        if now - last_poll >= args.poll_interval:
            last_poll = now
            try:
                robots = get_robots(base)
                tasks = get_tasks(base)
                # rules
                check_rules(robots)
                # activity signature
                activity = False
                for rid in target:
                    r = robots.get(rid) or {}
                    sig = robot_activity_signature(r)
                    if last_activity_sig.get(rid) != sig:
                        activity = True
                        last_activity_sig[rid] = sig
                if activity:
                    last_activity_ts = now
                # stall detection
                if live_task_count(tasks) > 0 and (now - last_activity_ts) >= args.stall_timeout:
                    raise RuntimeError(
                        f"stall_timeout: live_tasks={live_task_count(tasks)} no activity for {now-last_activity_ts:.1f}s"
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

        # Submit bursts (scenario phases)
        if now >= next_burst:
            next_burst = now + args.submit_burst_interval
            try:
                tasks = get_tasks(base)
                if live_task_count(tasks) >= args.max_live_tasks:
                    continue

                robots = get_robots(base)

                # Phase 0: all-to-one (merge queue)
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

                # Phase 1: opposite pairs (two pairs swap targets)
                elif phase % 4 == 1:
                    a, b, c, d = target
                    wp_a = rng.choice(hot)
                    wp_b = rng.choice([w for w in hot if w != wp_a] or hot)
                    record("PHASE", f"swap_pairs {a}<->{b} {c}<->{d} wps={wp_a},{wp_b}")
                    for rid, wp in [(a, wp_b), (b, wp_a), (c, wp_a), (d, wp_b)]:
                        rep.submit_total += 1
                        try:
                            tid = submit_task(base, wp, rid)
                            rep.submit_ok += 1
                            record("SUBMIT", f"{rid}->{wp} tid={tid}")
                        except Exception as e:  # noqa: BLE001
                            rep.submit_errors.append(str(e))

                # Phase 2: chase another robot's current waypoint (dynamic blocking)
                elif phase % 4 == 2:
                    record("PHASE", "chase_current_waypoints")
                    cur_wp = {}
                    for rid in target:
                        r = robots.get(rid) or {}
                        cur_wp[rid] = (r.get("current_waypoint") or "").strip()
                    # rotate targets
                    for i, rid in enumerate(target):
                        other = target[(i + 1) % 4]
                        wp = cur_wp.get(other) or rng.choice(hot)
                        rep.submit_total += 1
                        try:
                            tid = submit_task(base, wp, rid)
                            rep.submit_ok += 1
                            record("SUBMIT", f"{rid}->{wp} chase={other} tid={tid}")
                        except Exception as e:  # noqa: BLE001
                            rep.submit_errors.append(str(e))

                # Phase 3: random burst (contention + deadlock exposure)
                else:
                    record("PHASE", "random_burst")
                    for _ in range(4):
                        rid = rng.choice(target)
                        wp = rng.choice(hot)
                        rep.submit_total += 1
                        try:
                            tid = submit_task(base, wp, rid if rng.random() < 0.7 else None)
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

    ok = not rep.violations and not (rep.poll_errors and not args.allow_api_errors)
    if rep.violations:
        print("\n=== VIOLATIONS ===")
        for v in rep.violations[:20]:
            print(v)

    print(
        f"\n[summary] ok={ok} submit_ok={rep.submit_ok}/{rep.submit_total} "
        f"viol={len(rep.violations)} poll_err={len(rep.poll_errors)} submit_err={len(rep.submit_errors)}",
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


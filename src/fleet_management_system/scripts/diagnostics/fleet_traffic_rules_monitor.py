#!/usr/bin/env python3
"""Poll fleet Web API and verify traffic rules (same as user constraints ①②).

① 同一航点或同一航线上不能有多台车（离散：两车在 same waypoint；或两车在 same undirected edge）
② 若有车在航点 A，则不允许其它车出现在与 A 相连的航线上（一车在线段 u->v，且另一车在 u 或 v 航点且不是该车自己）

③（调度能力）仅能通过业务场景人工观察；本脚本不判失败。

用法:
  export FLEET_API_BASE=http://127.0.0.1:8080
  python3 scripts/diagnostics/fleet_traffic_rules_monitor.py --once
  python3 scripts/diagnostics/fleet_traffic_rules_monitor.py --interval 0.5 --duration 120

违反规则时进程以非零退出（若 --fail-fast），或打印 violation。
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import urllib.error
from pathlib import Path
from typing import Dict, List, Tuple

SCRIPTS_ROOT = Path(__file__).resolve().parents[1]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from _lib.http_api import http_json
from _lib.cli import add_base_url_arg, add_polling_args
from _lib.out import ok_line, ts_hms
from _lib.traffic_rules import check_traffic_rule_violations


def main() -> int:
    ap = argparse.ArgumentParser()
    add_base_url_arg(ap)
    add_polling_args(ap, default_interval=0.5, allow_duration=True, allow_once=True)
    ap.add_argument("--fail-fast", action="store_true", help="发现违反立即退出 2")
    args = ap.parse_args()
    base = args.base.rstrip("/")

    def snapshot() -> Tuple[List[str], Dict]:
        rdata = http_json(f"{base}/api/robots", timeout=5.0)
        robots = rdata.get("robots") or {}
        v = check_traffic_rule_violations(robots)
        return v, robots

    t0 = time.time()
    while True:
        try:
            viol, robots = snapshot()
        except urllib.error.URLError as e:
            print(f"{ts_hms()} [monitor] API 不可用: {e}", file=sys.stderr)
            return 3
        if viol:
            print(f"\n{ts_hms()} *** 规则违反 ***")
            for line in viol:
                print(line)
            if args.fail_fast:
                return 2
        else:
            brief = ", ".join(
                f"{rid}@{r.get('location_type','?')}"
                f":{r.get('current_waypoint') or r.get('current_segment') or '-'}"
                for rid, r in sorted(robots.items())
                if r.get("online")
            )
            print(ok_line("OK", brief), end="", flush=True)
        if args.once:
            print()
            return 0 if not viol else 2
        if args.duration > 0 and (time.time() - t0) >= args.duration:
            print()
            return 0 if not viol else 2
        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())

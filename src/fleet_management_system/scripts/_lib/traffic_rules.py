from __future__ import annotations

from typing import Dict, List, Optional, Tuple


def parse_segment(seg: str) -> Tuple[str, str]:
    if "<->" in seg:
        u, v = seg.split("<->", 1)
    elif "->" in seg:
        u, v = seg.split("->", 1)
    else:
        return "", ""
    u, v = u.strip(), v.strip()
    if not u or not v:
        return "", ""
    return u, v


def undirected_edge(a: str, b: str, sep: str = "^^^") -> str:
    return f"{a}{sep}{b}" if a <= b else f"{b}{sep}{a}"


def check_traffic_rule_violations(
    robots: Dict[str, dict],
    robots_filter: Optional[List[str]] = None,
) -> List[str]:
    violations: List[str] = []
    allowed = set(robots_filter) if robots_filter else None
    online = {
        rid: r
        for rid, r in robots.items()
        if r.get("online")
        and r.get("location_type") in ("waypoint", "segment", "unknown")
        and (allowed is None or rid in allowed)
    }

    wp_to_rids: Dict[str, List[str]] = {}
    for rid, r in online.items():
        if r.get("location_type") == "waypoint" and r.get("current_waypoint"):
            w = r["current_waypoint"]
            wp_to_rids.setdefault(w, []).append(rid)
    for w, rids in wp_to_rids.items():
        if len(rids) >= 2:
            violations.append(f"[①同航点] 航点 {w} 上有 {len(rids)} 台车: {sorted(rids)}")

    edge_to_rids: Dict[str, List[str]] = {}
    for rid, r in online.items():
        if r.get("location_type") != "segment":
            continue
        seg = r.get("current_segment") or ""
        u, v = parse_segment(seg)
        if not u or not v:
            continue
        key = undirected_edge(u, v)
        edge_to_rids.setdefault(key, []).append(rid)
    for e, rids in edge_to_rids.items():
        if len(rids) >= 2:
            violations.append(
                f"[①同航线] 边 {e.replace('^^^', '<->')} 上有 {len(rids)} 台车: {sorted(rids)}"
            )

    at_wp = {
        rid: r.get("current_waypoint")
        for rid, r in online.items()
        if r.get("location_type") == "waypoint" and r.get("current_waypoint")
    }
    for rid_seg, r in online.items():
        if r.get("location_type") != "segment":
            continue
        seg = r.get("current_segment") or ""
        u, v = parse_segment(seg)
        if not u or not v:
            continue
        for other_rid, wp in at_wp.items():
            if other_rid == rid_seg:
                continue
            if wp in (u, v):
                violations.append(
                    f"[②点线互斥] 车 {other_rid} 在航点 {wp}，车 {rid_seg} 在与其相连的线段 {seg}"
                )

    return violations

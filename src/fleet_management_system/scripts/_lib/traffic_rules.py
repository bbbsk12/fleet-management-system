"""交通规则检查工具模块。

提供机器人交通规则冲突检测功能，包括线段解析、无向边键生成
以及基于位置信息的多车同点、同边冲突和点线互斥等规则检查。
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple


def parse_segment(seg: str) -> Tuple[str, str]:
    """解析线段字符串，提取两端航点名称。

    支持 "<->"（双向）和 "->"（单向）两种线段分隔符。
    若格式不合法或解析后任一端为空，则返回空字符串对。

    Args:
        seg: 线段字符串，如 "wp1<->wp2" 或 "wp1->wp2"。

    Returns:
        包含两个端点航点名称的元组 (起点, 终点)。
        解析失败时返回 ("", "")。
    """
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
    """生成无向边键，使得 (a, b) 和 (b, a) 得到相同结果。

    通过将两个航点名称按字典序排列后拼接，构造唯一且顺序无关的边标识。

    Args:
        a: 第一个航点名称。
        b: 第二个航点名称。
        sep: 连接分隔符，默认为 "^^^"（避免与正常的 "<->" 混淆）。

    Returns:
        排序后的无向边键字符串。
    """
    return f"{a}{sep}{b}" if a <= b else f"{b}{sep}{a}"


def check_traffic_rule_violations(
    robots: Dict[str, dict],
    robots_filter: Optional[List[str]] = None,
) -> List[str]:
    """检查各机器人之间的交通规则冲突，返回违规描述列表。

    遍历所有在线机器人的位置信息，依次检查以下三类违规：
        1. 同航点冲突：多台车位于同一航点。
        2. 同边冲突：多台车位于同一线段上。
        3. 点线互斥：一台车在航点，另一台车在该航点相连的线段上。

    Args:
        robots: 机器人状态字典，键为机器人 ID，值为包含
                online、location_type、current_waypoint、
                current_segment 等字段的状态字典。
        robots_filter: 可选的机器人 ID 过滤列表，仅检查列表内
                       的机器人；为 None 时检查所有在线机器人。

    Returns:
        违规描述字符串列表，每条描述以中文标识符开头
        （如 [①同航点]、[①同航线]、[②点线互斥]）。
        若无违规则返回空列表。
    """
    # ---------- 第一步：筛选在线机器人 ----------
    violations: List[str] = []
    allowed = set(robots_filter) if robots_filter else None
    online = {
        rid: r
        for rid, r in robots.items()
        if r.get("online")
        and r.get("location_type") in ("waypoint", "segment", "unknown")
        and (allowed is None or rid in allowed)
    }

    # ---------- 第二步：检查同航点冲突 ----------
    # 将位于同一航点的机器人聚合，存在 2 台及以上则记为违规
    wp_to_rids: Dict[str, List[str]] = {}
    for rid, r in online.items():
        if r.get("location_type") == "waypoint" and r.get("current_waypoint"):
            w = r["current_waypoint"]
            wp_to_rids.setdefault(w, []).append(rid)
    for w, rids in wp_to_rids.items():
        if len(rids) >= 2:
            violations.append(f"[①同航点] 航点 {w} 上有 {len(rids)} 台车: {sorted(rids)}")

    # ---------- 第三步：检查同边冲突 ----------
    # 将位于同一无向边上的机器人聚合，存在 2 台及以上则记为违规
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

    # ---------- 第四步：检查点线互斥 ----------
    # 若某台车在航点，另一台车在该航点连接的线段上，则记为违规
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

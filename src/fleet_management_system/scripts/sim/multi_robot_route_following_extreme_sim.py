#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线循径多机器人不变量与最优策略测试
======================================

与简单贪婪仿真器的区别：
  - 每个机器人预先计算到目标的最短路径（航线）。
  - 运行时，机器人只能移动到路径上的下一个航点。
  - 若移动将导致不变量被违反，则机器人等待（保持原位）。

这更好地模拟了"按航线走"的调度语义。

功能说明：
  评估移动顺序策略（作为"谁让行"的代理）在以下场景中的表现：
    - 极端交接场景（wp_016 死胡同 + wp_007 瓶颈）
    - 随机场景（可选）

  不涉及可视化，仅检查硬不变量并报告可达性/死锁情况。
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

import yaml


def _edge_key(a: str, b: str) -> str:
    """生成无向边的标准化键值。"""
    return f"{a}<->{b}" if a <= b else f"{b}<->{a}"


class WaypointGraph:
    """航点图数据结构。"""

    def __init__(self, waypoints: Dict[str, Dict]) -> None:
        self.waypoints = waypoints
        self.adj: Dict[str, List[str]] = {}
        for wid, wp in waypoints.items():
            conns = wp.get("connections") or []
            self.adj[wid] = [str(x) for x in conns]

    def nodes(self) -> List[str]:
        """返回所有航点 ID。"""
        return list(self.waypoints.keys())

    def neighbors(self, wid: str) -> List[str]:
        """返回指定航点的相邻航点。"""
        return self.adj.get(wid, [])

    def has_node(self, wid: str) -> bool:
        """判断指定航点 ID 是否存在于图中。"""
        return wid in self.waypoints


def load_rmf_map_yaml(path: str) -> WaypointGraph:
    """从 RMF 地图 YAML 加载航点图。"""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    waypoints: Dict[str, Dict] = {}
    for wp in data.get("waypoints", []) or []:
        wid = str(wp.get("id"))
        waypoints[wid] = wp
    if not waypoints:
        raise RuntimeError(f"No waypoints parsed from {path}")
    return WaypointGraph(waypoints)


def bfs_shortest_path(graph: WaypointGraph, start: str, goal: str) -> List[str]:
    """使用 BFS 求最短路径，返回航点序列。"""
    if start == goal:
        return [start]
    q: List[str] = [start]
    parent: Dict[str, str] = {start: ""}
    for cur in q:
        for nb in graph.neighbors(cur):
            if nb in parent:
                continue
            parent[nb] = cur
            if nb == goal:
                break
            q.append(nb)
        if goal in parent:
            break
    if goal not in parent:
        return []
    # 重建路径
    cur = goal
    rev = [cur]
    while parent[cur] != "":
        cur = parent[cur]
        rev.append(cur)
    rev.reverse()
    return rev


def bfs_dist(graph: WaypointGraph, start: str) -> Dict[str, int]:
    """使用 BFS 计算到所有航点的最短距离。"""
    q: List[str] = [start]
    dist: Dict[str, int] = {start: 0}
    for cur in q:
        for nb in graph.neighbors(cur):
            if nb in dist:
                continue
            dist[nb] = dist[cur] + 1
            q.append(nb)
    return dist


@dataclass
class Violation:
    """不变量违规记录。"""
    rule: str
    detail: str
    step: int
    robots: List[str]


@dataclass
class Scenario:
    """测试场景定义。"""
    name: str
    start_pos: Dict[str, str]
    goals: Dict[str, str]


class RouteFollowingSimulator:
    """
    循径仿真器。

    每个机器人沿着预计算的最短路径移动，仅当移动不会违反不变量时才前进。
    支持多种移动顺序策略和规避阻塞的迂回机制。
    """

    def __init__(
        self,
        graph: WaypointGraph,
        robot_ids: Sequence[str],
        start_pos: Dict[str, str],
        goals: Dict[str, str],
        rng: random.Random,
        max_steps: int,
        move_order: str,
        allow_escape_on_block: bool,
    ) -> None:
        self.graph = graph
        self.robot_ids = list(robot_ids)
        self.start_pos = dict(start_pos)
        self.goals = dict(goals)
        self.rng = rng
        self.max_steps = max_steps
        self.move_order = move_order
        self.allow_escape_on_block = allow_escape_on_block

        self.pos_t: Dict[str, str] = dict(start_pos)
        self.route_index: Dict[str, int] = {rid: 0 for rid in self.robot_ids}
        self.route: Dict[str, List[str]] = {}
        for rid in self.robot_ids:
            st = self.pos_t[rid]
            gl = self.goals[rid]
            path = bfs_shortest_path(self.graph, st, gl)
            if not path:
                # 图中不可达 => 使用退化路径避免崩溃
                path = [st]
            self.route[rid] = path

        self.violations: List[Violation] = []
        self.deadlock_ticks = 0
        self.steps_taken: Optional[int] = None
        self.dist_to_goal: Dict[str, Dict[str, int]] = {
            rid: bfs_dist(self.graph, self.goals[rid]) for rid in self.robot_ids
        }

    def _move_order_robots(self, step: int) -> List[str]:
        """根据当前策略确定本步各机器人的移动顺序。"""

        def remaining_to_goal(rid: str) -> int:
            i = self.route_index[rid]
            return len(self.route[rid]) - 1 - i

        if self.move_order == "random":
            ids = list(self.robot_ids)
            self.rng.shuffle(ids)
            return ids
        if self.move_order == "near_first":
            return sorted(self.robot_ids,
                          key=lambda rid: (remaining_to_goal(rid), rid))
        if self.move_order == "far_first":
            return sorted(self.robot_ids,
                          key=lambda rid: (-remaining_to_goal(rid), rid))

        raise RuntimeError(f"unknown move_order={self.move_order}")

    def _is_move_feasible(
        self,
        rid: str,
        cur: str,
        nxt: str,
        reserved_next_wp: Set[str],
        reserved_edges: Set[str],
    ) -> bool:
        """判断从 cur 移动到 nxt 在当前预留集下是否可行。"""
        if nxt in reserved_next_wp:
            return False
        ek = _edge_key(cur, nxt)
        if ek in reserved_edges:
            return False

        # R3: 如果其他机器人位于任一端点，不能经过此航道
        endpoints = {cur, nxt}
        for other in self.robot_ids:
            if other == rid:
                continue
            if self.pos_t[other] in endpoints:
                return False
        return True

    def _check_invariants(
        self, step: int, pos_t1: Dict[str, str],
        traversed_edges: Dict[str, Optional[str]]
    ) -> None:
        """验证当前时间步的三条不变量。"""
        # R1: 同一航点唯一性
        wp_to_robots: Dict[str, List[str]] = {}
        for rid, wp in pos_t1.items():
            wp_to_robots.setdefault(wp, []).append(rid)
        for wp, rids in wp_to_robots.items():
            if len(rids) > 1:
                self.violations.append(
                    Violation(
                        rule="R1_same_waypoint",
                        detail=f"waypoint={wp} robots={rids}",
                        step=step,
                        robots=sorted(rids),
                    )
                )
                return

        # R2: 同一航道唯一性
        edge_to_robots: Dict[str, List[str]] = {}
        for rid, ek in traversed_edges.items():
            if not ek:
                continue
            edge_to_robots.setdefault(ek, []).append(rid)
        for ek, rids in edge_to_robots.items():
            if len(rids) > 1:
                self.violations.append(
                    Violation(
                        rule="R2_same_segment",
                        detail=f"edge={ek} robots={rids}",
                        step=step,
                        robots=sorted(rids),
                    )
                )
                return

        # R3: 航点-航段重叠
        for rid_move, ek in traversed_edges.items():
            if not ek:
                continue
            a, b = ek.split("<->", 1)
            for rid_other, wp_other_t in self.pos_t.items():
                if rid_other == rid_move:
                    continue
                if wp_other_t == a or wp_other_t == b:
                    self.violations.append(
                        Violation(
                            rule="R3_node_edge_overlap",
                            detail=f"move={rid_move} traverses {ek} "
                                   f"while {rid_other} at wp={wp_other_t} "
                                   f"at time t={step}",
                            step=step,
                            robots=sorted([rid_move, rid_other]),
                        )
                    )
                    return

    def step_once(self, step: int) -> bool:
        """
        执行一个仿真步。
        返回是否至少有一个机器人发生了移动。
        """
        reserved_next_wp: Set[str] = set()
        reserved_edges: Set[str] = set()

        pos_t1: Dict[str, str] = dict(self.pos_t)
        traversed_edges: Dict[str, Optional[str]] = {
            rid: None for rid in self.robot_ids
        }
        moved_any = False

        order = self._move_order_robots(step)
        for rid in order:
            cur = self.pos_t[rid]
            goal = self.goals[rid]
            if cur == goal:
                pos_t1[rid] = cur
                continue

            # 严格按预计算路径的下一跳移动
            i = self.route_index[rid]
            if i + 1 >= len(self.route[rid]):
                # 已到路径终点 => 等待
                pos_t1[rid] = cur
                continue
            desired_nxt = self.route[rid][i + 1]

            # 1) 尝试沿路径的期望移动
            if self._is_move_feasible(
                rid, cur, desired_nxt, reserved_next_wp, reserved_edges
            ):
                edge_key = _edge_key(cur, desired_nxt)
                pos_t1[rid] = desired_nxt
                traversed_edges[rid] = edge_key
                reserved_next_wp.add(desired_nxt)
                reserved_edges.add(edge_key)
                moved_any = True
                self.route_index[rid] = i + 1
                continue

            # 2) 若被阻塞：可选择迂回到其他可行邻居
            if not self.allow_escape_on_block:
                continue

            candidates = [nb for nb in self.graph.neighbors(cur) if nb != cur]
            feasible: List[str] = [
                nb for nb in candidates
                if self._is_move_feasible(
                    rid, cur, nb, reserved_next_wp, reserved_edges
                )
            ]
            if not feasible:
                continue

            best_nb: Optional[str] = None
            best_score: Optional[float] = None
            for nb in feasible:
                # 主要目标：向目标靠近
                d_goal = self.dist_to_goal[rid].get(nb, 10**9)
                # 次要目标：与其他机器人保持距离
                dist_from_nb = bfs_dist(self.graph, nb)
                other_wps = [self.pos_t[o] for o in self.robot_ids if o != rid]
                min_sep = min(
                    (dist_from_nb.get(wp, 0) for wp in other_wps), default=0
                )
                # 评分：越小 d_goal、越大 min_sep 越好
                score = (-float(d_goal)) + 0.15 * float(min_sep)
                if best_score is None or score > best_score:
                    best_score = score
                    best_nb = nb

            if best_nb is None:
                continue

            edge_key = _edge_key(cur, best_nb)
            pos_t1[rid] = best_nb
            traversed_edges[rid] = edge_key
            reserved_next_wp.add(best_nb)
            reserved_edges.add(edge_key)
            moved_any = True

            # 若迂回目标在预计算路径上，将路径索引向前跳跃
            try:
                next_i = self.route[rid].index(best_nb)
                if next_i > self.route_index[rid]:
                    self.route_index[rid] = next_i
            except ValueError:
                pass

        # 提交后验证不变量（按构造应始终通过）
        self._check_invariants(step, pos_t1, traversed_edges)
        self.pos_t = pos_t1
        return moved_any

    def run(self) -> bool:
        """运行仿真，返回是否所有机器人成功到达目标。"""
        for step in range(self.max_steps):
            if self.violations:
                return False
            if all(self.pos_t[rid] == self.goals[rid] for rid in self.robot_ids):
                self.steps_taken = step
                return True
            moved = self.step_once(step)
            if not moved:
                self.deadlock_ticks += 1
        self.steps_taken = self.max_steps
        return False


def build_extreme_scenarios(graph: WaypointGraph,
                            robot_ids: Sequence[str]) -> List[Scenario]:
    """
    构建围绕已知死胡同 + 瓶颈的硬编码极端场景：
      - wp_016 是死胡同，连接 wp_007
      - wp_007 是瓶颈，连接 wp_001 / wp_002 / wp_016
    """
    ids = list(robot_ids)
    if not (graph.has_node("wp_007") and graph.has_node("wp_016")):
        return []

    nodes = graph.nodes()

    def fill_random(extra_ids: Sequence[str], used: Set[str],
                    rng: random.Random) -> Dict[str, str]:
        """为额外机器人分配远离核心区域的随机航点。"""
        avoid = set(used)
        if graph.has_node("wp_007"):
            avoid.add("wp_007")
        if graph.has_node("wp_016"):
            avoid.add("wp_016")
        avail = [n for n in nodes if n not in avoid]
        if len(avail) < len(extra_ids):
            raise RuntimeError("Not enough nodes to fill extra robots")
        chosen = rng.sample(avail, k=len(extra_ids))
        out: Dict[str, str] = {}
        for rid, wp in zip(extra_ids, chosen):
            out[rid] = wp
        return out

    scenarios: List[Scenario] = []
    rng = random.Random(12345)

    # 场景 A：死胡同交接
    core_start = {"A": "wp_016", "B": "wp_007"}
    if graph.has_node("wp_002"):
        core_goals = {"A": "wp_002", "B": "wp_016"}
    else:
        core_goals = {"A": "wp_001", "B": "wp_016"}
    used_nodes = {core_start["A"], core_start["B"]}
    start_pos: Dict[str, str] = {}
    goals: Dict[str, str] = {}
    if len(ids) >= 2:
        start_pos[ids[0]] = core_start["A"]
        start_pos[ids[1]] = core_start["B"]
        goals[ids[0]] = core_goals["A"]
        goals[ids[1]] = core_goals["B"]
        extra_ids = ids[2:]
        if extra_ids:
            extra_starts = fill_random(extra_ids, used_nodes, rng)
            start_pos.update(extra_starts)
            for erid in extra_ids:
                avail_goals = [n for n in nodes if n != start_pos[erid]]
                goals[erid] = rng.choice(avail_goals) if avail_goals else start_pos[erid]
        scenarios.append(Scenario(
            name="ext_dead_end_handoff_wp016_wp007",
            start_pos=start_pos, goals=goals
        ))

    # 场景 B：对称出口侧
    if len(ids) >= 2 and graph.has_node("wp_001"):
        start_pos = {ids[0]: "wp_016", ids[1]: "wp_007"}
        goals = {ids[0]: "wp_001", ids[1]: "wp_016"}
        used_nodes = set(start_pos.values())
        extra_ids = ids[2:]
        if extra_ids:
            extra_starts = fill_random(extra_ids, used_nodes, rng)
            start_pos.update(extra_starts)
            for erid in extra_ids:
                avail_goals = [n for n in nodes if n != start_pos[erid]]
                goals[erid] = rng.choice(avail_goals) if avail_goals else start_pos[erid]
        scenarios.append(Scenario(
            name="ext_dead_end_handoff_exit_wp001",
            start_pos=start_pos, goals=goals
        ))

    # 场景 C：wp_002 <-> wp_007 边交换
    if len(ids) >= 2 and graph.has_node("wp_002"):
        start_pos = {ids[0]: "wp_002", ids[1]: "wp_007"}
        goals = {ids[0]: "wp_007", ids[1]: "wp_002"}
        used_nodes = set(start_pos.values())
        extra_ids = ids[2:]
        if extra_ids:
            extra_starts = fill_random(extra_ids, used_nodes, rng)
            start_pos.update(extra_starts)
            for erid in extra_ids:
                avail_goals = [n for n in nodes if n != start_pos[erid]]
                goals[erid] = rng.choice(avail_goals) if avail_goals else start_pos[erid]
        scenarios.append(Scenario(
            name="ext_edge_swap_wp002_wp007",
            start_pos=start_pos, goals=goals
        ))

    return scenarios


def generate_random_scenarios(
    graph: WaypointGraph,
    robot_ids: Sequence[str],
    scenarios: int,
    rng: random.Random,
    max_start_goal_tries: int,
    unique_goals: bool,
) -> List[Scenario]:
    """生成随机的测试场景列表。"""
    nodes = graph.nodes()
    out: List[Scenario] = []
    for si in range(scenarios):
        if len(nodes) < len(robot_ids):
            break
        starts = rng.sample(nodes, k=len(robot_ids))
        start_pos = {rid: starts[i] for i, rid in enumerate(robot_ids)}
        goals: Dict[str, str] = {}
        if unique_goals:
            available_goals = [n for n in nodes if n not in set(start_pos.values())]
            if len(available_goals) >= len(robot_ids):
                chosen_goals = rng.sample(available_goals, k=len(robot_ids))
            else:
                chosen_goals = rng.sample(nodes, k=len(robot_ids))
            for rid, g in zip(robot_ids, chosen_goals):
                goals[rid] = g
        else:
            for rid in robot_ids:
                for _ in range(max_start_goal_tries):
                    g = rng.choice(nodes)
                    if g != start_pos[rid]:
                        goals[rid] = g
                        break
                else:
                    goals[rid] = start_pos[rid]
        out.append(Scenario(name=f"rand_{si}", start_pos=start_pos, goals=goals))
    return out


def evaluate_policy_on_scenarios(
    graph: WaypointGraph,
    robot_ids: Sequence[str],
    scenarios: Sequence[Scenario],
    policy: str,
    seed: int,
    max_steps: int,
) -> Tuple[int, int, float, float]:
    """
    评估指定策略在一组场景上的表现。

    返回:
      (违规数, 失败数, 成功场景平均步数, 平均死锁次数)
    """
    violations = 0
    failed = 0
    steps_success: List[int] = []
    deadlocks: List[int] = []

    for idx, sc in enumerate(scenarios):
        sim = RouteFollowingSimulator(
            graph=graph,
            robot_ids=robot_ids,
            start_pos=sc.start_pos,
            goals=sc.goals,
            rng=random.Random(seed + idx * 1000),
            max_steps=max_steps,
            move_order=policy,
            allow_escape_on_block=True,
        )
        ok = sim.run()
        if sim.violations:
            violations += 1
            v = sim.violations[0]
            print(
                f"[VIOLATION] policy={policy} scenario={sc.name} "
                f"rule={v.rule} step={v.step} robots={v.robots}",
                file=sys.stderr
            )
            print(f"detail={v.detail}", file=sys.stderr)
            return violations, failed, 0.0, 0.0
        deadlocks.append(sim.deadlock_ticks)
        if not ok:
            failed += 1
        else:
            steps_success.append(sim.steps_taken or max_steps)

    avg_steps = (sum(steps_success) / len(steps_success)) if steps_success else float(max_steps)
    avg_deadlocks = sum(deadlocks) / len(deadlocks) if deadlocks else 0.0
    return violations, failed, avg_steps, avg_deadlocks


def main() -> int:
    """主函数：解析参数，构建场景，评估并选择最优策略。"""
    parser = argparse.ArgumentParser(
        description="Offline route-following extreme simulator"
    )
    parser.add_argument(
        "--map-yaml",
        default="src/fleet_management_system/maps/map0/rmf_map0.yaml"
    )
    parser.add_argument("--robot-count", type=int, default=4)
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--random-scenarios", type=int, default=50)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--policy", default="auto_best_extreme",
        help="far_first|near_first|random|auto_best_extreme"
    )
    parser.add_argument(
        "--unique-goals", action="store_true", default=True,
        help="Generate random scenarios with unique terminal waypoints"
    )
    parser.add_argument(
        "--no-unique-goals", action="store_false", dest="unique_goals"
    )
    args = parser.parse_args()

    graph = load_rmf_map_yaml(args.map_yaml)
    if len(graph.nodes()) < args.robot_count:
        print("ERROR: map has too few waypoints for requested robot-count",
              file=sys.stderr)
        return 2

    robot_ids = [f"r{i}" for i in range(args.robot_count)]
    rng = random.Random(args.seed)

    extreme_scenarios = build_extreme_scenarios(graph, robot_ids)
    random_scenarios = generate_random_scenarios(
        graph, robot_ids, args.random_scenarios, rng,
        max_start_goal_tries=200, unique_goals=args.unique_goals
    )

    scenarios_all = extreme_scenarios + random_scenarios
    print(f"[bootstrap] map={args.map_yaml} robots={args.robot_count} "
          f"extreme={len(extreme_scenarios)} random={len(random_scenarios)} "
          f"total={len(scenarios_all)}")
    if not scenarios_all:
        print("ERROR: no scenarios generated", file=sys.stderr)
        return 2

    candidate_policies = ["far_first", "near_first", "random"]
    selected_policy = args.policy
    if args.policy == "auto_best_extreme":
        # 仅在极端场景上评估"最优调度"
        if not extreme_scenarios:
            selected_policy = "far_first"
        else:
            best = None
            best_tuple = None
            for pol in ["far_first", "near_first"]:
                violations, failed, avg_steps, avg_deadlocks = \
                    evaluate_policy_on_scenarios(
                        graph, robot_ids, extreme_scenarios, pol,
                        seed=args.seed, max_steps=args.max_steps
                    )
                # 评价指标：失败数越少越好，其次死锁数，最后步数
                t = (failed, avg_deadlocks, avg_steps)
                print(f"[ext-eval] policy={pol} failed={failed}/"
                      f"{len(extreme_scenarios)} avg_deadlocks={avg_deadlocks:.1f} "
                      f"avg_steps_success={avg_steps:.1f}")
                if best is None or t < best_tuple:
                    best = pol
                    best_tuple = t
            selected_policy = best or "far_first"
            print(f"[ext-eval] selected best policy={selected_policy}")

    violations, failed, avg_steps, avg_deadlocks = evaluate_policy_on_scenarios(
        graph, robot_ids, scenarios_all, selected_policy,
        seed=args.seed, max_steps=args.max_steps
    )
    print(f"[result] policy={selected_policy} violations={violations} "
          f"failed={failed}/{len(scenarios_all)} "
          f"avg_deadlocks={avg_deadlocks:.1f} "
          f"avg_steps_success={avg_steps:.1f}")
    return 1 if violations else 0


if __name__ == "__main__":
    raise SystemExit(main())

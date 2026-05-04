#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线多机器人不变量仿真器（无需 ROS，无需 Nav2）
=================================================

功能说明：
  在从项目地图 YAML 提取的航点图上仿真机器人移动，并在每个离散步骤中强制执行以下
  三条硬安全不变量：

    1) 同一时间步内，不能有两台机器人位于同一航点。
    2) 同一时间步内，不能有两台机器人在同一航道（航段）上行驶。
    3) 若某台机器人在时间 t 位于航点 W，则时间 t→t+1 期间，
       没有其他机器人可以经过 W 所在边的任何航段（航点-航段重叠禁止）。

设计用途：
  验证候选调度策略能否在不产生调度不变量违规的情况下实现。
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import yaml


def _edge_key(a: str, b: str) -> str:
    """生成无向边的标准化键值。"""
    return f"{a}<->{b}" if a <= b else f"{b}<->{a}"


@dataclass
class StepState:
    """单个时间步的状态记录。"""
    t: int
    pos_t: Dict[str, str]                 # 当前时间步各机器人所在航点
    pos_t1: Dict[str, str]                # 下一时间步各机器人所在航点
    traversed_edges: Dict[str, Optional[str]]  # 各机器人行驶的边（无向）或 None


@dataclass
class Violation:
    """不变量违规记录。"""
    rule: str
    detail: str
    step: int
    robots: List[str]


class WaypointGraph:
    """航点图数据结构，维护航点及其连接关系。"""

    def __init__(self, waypoints: Dict[str, Dict]) -> None:
        self.waypoints = waypoints
        self.adj: Dict[str, List[str]] = {wid: [] for wid in waypoints.keys()}
        for wid, wp in waypoints.items():
            conns = wp.get("connections") or []
            self.adj[wid] = [str(x) for x in conns]

    def nodes(self) -> List[str]:
        """返回所有航点 ID 列表。"""
        return list(self.waypoints.keys())

    def neighbors(self, wid: str) -> List[str]:
        """返回指定航点的所有相邻航点。"""
        return self.adj.get(wid, [])


def load_rmf_map_yaml(path: str) -> WaypointGraph:
    """从 RMF 地图 YAML 文件加载航点图。"""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    waypoints = {}
    for wp in data.get("waypoints", []) or []:
        wid = str(wp.get("id"))
        waypoints[wid] = wp
    if not waypoints:
        raise RuntimeError(f"No waypoints parsed from {path}")
    return WaypointGraph(waypoints)


def bfs_dist(graph: WaypointGraph, start: str) -> Dict[str, int]:
    """使用 BFS 计算从起点到所有其他航点的最短距离。"""
    q: List[str] = [start]
    dist: Dict[str, int] = {start: 0}
    for cur in q:
        for nb in graph.neighbors(cur):
            if nb not in dist:
                dist[nb] = dist[cur] + 1
                q.append(nb)
    return dist


def shortest_next_hop(graph: WaypointGraph, cur: str, goal: str,
                       dist_to_goal: Dict[str, int]) -> Optional[str]:
    """
    在相邻航点中选择到目标距离最近的作为下一跳。
    若无相邻航点可减小距离则返回 None（机器人将等待）。
    """
    best_nb = None
    best_d = 10**9
    for nb in graph.neighbors(cur):
        d = dist_to_goal.get(nb)
        if d is None:
            continue
        if d < best_d:
            best_d = d
            best_nb = nb
    return best_nb


class InvariantSimulator:
    """
    不变量仿真器。

    在每一步中按贪婪策略为各机器人规划移动，并通过预留机制确保
    三条安全不变量不被违反。
    """

    def __init__(
        self,
        graph: WaypointGraph,
        robots: List[str],
        start_pos: Dict[str, str],
        goals: Dict[str, str],
        rng: random.Random,
        max_steps: int,
        greedy_noise: float,
    ) -> None:
        self.graph = graph
        self.robots = robots
        self.pos_t: Dict[str, str] = dict(start_pos)
        self.goals = dict(goals)
        self.rng = rng
        self.max_steps = max_steps
        self.greedy_noise = greedy_noise

        self.violations: List[Violation] = []
        self.deadlocks = 0

        # 预计算各机器人到目标的距离图（用于贪婪策略）
        self.dist_to_goal: Dict[str, Dict[str, int]] = {
            rid: bfs_dist(graph, goals[rid]) for rid in robots
        }

    def _check_step_invariants(
        self, step: int, pos_t: Dict[str, str], pos_t1: Dict[str, str],
        traversed_edges: Dict[str, Optional[str]]
    ) -> None:
        """
        验证当前时间步的三条不变量：
          R1: 同一航点唯一性
          R2: 同一航道唯一性
          R3: 航点-航段重叠禁止
        """
        # R1: 同一时间步各机器人位于不同航点
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

        # R2: 同一时间步各机器人行驶在不同航道
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

        # R3: 若某机器人位于航点 W，则其他机器人不得经过与 W 相连的航道
        for rid_move, ek in traversed_edges.items():
            if not ek:
                continue
            a, b = ek.split("<->", 1)
            for rid_other, wp_other_t in pos_t.items():
                if rid_other == rid_move:
                    continue
                if wp_other_t == a or wp_other_t == b:
                    self.violations.append(
                        Violation(
                            rule="R3_node_edge_overlap",
                            detail=f"move={rid_move} traverses {ek} "
                                   f"while {rid_other} is at waypoint {wp_other_t} "
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
        # 优先级排序：剩余距离较大的机器人优先选择
        def remaining_dist(rid: str) -> int:
            dmap = self.dist_to_goal.get(rid) or {}
            return dmap.get(self.pos_t[rid], 999999)

        # 加入小噪声以避免在极端图拓扑中出现确定性饥饿
        ordered = sorted(
            self.robots,
            key=lambda rid: (-remaining_dist(rid), rid),
        )

        # 按序规划，使用预留集保证不变量
        reserved_next_wp: Set[str] = set()
        reserved_edges: Set[str] = set()
        forbidden_next_wp: Set[str] = set()

        pos_t1: Dict[str, str] = dict(self.pos_t)
        traversed_edges: Dict[str, Optional[str]] = {rid: None for rid in self.robots}

        moved_any = False

        for rid in ordered:
            cur = self.pos_t[rid]
            goal = self.goals[rid]
            if cur == goal:
                pos_t1[rid] = cur
                continue

            # 贪婪候选列表：按到目标的距离升序排列邻居
            dist_map = self.dist_to_goal.get(rid) or {}
            candidates = list(self.graph.neighbors(cur))
            candidates_sorted = sorted(
                candidates,
                key=lambda nb: (dist_map.get(nb, 10**9), nb),
            )

            # 注入噪声：有时选择次优解以探索更多路径
            if candidates_sorted and self.greedy_noise > 0:
                if self.rng.random() < self.greedy_noise and len(candidates_sorted) > 1:
                    candidates_sorted = [candidates_sorted[1]] + candidates_sorted[0:1] + candidates_sorted[2:]

            # 将"等待"也作为候选（排在前面优先考虑）
            considered = [cur] + candidates_sorted

            chosen: Optional[str] = None
            chosen_edge: Optional[str] = None

            for nb in considered:
                if nb == cur:
                    # 等待本身不预留航道，但需检查航点是否已被预留
                    if nb in reserved_next_wp:
                        continue
                    if nb in forbidden_next_wp:
                        continue
                    chosen = nb
                    chosen_edge = None
                    break

                edge_key = _edge_key(cur, nb)
                # R2: 航道预留唯一性
                if edge_key in reserved_edges:
                    continue
                # R1: 下一时间步航点唯一性
                if nb in reserved_next_wp:
                    continue
                # R3 第二部分：若 nb 是已选移动边的端点，禁止再次前往
                if nb in forbidden_next_wp:
                    continue
                # R3 第一部分：严格检查与其他机器人当前位置的航点-航段重叠
                endpoints = {cur, nb}
                blocked_by_current = False
                for other in self.robots:
                    if other == rid:
                        continue
                    if self.pos_t[other] in endpoints:
                        blocked_by_current = True
                        break
                if blocked_by_current:
                    continue

                # 找到可行移动
                chosen = nb
                chosen_edge = edge_key
                break

            if chosen is None:
                # 无可行移动 => 在当前航点等待
                chosen = cur
                chosen_edge = None

            # 提交机器人的规划
            pos_t1[rid] = chosen
            if chosen_edge:
                traversed_edges[rid] = chosen_edge
                reserved_edges.add(chosen_edge)
                forbidden_next_wp.add(cur)
                forbidden_next_wp.add(chosen)
                moved_any = True
            reserved_next_wp.add(chosen)

        # 验证不变量
        self._check_step_invariants(step, self.pos_t, pos_t1, traversed_edges)
        self.pos_t = pos_t1

        if not moved_any:
            self.deadlocks += 1
        return moved_any

    def run(self) -> None:
        """运行仿真直到所有机器人到达目标、检测到违规或达到最大步数。"""
        for t in range(self.max_steps):
            if self.violations:
                return

            all_reached = True
            for rid in self.robots:
                if self.pos_t[rid] != self.goals[rid]:
                    all_reached = False
                    break
            if all_reached:
                return

            self.step_once(t)


def generate_random_scenario(
    graph: WaypointGraph,
    robot_ids: List[str],
    rng: random.Random,
    max_start_goal_tries: int,
) -> Tuple[Dict[str, str], Dict[str, str]]:
    """
    生成随机仿真场景。
    要求各机器人起点不同（避免 t=0 即违规），但目标航点可重叠。
    """
    nodes = graph.nodes()
    if len(nodes) < len(robot_ids):
        raise RuntimeError("Graph too small for scenario generation")

    # 选择不同的起点
    starts = rng.sample(nodes, k=len(robot_ids))
    start_pos = {rid: starts[i] for i, rid in enumerate(robot_ids)}

    goals: Dict[str, str] = {}
    for rid in robot_ids:
        for _ in range(max_start_goal_tries):
            g = rng.choice(nodes)
            if g != start_pos[rid]:
                goals[rid] = g
                break
        if rid not in goals:
            goals[rid] = start_pos[rid]
    return start_pos, goals


def main() -> int:
    """主函数：解析参数，加载地图，运行随机场景测试。"""
    parser = argparse.ArgumentParser(description="Offline invariant simulator")
    parser.add_argument(
        "--map-yaml",
        default="src/fleet_management_system/maps/map0/rmf_map0.yaml",
        help="Map YAML (with waypoints+connections)"
    )
    parser.add_argument("--robot-count", type=int, default=4,
                        help="Number of robots")
    parser.add_argument("--scenarios", type=int, default=50,
                        help="Number of random scenarios to test")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed")
    parser.add_argument("--max-steps", type=int, default=60,
                        help="Max discrete steps per scenario")
    parser.add_argument("--greedy-noise", type=float, default=0.1,
                        help="Explore noise for tie-breaking (0..1)")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    graph = load_rmf_map_yaml(args.map_yaml)

    robot_ids = [f"r{i}" for i in range(args.robot_count)]

    violations_found = False
    scenario_reports = []
    t0 = time.time()

    for si in range(args.scenarios):
        start_pos, goals = generate_random_scenario(
            graph, robot_ids, rng, max_start_goal_tries=500
        )
        sim = InvariantSimulator(
            graph=graph,
            robots=robot_ids,
            start_pos=start_pos,
            goals=goals,
            rng=rng,
            max_steps=args.max_steps,
            greedy_noise=args.greedy_noise,
        )
        sim.run()
        if sim.violations:
            violations_found = True
            v = sim.violations[0]
            print("\n=== VIOLATION ===")
            print(f"scenario={si} rule={v.rule} step={v.step} robots={v.robots}")
            print(f"detail={v.detail}")
            print(f"start_pos={start_pos}")
            print(f"goals={goals}")
            break

        scenario_reports.append(
            {
                "scenario": si,
                "deadlocks": sim.deadlocks,
                "all_reached": all(
                    sim.pos_t[rid] == sim.goals[rid] for rid in robot_ids
                ),
                "start_pos": start_pos,
                "goals": goals,
            }
        )

        if (si + 1) % 10 == 0:
            elapsed = time.time() - t0
            print(f"[progress] {si+1}/{args.scenarios} scenarios, "
                  f"elapsed={elapsed:.1f}s")

    elapsed = time.time() - t0
    print("\n=== SUMMARY ===")
    print(f"map={args.map_yaml}")
    print(f"robots={args.robot_count} scenarios={args.scenarios} "
          f"max_steps={args.max_steps}")
    print(f"seed={args.seed} greedy_noise={args.greedy_noise} "
          f"elapsed_sec={elapsed:.1f}")
    if violations_found:
        print("RESULT: FAILED (invariant violation detected)")
        return 1
    print("RESULT: PASSED (no invariant violations detected)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

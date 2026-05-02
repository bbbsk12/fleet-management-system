#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline route-following multi-robot invariant + best-policy test.

Differences from a naive greedy simulator:
  - Each robot precomputes a shortest path (route) to its goal.
  - At runtime, robots can only move to the *next waypoint on that route*.
  - If a move would break your invariants, the robot waits (holds).

This better matches "according to the route/航线走" scheduling semantics.

The script evaluates move-order policies (a proxy for "who yields") on:
  - Extreme handoff scenarios (wp_016 dead-end + wp_007 choke)
  - Plus optional random scenarios

It never visualizes anything; it checks hard invariants and reports reachability / deadlocks.
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
    return f"{a}<->{b}" if a <= b else f"{b}<->{a}"


class WaypointGraph:
    def __init__(self, waypoints: Dict[str, Dict]) -> None:
        self.waypoints = waypoints
        self.adj: Dict[str, List[str]] = {}
        for wid, wp in waypoints.items():
            conns = wp.get("connections") or []
            self.adj[wid] = [str(x) for x in conns]

    def nodes(self) -> List[str]:
        return list(self.waypoints.keys())

    def neighbors(self, wid: str) -> List[str]:
        return self.adj.get(wid, [])

    def has_node(self, wid: str) -> bool:
        return wid in self.waypoints


def load_rmf_map_yaml(path: str) -> WaypointGraph:
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
    # reconstruct
    cur = goal
    rev = [cur]
    while parent[cur] != "":
        cur = parent[cur]
        rev.append(cur)
    rev.reverse()
    return rev


def bfs_dist(graph: WaypointGraph, start: str) -> Dict[str, int]:
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
    rule: str
    detail: str
    step: int
    robots: List[str]


@dataclass
class Scenario:
    name: str
    start_pos: Dict[str, str]
    goals: Dict[str, str]


class RouteFollowingSimulator:
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
                # unreachable in this graph => keep a degenerate route to avoid crashing
                path = [st]
            self.route[rid] = path

        self.violations: List[Violation] = []
        self.deadlock_ticks = 0
        self.steps_taken: Optional[int] = None
        self.dist_to_goal: Dict[str, Dict[str, int]] = {
            rid: bfs_dist(self.graph, self.goals[rid]) for rid in self.robot_ids
        }

    def _move_order_robots(self, step: int) -> List[str]:
        def remaining_to_goal(rid: str) -> int:
            i = self.route_index[rid]
            return len(self.route[rid]) - 1 - i

        if self.move_order == "random":
            ids = list(self.robot_ids)
            self.rng.shuffle(ids)
            return ids
        if self.move_order == "near_first":
            return sorted(self.robot_ids, key=lambda rid: (remaining_to_goal(rid), rid))
        if self.move_order == "far_first":
            return sorted(self.robot_ids, key=lambda rid: (-remaining_to_goal(rid), rid))

        raise RuntimeError(f"unknown move_order={self.move_order}")

    def _is_move_feasible(
        self,
        rid: str,
        cur: str,
        nxt: str,
        reserved_next_wp: Set[str],
        reserved_edges: Set[str],
    ) -> bool:
        if nxt in reserved_next_wp:
            return False
        ek = _edge_key(cur, nxt)
        if ek in reserved_edges:
            return False

        # Rule 3: if any other robot is at either endpoint at time t, cannot traverse this edge.
        endpoints = {cur, nxt}
        for other in self.robot_ids:
            if other == rid:
                continue
            if self.pos_t[other] in endpoints:
                return False
        return True

    def _check_invariants(self, step: int, pos_t1: Dict[str, str], traversed_edges: Dict[str, Optional[str]]) -> None:
        # Rule 1: unique waypoint at t+1
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

        # Rule 2: unique edge traversal at t->t+1
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

        # Rule 3: node-edge overlap
        # For each moving robot edge (a<->b), no other robot may be at waypoint a or b at time t.
        # (pos_t is time t)
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
                            detail=f"move={rid_move} traverses {ek} while {rid_other} at wp={wp_other_t} at time t={step}",
                            step=step,
                            robots=sorted([rid_move, rid_other]),
                        )
                    )
                    return

    def step_once(self, step: int) -> bool:
        reserved_next_wp: Set[str] = set()
        reserved_edges: Set[str] = set()

        pos_t1: Dict[str, str] = dict(self.pos_t)
        traversed_edges: Dict[str, Optional[str]] = {rid: None for rid in self.robot_ids}
        moved_any = False

        order = self._move_order_robots(step)
        for rid in order:
            cur = self.pos_t[rid]
            goal = self.goals[rid]
            if cur == goal:
                pos_t1[rid] = cur
                continue

            # Next hop strictly on precomputed route.
            i = self.route_index[rid]
            if i + 1 >= len(self.route[rid]):
                # already at end of planned route => wait
                pos_t1[rid] = cur
                continue
            desired_nxt = self.route[rid][i + 1]

            # 1) Try the desired hop along the route.
            if self._is_move_feasible(rid, cur, desired_nxt, reserved_next_wp, reserved_edges):
                edge_key = _edge_key(cur, desired_nxt)
                pos_t1[rid] = desired_nxt
                traversed_edges[rid] = edge_key
                reserved_next_wp.add(desired_nxt)
                reserved_edges.add(edge_key)
                moved_any = True
                # advance route index
                self.route_index[rid] = i + 1
                continue

            # 2) If blocked: optionally escape by choosing another feasible neighbor.
            if not self.allow_escape_on_block:
                continue

            candidates = [nb for nb in self.graph.neighbors(cur) if nb != cur]
            feasible: List[str] = [
                nb for nb in candidates
                if self._is_move_feasible(rid, cur, nb, reserved_next_wp, reserved_edges)
            ]
            if not feasible:
                continue

            best_nb: Optional[str] = None
            best_score: Optional[float] = None
            for nb in feasible:
                # primary: progress towards goal
                d_goal = self.dist_to_goal[rid].get(nb, 10**9)
                # secondary: separation from other robots at time t
                dist_from_nb = bfs_dist(self.graph, nb)
                other_wps = [self.pos_t[o] for o in self.robot_ids if o != rid]
                min_sep = min((dist_from_nb.get(wp, 0) for wp in other_wps), default=0)
                # score: prefer smaller d_goal and larger separation
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

            # If escape lands on planned route, snap index forward to that point.
            try:
                next_i = self.route[rid].index(best_nb)
                if next_i > self.route_index[rid]:
                    self.route_index[rid] = next_i
            except ValueError:
                pass

        # Post-commit invariant validation (should always pass by construction).
        self._check_invariants(step, pos_t1, traversed_edges)
        self.pos_t = pos_t1
        return moved_any

    def run(self) -> bool:
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


def build_extreme_scenarios(graph: WaypointGraph, robot_ids: Sequence[str]) -> List[Scenario]:
    """
    Hardcode scenarios around the known dead-end + choke:
      - wp_016 is a dead-end connected to wp_007
      - wp_007 is a choke connected to wp_001/wp_002/wp_016
    """
    ids = list(robot_ids)
    if not (graph.has_node("wp_007") and graph.has_node("wp_016")):
        return []

    # Helper: pick a random unused node for extra robots
    nodes = graph.nodes()
    def fill_random(extra_ids: Sequence[str], used: Set[str], rng: random.Random) -> Dict[str, str]:
        # Keep extra robots away from the choke/dead-end core so we test the intended handoff.
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

    # We'll keep goals deterministic for the core robots and fill others randomly later.
    scenarios: List[Scenario] = []
    rng = random.Random(12345)

    # Case A: your dead-end handoff
    # A at wp_016 wants to go out to wp_002; B at wp_007 wants to enter wp_016.
    core_start = {"A": "wp_016", "B": "wp_007"}
    if graph.has_node("wp_002"):
        core_goals = {"A": "wp_002", "B": "wp_016"}
    else:
        core_goals = {"A": "wp_001", "B": "wp_016"}
    used_nodes = {core_start["A"], core_start["B"]}
    start_pos: Dict[str, str] = {}
    goals: Dict[str, str] = {}
    # Map ids[0]->A, ids[1]->B, rest fill randomly.
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
                # random goal distinct from its start sometimes
                avail_goals = [n for n in nodes if n != start_pos[erid]]
                goals[erid] = rng.choice(avail_goals) if avail_goals else start_pos[erid]
        scenarios.append(Scenario(name="ext_dead_end_handoff_wp016_wp007", start_pos=start_pos, goals=goals))

    # Case B: symmetric exit side
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
        scenarios.append(Scenario(name="ext_dead_end_handoff_exit_wp001", start_pos=start_pos, goals=goals))

    # Case C: edge swap on wp_002 <-> wp_007 (if available)
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
        scenarios.append(Scenario(name="ext_edge_swap_wp002_wp007", start_pos=start_pos, goals=goals))

    return scenarios


def generate_random_scenarios(
    graph: WaypointGraph,
    robot_ids: Sequence[str],
    scenarios: int,
    rng: random.Random,
    max_start_goal_tries: int,
    unique_goals: bool,
) -> List[Scenario]:
    nodes = graph.nodes()
    out: List[Scenario] = []
    for si in range(scenarios):
        if len(nodes) < len(robot_ids):
            break
        starts = rng.sample(nodes, k=len(robot_ids))
        start_pos = {rid: starts[i] for i, rid in enumerate(robot_ids)}
        goals: Dict[str, str] = {}
        if unique_goals:
            # Pick distinct goals for each robot.
            # Prefer goals different from each robot's start when possible.
            available_goals = [n for n in nodes if n not in set(start_pos.values())]
            if len(available_goals) >= len(robot_ids):
                chosen_goals = rng.sample(available_goals, k=len(robot_ids))
            else:
                # fallback: allow overlap with starts but still keep goals unique
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
    Returns:
      (violations, failed_count, avg_steps_taken_on_success, avg_deadlock_ticks)
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
            # Invariant violations should not happen; record and stop early.
            v = sim.violations[0]
            print(f"[VIOLATION] policy={policy} scenario={sc.name} rule={v.rule} step={v.step} robots={v.robots}", file=sys.stderr)
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
    parser = argparse.ArgumentParser(description="Offline route-following extreme simulator")
    parser.add_argument("--map-yaml", default="src/fleet_management_system/maps/map0/rmf_map0.yaml")
    parser.add_argument("--robot-count", type=int, default=4)
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--random-scenarios", type=int, default=50)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--policy", default="auto_best_extreme", help="far_first|near_first|random|auto_best_extreme")
    parser.add_argument("--unique-goals", action="store_true", default=True, help="Generate random scenarios with unique terminal waypoints")
    parser.add_argument("--no-unique-goals", action="store_false", dest="unique_goals")
    args = parser.parse_args()

    graph = load_rmf_map_yaml(args.map_yaml)
    if len(graph.nodes()) < args.robot_count:
        print("ERROR: map has too few waypoints for requested robot-count", file=sys.stderr)
        return 2

    robot_ids = [f"r{i}" for i in range(args.robot_count)]
    rng = random.Random(args.seed)

    extreme_scenarios = build_extreme_scenarios(graph, robot_ids)
    random_scenarios = generate_random_scenarios(
        graph, robot_ids, args.random_scenarios, rng, max_start_goal_tries=200, unique_goals=args.unique_goals
    )

    scenarios_all = extreme_scenarios + random_scenarios
    print(f"[bootstrap] map={args.map_yaml} robots={args.robot_count} extreme={len(extreme_scenarios)} random={len(random_scenarios)} total={len(scenarios_all)}")
    if not scenarios_all:
        print("ERROR: no scenarios generated", file=sys.stderr)
        return 2

    candidate_policies = ["far_first", "near_first", "random"]
    selected_policy = args.policy
    if args.policy == "auto_best_extreme":
        # Evaluate only extreme scenarios for "best scheduling".
        if not extreme_scenarios:
            selected_policy = "far_first"
        else:
            best = None
            best_tuple = None
            for pol in ["far_first", "near_first"]:
                violations, failed, avg_steps, avg_deadlocks = evaluate_policy_on_scenarios(
                    graph, robot_ids, extreme_scenarios, pol, seed=args.seed, max_steps=args.max_steps
                )
                # Prefer: fewer failed, then fewer deadlocks, then shorter steps.
                t = (failed, avg_deadlocks, avg_steps)
                print(f"[ext-eval] policy={pol} failed={failed}/{len(extreme_scenarios)} avg_deadlocks={avg_deadlocks:.1f} avg_steps_success={avg_steps:.1f}")
                if best is None or t < best_tuple:
                    best = pol
                    best_tuple = t
            selected_policy = best or "far_first"
            print(f"[ext-eval] selected best policy={selected_policy}")

    violations, failed, avg_steps, avg_deadlocks = evaluate_policy_on_scenarios(
        graph, robot_ids, scenarios_all, selected_policy, seed=args.seed, max_steps=args.max_steps
    )
    print(f"[result] policy={selected_policy} violations={violations} failed={failed}/{len(scenarios_all)} avg_deadlocks={avg_deadlocks:.1f} avg_steps_success={avg_steps:.1f}")
    return 1 if violations else 0


if __name__ == "__main__":
    raise SystemExit(main())


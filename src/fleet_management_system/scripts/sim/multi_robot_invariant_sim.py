#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline multi-robot invariant simulator (no ROS, no Nav2).

It simulates robots moving on the waypoint graph extracted from the project's map YAML
and enforces the user's 3 hard safety invariants at every discrete step:
  1) No two robots at the same waypoint at the same timestep.
  2) No two robots traversing the same edge (route segment) at the same timestep.
  3) If one robot is at waypoint W (at time t), no other robot may traverse any edge
     whose endpoints contain W during t->t+1. (node-edge overlap)

This script is meant to validate that a candidate dispatch policy can be implemented
without producing scheduling invariant violations.
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
    return f"{a}<->{b}" if a <= b else f"{b}<->{a}"


@dataclass
class StepState:
    t: int
    pos_t: Dict[str, str]
    pos_t1: Dict[str, str]
    traversed_edges: Dict[str, Optional[str]]  # robot_id -> edge_key (undirected) or None


@dataclass
class Violation:
    rule: str
    detail: str
    step: int
    robots: List[str]


class WaypointGraph:
    def __init__(self, waypoints: Dict[str, Dict]) -> None:
        self.waypoints = waypoints
        self.adj: Dict[str, List[str]] = {wid: [] for wid in waypoints.keys()}
        for wid, wp in waypoints.items():
            conns = wp.get("connections") or []
            self.adj[wid] = [str(x) for x in conns]

    def nodes(self) -> List[str]:
        return list(self.waypoints.keys())

    def neighbors(self, wid: str) -> List[str]:
        return self.adj.get(wid, [])


def load_rmf_map_yaml(path: str) -> WaypointGraph:
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
    q: List[str] = [start]
    dist: Dict[str, int] = {start: 0}
    for cur in q:
        for nb in graph.neighbors(cur):
            if nb not in dist:
                dist[nb] = dist[cur] + 1
                q.append(nb)
    return dist


def shortest_next_hop(graph: WaypointGraph, cur: str, goal: str, dist_to_goal: Dict[str, int]) -> Optional[str]:
    """
    Choose next waypoint among neighbors that reduces distance-to-goal.
    Return None if no neighbor helps (caller will make robot wait).
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

        # Precompute distance-to-goal for each robot for the greedy policy.
        self.dist_to_goal: Dict[str, Dict[str, int]] = {rid: bfs_dist(graph, goals[rid]) for rid in robots}

    def _check_step_invariants(self, step: int, pos_t: Dict[str, str], pos_t1: Dict[str, str], traversed_edges: Dict[str, Optional[str]]) -> None:
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

        # Rule 3: node-edge overlap (at time t, if any robot at W, no other traverses edge incident to W)
        # Implemented strictly with current positions at time t.
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
                            detail=f"move={rid_move} traverses {ek} while {rid_other} is at waypoint {wp_other_t} at time t={step}",
                            step=step,
                            robots=sorted([rid_move, rid_other]),
                        )
                    )
                    return

    def step_once(self, step: int) -> bool:
        """
        Return True if at least one robot moved.
        """
        # Priority: robots with larger remaining distance get earlier picks.
        def remaining_dist(rid: str) -> int:
            dmap = self.dist_to_goal.get(rid) or {}
            return dmap.get(self.pos_t[rid], 999999)

        # Add small noise to avoid deterministic starvation in corner graphs.
        ordered = sorted(
            self.robots,
            key=lambda rid: (-remaining_dist(rid), rid),
        )

        # Plan sequentially with reservation sets.
        reserved_next_wp: Set[str] = set()
        reserved_edges: Set[str] = set()
        forbidden_next_wp: Set[str] = set()  # endpoints of already-chosen traversed edges

        pos_t1: Dict[str, str] = dict(self.pos_t)
        traversed_edges: Dict[str, Optional[str]] = {rid: None for rid in self.robots}

        moved_any = False

        for rid in ordered:
            cur = self.pos_t[rid]
            goal = self.goals[rid]
            if cur == goal:
                pos_t1[rid] = cur
                continue

            # Greedy candidate list: neighbors that best reduce distance-to-goal first.
            dist_map = self.dist_to_goal.get(rid) or {}
            # Build candidate list (neighbors + wait)
            candidates = list(self.graph.neighbors(cur))
            # If greedy can't find a reducing move, still consider all neighbors.
            candidates_sorted = sorted(
                candidates,
                key=lambda nb: (dist_map.get(nb, 10**9), nb),
            )

            # Inject noise: sometimes pick 2nd best to explore.
            if candidates_sorted and self.greedy_noise > 0:
                if self.rng.random() < self.greedy_noise and len(candidates_sorted) > 1:
                    candidates_sorted = [candidates_sorted[1]] + candidates_sorted[0:1] + candidates_sorted[2:]

            # Consider wait first if waiting can reduce conflicts.
            considered = [cur] + candidates_sorted

            chosen: Optional[str] = None
            chosen_edge: Optional[str] = None

            for nb in considered:
                if nb == cur:
                    # waiting is always a valid vertex occupancy, but may violate rule 3 if someone else traverses incident edge,
                    # which is checked later against others' current positions (strict) and already chosen moves.
                    # waiting itself does not reserve an edge.
                    if nb in reserved_next_wp:
                        continue
                    if nb in forbidden_next_wp:
                        continue
                    chosen = nb
                    chosen_edge = None
                    break

                edge_key = _edge_key(cur, nb)
                # Rule 2: edge reservation uniqueness
                if edge_key in reserved_edges:
                    continue
                # Rule 1: next-vertex uniqueness
                if nb in reserved_next_wp:
                    continue
                # Rule 3, part B: if nb is endpoint of an already-chosen moving edge, forbid next occupancy there
                if nb in forbidden_next_wp:
                    continue
                # Rule 3, part A: strict node-edge overlap with other robots' current positions at time t:
                # if any other robot currently sits on either endpoint, cannot traverse.
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

                # If nb is ok, accept.
                chosen = nb
                chosen_edge = edge_key
                break

            if chosen is None:
                # No feasible move => wait at cur
                chosen = cur
                chosen_edge = None

            # Commit plan for rid
            pos_t1[rid] = chosen
            if chosen_edge:
                traversed_edges[rid] = chosen_edge
                reserved_edges.add(chosen_edge)
                forbidden_next_wp.add(cur)
                forbidden_next_wp.add(chosen)
                moved_any = True
            reserved_next_wp.add(chosen)

        # Verify invariants
        self._check_step_invariants(step, self.pos_t, pos_t1, traversed_edges)
        self.pos_t = pos_t1

        if not moved_any:
            self.deadlocks += 1
        return moved_any

    def run(self) -> None:
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
    nodes = graph.nodes()
    # We require unique start waypoints (to avoid immediate invariant violations at t=0),
    # but goals can overlap across robots (invariants will catch any hard conflicts later).
    if len(nodes) < len(robot_ids):
        raise RuntimeError("Graph too small for scenario generation")

    # Pick unique starts
    starts = rng.sample(nodes, k=len(robot_ids))
    start_pos = {rid: starts[i] for i, rid in enumerate(robot_ids)}

    goals: Dict[str, str] = {}
    for rid in robot_ids:
        # Ensure each goal differs from its start (and from other goals sometimes to create stress)
        for _ in range(max_start_goal_tries):
            g = rng.choice(nodes)
            if g != start_pos[rid]:
                goals[rid] = g
                break
        if rid not in goals:
            goals[rid] = start_pos[rid]
    return start_pos, goals


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline invariant simulator")
    parser.add_argument("--map-yaml", default="src/fleet_management_system/maps/map0/rmf_map0.yaml", help="Map YAML (with waypoints+connections)")
    parser.add_argument("--robot-count", type=int, default=4, help="Number of robots")
    parser.add_argument("--scenarios", type=int, default=50, help="Number of random scenarios to test")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--max-steps", type=int, default=60, help="Max discrete steps per scenario")
    parser.add_argument("--greedy-noise", type=float, default=0.1, help="Explore noise for tie-breaking (0..1)")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    graph = load_rmf_map_yaml(args.map_yaml)

    robot_ids = [f"r{i}" for i in range(args.robot_count)]

    violations_found = False
    scenario_reports = []
    t0 = time.time()

    for si in range(args.scenarios):
        start_pos, goals = generate_random_scenario(graph, robot_ids, rng, max_start_goal_tries=500)
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
                "all_reached": all(sim.pos_t[rid] == sim.goals[rid] for rid in robot_ids),
                "start_pos": start_pos,
                "goals": goals,
            }
        )

        if (si + 1) % 10 == 0:
            elapsed = time.time() - t0
            print(f"[progress] {si+1}/{args.scenarios} scenarios, elapsed={elapsed:.1f}s")

    elapsed = time.time() - t0
    print("\n=== SUMMARY ===")
    print(f"map={args.map_yaml}")
    print(f"robots={args.robot_count} scenarios={args.scenarios} max_steps={args.max_steps}")
    print(f"seed={args.seed} greedy_noise={args.greedy_noise} elapsed_sec={elapsed:.1f}")
    if violations_found:
        print("RESULT: FAILED (invariant violation detected)")
        return 1
    print("RESULT: PASSED (no invariant violations detected)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


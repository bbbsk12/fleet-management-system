# Fleet Logging Guide

## Goal

Unify runtime logs so operators can:
- Read scheduler state transitions directly from one line.
- Correlate robot/task/conflict/chassis events quickly.
- Locate root cause from `reason` and `detail` fields.

## Canonical Log Shape

Use this key order whenever possible:

`event=<name> task=<task_id|-> robot=<robot_id|-> state_prev=<state|-> state_new=<state|-> reason=<code|-> detail=<kv pairs>`

Examples:
- `event=task.assign task=task_... robot=robot1 state_prev=pending state_new=assigned reason=NEAREST_AVAILABLE detail=distance_m:1.24`
- `event=conflict.offline_block task=task_... robot=robot2 state_prev=in_progress state_new=failed reason=OFFLINE_BLOCKER_TIMEOUT detail=blocker:robot3 blocked_sec:5.2`
- `event=chassis.timeout task=task_... robot=robot1 state_prev=executing state_new=failed reason=EXEC_TIMEOUT detail=timeout_sec=30.0`

## Event Families

- `task.*`: submit/assign/defer/state_change/complete/fail/cancel
- `nav.*`: start/defer/goal_rejected/result/arrival_check
- `conflict.*`: offline_block/deadlock.resolve/avoidance decisions
- `hold.*`: apply/release/fail
- `chassis.*`: send/handshake/ack/complete/error/timeout
- `robot.*`: connection_change/remove/rejoin/offline_guard/online_recover
- `scheduler.*`: no_capacity/preempt/orphan recovery

## Reason Code Rules

- Use stable machine-readable codes in `reason`.
- Put free text only in `detail`.
- Keep reason codes short, uppercase, and action-oriented.

Suggested reasons:
- `NO_AVAILABLE_ROBOTS`
- `ROBOT_BUSY`
- `HARD_HOLD_ACTIVE`
- `NAV_SERVER_NOT_READY`
- `OFFLINE_BLOCKER_TIMEOUT`
- `DEADLOCK_CYCLE`
- `HANDSHAKE_TIMEOUT_EXHAUSTED`
- `EXEC_TIMEOUT`
- `ROBOT_REMOVED`

## Reason Lookup

- `NO_AVAILABLE_ROBOTS`: scheduler sees pending tasks but no eligible robot.
- `ROBOT_BUSY`: robot has active goal/internal nav state and cannot accept new task.
- `HARD_HOLD_ACTIVE`: robot is intentionally paused by hold policy.
- `NAV_SERVER_NOT_READY`: Nav2 action server unavailable at dispatch time.
- `OFFLINE_BLOCKER_TIMEOUT`: task blocked by offline robot beyond timeout.
- `DEADLOCK_CYCLE`: cyclic blocking detected; one robot is selected to yield.
- `HANDSHAKE_TIMEOUT_EXHAUSTED`: chassis handshake retries exhausted.
- `EXEC_TIMEOUT`: chassis execution timeout after handshake success.
- `ROBOT_REMOVED`: task failed because robot was removed from fleet.

## Quick Query Patterns

- Task timeline: search `task=<task_id>`
- Robot timeline: search `robot=<robot_id>`
- Failures only: search `state_new=failed`
- Defer loops: search `event=task.defer` and `event=nav.defer`
- Offline cascades: search `reason=OFFLINE_BLOCKER_TIMEOUT` or `event=robot.connection_change`

## One-Command Tracing

Use `src/fleet_management_system/scripts/tools/log_trace.sh`:

- `log_trace.sh task <task_id>`: full task timeline + fail/defer hotspots
- `log_trace.sh robot <robot_id>`: full robot timeline + conflict/offline hotspots
- `log_trace.sh failed`: global failures
- `log_trace.sh chassis`: chassis pipeline events
- `log_trace.sh deadlock`: deadlock/conflict events

Optional third arg overrides log glob:
- `log_trace.sh task <task_id> "test_logs/fleet_manager_*.log"`

Time window filter:
- `log_trace.sh --since 10m task <task_id>`
- `log_trace.sh --since 2h robot <robot_id>`
- `--since` supports `s/m/h/d` suffix, e.g. `30s`, `15m`, `6h`, `1d`.

## Troubleshooting Flow

For a stuck or failed task:
1. Search by `task=<id>` and locate latest `task.state_change`.
2. If `state_new=waiting_fleet` or repeated `task.defer`, inspect matching `hold.*` and `conflict.*`.
3. If `state_new=executing`, inspect `chassis.*` for handshake/exec timeout.
4. If no progress and robot offline transitions exist, inspect `robot.connection_change` and `robot.offline_guard`.

For a blocked robot:
1. Search `robot=<id>`.
2. Check latest `nav.start` and nearest `conflict.*`.
3. If deadlock appears, inspect `deadlock.resolve`.

## Implementation Notes

- ROS C++ logs use `RCLCPP_*` with structured message body.
- Persistent diagnostics use `PersistLogger` in parallel for long-run trace.
- Web backend logs mirror the same `event` style to align with ROS-side troubleshooting.

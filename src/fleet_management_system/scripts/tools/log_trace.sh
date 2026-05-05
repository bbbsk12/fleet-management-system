#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  log_trace.sh [--since <window>] task <task_id> [log_glob]
  log_trace.sh [--since <window>] robot <robot_id> [log_glob]
  log_trace.sh [--since <window>] failed [log_glob]
  log_trace.sh [--since <window>] chassis [log_glob]
  log_trace.sh [--since <window>] deadlock [log_glob]

Examples:
  log_trace.sh task task_1713341111222_0
  log_trace.sh --since 10m task task_1713341111222_0
  log_trace.sh --since 2h robot robot_1
  log_trace.sh robot robot_1 "test_logs/fleet_manager_*.log"
  log_trace.sh failed

Notes:
  - Run from repository root.
  - Default log glob: test_logs/*.log
  - --since supports: 30s / 10m / 2h / 1d
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

SINCE_WINDOW=""
if [[ "${1:-}" == "--since" ]]; then
  if [[ $# -lt 3 ]]; then
    echo "--since requires a value (e.g. 10m)" >&2
    exit 1
  fi
  SINCE_WINDOW="$2"
  shift 2
fi

MODE="${1:-}"
KEY="${2:-}"
LOG_GLOB="${3:-test_logs/*.log}"

shopt -s nullglob
LOG_FILES=( $LOG_GLOB )
shopt -u nullglob

if [[ ${#LOG_FILES[@]} -eq 0 ]]; then
  echo "No log files matched: $LOG_GLOB" >&2
  exit 1
fi

run_rg() {
  local pattern="$1"
  if command -v rg >/dev/null 2>&1; then
    rg -n --no-heading "$pattern" "${LOG_FILES[@]}" || true
  else
    grep -En "$pattern" "${LOG_FILES[@]}" || true
  fi
}

parse_since_seconds() {
  local value="$1"
  if [[ "$value" =~ ^([0-9]+)([smhd])$ ]]; then
    local n="${BASH_REMATCH[1]}"
    local unit="${BASH_REMATCH[2]}"
    case "$unit" in
      s) echo "$n" ;;
      m) echo $((n * 60)) ;;
      h) echo $((n * 3600)) ;;
      d) echo $((n * 86400)) ;;
      *) return 1 ;;
    esac
  else
    return 1
  fi
}

apply_since_filter() {
  local cutoff_epoch="$1"
  awk -v cutoff="$cutoff_epoch" '
  function ts_to_epoch(ts,    base, dt, parts) {
    # ts format: YYYY-MM-DDTHH:MM:SS.mmmZ
    base = substr(ts, 1, 19)
    gsub("T", " ", base)
    split(base, dt, /[- :]/)
    return mktime(dt[1] " " dt[2] " " dt[3] " " dt[4] " " dt[5] " " dt[6])
  }
  {
    if (match($0, /[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{3}Z/)) {
      ts = substr($0, RSTART, RLENGTH)
      if (ts_to_epoch(ts) >= cutoff) {
        print $0
      }
    } else {
      # If no timestamp in line, keep it to avoid dropping context lines
      print $0
    }
  }'
}

print_header() {
  echo "==== $1 ===="
}

run_query() {
  local pattern="$1"
  if [[ -n "$SINCE_WINDOW" ]]; then
    local since_seconds
    since_seconds="$(parse_since_seconds "$SINCE_WINDOW")" || {
      echo "Invalid --since value: $SINCE_WINDOW (expected 30s/10m/2h/1d)" >&2
      exit 1
    }
    local now_epoch cutoff_epoch
    now_epoch="$(date +%s)"
    cutoff_epoch=$((now_epoch - since_seconds))
    run_rg "$pattern" | apply_since_filter "$cutoff_epoch"
  else
    run_rg "$pattern"
  fi
}

case "$MODE" in
  task)
    if [[ -z "$KEY" ]]; then
      echo "task mode requires <task_id>" >&2
      exit 1
    fi
    print_header "TASK TIMELINE: $KEY"
    run_query "task=${KEY}"
    print_header "TASK FAIL/DEFER HOTSPOTS: $KEY"
    run_query "task=${KEY}.*(state_new=failed|event=task\\.fail|event=chassis\\.(error|timeout)|event=(task|nav)\\.defer|event=hold\\.)"
    ;;
  robot)
    if [[ -z "$KEY" ]]; then
      echo "robot mode requires <robot_id>" >&2
      exit 1
    fi
    print_header "ROBOT TIMELINE: $KEY"
    run_query "robot=${KEY}"
    print_header "ROBOT BLOCK/OFFLINE HOTSPOTS: $KEY"
    run_query "robot=${KEY}.*(event=conflict\\.|event=deadlock\\.resolve|event=robot\\.|reason=OFFLINE_BLOCKER_TIMEOUT)"
    ;;
  failed)
    print_header "GLOBAL FAILURES"
    run_query "(state_new=failed|event=task\\.fail|event=chassis\\.(error|timeout))"
    ;;
  chassis)
    print_header "CHASSIS PIPELINE"
    run_query "event=chassis\\.(send|handshake|ack|complete|error|timeout)"
    ;;
  deadlock)
    print_header "DEADLOCK/CONFLICT"
    run_query "event=(deadlock\\.resolve|conflict\\.)"
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown mode: $MODE" >&2
    usage
    exit 1
    ;;
esac

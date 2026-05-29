#!/usr/bin/env bash
# run.sh — run a scenario against an ALREADY-RUNNING stack, then clean up.
#
# Usage:
#   ./run.sh scenarios/smoke.yaml [extra harness launch args...]
#
# Assumes the Gazebo sim (simulation/bringup.launch.py) and the nav stack
# (nav_bringup/emmn.launch.py sim:=true) are already up — bring them up yourself
# first. This script does NOT launch or tear down the sim/nav.
#
# Steps:
#   1. read world_name from the scenario YAML (selects the create service)
#   2. launch nav_sim_harness/harness.launch.py — just the /world/<name>/create
#      service bridge the spawner needs — in the background
#   3. run the scenario through `python3 -m nav_sim_harness.runner`
#   4. tear down only the bridge launch (the sim/nav are left running) on exit
#
# Exit code mirrors the runner: 0 = all steps passed, non-zero otherwise.
set -uo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <scenario.yaml> [--gui] [extra launch args...]" >&2
  exit 2
fi

SCENARIO="$1"; shift
if [[ ! -f "$SCENARIO" ]]; then
  echo "scenario not found: $SCENARIO" >&2
  exit 2
fi

EXTRA_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --gui|--headless) ;;  # no-op: the sim is launched separately now
    *)                EXTRA_ARGS+=("$arg") ;;
  esac
done

# Pull world_name straight from the scenario so the create-service bridge targets
# the same world the runner spawns into.
WORLD_NAME=$(python3 - "$SCENARIO" <<'PY'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1])) or {}
print(d.get("world_name", "default"))
PY
)

echo "[harness] scenario=$SCENARIO world_name=$WORLD_NAME (assuming sim + nav already running)"

# Launch the create-service bridge in its own process group so we can kill it
# (and only it) cleanly — the sim/nav stack is left untouched.
setsid ros2 launch nav_sim_harness harness.launch.py \
  world_name:="$WORLD_NAME" \
  "${EXTRA_ARGS[@]}" &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= "$LAUNCH_PID" 2>/dev/null | tr -d ' ')

teardown() {
  echo "[harness] tearing down bridge (launch pid=$LAUNCH_PID pgid=${LAUNCH_PGID:-?})"
  if [[ -n "${LAUNCH_PGID:-}" ]]; then
    kill -INT -- "-$LAUNCH_PGID" 2>/dev/null || true
    sleep 3
    kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null || true
  else
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
  fi
  # NOTE: the sim/nav stack is externally managed — we deliberately do NOT kill
  # Gazebo or the nav nodes here.
}
trap teardown EXIT INT TERM

# The runner blocks on the action server + /nav_status before doing anything,
# so no fixed sleep is needed here.
python3 -m nav_sim_harness.runner "$SCENARIO"
RC=$?

echo "[harness] runner exit code: $RC"
exit $RC

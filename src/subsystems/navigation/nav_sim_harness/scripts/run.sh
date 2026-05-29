#!/usr/bin/env bash
# run.sh — launch the full nav harness, run a scenario, then tear everything down.
#
# Usage:
#   ./run.sh scenarios/smoke.yaml [--gui] [extra harness launch args...]
#
# Steps:
#   1. read world / world_name from the scenario YAML
#   2. launch nav_sim_harness/harness.launch.py in the background (headless by default)
#   3. run the scenario through `python3 -m nav_sim_harness.runner`
#   4. tear down the launch (and the whole process group) on exit
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

HEADLESS=true
EXTRA_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --gui)      HEADLESS=false ;;
    --headless) HEADLESS=true ;;
    *)          EXTRA_ARGS+=("$arg") ;;
  esac
done

# Pull world/world_name straight from the scenario so the launched world matches
# what the runner spawns into.
read -r WORLD WORLD_NAME < <(python3 - "$SCENARIO" <<'PY'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1])) or {}
print(d.get("world", "terrain_world.sdf"), d.get("world_name", "default"))
PY
)

echo "[harness] scenario=$SCENARIO world=$WORLD world_name=$WORLD_NAME headless=$HEADLESS"

# Launch the stack in its own process group so we can kill the whole tree.
setsid ros2 launch nav_sim_harness harness.launch.py \
  world:="$WORLD" world_name:="$WORLD_NAME" headless:="$HEADLESS" \
  "${EXTRA_ARGS[@]}" &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= "$LAUNCH_PID" 2>/dev/null | tr -d ' ')

teardown() {
  echo "[harness] tearing down (launch pid=$LAUNCH_PID pgid=${LAUNCH_PGID:-?})"
  if [[ -n "${LAUNCH_PGID:-}" ]]; then
    kill -INT -- "-$LAUNCH_PGID" 2>/dev/null || true
    sleep 3
    kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null || true
  else
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
  fi
  # Gazebo can linger; make sure it's gone.
  pkill -f 'gz sim' 2>/dev/null || true
}
trap teardown EXIT INT TERM

# The runner blocks on the action server + /nav_status before doing anything,
# so no fixed sleep is needed here.
python3 -m nav_sim_harness.runner "$SCENARIO"
RC=$?

echo "[harness] runner exit code: $RC"
exit $RC

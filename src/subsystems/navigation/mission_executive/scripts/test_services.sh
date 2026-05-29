#!/usr/bin/env bash
# Smoke test for the mission_executive queue/services rework.
# Run inside the devcontainer with the workspace sourced.
set -u

PASS=0
FAIL=0
ok()   { echo "  [PASS] $*"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }
step() { echo; echo "=== $* ==="; }

CACHE="${HOME}/.athena/test_waypoints.json"
rm -f "$CACHE"

# Wait until the cache file has settled to a state that satisfies the given
# python predicate (receives the parsed JSON as `q`). Polling avoids races
# between service-call return and the persist+publish that follows.
wait_cache() {
  local pred=$1 deadline=$(( SECONDS + 5 ))
  while (( SECONDS < deadline )); do
    if [[ -f "$CACHE" ]] && python3 -c "
import json, sys
try:
    q = json.load(open('$CACHE'))
except Exception:
    sys.exit(1)
sys.exit(0 if ($pred) else 1)
" 2>/dev/null; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

dump_cache() { python3 -c "import json; print(json.dumps(json.load(open('$CACHE')), indent=2))"; }

cleanup() {
  [[ -n "${ECHO_PID:-}" ]] && kill "$ECHO_PID" 2>/dev/null || true
  # `ros2 run` exec's the node so the pgid kill is more reliable than the bg pid
  pkill -9 -x mission_executive_node 2>/dev/null || true
  pkill -9 -x static_transform_publisher 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

# Pre-clean: don't race with leftover nodes from a previous failed run
pkill -9 -x mission_executive_node 2>/dev/null || true
pkill -9 -x static_transform_publisher 2>/dev/null || true
sleep 0.3

start_node() {
  ros2 run mission_executive mission_executive_node --ros-args \
    -p waypoint_cache_path:="$CACHE" \
    -p replan_throttle_s:=10.0 \
    >/tmp/me.log 2>&1 &
  NODE_PID=$!
  for _ in $(seq 1 50); do
    ros2 service list 2>/dev/null | grep -q '/mission_executive/advance' && return 0
    sleep 0.2
  done
  return 1
}

step "Start static map->base_link TF + mission_executive"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link \
  >/tmp/tf.log 2>&1 &
TF_PID=$!

start_node \
  && ok "node up, services registered" \
  || { bad "node never came up"; cat /tmp/me.log; exit 1; }

step "Add three waypoints via ~/set_target (insertion order: far, near, mid)"
add_target() {
  local id=$1 x=$2 y=$3
  ros2 service call /mission_executive/set_target msgs/srv/SetTarget \
    "{target_id: '$id', goal_type: 1, target_type: 0, x_m: $x, y_m: $y, tolerance_m: 2.0}" \
    >/dev/null
}
add_target wp_far  50.0 0.0
add_target wp_near  5.0 0.0
add_target wp_mid  20.0 0.0

if wait_cache "len(q)==3 and all(e['visited']==False and e['skipped']==False for e in q)"; then
  python3 -c "
import json
q = json.load(open('$CACHE'))
ids = [e['id'] for e in q]
assert ids == ['wp_near','wp_mid','wp_far'], f'expected near,mid,far got {ids}'
print('  cache order:', ids)
" && ok "queue auto-sorted by distance from robot (0,0)" || bad "queue order wrong: $(dump_cache)"
else
  bad "cache never reached 3-pending state"; dump_cache
fi

step "~/advance: pops wp_near, marks visited=false, becomes active"
ros2 service call /mission_executive/advance std_srvs/srv/Trigger >/dev/null
# Active target isn't reflected in the cache's visited/skipped fields, so check
# the latched /waypoint_queue topic by parsing the status array. Use python to
# decode the YAML-quoted JSON robustly.
read_queue_topic() {
  timeout 3 ros2 topic echo --once /waypoint_queue std_msgs/msg/String 2>/dev/null \
    | python3 -c "
import sys, yaml
for doc in yaml.safe_load_all(sys.stdin.read()):
    if isinstance(doc, dict) and 'data' in doc:
        print(doc['data']); break
"
}

check_active() {
  local want=$1
  for _ in $(seq 1 20); do
    if read_queue_topic | WANT="$want" python3 -c "
import json, os, sys
data = sys.stdin.read().strip()
if not data: sys.exit(2)
q = json.loads(data)
active = [e for e in q if e['status']=='ACTIVE']
sys.exit(0 if (len(active)==1 and active[0]['id']==os.environ['WANT']) else 1)
"; then return 0; fi
    sleep 0.2
  done
  return 1
}

check_active wp_near && ok "wp_near is ACTIVE in /waypoint_queue" || bad "wp_near not ACTIVE: $(read_queue_topic)"

step "~/skip: wp_near becomes SKIPPED, no ACTIVE in queue"
ros2 service call /mission_executive/skip std_srvs/srv/Trigger >/dev/null
if wait_cache "next(e for e in q if e['id']=='wp_near')['skipped']==True"; then
  ok "wp_near skipped=true in cache"
else
  bad "wp_near not marked skipped: $(dump_cache)"
fi

step "~/advance again: should pop wp_mid (next pending by distance)"
echo "  cache BEFORE 2nd advance:"; python3 -c "import json; print(' ', [(e['id'],e['visited'],e['skipped']) for e in json.load(open('$CACHE'))])"
ros2 service call /mission_executive/advance std_srvs/srv/Trigger >/dev/null
sleep 0.5
echo "  cache AFTER  2nd advance:"; python3 -c "import json; print(' ', [(e['id'],e['visited'],e['skipped']) for e in json.load(open('$CACHE'))])"
check_active wp_mid && ok "wp_mid is ACTIVE" || bad "wp_mid not ACTIVE: $(read_queue_topic)"

step "Replan on /local_planner/stuck rising edge (throttle=10s)"
# Echo goal_pose stamps; count one line per emitted message via the '---' separator.
( ros2 topic echo /goal_pose --field header.stamp.sec >/tmp/goals.log 2>&1 ) &
ECHO_PID=$!
sleep 1.0
count_goals() { grep -c '^---' /tmp/goals.log || true; }
BEFORE=$(count_goals)

publish_stuck() {
  local v=$1
  ros2 topic pub --once /local_planner/stuck msgs/msg/LocalPlannerStuck \
    "{header: {frame_id: 'map'}, stuck: $v, best_forward_clearance_m: 0.0, best_reverse_clearance_m: 0.0}" \
    >/dev/null 2>&1
}

publish_stuck false; sleep 0.3
publish_stuck true;  sleep 0.7   # rising edge -> 1 republish
RISE1=$(count_goals)
publish_stuck true;  sleep 0.7   # still stuck, not an edge
NOEDGE=$(count_goals)
publish_stuck false; sleep 0.3
publish_stuck true;  sleep 0.7   # rising edge but within 10s throttle
THROTTLED=$(count_goals)

kill $ECHO_PID 2>/dev/null || true
ECHO_PID=

echo "  goal counts: before=$BEFORE rise1=$RISE1 noedge=$NOEDGE throttled=$THROTTLED"
echo "  --- /tmp/goals.log (last 40) ---"; tail -n 40 /tmp/goals.log; echo "  ---"
(( RISE1 > BEFORE ))    && ok "rising edge republished goal" || bad "rising edge did not republish"
(( NOEDGE == RISE1 ))   && ok "non-edge stuck=true did not republish" || bad "non-edge republished"
(( THROTTLED == RISE1 )) && ok "throttled rising edge suppressed (10s window)" || bad "throttle not honored"

step "Persistence across restart"
pkill -9 -x mission_executive_node 2>/dev/null || true
wait 2>/dev/null || true
NODE_PID=
sleep 0.5
start_node || { bad "node restart failed"; cat /tmp/me.log; exit 1; }

if wait_cache "len(q)==3"; then
  python3 -c "
import json
q = json.load(open('$CACHE'))
by = {e['id']: (e['visited'], e['skipped']) for e in q}
assert by['wp_near']==(False, True), by
assert by['wp_far'] ==(False, False), by
print('  restored:', by)
" && ok "queue restored from cache (wp_near skipped, wp_far pending)" \
  || bad "restart state wrong: $(dump_cache)"
else
  bad "cache disappeared on restart"
fi

step "Result"
echo "  PASS=$PASS  FAIL=$FAIL"
[[ $FAIL -eq 0 ]]

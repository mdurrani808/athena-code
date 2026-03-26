# Navigation Stack — Implementation Progress

## Legend
- [x] Done
- [ ] Not started

---

## Plan 1 — `gps_pose_publisher` ✅
- [x] New package: `gps_pose_publisher/`
- [x] WGS84 → ENU via `GeographicLib::LocalCartesian`
- [x] Publishes `/robot_pose` (PoseStamped, frame: map)
- [x] Broadcasts `map → base_link` TF
- [x] Publishes `/gps_status` (String)
- [x] Service `~/latlon_to_enu` (msgs/srv/LatLonToENU)
- [x] `use_start_gate_ref` param — waits for `/start_gate_ref` NavSatFix if true
- [x] Heading conversion: compass deg → ENU rad (`(90 - deg) * π/180`)
- [x] Only publishes once both GPS fix AND heading are available

## Plan 2 — `ackermann_mppi` modifications ✅
- [x] Input topic renamed: `/plan` → `/global_path` (transient_local QoS)
- [x] `nav_enabled_topic` param (default `/nav_enabled`)
- [x] Subscribe `std_msgs/Bool` on nav_enabled topic
- [x] `controlLoop()` short-circuits (zero cmd_vel, DEBUG log) when nav disabled
- [x] `std::atomic<bool> nav_enabled_` — thread-safe with MultiThreadedExecutor

## Plan 3 — `athena_smac_planner` + `global_planner` ✅
- [x] Package rename: `nav2_smac_planner` → `athena_smac_planner` (package.xml + CMakeLists.txt)
- [x] Library rename: `nav2_smac_planner` → `athena_smac_planner`
- [x] Removed nonexistent plugin XML exports from package.xml
- [x] `global_planner_node.cpp` — subscribes `/goal_pose`, `/robot_pose`
- [x] Owns `Costmap2DROS` under `"global_costmap"` namespace (StaticLayer from `/map`)
- [x] Uses `SmacPlannerHybrid::createPlan()` directly (no lifecycle)
- [x] Publishes `/global_path` (transient_local) and `/planner_event`
- [x] Emits `PLAN_FAILED` event on exception; `PLAN_SUCCEEDED` on success
- [x] WARN if `/goal_pose` arrives before `/robot_pose`
- [x] MultiThreadedExecutor with node + costmap node

---

## Plan 4 — `mission_executive` ✅
- [x] Package: `mission_executive/` (package.xml, CMakeLists.txt)
- [x] Action server `~/navigate_to_target` (NavigateToTarget)
- [x] Services: `~/abort` (Trigger), `~/set_target` (SetTarget), `~/teleop` (SetBool)
- [x] Publishes: `/goal_pose`, `/nav_enabled`, `/nav_mode`, `/active_target`, `/nav_status`
- [x] Subscribes: `/robot_pose`, `/odom`, `/planner_event`, `/global_path`
- [x] States: IDLE, NAVIGATING, ARRIVING, STOPPED_AT_TARGET, ABORTING, RETURNING, STOPPED_AT_RETURN, TELEOP
- [x] Full transition table from PLAN.md §3.2.2
- [x] Calls `gps_pose_publisher/latlon_to_enu` for coordinate conversion
- [x] MultiThreadedExecutor + reentrant callback group (deadlock-safe)
- [x] Cross-track error replanning: replan when xtrack > `replan_distance_m`
- [x] Stop detection: `|twist.linear| < velocity_zero_threshold` held for `arrival_hold_time` seconds
- [x] Target registry via `~/set_target`; RETURN requires visited target
- [x] Preemption: new GO_TO aborts current active goal

## Plan 5 — `nav.launch.py` + unified build ✅
- [x] Package: `nav_bringup/` (package.xml, CMakeLists.txt)
- [x] `config/nav_params.yaml` — single param file for all nodes (from PLAN.md §7)
- [x] `launch/nav.launch.py` — includes `athena_gps/launch/gps_launch.py`
- [x] `/odom` → `/odom/ground_truth` remapped for `mission_executive` + `ackermann_mppi`
- [x] twist_stamper node: `cmd_vel_nav` → `/rear_ackermann_controller/reference`
- [x] Launches: `gps_pose_publisher`, `map_node`, `global_planner`, `ackermann_mppi`, `mission_executive`
- [x] Single `nav_params.yaml` passed to all nodes

---

## Notes
- `msgs` package (at `src/msgs/`) already contains all required message/service/action definitions
- Phase 2 extensions (obstacle layers, ArUco, object detection) are deferred per PLAN.md §5

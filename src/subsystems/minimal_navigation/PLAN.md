# Athena Navigation Stack — Implementation Spec

A GPS-only, Nav2-free navigation system in a single ROS 2 package. No IMU-GNSS fusion — localization is GPS + compass heading only.


---

## 1. Architecture Overview

Four nodes form the runtime pipeline:

```
gps_pose_publisher → mission_executive → global_planner → mppi_runner
       │                    │                   │               │
   map→base_link TF     state machine       Hybrid-A*       MPPI control
   /robot_pose          /goal_pose          /global_path     /cmd_vel
```

**TF tree:** `map → base_link` (no `odom` frame — see §1.2).

### 1.1 Existing Code

| Source | Role | Required Changes |
|--------|------|------------------|
| `ackermann_mppi/` | Local planner / path follower | Rename `/plan` → `/global_path`; add `/nav_enabled` guard; add costmap debug publisher |
| `smac/` | Global planner (Hybrid-A\*) | Rename package to `athena_smac_planner`; integrate `athena_map` costmap input |
| `athena_map/` | Publishes `nav_msgs/OccupancyGrid` on `/map` | Ensure `frame_id: "map"`, `transient_local` QoS, and resolution matches `global_costmap` (0.5 m). We control this node — adapt as needed for clean integration. |
| `athena_planner/` | BT nodes (ArUco pose extraction, etc.) | Reuse in Phase 2 |
| `aruco_detection/` | ArUco refinement (~80% complete) | Wire in Phase 2c |

**Do not use `localizer/`** — IMU-GNSS fusion is explicitly excluded from the localization path.

### 1.2 TF Convention — Intentional `odom` Omission

REP-105 expects `map → odom → base_link`. This stack uses `map → base_link` directly because there is no fused odometry source suitable for the `odom` frame. `gps_pose_publisher` broadcasts this transform.

The ZED SDK provides `/odom` (`nav_msgs/Odometry`) with visual-inertial velocity estimates. This is used **only** for measured velocity in stop detection (§3.2.4) — it does not participate in localization or TF.

### 1.3 Package Rename: `smac/` → `athena_smac_planner`

The vendored `smac/` package declared `<name>nav2_smac_planner</name>` in `package.xml`, colliding with the system-installed `ros-$ROS_DISTRO-nav2-smac-planner`. Fix: rename to `athena_smac_planner` in both `package.xml` and `CMakeLists.txt`.

---

## 2. Operator Interface

All operator commands use ROS 2 services and actions (the legacy `OperatorCmd.msg` topic is retired).

| Command | Interface | Type | Rationale |
|---------|-----------|------|-----------|
| `GO_TO` | Action | `NavigateToTarget` | Long-running; preemptable; provides distance feedback |
| `RETURN` | Action | `NavigateToTarget` | Same semantics as `GO_TO` (with `is_return=true`) |
| `ABORT` | Service | `std_srvs/Trigger` | Instant; needs confirmation |
| `SET_TARGET` | Service | `SetTarget` | Pre-register a target by ID for later navigation |
| `TELEOP_ON` / `TELEOP_OFF` | Service | `std_srvs/SetBool` | Needs ack; returns success + optional message |

**Intended workflow:** Use `SetTarget` to pre-load named targets (with coordinates, type, and tolerance) into `mission_executive`'s target registry. Then use `NavigateToTarget` with just a `target_id` (+ `is_return` flag) to begin navigation. `NavigateToTarget` also accepts inline coordinates for ad-hoc goals that don't need pre-registration — when `target_id` is empty, the coordinates in the goal message are used directly.

All custom types live in the `navigation_msgs` package (see §6).

---

## 3. Node Specifications

### 3.1 `gps_pose_publisher`

Converts raw GPS + heading into a `map`-frame pose and broadcasts `map → base_link`.

**Subscriptions:**

| Topic | Type | Notes |
|-------|------|-------|
| `/gps/fix` | `sensor_msgs/NavSatFix` | |
| `/gps/heading` (configurable) | `std_msgs/Float64` | Input always in degrees. Internally converted to ENU radians (0 = East, CCW+). |

**Publications:**

| Topic | Type | Notes |
|-------|------|-------|
| `/robot_pose` | `geometry_msgs/PoseStamped` | Frame: `map` |
| `/gps_status` | `std_msgs/String` | Fix quality + heading source each update |
| TF: `map → base_link` | | |

**Services:**

| Service | Type | Purpose |
|---------|------|---------|
| `~/latlon_to_enu` | `LatLonToENU` | Centralizes WGS84 → ENU projection |

**Core logic:**

- WGS84 → ENU via `GeographicLib::LocalCartesian`:
  ```cpp
  GeographicLib::LocalCartesian proj(lat0, lon0, alt0);
  proj.Forward(lat, lon, alt, e, n, u);
  ```
- Origin set from first fix, or from `/start_gate_ref` if `use_start_gate_ref: true`.
- Heading conversion: input degrees → ENU radians, then `tf2::Quaternion::setRPY(0, 0, heading_rad)`.

---

### 3.2 `mission_executive`

Operator-driven state machine managing target sequencing, abort/return, mode switching, and arrival detection.

#### 3.2.1 States

`IDLE`, `NAVIGATING`, `ARRIVING`, `STOPPED_AT_TARGET`, `ABORTING`, `RETURNING`, `STOPPED_AT_RETURN`, `TELEOP`.

`TELEOP_ON` from any state → `TELEOP`. `TELEOP_OFF` → `IDLE` (never auto-resumes navigation).

#### 3.2.2 Transition Table

| State | GO_TO | ABORT | RETURN | `PLAN_FAILED` | TELEOP_ON | TELEOP_OFF |
|-------|-------|-------|--------|---------------|-----------|------------|
| `IDLE` | → `NAVIGATING` | ignore | → `RETURNING`\* | ignore | → `TELEOP` | ignore |
| `NAVIGATING` | → `NAVIGATING` (preempt) | → `ABORTING` | ignore | → `ABORTING` | → `TELEOP` | ignore |
| `ARRIVING` | → `NAVIGATING` (cancel) | → `ABORTING` | ignore | ignore | → `TELEOP` | ignore |
| `STOPPED_AT_TARGET` | → `NAVIGATING` | ignore | → `RETURNING`\* | ignore | → `TELEOP` | ignore |
| `ABORTING` | queued | ignore | queued | ignore | → `TELEOP` | ignore |
| `RETURNING` | → `NAVIGATING` (preempt) | → `ABORTING` | ignore | → `ABORTING` | → `TELEOP` | ignore |
| `STOPPED_AT_RETURN` | → `NAVIGATING` | ignore | → `RETURNING`\* | ignore | → `TELEOP` | ignore |
| `TELEOP` | ignore | ignore | ignore | ignore | ignore | → `IDLE` |

\* `RETURN` requires the target to have been previously visited. Otherwise the action server returns failure immediately.

#### 3.2.3 `PLAN_FAILED` Handling

Subscribes to `/planner_event` (`PlannerEvent.msg`). On `event == PLAN_FAILED`:

- In `NAVIGATING` or `RETURNING`: transition to `ABORTING`, publish `/nav_enabled = false`.
- In all other states: ignore (the executive is authoritative on state).

#### 3.2.4 Stopped Detection

Arrival confirmation uses **measured velocity from `/odom`** (published by the ZED SDK — not commanded `/cmd_vel`). The robot is considered stopped when `|twist.linear| < velocity_zero_threshold` is held for `arrival_hold_time` seconds.

#### 3.2.5 Replanning

The global planner does **not** self-trigger replans. `mission_executive` computes the robot's cross-track error (perpendicular distance from `/robot_pose` to the nearest segment of the current `/global_path`). When this exceeds `replan_distance_m`, the executive re-publishes `/goal_pose` to trigger a fresh plan. This keeps replanning fully observable via the `/goal_pose` topic stream.

The executive caches the latest `/global_path` (subscribed with `transient_local` QoS) for this computation.

#### 3.2.6 Executor Configuration

`mission_executive` calls `gps_pose_publisher`'s `latlon_to_enu` service during `GO_TO` goal conversion. To avoid deadlock from blocking on `future.get()` inside a callback, **use a `MultiThreadedExecutor` with a reentrant callback group** for the service client.

#### 3.2.7 Interfaces

**Subscriptions:**

| Topic | Type | Purpose |
|-------|------|---------|
| `/robot_pose` | `PoseStamped` | Current position |
| `/odom` | `nav_msgs/Odometry` | Measured velocity for stop detection (ZED SDK) |
| `/planner_event` | `PlannerEvent` | `PLAN_FAILED` triggers abort |
| `/global_path` | `nav_msgs/Path` | Cached for cross-track error (QoS: `transient_local`) |

**Action servers:**

| Name | Type | Handles |
|------|------|---------|
| `~/navigate_to_target` | `NavigateToTarget` | `GO_TO`, `RETURN` |

**Services:**

| Name | Type | Purpose |
|------|------|---------|
| `~/abort` | `std_srvs/Trigger` | Transition to `ABORTING` |
| `~/set_target` | `SetTarget` | Register/update a named target in the registry |

**Publications:**

| Topic | Type | Notes |
|-------|------|-------|
| `/goal_pose` | `PoseStamped` | Always ENU / `map` frame; consumed by global planner |
| `/nav_enabled` | `std_msgs/Bool` | `false` during teleop/stop; MPPI respects this |
| `/nav_mode` | `std_msgs/String` | `"autonomous"` / `"teleop"` / `"stopped"` |
| `/active_target` | `ActiveTarget` | Target metadata + status |
| `/nav_status` | `NavStatus` | 2 Hz + on every state change |

#### 3.2.8 Target Types

| Type | Tolerance | Notes |
|------|-----------|-------|
| `GNSS_ONLY` | 3.0 m | GPS-sourced goal |
| `ARUCO_POST` | 2.0 m | GNSS approach → ArUco refines (Phase 2c) |
| `OBJECT` | stop on detection | First two: GNSS < 3 m; third: < 10 m |
| `LOCAL` | user-defined | Meter-sourced goal; no GPS required |

---

### 3.3 `global_planner` (Hybrid-A\* via `athena_smac_planner`)

Wraps the vendored SMAC planner with `athena_map` costmap input. Plans **only** on receipt of a new `/goal_pose` — no self-triggered replanning.

**Startup behavior:** On receiving a `/goal_pose` before a valid `/robot_pose` has been received, the planner logs a `WARN` and waits (does not publish `PLAN_FAILED`). Planning proceeds once both pose and map are available. This handles the GPS cold-start window gracefully.

**Subscriptions:**

| Topic | Type | Notes |
|-------|------|-------|
| `/goal_pose` | `PoseStamped` | Triggers planning |
| `/robot_pose` | `PoseStamped` | Planner start pose — must be received before planning |
| `/map` | `nav_msgs/OccupancyGrid` | Feeds `StaticLayer` (QoS: `transient_local`) |

**Publications:**

| Topic | Type | QoS | Notes |
|-------|------|-----|-------|
| `/global_path` | `nav_msgs/Path` | `transient_local` | Consumed by MPPI + mission_executive |
| `/planner_event` | `PlannerEvent` | | Enum-based event (see §6) |

---

### 3.4 `mppi_runner` (existing `ackermann_mppi`)

Minimal changes to the existing package.

**Changes required:**

1. Rename input topic `/plan` → `/global_path`
2. Subscribe to `/nav_enabled` — skip `evalControl()` when false
3. Add costmap debug publisher via `nav2_costmap_2d::CostmapPublisher`

**Local costmap** is owned by `mppi_runner` as a child `Costmap2DROS` node, configured under the `local_costmap` namespace nested within `mppi_runner` parameters (see §7).

**Subscriptions:** `/robot_pose`, `/odom`, `/global_path`, `/goal_pose`, `/nav_enabled`

**Publications:** `/cmd_vel` (`TwistStamped`)

**Control loop:** Timer at `controller_frequency` Hz (default 20). Calls `evalControl()` only when pose + path + goal are all available **and** `/nav_enabled` is true.

**Initial critics:**
`ConstraintCritic`, `GoalCritic`, `GoalAngleCritic`, `PathFollowCritic`, `PathAlignCritic`, `PreferForwardCritic`

---

## 4. Debugging & Observability

### 4.1 `/nav_status` (published by `mission_executive`)

Published at 2 Hz and on every state change. See §6 for full message definition.

### 4.2 Logging Conventions

| Event | Level | Format |
|-------|-------|--------|
| State transition | `INFO` | `[mission_executive] <OLD> → <NEW>: <reason>` |
| Replan trigger | `INFO` | `[mission_executive] replan triggered: xtrack=X.Xm` |
| `evalControl()` skip | `DEBUG` | (fires at 20 Hz — never INFO) |
| GPS origin set | `INFO` | `[gps_pose_publisher] origin set: lat=X lon=Y alt=Z` |
| `PLAN_FAILED` received | `WARN` | `[mission_executive] PLAN_FAILED — transitioning to ABORTING` |
| Planner waiting for fix | `WARN` | `[global_planner] goal received but no robot_pose yet — waiting` |
| Service/action rejection | `WARN` | Reason logged; error returned to caller |

### 4.3 Topic QoS

| Topic | QoS |
|-------|-----|
| `/global_path` | `transient_local` (rviz2/Foxglove sees current path on connect) |
| `/map` | `transient_local` (from `athena_map`) |
| `/nav_status` | `keep_last(1)`, reliable |

---

## 5. Extension Phases

| Phase | Feature | Effort |
|-------|---------|--------|
| **2a** | Depth camera obstacle avoidance | Config only — add `ObstacleLayer` + `InflationLayer` to MPPI local costmap plugins, enable `CostCritic` + `ObstaclesCritic` |
| **2b** | GeoTIFF terrain obstacles | Config only — set `geotiff_path` on `global_planner` |
| **2c** | ArUco post refinement | Wire `correction_node` output to override `/goal_pose` when `ARUCO_POST` target is within ~10 m. Reuse `GetArucoPose` BT node. |
| **2d** | Object detection | `yolo_ros/` detects objects within tolerance → publish to C2 overlay → `mission_executive` transitions to `STOPPED_AT_TARGET` |
| **2e** | Spiral/lawnmower search | `SpiralCoverageAction` BT node (Archimedean spiral, $r = 15$ m, spacing $= 2$ m). Sub-mode when target reached via GNSS but not visually acquired. |

---

## 6. Custom Message / Service / Action Definitions

All defined in the `navigation_msgs` package.

### Services

```
# LatLonToENU.srv
float64 lat
float64 lon
---
float64 x
float64 y
float64 z
```

```
# SetTarget.srv — pre-register a named target
string  target_id
float64 lat              # GPS goal (WGS84) — ignored if goal_type == METER
float64 lon
float64 x_m             # Meter goal (ENU) — ignored if goal_type == GPS
float64 y_m
uint8   goal_type        # GPS=0, METER=1
uint8   GPS=0
uint8   METER=1
uint8   target_type      # GNSS_ONLY=0, ARUCO_POST=1, OBJECT=2, LOCAL=3
uint8   GNSS_ONLY=0
uint8   ARUCO_POST=1
uint8   OBJECT=2
uint8   LOCAL=3
float64 tolerance_m
---
bool   success
string message
```

### Actions

```
# NavigateToTarget.action
# --- Goal ---
# If target_id is non-empty, coordinates are looked up from the registry
# (previously loaded via SetTarget). If target_id is empty, inline
# coordinates are used as an ad-hoc goal.
string  target_id        # empty string = use inline coordinates
float64 lat              # ad-hoc GPS goal (ignored if target_id set)
float64 lon
float64 x_m             # ad-hoc meter goal (ignored if target_id set)
float64 y_m
uint8   goal_type        # GPS=0, METER=1
uint8   GPS=0
uint8   METER=1
uint8   target_type      # GNSS_ONLY=0, ARUCO_POST=1, OBJECT=2, LOCAL=3
uint8   GNSS_ONLY=0
uint8   ARUCO_POST=1
uint8   OBJECT=2
uint8   LOCAL=3
float64 tolerance_m      # only used for ad-hoc goals; registry targets use stored tolerance
bool    is_return
---
# --- Result ---
bool   success
string message
---
# --- Feedback ---
float64 distance_to_goal_m
float64 cross_track_error_m
string  state
```

### Messages

```
# PlannerEvent.msg
uint8 NEW_GOAL=0
uint8 PLANNING=1
uint8 PLAN_SUCCEEDED=2
uint8 PLAN_FAILED=3
uint8 event
```

```
# ActiveTarget.msg
string                       target_id
uint8                        target_type      # GNSS_ONLY=0, ARUCO_POST=1, OBJECT=2, LOCAL=3
float64                      tolerance_m
geometry_msgs/PoseStamped    goal_enu
uint8                        goal_source      # GPS=0, METER=1
string                       status           # NAVIGATING, SEARCHING, ARRIVED, ABORTED
```

```
# NavStatus.msg
string  state                # IDLE, NAVIGATING, ARRIVING, STOPPED_AT_TARGET,
                             # ABORTING, RETURNING, STOPPED_AT_RETURN, TELEOP
string  active_target_id
uint8   active_target_type   # GNSS_ONLY=0, ARUCO_POST=1, OBJECT=2, LOCAL=3
uint8   goal_source          # GPS=0, METER=1
float64 distance_to_goal_m   # -1.0 if no active goal
float64 cross_track_error_m  # -1.0 if no active path
float64 heading_error_rad
float64 robot_speed_mps      # from /odom twist (ZED SDK)
bool    is_return
uint8   last_planner_event   # last PlannerEvent.event value
```

---

## 7. Parameter Reference (`nav_params.yaml`)

```yaml
# ── Global ──────────────────────────────────────────────
use_sim_time: false          # propagated to all nodes via launch

# ── Robot geometry (referenced by costmaps + planner) ───
robot_radius: &robot_radius 0.45   # meters — inscribed radius

# ── gps_pose_publisher ─────────────────────────────────
gps_pose_publisher:
  ros__parameters:
    use_sim_time: false
    heading_topic: "/gps/heading"
    origin_lat: NaN              # auto-set from first fix if NaN
    origin_lon: NaN
    origin_alt: 0.0
    use_start_gate_ref: true     # if true, /start_gate_ref sets origin

# ── mission_executive ──────────────────────────────────
mission_executive:
  ros__parameters:
    use_sim_time: false
    default_targets: []
    velocity_zero_threshold: 0.05  # m/s — applied to /odom twist
    arrival_hold_time: 1.0         # seconds of confirmed zero velocity
    replan_distance_m: 3.0         # cross-track error threshold to trigger replan
    latlon_to_enu_service: "/gps_pose_publisher/latlon_to_enu"

# ── global_planner (athena_smac_planner) ───────────────
global_planner:
  ros__parameters:
    use_sim_time: false
    geotiff_path: ""
    grid_size_m: 500.0
    grid_resolution_m: 0.5
    obstacle_threshold: 50
    min_turning_r: 1.2             # must match mppi_runner
    allow_reverse: false

global_costmap:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    robot_radius: 0.45
    update_frequency: 1.0
    publish_frequency: 0.5
    transform_tolerance: 1.0       # generous — GPS-only localization has jitter
    rolling_window: false
    plugins: ["static_layer", "inflation_layer"]
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_topic: /map
      subscribe_to_updates: true
      map_subscribe_transient_local: true
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 1.0        # >= robot_radius

# ── mppi_runner (ackermann_mppi) ───────────────────────
mppi_runner:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    motion_model: "Ackermann"
    min_turning_r: 1.2
    vx_max: 3.0
    vx_min: 0.0
    wz_max: 1.0
    time_steps: 56
    batch_size: 1000
    model_dt: 0.05
    critics:
      - ConstraintCritic
      - GoalCritic
      - GoalAngleCritic
      - PathFollowCritic
      - PathAlignCritic
      - PreferForwardCritic
    # Local costmap — child Costmap2DROS node owned by mppi_runner
    local_costmap:
      ros__parameters:
        use_sim_time: false
        global_frame: map
        robot_base_frame: base_link
        robot_radius: 0.45
        update_frequency: 5.0
        publish_frequency: 2.0
        transform_tolerance: 1.0
        rolling_window: true
        width: 20
        height: 20
        resolution: 0.1
        plugins: []                # Phase 2a: add ObstacleLayer + InflationLayer
```

---

## 8. Implementation Order

1. **`gps_pose_publisher`** — full implementation; validates TF + ENU projection before anything else runs.
2. **`ackermann_mppi` modifications** — topic rename, `/nav_enabled` guard, costmap debug publisher.
3. **`global_planner` / `athena_smac_planner`** — package rename, `athena_map` integration, costmap wiring.
4. **`mission_executive`** — state machine, action/service servers, all operator commands.
5. **`nav.launch.py`** + `CMakeLists.txt` + `package.xml` — single-package build and launch.

In the final launch file, you need to launch main/athena-code/src/subsystems/navigation/athena_gps/launch/gps_launch.py, and we will use the /odom/ground_truth topic for odom. you also need to map the output using a twist stamper like

    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='cmd_vel_stamper',
        parameters=[{'use_sim_time': sim}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel_nav'),
            ('cmd_vel_out', '/rear_ackermann_controller/reference'),
        ],
    )


# `nav_sim_harness` — Progress

Tracks status against [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md). Update as
phases land. Status key: ✅ done · 🚧 in progress · ⬜ not started.

_Last updated: 2026-05-28 (Phases 1–3 implemented; live smoke run pending)._

> **Build status:** `colcon build --packages-select nav_sim_harness simulation`
> passes; `ament_flake8` clean; all three scenarios parse; `harness.launch.py`
> declares `world`/`world_name`/`headless` and resolves its includes.
>
> **First headless run (2026-05-28):** `./scripts/run.sh scenarios/smoke.yaml`
> brought the whole stack up **headless** (my `-s` flag confirmed working: DEM
> loaded, costmap generated, `front_ackermann_controller` configured), the runner
> connected, the goal was **accepted**, `/nav_status` feedback streamed
> (`NAVIGATING`), and scoring + teardown + exit-code propagation all ran. The nav
> itself did **not** progress (`dist` stuck, `has_path=0`) because the container
> already had a competing `mission_executive` stack running (`test_services.sh` +
> ~9 `mission_executive_node` instances) → duplicate action servers /
> controller_manager conflicts. **A clean PASS requires a container with no other
> nav stack running** — re-run on a fresh container to close the "Done when"
> gates. The harness machinery itself is validated end-to-end.

> **Package layout note:** built as **`ament_python`** (`setup.py`/`setup.cfg`),
> not `ament_cmake` as the plan's layout sketch implied — it matches the repo's
> other Python nav packages (`aruco_bt`, `led_indicator`, …) and is the natural
> fit for a pure-Python harness. `runner`/`spawner` are exposed both as
> `python3 -m nav_sim_harness.<mod>` and as console scripts.

---

## Phase 0 — Plan review & verification ✅

- ✅ Verified `mission_cli` command surface, blocking behaviour, action server name
  (`/mission_executive/navigate_to_target`). Found exit code does NOT signal nav
  pass/fail — score off `/nav_status` instead.
- ✅ Verified observation topics/msgs: `/led_status` (LedStatus), `/nav_status`
  (NavStatus — fields `state`, `distance_to_goal_m`, `cross_track_error_m`),
  `/active_target`, `/odom/ground_truth`. Confirmed all 12 nav state strings
  (`STOPPED_AT_TARGET`, `SPIRAL_DONE`, …).
- ✅ Verified sim launch files: `gz_sim.launch.py` (no `headless` yet; `gz_args`
  appends `-r -v 4`), `bringup.launch.py` (exposes `publish_sim_heading`, `world`,
  `world_name`), `bridge.launch.py` (no spawn service bridged). Corrected the plan's
  `GZ_SIM_RESOURCE_PATH` attribution (set in `gz_sim.launch.py`, not `spawn`).
- ✅ Confirmed `nav_bringup/emmn.launch.py` accepts `sim:=true`; `gps_pose_publisher`
  always broadcasts `map→base_link`, so `publish_ground_truth_tf` is NOT needed
  (closed plan open-item #1).
- ✅ Detection node uses `DICT_4X4_50` (old `Dictionary_get` API) — drives the tag
  dictionary choice below.
- ✅ Plan updated with all corrections; `aruco_texture.py` reframed as a one-shot
  tool; `launch_testing`/`colcon test` smoke dropped in favour of `smoke.yaml`.

## Phase 1 — Launch-file flag + spawner (1 day) ✅

- ✅ **ArUco post model** `src/description/models/aruco_post/`:
  - `model.config`, `model.sdf` — 3-sided triangular prism (3 faces @ yaw 0/120/240°,
    apothem 0.057735 m, centred z=1.0 m) + support post; PBR `albedo_map` per face.
  - `materials/textures/aruco_4x4_50_id4.png` — baked id=4 tag, **verified detected
    as id 4** under `DICT_4X4_50` via detector round-trip.
- ✅ **One-shot generator** `scripts/gen_aruco_texture.py` — `cv2.aruco` (new+old API),
  6×6 marker + 1-cell white quiet zone = 8 cells = 20 cm @ 2.5 cm/cell. Not imported
  at runtime.
- ✅ Added `headless` arg to `simulation/launch/gz_sim.launch.py` (appends ` -s`
  to `gz_args` via a `PythonExpression` when true) and forwarded it from
  `bringup.launch.py`. Verified both via `ros2 launch … --show-args`.
- ✅ `spawner.py`: `Spawner` rclpy client on `/world/<name>/create`
  (`ros_gz_interfaces/srv/SpawnEntity`); `spawn_box(name, pose, size, rgba)` and
  `spawn_aruco_post(name, pose)` (the latter `<include>`s `model://aruco_post` —
  no runtime texture work). Also a standalone CLI (`python3 -m
  nav_sim_harness.spawner box|aruco --pose …`).
- ✅ The `/world/<name>/create` bridge lands in `harness.launch.py` (Phase 3) as a
  `parameter_bridge` service-bridge node. Confirmed this `ros_gz_bridge`
  (0.244.20, Humble) supports the CLI service-bridge form
  `…/create@ros_gz_interfaces/srv/SpawnEntity`.
- 🚧 **Done when:** script drops a box + post into a running `terrain_world.sdf`
  headless — code complete; **live drop not yet executed**.

## Phase 2 — Scenario runner via mission_cli (1 day) ✅

- ✅ `runner.py`: YAML loader (`_load_scenario` → `Scenario`/`Spawn`/`Step`
  dataclasses); waits for the action server + first `/nav_status`; runs `setup:`
  spawns then ordered `steps:`; each step shells out to `ros2 run
  mission_executive mission_cli …`; background `Observers` subscribe to
  `/nav_status`, `/led_status`, `/odom/ground_truth` (flashing-green latched
  per-step); per-step `assert:` evaluated after the nav returns.
- ✅ Scoring off `/nav_status` (never the subprocess exit code): `gnss`
  (`expect_state` + `distance_to_goal_m < max_dist_m`, default 3.0); `aruco_post`
  (`expect_state` + flashing-green LED seen + ground-truth dist to the named
  spawned post `< max_dist_m`, default 2.0). Steps with no `assert:` (e.g.
  `set-target`) are non-scored setup calls.
- 🚧 **Done when:** `./run.sh scenarios/single_aruco_post.yaml` exits 0 — code
  complete; **live run not yet executed**.

## Phase 3 — Launch wrapper + URC scenario (0.5 day) ✅

- ✅ `harness.launch.py`: includes `bringup.launch.py` (`publish_sim_heading:=true`,
  `world`, `world_name`, `headless` forwarded; default world `terrain_world.sdf`,
  headless default `true`) + `emmn.launch.py` (`sim:=true`) + the
  `/world/<world_name>/create` service bridge node.
- ✅ `scripts/run.sh`: reads `world`/`world_name` from the scenario, launches the
  harness in its own process group (headless by default; `--gui` to override),
  runs `python3 -m nav_sim_harness.runner`, tears the whole tree down on exit,
  and mirrors the runner's exit code.
- ✅ Scenarios: `smoke.yaml` (datum-free metre-frame GNSS nav — the smoke test),
  `single_aruco_post.yaml` (post @ ENU (12,3), GPS goal computed from the
  terrain_world datum), `multi_target.yaml` (GNSS via `set-target`+`nav-by-id`,
  then an ArUco post via `nav post1`). Chose `multi_target.yaml` over the layout
  sketch's `urc_full_mission.yaml` (resolves the filename-drift open item).
- 🚧 **Done when:** `./run.sh scenarios/smoke.yaml` exits 0 — code complete;
  **live run not yet executed** (next step).

---

## Future work (post-v1)

- ⬜ YOLO / object-detection scoring: `spawn_mallet`/`spawn_rock_pick`/
  `spawn_water_bottle`, `/yolo_detection` observer, `object` assert kind. (`nav object`
  already exists in `mission_cli` — only scoring is missing.)
- ⬜ Timed / mid-run spawns (drop an obstacle after a waypoint).
- ⬜ Fault injection (drop GPS for N s, inject TF jumps).

## Open items to confirm on first dry run

- ✅ Scenario filename drift resolved: shipping `multi_target.yaml` (not
  `urc_full_mission.yaml`).
- ✅ Post height: kept the model default (faces centred at z=1.0 m); scenarios
  spawn at ground z=0. Tunable later via the spawn pose if needed.
- ✅ Spawn timing: v1 spawns in `setup:` before any nav step (timed/mid-run
  deferred to Future work).
- ⬜ **GPS-datum assumption (must verify on first run):** the post scenarios
  assume the sim's first GPS fix equals the `terrain_world.sdf`
  `spherical_coordinates` datum (lat0=38.42391162772634, lon0=-110.78490558433397),
  so ENU origin = robot spawn. The `nav post1` goal lat/lon were back-computed
  from that. If the rover stops short of the post, adjust the scenario goal
  lat/lon (or widen `max_dist_m`).
- ⬜ Confirm `emmn.launch.py`'s `aruco_bt/aruco.launch.py` brings up the real
  Detection2D `aruco_node.py` (publishes `/aruco_loc`), not a dummy `BB` node —
  needed for the ArUco-approach branch the post scenarios exercise.
- ✅ `/gps/fix` IS bridged in sim — `athena_gps/gps_launch.py` adds
  `gps_bridge` (`/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat`) and
  `imu_bridge` under `IfCondition(sim)`, pulled in via
  `athena_sensors/sensors.launch.py` → `emmn.launch.py`. So the harness needs no
  extra sensor bridge beyond the create-service one.

## How to run

```bash
cd src/subsystems/navigation/nav_sim_harness
./scripts/run.sh scenarios/smoke.yaml              # headless smoke test
./scripts/run.sh scenarios/single_aruco_post.yaml  # spawn + ArUco approach
./scripts/run.sh scenarios/multi_target.yaml --gui # chained, with Gazebo GUI
```
(Run inside the ROS2 devcontainer with the workspace built and sourced.)

### Nodes (updated)

**`gps_pose_publisher`** — unchanged
**`mission_executive`** — unchanged
**`dem_costmap_converter`** — exists, no changes needed
**`global_planner`** — new, described below
**`vector_field_planner`** — unchanged

---

## `dem_costmap_converter` role

Runs independently at startup. Loads the GeoTIFF, computes a slope map via Sobel gradients, converts slope degrees to nav2-convention cost values (0–254), and republishes the `OccupancyGrid` at 1 Hz with transient_local QoS. The `global_planner` subscribes to this and treats it as a static cost lookup table — nothing else changes in `dem_costmap_converter`.

---

## `global_planner` node

**Inputs:**
- `/goal_pose` (`geometry_msgs/PoseStamped`) — from `mission_executive`
- `map → base_link` TF — start position
- `/map` (`nav_msgs/OccupancyGrid`, transient_local) — from `dem_costmap_converter`, optional

**Output:**
- `/global_path` (`nav_msgs/Path`, transient_local)

### Phase 1 (now): straight-line interpolation

When a goal arrives and either `use_costmap` is false or no costmap has been received yet, sample evenly-spaced poses along the straight line from robot to goal at `path_resolution_m` spacing. Publish immediately.

### Phase 2 (later): costmap-weighted A\*

When `use_costmap: true` and a costmap has been received, run A\* on the occupancy grid. The cost to enter a cell is read directly from the grid value published by `dem_costmap_converter` — cells at 254 are treated as impassable walls, everything below scales traversal cost. A `slope_weight` parameter scales how much the cost value penalizes the path versus raw distance, giving field-tunable behavior between "shortest path" and "flattest path":

$$g(cell) = \text{dist\_to\_neighbor} + \alpha \cdot \frac{\text{grid\_value}(cell)}{254}$$

where $\alpha$ is `slope_weight`.

The fallback is always available: if A\* fails to find a path (e.g. goal is in a lethal cell), the planner logs a warning and falls back to straight-line, which `mission_executive` will eventually replan anyway via its cross-track error monitor.

### Transition between phases

The planner checks `use_costmap` and costmap availability on every new `/goal_pose`. Switching from Phase 1 to Phase 2 in the field requires only:
1. Setting `use_costmap: true` in the YAML
2. Setting `dem_file_path` and launching `dem_costmap_converter`

No code changes, no interface changes.

---

## Updated `nav_params.yaml`

```
dem_costmap_converter: dem_file_path, map_resolution,
                       max_passable_slope_degrees,
                       output_frame, origin_x, origin_y

gps_pose_publisher:    heading_topic, heading_offset_deg,
                       use_start_gate_ref, origin_lat/lon/alt

mission_executive:     velocity_zero_threshold, arrival_hold_time,
                       replan_distance_m, latlon_to_enu_service, odom_topic

global_planner:        path_resolution_m,
                       use_costmap,
                       slope_weight

vector_field_planner:  map_frame, base_frame, tf_timeout_s,
                       max_speed_mps, max_steering_angle_rad,
                       lookahead_dist_m, k_p_steering,
                       repulsion_gain, repulsion_cutoff_m,
                       goal_tolerance_m, publish_debug_markers
```

`elevation_map_path` is gone — that's now entirely owned by `dem_costmap_converter` via `dem_file_path`. The `global_planner` never touches the GeoTIFF directly.

---

## Updated topic graph

| Publisher | Topic | Subscriber(s) |
|---|---|---|
| `gps_pose_publisher` | TF `map→base_link` | `global_planner`, `vector_field_planner`, `mission_executive` |
| `dem_costmap_converter` | `/map` | `global_planner` |
| `mission_executive` | `/nav_enabled` | `vector_field_planner` |
| `mission_executive` | `/goal_pose` | `global_planner` |
| `global_planner` | `/global_path` | `mission_executive`, `vector_field_planner` |
| `vector_field_planner` | `/cmd_vel` | chassis driver |
| `vector_field_planner` | `~/debug_markers` | RViz |

---

## What changes when moving to Phase 2

1. Launch `dem_costmap_converter` with `dem_file_path` pointing at your GeoTIFF
2. Set `use_costmap: true` and tune `slope_weight` in the YAML
3. Nothing else — same topics, same interfaces, `mission_executive` and `vector_field_planner` are completely unaware of the change


minimal_nav_bringup is an old package that oyu will have to update the params and launch file for, but a similar pattern should be followed. 


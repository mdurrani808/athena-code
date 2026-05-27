// Copyright (c) 2025 UMD Loop
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace vector_field_planner {

struct Pose2D {
  double x;
  double y;
};

struct ObstaclePoint {
  double x;  // map frame
  double y;  // map frame
};

enum class NavState : std::uint8_t {
  NAVIGATE = 0,
  SLOW     = 1,
  PROBE    = 2,
  ESCAPE   = 3,
  REPLAN   = 4,
};

struct PlannerParams {
  // --- path following ---
  double lookahead_dist_m              = 3.0;
  double k_p_steering                  = 1.5;  // pure-pursuit fallback (avoidance off)
  double max_steering_angle_rad        = 0.5;
  double max_speed_mps                 = 1.5;
  double goal_tolerance_m              = 1.5;
  double min_approach_linear_velocity  = 0.3;

  // --- avoidance master switch ---
  bool   obstacle_avoidance_enabled    = false;

  // --- robot footprint / kinematics ---
  double robot_radius_m                = 0.35;
  double min_turn_radius_m             = 1.0;   // tightest physically reachable arc

  // --- local grid ---
  double local_grid_size_m             = 10.0;  // square, base-frame, centered on robot
  double local_grid_resolution_m       = 0.1;
  double inflate_margin_m              = 0.15;  // added to robot_radius for dilation
  double scan_buffer_max_dist_m        = 5.0;   // node-side scan filter; surfaced here for parity

  // --- tentacles ---
  double tentacle_length_forward_m     = 3.0;
  double tentacle_length_reverse_m     = 1.5;
  double tentacle_sample_step_m        = 0.1;
  int    num_tentacles_per_side        = 6;     // total per direction = 2*N + 1 (incl. straight)

  // --- safety / FSM thresholds ---
  double r_stop_hard_m                 = 0.55;  // hard zero-vel floor
  double r_stop_m                      = 0.70;  // NAVIGATE/SLOW → PROBE
  double r_slow_m                      = 1.35;  // SLOW band upper edge
  double v_escape_mps                  = 0.4;   // reverse speed magnitude in ESCAPE
  double t_escape_max_s                = 4.0;
  int    max_escape_attempts           = 3;
  double tick_period_s                 = 0.05;  // matches node timer

  // --- tentacle scoring weights ---
  double w_clear                       = 1.0;
  double w_goal                        = 0.6;
  double w_smooth                      = 0.15;
  double w_reverse                     = 0.3;
};

struct PlannerResult {
  double linear_vel               = 0.0;
  double angular_vel              = 0.0;
  bool   goal_reached             = false;

  // Pure-pursuit / carrot diagnostics (unchanged).
  double lookahead_x              = 0.0;
  double lookahead_y              = 0.0;
  bool   lookahead_interpolated   = false;
  bool   lookahead_behind         = false;
  std::size_t closest_idx         = 0;
  double heading_err              = 0.0;
  double steering_unclamped       = 0.0;
  bool   clamped                  = false;
  double effective_lookahead_dist = 0.0;
  double approach_velocity_scale  = 1.0;

  // Avoidance diagnostics.
  double closest_r                = -1.0;  // nearest inflated obstacle, m; -1 if none
  int    active_points            = 0;     // obstacles within local grid

  // Tentacle / FSM diagnostics.
  NavState nav_state              = NavState::NAVIGATE;
  int    chosen_tentacle_idx      = -1;
  double chosen_curvature         = 0.0;
  double chosen_direction         = 1.0;   // +1 forward, -1 reverse
  double chosen_clearance         = 0.0;
  double best_forward_clearance   = 0.0;
  double best_reverse_clearance   = 0.0;
  // Indices into VectorFieldPlannerAlgo::tentacles() of the best-scoring
  // forward/reverse arcs this tick. Always populated (even in PROBE/REPLAN
  // when no command was applied), so consumers can derive an escape
  // direction without re-running the scoring pass.
  int    best_forward_idx         = -1;
  int    best_reverse_idx         = -1;
  bool   request_replan           = false; // raised in REPLAN until a new path arrives
};

// ---------------------------------------------------------------------------
// Internal types — exposed for testability / debug consumers.
// ---------------------------------------------------------------------------

struct Tentacle {
  double curvature;                // signed, 1/m (body-frame)
  double direction;                // +1 forward, -1 reverse
  double length;                   // m
  std::vector<Pose2D> samples;     // base-frame, evenly spaced; samples[0] = origin
};

struct TentacleScore {
  int    idx        = -1;
  double clearance  = 0.0;
  double goal_align = 0.0;
  double smoothness = 0.0;
  double total      = -std::numeric_limits<double>::infinity();
  bool   collides   = true;        // true if tentacle hits within r_stop_hard_m
};

struct FsmContext {
  NavState state                          = NavState::NAVIGATE;
  int      ticks_in_state                 = 0;
  double   best_clearance_seen_in_state   = 0.0;
  int      escape_attempts                = 0;
  double   prev_curvature                 = 0.0;
};

class VectorFieldPlannerAlgo {
 public:
  VectorFieldPlannerAlgo() = default;

  /*
   * Sets planner parameters. Rebuilds the tentacle library on the next compute().
   */
  void setParams(const PlannerParams& params) {
    params_       = params;
    tentacles_dirty_ = true;
  }

  const PlannerParams& getParams() const { return params_; }

  /*
   * Sets the path to follow. Resets the FSM (and escape counter) — a new path
   * is treated as a fresh start, including after a REPLAN cycle.
   */
  void setPath(const std::vector<Pose2D>& path) {
    path_ = path;
    fsm_  = FsmContext{};
  }

  /*
   * Updates the local obstacle set (map-frame points, already age-filtered by node).
   */
  void updateObstacles(const std::vector<ObstaclePoint>& obstacles) {
    obstacles_ = obstacles;
  }

  /*
   * Compute velocity + steering commands.
   *   rx, ry, yaw   — robot pose in map frame
   *   current_speed — last commanded linear speed (m/s), used for velocity-scaled lookahead
   */
  PlannerResult compute(double rx, double ry, double yaw, double current_speed);

  // -- exposed for tests --
  std::size_t findClosestIndex(double rx, double ry) const;

  struct LookaheadResult {
    double x;
    double y;
    bool   interpolated;
  };
  LookaheadResult findLookahead(double rx, double ry, std::size_t closest_idx,
                                double lookahead_dist) const;

  const std::vector<Tentacle>& tentacles() const { return tentacles_; }
  const FsmContext& fsm() const { return fsm_; }

 private:
  PlannerParams                params_;
  std::vector<Pose2D>          path_;
  std::vector<ObstaclePoint>   obstacles_;
  FsmContext                   fsm_;

  // Tentacle library (rebuilt on param change).
  std::vector<Tentacle>        tentacles_;
  bool                         tentacles_dirty_ = true;

  // Local grid (rebuilt every compute()).
  std::vector<std::uint8_t>    grid_;          // 0=free, 1=blocked; row-major
  int                          grid_n_         = 0;
  double                       grid_res_       = 0.1;
  // Disk kernel cell offsets used for obstacle dilation (recomputed with params).
  std::vector<std::pair<int,int>> disk_offsets_;

  // ----- helpers -----
  double effectiveLookaheadDist(double current_speed) const;
  double approachVelocityScale(double dist_to_goal) const;

  void          ensureTentaclesBuilt();
  void          buildTentacles();
  void          buildDiskKernel();

  void          buildLocalGrid(double rx, double ry, double yaw);
  bool          gridAt(double x, double y) const;
  void          paintDisk(int cx, int cy);

  TentacleScore scoreTentacle(const Tentacle& t, int idx, double carrot_bearing_base) const;

  void          stepFsm(const TentacleScore& best_fwd,
                        const TentacleScore& best_rev,
                        double approach_vel,
                        PlannerResult& res);

  void          applyTentacleCommand(const TentacleScore& sc, double linear_vel,
                                     PlannerResult& res);

  double        nearestObstacleDistBase(double rx, double ry, double yaw) const;
};

}  // namespace vector_field_planner

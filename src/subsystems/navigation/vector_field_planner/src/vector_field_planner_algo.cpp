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

#include "vector_field_planner/vector_field_planner_algo.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vector_field_planner {

namespace {

inline double wrapAngle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace

// ---------------------------------------------------------------------------
// Carrot / lookahead (unchanged behaviour, kept for path following).
// ---------------------------------------------------------------------------

double VectorFieldPlannerAlgo::effectiveLookaheadDist(double current_speed) const {
  const double t = std::clamp(std::fabs(current_speed) / params_.max_speed_mps, 0.0, 1.0);
  return params_.lookahead_dist_m * (1.0 + t);
}

double VectorFieldPlannerAlgo::approachVelocityScale(double dist_to_goal) const {
  return std::clamp(dist_to_goal / params_.lookahead_dist_m, 0.0, 1.0);
}

VectorFieldPlannerAlgo::LookaheadResult VectorFieldPlannerAlgo::findLookahead(
    double rx, double ry, std::size_t closest_idx, double lookahead_dist) const
{
  if (path_.empty()) return {rx, ry, false};

  const double ld2 = lookahead_dist * lookahead_dist;
  for (std::size_t i = closest_idx; i + 1 < path_.size(); ++i) {
    const double dx = path_[i + 1].x - path_[i].x;
    const double dy = path_[i + 1].y - path_[i].y;
    const double fx = path_[i].x - rx;
    const double fy = path_[i].y - ry;

    const double a = dx * dx + dy * dy;
    if (a < 1e-12) continue;

    const double b = 2.0 * (fx * dx + fy * dy);
    const double c = fx * fx + fy * fy - ld2;
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) continue;

    const double t = (-b + std::sqrt(disc)) / (2.0 * a);
    if (t >= 0.0 && t <= 1.0) {
      return {path_[i].x + t * dx, path_[i].y + t * dy, true};
    }
  }
  return {path_.back().x, path_.back().y, false};
}

std::size_t VectorFieldPlannerAlgo::findClosestIndex(double rx, double ry) const {
  std::size_t best = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path_.size(); ++i) {
    const double dx = path_[i].x - rx;
    const double dy = path_[i].y - ry;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) { best_d2 = d2; best = i; }
  }
  return best;
}

// ---------------------------------------------------------------------------
// Tentacle library
// ---------------------------------------------------------------------------

void VectorFieldPlannerAlgo::ensureTentaclesBuilt() {
  if (!tentacles_dirty_) return;
  buildTentacles();
  buildDiskKernel();
  tentacles_dirty_ = false;
}

void VectorFieldPlannerAlgo::buildTentacles() {
  tentacles_.clear();

  const int n_side = std::max(0, params_.num_tentacles_per_side);
  const double k_max =
    (params_.min_turn_radius_m > 1e-6) ? (1.0 / params_.min_turn_radius_m) : 0.0;
  const double step = std::max(1e-3, params_.tentacle_sample_step_m);

  auto build_one = [&](double curvature, double direction, double length) {
    Tentacle t;
    t.curvature = curvature;
    t.direction = direction;
    t.length    = length;

    double x = 0.0, y = 0.0, theta = 0.0;
    t.samples.push_back({x, y});
    const int n_steps = static_cast<int>(std::floor(length / step));
    for (int i = 0; i < n_steps; ++i) {
      x     += direction * step * std::cos(theta);
      y     += direction * step * std::sin(theta);
      theta += direction * step * curvature;
      t.samples.push_back({x, y});
    }
    tentacles_.push_back(std::move(t));
  };

  // Forward family: 2*n_side + 1 curvatures evenly spaced in [-k_max, +k_max].
  for (int i = -n_side; i <= n_side; ++i) {
    const double k = (n_side > 0) ? (k_max * static_cast<double>(i) / n_side) : 0.0;
    build_one(k, +1.0, params_.tentacle_length_forward_m);
  }
  // Reverse family.
  for (int i = -n_side; i <= n_side; ++i) {
    const double k = (n_side > 0) ? (k_max * static_cast<double>(i) / n_side) : 0.0;
    build_one(k, -1.0, params_.tentacle_length_reverse_m);
  }
}

void VectorFieldPlannerAlgo::buildDiskKernel() {
  disk_offsets_.clear();
  const double inflate_r = params_.robot_radius_m + params_.inflate_margin_m;
  const double res = std::max(1e-3, params_.local_grid_resolution_m);
  const int r_cells = static_cast<int>(std::ceil(inflate_r / res));
  const double r2 = (inflate_r / res) * (inflate_r / res);
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      if (static_cast<double>(dx * dx + dy * dy) <= r2) {
        disk_offsets_.emplace_back(dx, dy);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Local grid (base frame, centered on robot)
// ---------------------------------------------------------------------------

void VectorFieldPlannerAlgo::buildLocalGrid(double rx, double ry, double yaw) {
  grid_res_ = std::max(1e-3, params_.local_grid_resolution_m);
  grid_n_   = std::max(2, static_cast<int>(std::round(params_.local_grid_size_m / grid_res_)));
  // Ensure even count so the robot sits cleanly at cell-center origin.
  if (grid_n_ % 2 != 0) ++grid_n_;

  grid_.assign(static_cast<std::size_t>(grid_n_) * grid_n_, 0);

  const double c = std::cos(yaw), s = std::sin(yaw);
  const int half = grid_n_ / 2;

  for (const auto& p : obstacles_) {
    const double dx = p.x - rx;
    const double dy = p.y - ry;
    // Map → base: rotate by -yaw.
    const double bx =  c * dx + s * dy;
    const double by = -s * dx + c * dy;

    const int cx = static_cast<int>(std::floor(bx / grid_res_)) + half;
    const int cy = static_cast<int>(std::floor(by / grid_res_)) + half;
    if (cx < -half || cx >= grid_n_ + half) continue;  // beyond any reachable inflation
    paintDisk(cx, cy);
  }
}

void VectorFieldPlannerAlgo::paintDisk(int cx, int cy) {
  for (const auto& off : disk_offsets_) {
    const int x = cx + off.first;
    const int y = cy + off.second;
    if (x < 0 || x >= grid_n_ || y < 0 || y >= grid_n_) continue;
    grid_[static_cast<std::size_t>(y) * grid_n_ + x] = 1;
  }
}

bool VectorFieldPlannerAlgo::gridAt(double x, double y) const {
  const int half = grid_n_ / 2;
  const int gx = static_cast<int>(std::floor(x / grid_res_)) + half;
  const int gy = static_cast<int>(std::floor(y / grid_res_)) + half;
  if (gx < 0 || gx >= grid_n_ || gy < 0 || gy >= grid_n_) return false;
  return grid_[static_cast<std::size_t>(gy) * grid_n_ + gx] != 0;
}

double VectorFieldPlannerAlgo::nearestObstacleDistBase(double rx, double ry, double yaw) const {
  double best = std::numeric_limits<double>::infinity();
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (const auto& p : obstacles_) {
    const double dx = p.x - rx;
    const double dy = p.y - ry;
    const double bx =  c * dx + s * dy;
    const double by = -s * dx + c * dy;
    const double d  = std::hypot(bx, by);
    if (d < best) best = d;
  }
  return best;
}

// ---------------------------------------------------------------------------
// Tentacle scoring
// ---------------------------------------------------------------------------

TentacleScore VectorFieldPlannerAlgo::scoreTentacle(const Tentacle& t, int idx,
                                                    double carrot_bearing_base) const
{
  TentacleScore sc;
  sc.idx = idx;

  const double step = std::max(1e-3, params_.tentacle_sample_step_m);

  // Walk samples (skip the origin) until the first blocked cell.
  double clearance = t.length;
  for (std::size_t i = 1; i < t.samples.size(); ++i) {
    if (gridAt(t.samples[i].x, t.samples[i].y)) {
      clearance = static_cast<double>(i - 1) * step;
      break;
    }
  }
  sc.clearance = clearance;
  sc.collides  = (clearance < params_.r_stop_hard_m);

  // Goal alignment: bearing from origin to tentacle endpoint vs. carrot bearing.
  // Endpoint bearing is well-defined as long as the tentacle has at least one step.
  const auto& end = t.samples.back();
  const double end_bearing = std::atan2(end.y, end.x);
  sc.goal_align = std::cos(wrapAngle(end_bearing - carrot_bearing_base));

  // Smoothness: penalize curvature change from previous chosen tentacle.
  const double k_max = (params_.min_turn_radius_m > 1e-6)
                       ? (1.0 / params_.min_turn_radius_m) : 1.0;
  const double dk_norm = std::clamp(std::fabs(t.curvature - fsm_.prev_curvature) / (2.0 * k_max),
                                    0.0, 1.0);
  sc.smoothness = 1.0 - dk_norm;

  const double clear_norm = (t.length > 1e-6) ? (clearance / t.length) : 0.0;
  const double reverse_pen = (t.direction < 0.0) ? params_.w_reverse : 0.0;

  sc.total = params_.w_clear  * clear_norm
           + params_.w_goal   * sc.goal_align
           + params_.w_smooth * sc.smoothness
           - reverse_pen;

  // Colliding tentacles get a very low score regardless of other terms, so they
  // never "win" a pool if any non-colliding option exists.
  if (sc.collides) {
    sc.total -= 100.0;
  }
  return sc;
}

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------

void VectorFieldPlannerAlgo::stepFsm(const TentacleScore& best_fwd,
                                     const TentacleScore& best_rev,
                                     double approach_vel,
                                     PlannerResult& res)
{
  const double fwd_clear = best_fwd.collides ? 0.0 : best_fwd.clearance;
  const double rev_clear = best_rev.collides ? 0.0 : best_rev.clearance;
  const double any_clear = std::max(fwd_clear, rev_clear);

  res.best_forward_clearance = fwd_clear;
  res.best_reverse_clearance = rev_clear;
  res.best_forward_idx       = best_fwd.idx;
  res.best_reverse_idx       = best_rev.idx;

  const NavState prev_state = fsm_.state;

  // ---- transitions ----
  NavState next = fsm_.state;
  switch (fsm_.state) {
    case NavState::NAVIGATE:
      if (fwd_clear >= params_.r_slow_m)         next = NavState::NAVIGATE;
      else if (fwd_clear >= params_.r_stop_m)    next = NavState::SLOW;
      else                                       next = NavState::PROBE;
      break;

    case NavState::SLOW:
      if (fwd_clear >= params_.r_slow_m)         next = NavState::NAVIGATE;
      else if (fwd_clear < params_.r_stop_m)     next = NavState::PROBE;
      else                                       next = NavState::SLOW;
      break;

    case NavState::PROBE:
      if (fwd_clear >= params_.r_stop_m) {
        next = (fwd_clear >= params_.r_slow_m) ? NavState::NAVIGATE : NavState::SLOW;
      } else if (rev_clear >= params_.r_stop_m
                 && fsm_.escape_attempts < params_.max_escape_attempts) {
        next = NavState::ESCAPE;
      } else {
        next = NavState::REPLAN;
      }
      break;

    case NavState::ESCAPE: {
      // Re-enter NAVIGATE as soon as a clean forward arc opens up.
      if (fwd_clear >= params_.r_slow_m) {
        next = NavState::NAVIGATE;
      } else {
        const double elapsed = fsm_.ticks_in_state * params_.tick_period_s;
        const bool   improving =
          (any_clear > fsm_.best_clearance_seen_in_state + 1e-3);
        if (elapsed > params_.t_escape_max_s && !improving) {
          next = NavState::REPLAN;
        } else if (rev_clear < params_.r_stop_m) {
          // Lost our reverse option mid-escape — re-triage.
          next = NavState::PROBE;
        } else {
          next = NavState::ESCAPE;
        }
      }
      break;
    }

    case NavState::REPLAN:
      // Sticky until setPath() resets the FSM.
      next = NavState::REPLAN;
      break;
  }

  // ---- state entry bookkeeping ----
  if (next != prev_state) {
    fsm_.state                        = next;
    fsm_.ticks_in_state               = 0;
    fsm_.best_clearance_seen_in_state = any_clear;
    if (next == NavState::ESCAPE) ++fsm_.escape_attempts;
  } else {
    ++fsm_.ticks_in_state;
    if (any_clear > fsm_.best_clearance_seen_in_state) {
      fsm_.best_clearance_seen_in_state = any_clear;
    }
  }

  // ---- actions ----
  switch (fsm_.state) {
    case NavState::NAVIGATE: {
      const double v = std::min(params_.max_speed_mps, approach_vel);
      applyTentacleCommand(best_fwd, v, res);
      break;
    }
    case NavState::SLOW: {
      const double band = std::max(1e-6, params_.r_slow_m - params_.r_stop_m);
      const double scale = std::clamp((fwd_clear - params_.r_stop_m) / band, 0.0, 1.0);
      const double v = std::min(params_.max_speed_mps, approach_vel) * scale;
      applyTentacleCommand(best_fwd, v, res);
      break;
    }
    case NavState::PROBE: {
      res.linear_vel  = 0.0;
      res.angular_vel = 0.0;
      break;
    }
    case NavState::ESCAPE: {
      // Negative linear vel; ω = κ * v handles Ackermann reverse correctly.
      applyTentacleCommand(best_rev, -std::fabs(params_.v_escape_mps), res);
      break;
    }
    case NavState::REPLAN: {
      res.linear_vel     = 0.0;
      res.angular_vel    = 0.0;
      res.request_replan = true;
      break;
    }
  }

  res.nav_state = fsm_.state;
}

void VectorFieldPlannerAlgo::applyTentacleCommand(const TentacleScore& sc, double linear_vel,
                                                  PlannerResult& res)
{
  if (sc.idx < 0 || sc.idx >= static_cast<int>(tentacles_.size())) {
    res.linear_vel  = 0.0;
    res.angular_vel = 0.0;
    return;
  }
  const auto& t = tentacles_[sc.idx];
  res.linear_vel        = linear_vel;
  res.angular_vel       = linear_vel * t.curvature;  // ω = v * κ (Ackermann, signed v)
  res.chosen_tentacle_idx = sc.idx;
  res.chosen_curvature    = t.curvature;
  res.chosen_direction    = t.direction;
  res.chosen_clearance    = sc.clearance;
  fsm_.prev_curvature     = t.curvature;
}

// ---------------------------------------------------------------------------
// Main compute
// ---------------------------------------------------------------------------

PlannerResult VectorFieldPlannerAlgo::compute(double rx, double ry, double yaw,
                                              double current_speed)
{
  PlannerResult res;
  res.closest_r = -1.0;

  if (path_.empty()) return res;

  const auto& goal_pos = path_.back();
  const double dist_to_goal = std::hypot(goal_pos.x - rx, goal_pos.y - ry);
  if (dist_to_goal < params_.goal_tolerance_m) {
    res.goal_reached = true;
    return res;
  }

  // Path-following intermediates.
  res.closest_idx              = findClosestIndex(rx, ry);
  res.effective_lookahead_dist = effectiveLookaheadDist(current_speed);

  const auto lk = findLookahead(rx, ry, res.closest_idx, res.effective_lookahead_dist);
  res.lookahead_x            = lk.x;
  res.lookahead_y            = lk.y;
  res.lookahead_interpolated = lk.interpolated;

  const double cyaw = std::cos(yaw), syaw = std::sin(yaw);
  const double carrot_dx = lk.x - rx;
  const double carrot_dy = lk.y - ry;
  res.lookahead_behind = (carrot_dx * cyaw + carrot_dy * syaw < 0.0);
  res.heading_err      = wrapAngle(std::atan2(carrot_dy, carrot_dx) - yaw);

  // Approach velocity (used by both pure-pursuit and tentacle paths).
  res.approach_velocity_scale = approachVelocityScale(dist_to_goal);
  const double approach_vel = params_.min_approach_linear_velocity +
    res.approach_velocity_scale *
    (params_.max_speed_mps - params_.min_approach_linear_velocity);

  // -------------------------------------------------------------------------
  // Avoidance OFF → pure-pursuit fallback (legacy behaviour preserved).
  // -------------------------------------------------------------------------
  if (!params_.obstacle_avoidance_enabled) {
    res.steering_unclamped = params_.k_p_steering * res.heading_err;
    res.angular_vel        = std::clamp(res.steering_unclamped,
                                        -params_.max_steering_angle_rad,
                                        params_.max_steering_angle_rad);
    res.clamped            = std::fabs(res.steering_unclamped) > params_.max_steering_angle_rad;
    res.linear_vel         = std::min(params_.max_speed_mps, approach_vel);
    res.nav_state          = NavState::NAVIGATE;
    return res;
  }

  // -------------------------------------------------------------------------
  // Tentacle planner
  // -------------------------------------------------------------------------
  ensureTentaclesBuilt();
  buildLocalGrid(rx, ry, yaw);

  // Carrot bearing in base frame.
  const double cb_x =  cyaw * carrot_dx + syaw * carrot_dy;
  const double cb_y = -syaw * carrot_dx + cyaw * carrot_dy;
  const double carrot_bearing_base = std::atan2(cb_y, cb_x);

  TentacleScore best_fwd, best_rev;
  for (std::size_t i = 0; i < tentacles_.size(); ++i) {
    const auto sc = scoreTentacle(tentacles_[i], static_cast<int>(i), carrot_bearing_base);
    if (tentacles_[i].direction > 0.0) {
      if (sc.total > best_fwd.total) best_fwd = sc;
    } else {
      if (sc.total > best_rev.total) best_rev = sc;
    }
  }

  // Diagnostics (raw obstacle distance — independent of grid dilation).
  const double nearest = nearestObstacleDistBase(rx, ry, yaw);
  res.closest_r     = std::isinf(nearest) ? -1.0 : nearest;
  res.active_points = static_cast<int>(obstacles_.size());

  // FSM picks state + command.
  stepFsm(best_fwd, best_rev, approach_vel, res);

  // Hard safety stop — independent of FSM, last word on forward motion.
  if (res.closest_r > 0.0 && res.closest_r < params_.r_stop_hard_m && res.linear_vel > 0.0) {
    res.linear_vel  = 0.0;
    res.angular_vel = 0.0;
  }

  return res;
}

}  // namespace vector_field_planner

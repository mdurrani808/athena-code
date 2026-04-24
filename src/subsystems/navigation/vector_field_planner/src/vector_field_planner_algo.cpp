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
#include <queue>

namespace vector_field_planner {

// VFH* Constants (§9.2)
constexpr double kAlphaDeg = 5.0;
constexpr double kAlphaRad = kAlphaDeg * M_PI / 180.0;
constexpr int kNumBins = static_cast<int>(360.0 / kAlphaDeg);
constexpr int kSmoothL = 3;   // 7-bin smoothing window
constexpr int kSMax = 18;    // wide valley threshold (bins)
constexpr int kNg = 5;       // look-ahead depth
constexpr double kDs = 1.0;  // step distance (meters, ~robot diameter)
constexpr double kLambda = 0.85;

// Cost weights
constexpr double kMu1 = 6.0, kMu2 = 3.0, kMu3 = 2.0;   // depth 0
constexpr double kMu1p = 5.0, kMu2p = 2.0, kMu3p = 1.0; // depth > 0

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

double VectorFieldPlannerAlgo::effectiveLookaheadDist(double current_speed) const {
  // Scale linearly from lookahead_dist_m at rest to 2x at max_speed_mps.
  // Derived entirely from existing params — no extra tuning knobs needed.
  const double t = std::clamp(std::fabs(current_speed) / params_.max_speed_mps, 0.0, 1.0);
  return params_.lookahead_dist_m * (1.0 + t);
}

double VectorFieldPlannerAlgo::approachVelocityScale(double dist_to_goal) const {
  // Ramp begins when goal enters the lookahead window (lookahead_dist_m).
  return std::clamp(dist_to_goal / params_.lookahead_dist_m, 0.0, 1.0);
}

// ---------------------------------------------------------------------------
// Interpolated lookahead carrot
// ---------------------------------------------------------------------------

VectorFieldPlannerAlgo::LookaheadResult VectorFieldPlannerAlgo::findLookahead(
    double rx, double ry, size_t closest_idx, double lookahead_dist) const
{
  if (path_.empty()) {
    return {rx, ry, false};
  }

  // Walk segments from closest_idx forward. For each segment [path_[i], path_[i+1]]
  // solve the quadratic |path_[i] + t*(path_[i+1]-path_[i]) - robot|^2 = ld^2.
  // Take the larger-t root (further along path). If t in [0,1], interpolate and return.
  const double ld2 = lookahead_dist * lookahead_dist;

  for (size_t i = closest_idx; i + 1 < path_.size(); ++i) {
    const double dx = path_[i + 1].x - path_[i].x;
    const double dy = path_[i + 1].y - path_[i].y;
    const double fx = path_[i].x - rx;
    const double fy = path_[i].y - ry;

    const double a = dx * dx + dy * dy;
    if (a < 1e-12) continue;  // degenerate segment

    const double b = 2.0 * (fx * dx + fy * dy);
    const double c = fx * fx + fy * fy - ld2;

    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) continue;  // circle misses this segment

    // Prefer the forward intersection (larger t)
    const double t = (-b + std::sqrt(disc)) / (2.0 * a);
    if (t >= 0.0 && t <= 1.0) {
      return {
        path_[i].x + t * dx,
        path_[i].y + t * dy,
        true
      };
    }
  }

  // No segment intersection — robot is within lookahead_dist of the last point,
  // or path spacing is coarser than lookahead. Return last point.
  return {path_.back().x, path_.back().y, false};
}

// ---------------------------------------------------------------------------
// VFH* Internals
// ---------------------------------------------------------------------------

std::vector<double> VectorFieldPlannerAlgo::buildHistogram(
    const std::vector<ObstaclePoint>& obs, double rx, double ry) const
{
  std::vector<double> h(kNumBins, 0.0);
  const double d_max = params_.repulsion_cutoff_m;

  for (const auto& p : obs) {
    const double dx = p.x - rx;
    const double dy = p.y - ry;
    const double dist = std::hypot(dx, dy);

    if (dist >= d_max || dist < 0.1) continue;

    // Magnitude calculation (VFH+)
    // m = c^2 * (a - b*d)
    // Simplified: weight decreases with distance
    const double magnitude = (d_max - dist) / d_max;
    
    double angle = std::atan2(dy, dx);
    if (angle < 0) angle += 2.0 * M_PI;
    
    int bin = static_cast<int>(angle / kAlphaRad) % kNumBins;
    h[bin] += magnitude;
  }
  return h;
}

std::vector<double> VectorFieldPlannerAlgo::smoothHistogram(const std::vector<double>& h) const
{
  std::vector<double> h_smooth(kNumBins, 0.0);
  for (int i = 0; i < kNumBins; ++i) {
    double sum = 0.0;
    for (int l = -kSmoothL; l <= kSmoothL; ++l) {
      int idx = (i + l + kNumBins) % kNumBins;
      // Linear weighting 4, 3, 2, 1 for center to edge
      sum += h[idx] * (kSmoothL + 1 - std::abs(l));
    }
    h_smooth[i] = sum / (kSmoothL + 1);
  }
  return h_smooth;
}

std::vector<int> VectorFieldPlannerAlgo::findCandidates(
    const std::vector<double>& h, int k_target, int k_prev) const
{
  std::vector<int> candidates;
  
  // Find valleys (consecutive bins below threshold)
  std::vector<std::pair<int, int>> valleys;
  int start = -1;
  for (int i = 0; i < kNumBins; ++i) {
    if (h[i] < params_.vfh_threshold) {
      if (start == -1) start = i;
    } else {
      if (start != -1) {
        valleys.push_back({start, i - 1});
        start = -1;
      }
    }
  }
  if (start != -1) {
    // Handle wrap around valley
    if (!valleys.empty() && valleys[0].first == 0) {
      valleys[0].first = start;
    } else {
      valleys.push_back({start, kNumBins - 1});
    }
  }

  for (auto& v : valleys) {
    int size = (v.second - v.first + kNumBins) % kNumBins + 1;
    if (size >= kSMax) {
      // Wide valley: candidates are edges and goal/prev if inside
      candidates.push_back((v.first + kSMax/2) % kNumBins);
      candidates.push_back((v.second - kSMax/2 + kNumBins) % kNumBins);
      
      // Check if target or prev is in valley
      auto in_valley = [&](int k) {
        if (v.first <= v.second) return k >= v.first && k <= v.second;
        return k >= v.first || k <= v.second;
      };
      if (in_valley(k_target)) candidates.push_back(k_target);
      if (in_valley(k_prev)) candidates.push_back(k_prev);
    } else {
      // Narrow valley: candidate is the middle
      int mid = (v.first + size / 2) % kNumBins;
      candidates.push_back(mid);
    }
  }

  // Deduplicate and filter candidates that are too far from target? 
  // No, A* will handle cost. Just deduplicate.
  std::sort(candidates.begin(), candidates.end());
  candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

  return candidates;
}

static int angular_diff(int k1, int k2) {
  int diff = std::abs(k1 - k2);
  if (diff > kNumBins / 2) diff = kNumBins - diff;
  return diff;
}

struct VfhNode {
  double rx, ry, yaw;
  int k_prev;
  int root_k;
  int depth;
  double g;
  double f;

  bool operator>(const VfhNode& other) const { return f > other.f; }
};

int VectorFieldPlannerAlgo::runAstar(double rx, double ry, double yaw,
                                     int k_target, int k_prev) const
{
  std::priority_queue<VfhNode, std::vector<VfhNode>, std::greater<VfhNode>> open;

  auto h0 = buildHistogram(obstacles_, rx, ry);
  auto h0s = smoothHistogram(h0);
  auto candidates0 = findCandidates(h0s, k_target, k_prev);

  if (candidates0.empty()) return k_target;
  if (candidates0.size() == 1) return candidates0[0];

  int current_k = static_cast<int>(std::fmod(yaw + 2.0*M_PI, 2.0*M_PI) / kAlphaRad);

  for (int ck : candidates0) {
    // cost_primary (depth 0)
    // g = mu1*delta(ck, kt) + mu2*delta(orient, kt) + mu3*delta(ck, kp)
    double g = kMu1 * angular_diff(ck, k_target) +
               kMu2 * angular_diff(current_k, k_target) +
               kMu3 * angular_diff(ck, k_prev);
    
    // Simple admissible heuristic (Dijkstra if h=0, but we'll use a small goal-bias)
    double h = kMu1p * std::pow(kLambda, 1) * angular_diff(ck, k_target);
    
    open.push({rx, ry, yaw, ck, ck, 0, g, g + h});
  }

  if (open.empty()) return candidates0[0];

  while (!open.empty()) {
    VfhNode n = open.top();
    open.pop();

    if (n.depth >= kNg) return n.root_k;

    // Project next position assuming we moved in direction ck
    double move_yaw = n.k_prev * kAlphaRad;
    double next_rx = n.rx + kDs * std::cos(move_yaw);
    double next_ry = n.ry + kDs * std::sin(move_yaw);
    double next_yaw = move_yaw; // Simple model: orientation becomes direction of motion

    auto hi = buildHistogram(obstacles_, next_rx, next_ry);
    auto his = smoothHistogram(hi);
    auto candidatesi = findCandidates(his, k_target, n.k_prev);

    // Branching reduction: pick only the best candidate per side of target (plus target itself)
    int best_left = -1, best_right = -1;
    double min_g_left = std::numeric_limits<double>::infinity();
    double min_g_right = std::numeric_limits<double>::infinity();

    for (int ck : candidatesi) {
      int next_depth = n.depth + 1;
      double cost_ck = std::pow(kLambda, next_depth) * (
          kMu1p * angular_diff(ck, k_target) +
          kMu2p * angular_diff(static_cast<int>(next_yaw/kAlphaRad), k_target) +
          kMu3p * angular_diff(ck, n.k_prev)
      );

      // Simple branching reduction logic
      int diff = ck - k_target;
      if (diff > kNumBins/2) diff -= kNumBins;
      if (diff < -kNumBins/2) diff += kNumBins;

      if (diff < 0 && cost_ck < min_g_left) {
        best_left = ck; min_g_left = cost_ck;
      } else if (diff > 0 && cost_ck < min_g_right) {
        best_right = ck; min_g_right = cost_ck;
      } else if (diff == 0) {
        // Target itself is always expanded if found
        double h_new = kMu1p * std::pow(kLambda, next_depth + 1) * angular_diff(ck, k_target);
        open.push({next_rx, next_ry, next_yaw, ck, n.root_k, next_depth, n.g + cost_ck, n.g + cost_ck + h_new});
      }
    }

    if (best_left != -1) {
      double h_new = kMu1p * std::pow(kLambda, n.depth + 2) * angular_diff(best_left, k_target);
      open.push({next_rx, next_ry, next_yaw, best_left, n.root_k, n.depth + 1, n.g + min_g_left, n.g + min_g_left + h_new});
    }
    if (best_right != -1) {
      double h_new = kMu1p * std::pow(kLambda, n.depth + 2) * angular_diff(best_right, k_target);
      open.push({next_rx, next_ry, next_yaw, best_right, n.root_k, n.depth + 1, n.g + min_g_right, n.g + min_g_right + h_new});
    }
    
    if (open.size() > 200) break;
  }

  return k_target;
}

// ---------------------------------------------------------------------------
// Closest index
// ---------------------------------------------------------------------------

size_t VectorFieldPlannerAlgo::findClosestIndex(double rx, double ry) const {
  size_t best = 0;
  double best_dist2 = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < path_.size(); ++i) {
    const double dx = path_[i].x - rx;
    const double dy = path_[i].y - ry;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_dist2) {
      best_dist2 = d2;
      best = i;
    }
  }

  return best;
}

// ---------------------------------------------------------------------------
// Main compute
// ---------------------------------------------------------------------------

PlannerResult VectorFieldPlannerAlgo::compute(double rx, double ry, double yaw, double current_speed)
{
  PlannerResult res;
  res.closest_r = std::numeric_limits<double>::infinity();

  if (path_.empty()) {
    return res;
  }

  const auto& goal_pos = path_.back();
  const double dist_to_goal = std::hypot(goal_pos.x - rx, goal_pos.y - ry);
  if (dist_to_goal < params_.goal_tolerance_m) {
    res.goal_reached = true;
    res.linear_vel = 0.0;
    res.angular_vel = 0.0;
    return res;
  }

  res.closest_idx = findClosestIndex(rx, ry);

  // Feature 2: velocity-scaled lookahead
  res.effective_lookahead_dist = effectiveLookaheadDist(current_speed);

  // Feature 3: interpolated carrot
  const auto lk = findLookahead(rx, ry, res.closest_idx, res.effective_lookahead_dist);
  res.lookahead_x = lk.x;
  res.lookahead_y = lk.y;
  res.lookahead_interpolated = lk.interpolated;

  const double fwd_dot = (lk.x - rx) * std::cos(yaw) + (lk.y - ry) * std::sin(yaw);
  res.lookahead_behind = (fwd_dot < 0.0);

  res.heading_err = std::atan2(lk.y - ry, lk.x - rx) - yaw;
  while (res.heading_err > M_PI) res.heading_err -= 2.0 * M_PI;
  while (res.heading_err < -M_PI) res.heading_err += 2.0 * M_PI;

  // Obstacle avoidance (VFH*)
  // Skip VFH* when lookahead is behind: the kinematic constraint prevents large
  // heading corrections and the robot needs a hard turn, not micro-adjustments.
  if (params_.obstacle_avoidance_enabled && !res.lookahead_behind) {
    double target_heading = std::atan2(lk.y - ry, lk.x - rx);
    if (target_heading < 0) target_heading += 2.0 * M_PI;
    int k_target = static_cast<int>(target_heading / kAlphaRad) % kNumBins;

    // Initialize prev_k_ to current yaw on first call
    static bool first_run = true;
    if (first_run) {
      prev_k_ = static_cast<int>(std::fmod(yaw + 2.0*M_PI, 2.0*M_PI) / kAlphaRad);
      first_run = false;
    }

    int k_best = runAstar(rx, ry, yaw, k_target, prev_k_);
    
    double desired_heading = k_best * kAlphaRad;
    res.heading_err = desired_heading - yaw;
    while (res.heading_err > M_PI) res.heading_err -= 2.0 * M_PI;
    while (res.heading_err < -M_PI) res.heading_err += 2.0 * M_PI;

    res.vfh_k_best = k_best;
    prev_k_ = k_best;

    // Track active points and closest_r for diagnostics
    for (const auto& p : obstacles_) {
      const double dx = rx - p.x;
      const double dy = ry - p.y;
      const double dist = std::hypot(dx, dy);
      if (dist < params_.repulsion_cutoff_m) {
        res.closest_r = std::min(res.closest_r, dist);
        res.active_points++;
      }
    }
  }

  res.steering_unclamped = params_.k_p_steering * res.heading_err;
  res.angular_vel = std::clamp(res.steering_unclamped,
                               -params_.max_steering_angle_rad,
                               params_.max_steering_angle_rad);
  res.clamped = std::abs(res.steering_unclamped) > params_.max_steering_angle_rad;

  // Feature 1: approach velocity scaling
  res.approach_velocity_scale = approachVelocityScale(dist_to_goal);
  const double approach_vel = params_.min_approach_linear_velocity +
    res.approach_velocity_scale * (params_.max_speed_mps - params_.min_approach_linear_velocity);
  res.linear_vel = std::min(params_.max_speed_mps, approach_vel);

  if (std::isinf(res.closest_r)) {
    res.closest_r = -1.0;
  }

  return res;
}

}  // namespace vector_field_planner

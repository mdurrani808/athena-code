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

#include "global_planner/global_planner_algo.hpp"

#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <algorithm>

namespace global_planner {

std::vector<Pose2D> GlobalPlannerAlgo::planStraightLine(
    double sx, double sy, double gx, double gy) const {
  std::vector<Pose2D> path;

  const double dist = std::hypot(gx - sx, gy - sy);
  const double yaw = std::atan2(gy - sy, gx - sx);

  if (dist < 1e-6) {
    path.push_back({gx, gy, yaw});
    return path;
  }

  const int n = std::max(2, static_cast<int>(std::ceil(dist / params_.path_resolution_m)) + 1);

  path.reserve(n);
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / (n - 1);
    path.push_back({sx + t * (gx - sx), sy + t * (gy - sy), yaw});
  }

  return path;
}

std::optional<std::vector<Pose2D>> GlobalPlannerAlgo::planAstar(
    double sx, double sy, double gx, double gy) const {
  if (!costmap_.has_value()) {
    return std::nullopt;
  }

  const auto& cmap = costmap_.value();
  const int W = static_cast<int>(cmap.width);
  const int H = static_cast<int>(cmap.height);
  const double res = cmap.resolution;
  const double ox = cmap.origin_x;
  const double oy = cmap.origin_y;

  auto toGrid = [&](double wx, double wy, int& col, int& row) -> bool {
    col = static_cast<int>((wx - ox) / res);
    row = static_cast<int>((wy - oy) / res);
    return col >= 0 && col < W && row >= 0 && row < H;
  };

  auto toWorld = [&](int col, int row, double& wx, double& wy) {
    wx = ox + (col + 0.5) * res;
    wy = oy + (row + 0.5) * res;
  };

  int sc, sr, gc, gr;
  if (!toGrid(sx, sy, sc, sr)) return std::nullopt;
  if (!toGrid(gx, gy, gc, gr)) return std::nullopt;

  const auto goal_val = static_cast<uint8_t>(cmap.data[gr * W + gc]);
  if (goal_val >= 254) return std::nullopt;

  using State = std::tuple<double, int, int>;
  std::priority_queue<State, std::vector<State>, std::greater<State>> open;

  std::vector<double> g_cost(W * H, std::numeric_limits<double>::infinity());
  std::vector<int> parent(W * H, -1);

  const int s_idx = sr * W + sc;
  g_cost[s_idx] = 0.0;
  open.emplace(0.0, sc, sr);

  constexpr int dcol[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  constexpr int drow[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  constexpr double step[8] = {1, 1, 1, 1, M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};

  bool found = false;
  while (!open.empty()) {
    auto [f, cx, cy] = open.top();
    open.pop();

    const int idx = cy * W + cx;
    if (f > g_cost[idx] + 1e-9) continue;
    if (cx == gc && cy == gr) { found = true; break; }

    for (int d = 0; d < 8; ++d) {
      const int nx = cx + dcol[d];
      const int ny = cy + drow[d];
      if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;

      const auto cell_val = static_cast<uint8_t>(cmap.data[ny * W + nx]);
      if (cell_val >= 254) continue;

      const double move_cost = step[d] * res + params_.slope_weight * static_cast<double>(cell_val) / 254.0;
      const double ng = g_cost[idx] + move_cost;
      const int n_idx = ny * W + nx;

      if (ng < g_cost[n_idx]) {
        g_cost[n_idx] = ng;
        parent[n_idx] = idx;
        const double h = std::hypot(nx - gc, ny - gr) * res;
        open.emplace(ng + h, nx, ny);
      }
    }
  }

  if (!found) return std::nullopt;

  std::vector<std::pair<int, int>> cells;
  for (int cur = gr * W + gc; cur != -1; cur = parent[cur]) {
    cells.emplace_back(cur % W, cur / W);
  }
  std::reverse(cells.begin(), cells.end());

  std::vector<Pose2D> path;
  path.reserve(cells.size());

  for (size_t i = 0; i < cells.size(); ++i) {
    double wx, wy;
    toWorld(cells[i].first, cells[i].second, wx, wy);
    
    double yaw = 0.0;
    if (i + 1 < cells.size()) {
      double nwx, nwy;
      toWorld(cells[i + 1].first, cells[i + 1].second, nwx, nwy);
      yaw = std::atan2(nwy - wy, nwx - wx);
    } else if (!path.empty()) {
      yaw = path.back().yaw;
    }
    
    path.push_back({wx, wy, yaw});
  }

  return path;
}

} // namespace global_planner
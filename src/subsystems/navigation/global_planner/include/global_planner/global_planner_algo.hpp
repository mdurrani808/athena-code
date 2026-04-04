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

#include <vector>
#include <optional>
#include <cstdint>

namespace global_planner {

struct Pose2D {
  double x;
  double y;
  double yaw;
};

struct Costmap {
  uint32_t width = 0;
  uint32_t height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<int8_t> data;
};

struct PlannerParams {
  double path_resolution_m = 1.0;
  bool use_costmap = false;
  double slope_weight = 10.0;
};

class GlobalPlannerAlgo {
public:
  GlobalPlannerAlgo() = default;

  /*
   * Sets the global planner parameters (e.g., costmap usage, resolution).
   * Param: params - The configuration parameters.
   */
  void setParams(const PlannerParams& params) { params_ = params; }

  /*
   * Returns the current configuration parameters.
   */
  const PlannerParams& getParams() const { return params_; }

  /*
   * Sets the costmap used for A* planning.
   * Param: costmap - The 2D grid costmap representing obstacles and terrain slopes.
   */
  void setCostmap(const Costmap& costmap) { costmap_ = costmap; }

  /*
   * Clears the internal costmap, forcing straight-line fallback.
   */
  void clearCostmap() { costmap_ = std::nullopt; }

  /*
   * Checks if a costmap is currently loaded.
   * Returns: True if a costmap is present.
   */
  bool hasCostmap() const { return costmap_.has_value(); }

  /*
   * Generates a simple straight-line path from start to goal.
   * Ignores obstacles, used as a baseline or fallback.
   * Param: sx - Start X position.
   * Param: sy - Start Y position.
   * Param: gx - Goal X position.
   * Param: gy - Goal Y position.
   * Returns: A sequence of poses forming a straight line.
   */
  std::vector<Pose2D> planStraightLine(double sx, double sy, double gx, double gy) const;

  /*
   * Generates an optimal path using A* over the loaded costmap.
   * Evaluates cost based on distance and cell weights (slopes).
   * Param: sx - Start X position.
   * Param: sy - Start Y position.
   * Param: gx - Goal X position.
   * Param: gy - Goal Y position.
   * Returns: A sequence of poses if a path is found, or std::nullopt if unreachable.
   */
  std::optional<std::vector<Pose2D>> planAstar(double sx, double sy, double gx, double gy) const;

private:
  PlannerParams params_;
  std::optional<Costmap> costmap_;
};

} // namespace global_planner

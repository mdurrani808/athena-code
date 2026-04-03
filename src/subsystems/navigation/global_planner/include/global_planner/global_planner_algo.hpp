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

  void setParams(const PlannerParams& params) { params_ = params; }
  const PlannerParams& getParams() const { return params_; }

  void setCostmap(const Costmap& costmap) { costmap_ = costmap; }
  void clearCostmap() { costmap_ = std::nullopt; }
  bool hasCostmap() const { return costmap_.has_value(); }

  std::vector<Pose2D> planStraightLine(double sx, double sy, double gx, double gy) const;
  std::optional<std::vector<Pose2D>> planAstar(double sx, double sy, double gx, double gy) const;

private:
  PlannerParams params_;
  std::optional<Costmap> costmap_;
};

} // namespace global_planner

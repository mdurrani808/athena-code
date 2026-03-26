// Copyright (c) 2020, Carlos Luis
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
// limitations under the License. Reserved.

#ifndef ATHENA_SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_
#define ATHENA_SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_

#include <memory>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "athena_smac_planner/constants.hpp"

namespace athena_smac_planner
{

/**
 * @class athena_smac_planner::CostmapDownsampler
 * @brief A costmap downsampler for more efficient path planning
 */
class CostmapDownsampler
{
public:
  /**
   * @brief A constructor for CostmapDownsampler
   */
  CostmapDownsampler();

  /**
   * @brief A destructor for CostmapDownsampler
   */
  ~CostmapDownsampler();

  /**
   * @brief Configure the downsampled costmap object
   * @param costmap The costmap we want to downsample
   * @param downsampling_factor Multiplier for the costmap resolution
   * @param use_min_cost_neighbor If true, min function is used instead of max for downsampling
   */
  void on_configure(
    nav2_costmap_2d::Costmap2D * const costmap,
    const unsigned int & downsampling_factor,
    const bool & use_min_cost_neighbor = false);

  /**
   * @brief Cleanup the downsampled costmap
   */
  void on_cleanup();

  /**
   * @brief Downsample the given costmap by the downsampling factor, and publish the downsampled costmap
   * @param downsampling_factor Multiplier for the costmap resolution
   * @return A ptr to the downsampled costmap
   */
  nav2_costmap_2d::Costmap2D * downsample(const unsigned int & downsampling_factor);

  /**
   * @brief Resize the downsampled costmap. Used in case the costmap changes and we need to update the downsampled version
   */
  void resizeCostmap();

protected:
  /**
   * @brief Update the sizes X-Y of the costmap and its downsampled version
   */
  void updateCostmapSize();

  /**
   * @brief Explore all subcells of the original costmap and assign the max cost to the new (downsampled) cell
   * @param new_mx The X-coordinate of the cell in the new costmap
   * @param new_my The Y-coordinate of the cell in the new costmap
   */
  void setCostOfCell(
    const unsigned int & new_mx,
    const unsigned int & new_my);

  unsigned int _size_x;
  unsigned int _size_y;
  unsigned int _downsampled_size_x;
  unsigned int _downsampled_size_y;
  unsigned int _downsampling_factor;
  bool _use_min_cost_neighbor;
  float _downsampled_resolution;
  nav2_costmap_2d::Costmap2D * _costmap;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> _downsampled_costmap;
};

}  // namespace athena_smac_planner

#endif  // ATHENA_SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_

// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef ACKERMANN_MPPI__CRITICS__PATH_FOLLOW_CRITIC_HPP_
#define ACKERMANN_MPPI__CRITICS__PATH_FOLLOW_CRITIC_HPP_

#include "ackermann_mppi/critic_function.hpp"
#include "ackermann_mppi/models/state.hpp"
#include "ackermann_mppi/tools/utils.hpp"

namespace mppi::critics
{

class PathFollowCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  float threshold_to_consider_{0};
  size_t offset_from_furthest_{0};
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // ACKERMANN_MPPI__CRITICS__PATH_FOLLOW_CRITIC_HPP_

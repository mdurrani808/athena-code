// Copyright (c) 2021, Samsung Research America
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

#include "athena_smac_planner/node_basic.hpp"

namespace athena_smac_planner
{

template<typename NodeT>
void NodeBasic<NodeT>::processSearchNode()
{
}

template<>
void NodeBasic<NodeHybrid>::processSearchNode()
{
  // We only want to override the node's pose if it has not yet been visited
  // to prevent the case that a node has been queued multiple times and
  // a new branch is overriding one of lower cost already visited.
  if (!this->graph_node_ptr->wasVisited()) {
    this->graph_node_ptr->pose = this->pose;
    this->graph_node_ptr->setMotionPrimitiveIndex(this->motion_index, this->turn_dir);
  }
}

template<typename NodeT>
void NodeBasic<NodeT>::populateSearchNode(NodeT * & node)
{
  this->graph_node_ptr = node;
}

template<>
void NodeBasic<NodeHybrid>::populateSearchNode(NodeHybrid * & node)
{
  this->pose = node->pose;
  this->graph_node_ptr = node;
  this->motion_index = node->getMotionPrimitiveIndex();
  this->turn_dir = node->getTurnDirection();
}

template class NodeBasic<NodeHybrid>;

}  // namespace athena_smac_planner

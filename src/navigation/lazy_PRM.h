// Copyright 2018 slane@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <utility>
#include <vector>

#include "constants/includes.h"
#include "graph/vertex.h"
#include "logging/logger.h"
#include "navigation/PRM.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"

#ifndef SRC_NAVIGATION_LAZY_PRM_H_
#define SRC_NAVIGATION_LAZY_PRM_H_

namespace navigation {
class LazyPRM : public PRM {
 public:
  LazyPRM() = delete;
  ~LazyPRM() = default;

  LazyPRM(const graph::Graph& initial_graph,
          const obstacle::ObstacleFlag& obstacles,
          const obstacle::SafetyMargin& safety_margin,
          const bool use_scaffolding,
          const int number_of_layers, const int vertices_per_layer,
          const float layer_offset, const size_t seed);

  LazyPRM(const obstacle::ObstacleFlag& obstacles,
          const obstacle::SafetyMargin& safety_margin,
          const size_t base_graph_vertices, const bool use_scaffolding,
          const int number_of_layers, const int vertices_per_layer,
          const float layer_offset, const size_t seed);

  void Update(const obstacle::ObstacleFlag& obstacles,
              const Eigen::Vector2f& current_pose,
              const Eigen::Vector2f& goal_pose,
              logger::Logger* logger);

  void Update(const obstacle::ObstacleFlag& obstacles,
              const obstacle::SafetyMargin& safety_margin,
              const Eigen::Vector2f& current_pose,
              const Eigen::Vector2f& goal_pose,
              logger::Logger* logger);

  std::pair<bool, std::vector<Eigen::Vector2f>> Plan(
      logger::Logger* logger);

  bool AStarSearch(
      graph::GraphIndex start_graph, graph::VertexIndex start_index,
      graph::GraphIndex goal_graph, graph::VertexIndex goal_index,
      std::vector<std::pair<graph::GraphIndex, graph::VertexIndex>>*
      waypoint_list) const;
};
}  // namespace navigation

#endif  // SRC_NAVIGATION_LAZY_PRM_H_

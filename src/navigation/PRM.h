// Copyright 2016 - 2019 kvedder@umass.edu slane@cs.umass.edu
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

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "constants/includes.h"
#include "eigen3/Eigen/Core"
#include "graph/fastmultigraph.h"
#include "graph/graph.h"
#include "graph/vertex.h"
#include "math/poses_2d.h"
#include "navigation/navigation.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"
#include "util/random.h"

#ifndef SRC_NAVIGATION_PRM_H_
#define SRC_NAVIGATION_PRM_H_

namespace navigation {

class PRM : public Navigation {
 public:
  struct PriorityQueueVertex {
    float total_distance;
    float heuristic_value;
    graph::GraphIndex vertex_graph;
    graph::VertexIndex vertex_index;
    graph::GraphIndex prev_vertex_graph;
    graph::VertexIndex prev_vertex_index;
    PriorityQueueVertex(float total_distance, float heuristic_value,
                        graph::GraphIndex vertex_graph,
                        graph::VertexIndex vertex_index,
                        graph::GraphIndex prev_vertex_graph,
                        graph::VertexIndex prev_vertex_index)
        : total_distance(total_distance),
          heuristic_value(heuristic_value),
          vertex_graph(vertex_graph),
          vertex_index(vertex_index),
          prev_vertex_graph(prev_vertex_graph),
          prev_vertex_index(prev_vertex_index) {}

    ~PriorityQueueVertex() {}
  };

  PRM() = delete;
  PRM(const graph::Graph& initial_graph,
      const obstacle::ObstacleFlag& obstacles,
      const obstacle::SafetyMargin& safety_margin, const bool use_scaffolding,
      const int number_of_layers, const int vertices_per_layer,
      const float layer_offset, const size_t seed);
  PRM(const obstacle::ObstacleFlag& obstacles,
      const obstacle::SafetyMargin& safety_margin,
      const size_t base_graph_vertices, const bool use_scaffolding,
      const int number_of_layers, const int vertices_per_layer,
      const float layer_offset, const size_t seed);

  ~PRM() = default;

  void Update(const obstacle::ObstacleFlag& obstacles,
              const Eigen::Vector2f& current_pose,
              const Eigen::Vector2f& goal_pose, logger::Logger* logger);

  void Update(const obstacle::ObstacleFlag& obstacles,
              const obstacle::SafetyMargin& safety_margin,
              const Eigen::Vector2f& current_pose,
              const Eigen::Vector2f& goal_pose,
              logger::Logger* logger) override;

  std::pair<bool, std::vector<Eigen::Vector2f>> Plan(
      logger::Logger* logger) override;

  const graph::multi_graph::FastMultiGraph& GetGraph() const;

  bool AStarSearch(
      graph::GraphIndex start_graph, graph::VertexIndex start_index,
      graph::GraphIndex goal_graph, graph::VertexIndex goal_index,
      std::vector<std::pair<graph::GraphIndex, graph::VertexIndex>>*
          waypoint_list) const;

  bool ClosestFreeVertex(const Eigen::Vector2f& position,
                         graph::GraphIndex* graph_index,
                         graph::VertexIndex* index);

 protected:
  void InvalidateBlockedEdges();

  // Sampling PRM Primitive Function
  // Returns a vector of new points.
  // 0,0 is center of the field, positive X is towards the enemy goal
  // Units are in millimeters.
  void SamplePoint(Eigen::Vector2f* sample_position);

  static bool CompareVertices(const PRM::PriorityQueueVertex& pe1,
                              const PRM::PriorityQueueVertex& pe2);

  // Nearest Vertices PRM Primitive Function
  // Returns a vector of all points within a specific radius.
  std::vector<std::pair<graph::GraphIndex, graph::VertexIndex>>
  NearestNeighbors(const Eigen::Vector2f& input_position) const;

  obstacle::ObstacleFlag obstacles_;
  obstacle::SafetyMargin safety_margin_;

  // In free space current and goal.
  Eigen::Vector2f start_position_;
  Eigen::Vector2f goal_position_;

  // Indices of start and goal on graph
  graph::GraphIndex start_graph_;
  graph::VertexIndex start_index_;
  graph::GraphIndex goal_graph_;
  graph::VertexIndex goal_index_;

  graph::GraphIndex base_graph_index_;
  graph::multi_graph::FastMultiGraph multi_graph_;

  util_random::Random random_;

  // Flag to run one shot mode under certain conditions
  bool abort_search_;

  const bool use_scaffolding_;
  const int number_of_layers_;
  const int vertices_per_layer_;
  const float layer_offset_;

  // Predecessor index for start node for PRM.
  const graph::VertexIndex kStartNodePredecessor = graph::VertexIndex(0);
};
}  // namespace navigation
#endif  // SRC_NAVIGATION_PRM_H_

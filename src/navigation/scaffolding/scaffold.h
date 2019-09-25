// Copyright 2017 - 2018 kvedder@umass.edu, slane@cs.umass.edu
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

#include <vector>

#include "constants/constants.h"
#include "eigen3/Eigen/Core"
#include "graph/fastmultigraph.h"
#include "graph/graph.h"
#include "math/poses_2d.h"

#ifndef SRC_NAVIGATION_SCAFFOLDING_SCAFFOLD_H_
#define SRC_NAVIGATION_SCAFFOLDING_SCAFFOLD_H_

namespace navigation {
namespace scaffolding {

void ConnectPointToScaffold(
    const graph::GraphIndex scaffold_graph_index,
    const graph::GraphIndex base_graph_index,
    const graph::VertexIndex& base_graph_vertex_index,
    const obstacle::ObstacleFlag& obstacle_flag,
    const obstacle::SafetyMargin& safety_margin, const float distance_threshold,
    const obstacle::Obstacle* obstacle, const float layer_offset,
    const int num_layers, const int vertices_per_layer,
    graph::multi_graph::FastMultiGraph* fast_multi_graph);

void LazyConnectPointToScaffold(
  const graph::GraphIndex scaffold_graph_index,
  const graph::GraphIndex base_graph_index,
  const graph::VertexIndex& base_graph_vertex_index,
  const obstacle::ObstacleFlag& obstacle_flag,
  const obstacle::SafetyMargin& safety_margin, const float distance_threshold,
  const obstacle::Obstacle* obstacle, const float layer_offset,
  const int num_layers, const int vertices_per_layer,
  graph::multi_graph::FastMultiGraph* fast_multi_graph);

void InsertScaffolds(const graph::GraphIndex& base_graph_index,
                     const obstacle::ObstacleFlag& obstacle_flag,
                     const obstacle::SafetyMargin& safety_margin,
                     const float distance_threshold, const float layer_offset,
                     const int num_layers, const int vertices_per_layer,
                     graph::multi_graph::FastMultiGraph* fast_multi_graph);

void LazyInsertScaffolds(const graph::GraphIndex& base_graph_index,
                         const obstacle::ObstacleFlag& obstacle_flag,
                         const obstacle::SafetyMargin& safety_margin,
                         const float distance_threshold,
                         const float layer_offset, const int num_layers,
                         const int vertices_per_layer,
                         graph::multi_graph::FastMultiGraph* fast_multi_graph);

graph::Graph GenerateCircleScaffold(const pose_2d::Pose2Df& pose,
                                    const float radius, int number_of_layers,
                                    int vertices_per_layer, float layer_offset);

graph::Graph GenerateRectangleScaffold(const pose_2d::Pose2Df& pose,
                                       const float width, const float height,
                                       const float scaffold_distance,
                                       int number_of_layers,
                                       int vertices_per_layer,
                                       float layer_offset);

graph::Graph GenerateTempCircleScaffold(const pose_2d::Pose2Df& pose);

void ModifyScaffoldPose(const pose_2d::Pose2Df& old_pose,
                        const pose_2d::Pose2Df& new_pose, graph::Graph* graph);

}  // namespace scaffolding
}  // namespace navigation

#endif  // SRC_NAVIGATION_SCAFFOLDING_SCAFFOLD_H_

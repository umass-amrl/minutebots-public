// Copyright 2017-2018 kvedder@umass.edu, slane@cs.umass.edu
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
#include "navigation/scaffolding/scaffold.h"

#include <algorithm>
#include <limits>
#include <utility>

STANDARD_USINGS;

using graph::Graph;
using graph::GraphIndex;
using graph::multi_graph::FastMultiGraph;
using pose_2d::Pose2Df;
using obstacle::ObstacleFlag;
using obstacle::SafetyMargin;

namespace navigation {
namespace scaffolding {

graph::VertexIndex NearestNeighborInScaffold(
    const Graph& graph, const Vector2f& position,
    const graph::VertexIndex& ring_lower_vertex_index,
    const graph::VertexIndex& ring_upper_vertex_index) {
  size_t min_index = 0;
  float min_distance = std::numeric_limits<float>::max() - 1;

  size_t i = 0;
  for (const auto& vertex : graph.GetVertices()) {
    const auto current_distance = (vertex.position - position).squaredNorm();
    if (i >= ring_lower_vertex_index.index &&
        i <= ring_upper_vertex_index.index &&
        min_distance >= current_distance) {
      min_distance = current_distance;
      min_index = i;
    }
    ++i;
  }
  return graph::VertexIndex(min_index);
}

graph::VertexIndex ShiftVertexIndex(
    const graph::VertexIndex& index, const int amount,
    const graph::VertexIndex& ring_lower_vertex_index,
    const graph::VertexIndex& ring_upper_vertex_index) {
  //             << ring_upper_vertex_index.index << "], val: " << index.index
  //             << " amt: " << amount;
  if (index.index <= ring_lower_vertex_index.index && amount < 0) {
    const auto i =
        static_cast<int>(ring_upper_vertex_index.index) + (amount + 1);
    return graph::VertexIndex(i);
  } else if (index.index >= ring_upper_vertex_index.index && amount > 0) {
    const auto i =
        static_cast<int>(ring_lower_vertex_index.index) + (amount - 1);
    return graph::VertexIndex(i);
  } else {
    const auto i = static_cast<int>(index.index) + amount;
    return graph::VertexIndex(i);
  }
}

bool NoLineCollision(const Vector2f& p1, const Vector2f& p2,
                     const ObstacleFlag& obstacle_flag,
                     const SafetyMargin& safety_margin) {
  for (const auto* obstacle : obstacle_flag) {
    if (obstacle->LineCollision(p1, p2,
                                safety_margin.GetMargin(obstacle->GetType()))) {
      return false;
    }
  }
  return true;
}

// Connects the specified point in the base graph to the specified scaffold
// graph.
void ConnectPointToLayer(const GraphIndex scaffold_graph_index,
                         const GraphIndex base_graph_index,
                         const graph::VertexIndex& base_graph_vertex_index,
                         const ObstacleFlag& obstacle_flag,
                         const SafetyMargin& safety_margin,
                         const graph::VertexIndex& ring_lower_vertex_index,
                         const graph::VertexIndex& ring_upper_vertex_index,
                         FastMultiGraph* fast_multi_graph) {
  if (!kProduction) {
    if (ring_lower_vertex_index.index >=
        fast_multi_graph->GetGraph(scaffold_graph_index).GetNumVertices()) {
      LOG(FATAL)
          << "Lower vertex index bound out of range ("
          << ring_lower_vertex_index.index << " vs "
          << fast_multi_graph->GetGraph(scaffold_graph_index).GetNumVertices()
          << ")";
    }
    if (ring_upper_vertex_index.index >=
        fast_multi_graph->GetGraph(scaffold_graph_index).GetNumVertices()) {
      LOG(FATAL)
          << "Lower vertex index bound out of range("
          << ring_upper_vertex_index.index << " vs "
          << fast_multi_graph->GetGraph(scaffold_graph_index).GetNumVertices()
          << ")";
    }
  }

  //   fast_multi_graph->GetMutableGraph(graph_index)
  const graph::VertexIndex nearest_scaffold_vertex_index =
      NearestNeighborInScaffold(
          fast_multi_graph->GetGraph(scaffold_graph_index),
          fast_multi_graph->GetGraph(base_graph_index)
              .GetVertex(base_graph_vertex_index)
              .position,
          ring_lower_vertex_index, ring_upper_vertex_index);
  //             << nearest_scaffold_vertex_index.index;

  const graph::VertexIndex neighbor_a =
      ShiftVertexIndex(nearest_scaffold_vertex_index, 1,
                       ring_lower_vertex_index, ring_upper_vertex_index);
  const graph::VertexIndex neighbor_b =
      ShiftVertexIndex(nearest_scaffold_vertex_index, -1,
                       ring_lower_vertex_index, ring_upper_vertex_index);

  const auto& base_graph_point = fast_multi_graph->GetGraph(base_graph_index)
                                     .GetVertex(base_graph_vertex_index)
                                     .position;
  const auto& scaffold_nearest_point =
      fast_multi_graph->GetGraph(scaffold_graph_index)
          .GetVertex(nearest_scaffold_vertex_index)
          .position;
  const auto& scaffold_a_point =
      fast_multi_graph->GetGraph(scaffold_graph_index)
          .GetVertex(neighbor_a)
          .position;
  const auto& scaffold_b_point =
      fast_multi_graph->GetGraph(scaffold_graph_index)
          .GetVertex(neighbor_b)
          .position;
  if (NoLineCollision(base_graph_point, scaffold_nearest_point, obstacle_flag,
                      safety_margin)) {
    const auto weight = (base_graph_point - scaffold_nearest_point).norm();
    fast_multi_graph->AddInterGraphEdge(graph::edges::InterGraphEdge(
        true, weight, base_graph_index, base_graph_vertex_index,
        base_graph_point, scaffold_graph_index, nearest_scaffold_vertex_index,
        scaffold_nearest_point));

    //               << base_graph_vertex_index.index << " to " <<
    //               scaffold_graph_index
    //               << ", " << nearest_scaffold_vertex_index.index;
  }
  if (NoLineCollision(base_graph_point, scaffold_a_point, obstacle_flag,
                      safety_margin)) {
    const auto weight = (base_graph_point - scaffold_a_point).norm();
    fast_multi_graph->AddInterGraphEdge(graph::edges::InterGraphEdge(
        true, weight, base_graph_index, base_graph_vertex_index,
        base_graph_point, scaffold_graph_index, neighbor_a, scaffold_a_point));
    //               << base_graph_vertex_index.index << " to " <<
    //               scaffold_graph_index
    //               << ", " << neighbor_a.index;
  }
  if (NoLineCollision(base_graph_point, scaffold_b_point, obstacle_flag,
                      safety_margin)) {
    const auto weight = (base_graph_point - scaffold_b_point).norm();
    fast_multi_graph->AddInterGraphEdge(graph::edges::InterGraphEdge(
        true, weight, base_graph_index, base_graph_vertex_index,
        base_graph_point, scaffold_graph_index, neighbor_b, scaffold_b_point));
    //               << base_graph_vertex_index.index << " to " <<
    //               scaffold_graph_index
    //               << ", " << neighbor_b.index;
  }
}

void ConnectPointToScaffold(
    const GraphIndex scaffold_graph_index, const GraphIndex base_graph_index,
    const graph::VertexIndex& base_graph_vertex_index,
    const ObstacleFlag& obstacle_flag, const SafetyMargin& safety_margin,
    const float distance_threshold, const obstacle::Obstacle* obstacle,
    const float layer_offset, const int num_layers,
    const int vertices_per_layer, FastMultiGraph* fast_multi_graph) {
  const auto& graph_position = fast_multi_graph->GetGraph(base_graph_index)
                                   .GetVertex(base_graph_vertex_index)
                                   .position;
  const float max_radius = obstacle->GetRadius() +
                           safety_margin.GetMargin(obstacle->GetType()) +
                           layer_offset * num_layers + distance_threshold;
  if ((obstacle->GetPose().translation - graph_position).squaredNorm() >
      Sq(max_radius)) {
    return;
  }

  // This will be -1 when inside of the innermost layer.
  const int lower_layer_index = std::max(
      -1,
      std::min(num_layers - 1,
               static_cast<int>(floor(
                   ((obstacle->GetPose().translation - graph_position).norm() -
                    (obstacle->GetRadius() +
                     safety_margin.GetMargin(obstacle->GetType()))) /
                   static_cast<float>(layer_offset)))));


  if (lower_layer_index >= num_layers - 1) {
    int min_index_val =
        std::max(lower_layer_index - 1, 0) * vertices_per_layer + 1;
    int max_index_val = (lower_layer_index + 1) * vertices_per_layer - 1;
    const graph::VertexIndex min_index(min_index_val);
    const graph::VertexIndex max_index(max_index_val);

    //               << " Max index: " << max_index.index;

    ConnectPointToLayer(scaffold_graph_index, base_graph_index,
                        base_graph_vertex_index, obstacle_flag, safety_margin,
                        min_index, max_index, fast_multi_graph);
  } else if (lower_layer_index < 0) {
    const graph::VertexIndex min_index((lower_layer_index + 1) *
                                       vertices_per_layer);
    const graph::VertexIndex max_index(
        (lower_layer_index + 2) * vertices_per_layer - 1);

    //               << " Max index: " << max_index.index;

    ConnectPointToLayer(scaffold_graph_index, base_graph_index,
                        base_graph_vertex_index, obstacle_flag, safety_margin,
                        min_index, max_index, fast_multi_graph);
  } else {
    const graph::VertexIndex min_index1((lower_layer_index + 1) *
                                        vertices_per_layer);
    const graph::VertexIndex max_index1(
        (lower_layer_index + 2) * vertices_per_layer - 1);

    //               << " Max index: " << max_index1.index;

    ConnectPointToLayer(scaffold_graph_index, base_graph_index,
                        base_graph_vertex_index, obstacle_flag, safety_margin,
                        min_index1, max_index1, fast_multi_graph);
    const graph::VertexIndex min_index2((lower_layer_index)*vertices_per_layer);
    const graph::VertexIndex max_index2(
        (lower_layer_index + 1) * vertices_per_layer - 1);

    //               << " Max index: " << max_index2.index;

    ConnectPointToLayer(scaffold_graph_index, base_graph_index,
                        base_graph_vertex_index, obstacle_flag, safety_margin,
                        min_index2, max_index2, fast_multi_graph);
  }
}

void InsertScaffolds(const graph::GraphIndex& base_graph_index,
                     const ObstacleFlag& obstacle_flag,
                     const SafetyMargin& safety_margin,
                     const float distance_threshold, const float layer_offset,
                     const int num_layers, const int vertices_per_layer,
                     FastMultiGraph* fast_multi_graph) {
  const Graph& base_graph = fast_multi_graph->GetGraph(base_graph_index);

  // Connects each vertex in the base graph to the scaffold graphs.
  for (size_t base_graph_vertex_index_value = 0;
       base_graph_vertex_index_value < base_graph.GetNumVertices();
       ++base_graph_vertex_index_value) {
    const graph::VertexIndex base_graph_vertex_index(
        base_graph_vertex_index_value);

    auto obs_flag_itr = obstacle_flag.begin();
    for (graph::GraphIndex scaffold_index = 0;
         scaffold_index < fast_multi_graph->GetGraphs().size();
         ++scaffold_index) {
      if (scaffold_index != base_graph_index) {
        ConnectPointToScaffold(
            scaffold_index, base_graph_index, base_graph_vertex_index,
            obstacle_flag, safety_margin, distance_threshold, *obs_flag_itr,
            layer_offset, num_layers, vertices_per_layer, fast_multi_graph);
        ++obs_flag_itr;
      }
    }
  }

  auto outer_obs_flag_itr = obstacle_flag.begin();
  for (graph::GraphIndex scaffold_index_outer = 0;
       scaffold_index_outer < fast_multi_graph->GetGraphs().size();
       ++scaffold_index_outer) {
    if (scaffold_index_outer == base_graph_index) {
      continue;
    }
    auto inner_obs_flag_itr = obstacle_flag.begin();
    for (graph::GraphIndex scaffold_index_inner = 0;
         scaffold_index_inner < fast_multi_graph->GetGraphs().size();
         ++scaffold_index_inner) {
      if (scaffold_index_inner == base_graph_index) {
        continue;
      }
      if (scaffold_index_outer != scaffold_index_inner) {
        const auto& outer_position =
            (*outer_obs_flag_itr)->GetPose().translation;
        const auto& inner_position =
            (*inner_obs_flag_itr)->GetPose().translation;
        const float max_distance = (*outer_obs_flag_itr)->GetRadius() +
                                   (*inner_obs_flag_itr)->GetRadius() +
                                   (2 * num_layers * layer_offset) +
                                   distance_threshold;
        if ((outer_position - inner_position).squaredNorm() <
            Sq(max_distance)) {
          for (size_t inner_vertex_index_value = 0;
               inner_vertex_index_value <
               fast_multi_graph->GetGraph(scaffold_index_inner)
                   .GetNumVertices();
               ++inner_vertex_index_value) {
            const graph::VertexIndex inner_vertex_index(
                inner_vertex_index_value);
            ConnectPointToScaffold(
                scaffold_index_outer, scaffold_index_inner, inner_vertex_index,
                obstacle_flag, safety_margin, distance_threshold,
                *outer_obs_flag_itr, layer_offset, num_layers,
                vertices_per_layer, fast_multi_graph);
          }
        }
      }
      ++inner_obs_flag_itr;
    }
    ++outer_obs_flag_itr;
  }
}

static const float kEpsilonPad = 0.1;

Graph GenerateCircleScaffold(const Pose2Df& pose, const float radius,
                             int number_of_layers, int vertices_per_layer,
                             float layer_offset) {
  Graph scaffold;
  const Vector2f& center = pose.translation;

  float scaffold_radius =
      2.0f * radius /
          (sqrt(2.0f) * sqrt(cos(2.0f * M_PI / vertices_per_layer) + 1.0f)) +
      kEpsilonPad;

  const float kPositionParam = 2.0f * M_PI / vertices_per_layer;
  for (int j = 0; j < number_of_layers; j++) {
    // Add vertices for the layer.
    for (int i = 0; i < vertices_per_layer; i++) {
      const float base_theta = i * kPositionParam;
      const float offset = (((j % 2) != 0) ? kPositionParam / 2.0f : 0) +
                           kPositionParam * (j / 2);
      const float theta = base_theta + offset;
      const Vector2f position(scaffold_radius * cos(theta),
                              scaffold_radius * sin(theta));

      scaffold.AddVertex(position + center);
    }

    // Connect edges within the layer.
    for (int i = 0; i < vertices_per_layer; i++) {
      graph::VertexIndex from_index(i + vertices_per_layer * j);
      graph::VertexIndex to_index(i - 1 + vertices_per_layer * j);
      if (i == 0) {
        to_index =
            graph::VertexIndex(vertices_per_layer - 1 + vertices_per_layer * j);
      }
      const float weight = (scaffold.GetVertex(from_index).position -
                            scaffold.GetVertex(to_index).position)
                               .norm();
      scaffold.AddEdge(from_index, to_index, weight);
    }

    // Connect edges between layers.
    if (j > 0) {
      for (int i = 0; i < vertices_per_layer; i++) {
        graph::VertexIndex current_vertex(i + vertices_per_layer * j);
        graph::VertexIndex prev_smaller_vertex(i +
                                               vertices_per_layer * (j - 1));
        graph::VertexIndex prev_larger_vertex(i + 1 +
                                              vertices_per_layer * (j - 1));

        if (i >= vertices_per_layer - 1) {
          prev_larger_vertex =
              graph::VertexIndex(0 + vertices_per_layer * (j - 1));
        }

        const float smaller_weight =
            (scaffold.GetVertex(current_vertex).position -
             scaffold.GetVertex(prev_smaller_vertex).position)
                .norm();
        const float larger_weight =
            (scaffold.GetVertex(current_vertex).position -
             scaffold.GetVertex(prev_larger_vertex).position)
                .norm();
        scaffold.AddEdge(current_vertex, prev_smaller_vertex, smaller_weight);
        scaffold.AddEdge(current_vertex, prev_larger_vertex, larger_weight);
      }
    }
    scaffold_radius += layer_offset;
  }
  return scaffold;
}

Graph GenerateRectangleScaffold(const Pose2Df& pose, const float width,
                                const float height,
                                const float scaffold_distance,
                                int number_of_layers, int vertices_per_layer,
                                float layer_offset) {
  Graph scaffold;

  Eigen::Rotation2Df rotation(pose.angle);

  const std::vector<Vector2f> corner_normals = {
      (rotation * Vector2f(width, height)).normalized(),
      (rotation * Vector2f(width, -height)).normalized(),
      (rotation * Vector2f(-width, -height)).normalized(),
      (rotation * Vector2f(-width, height)).normalized()};

  const std::vector<Vector2f> corner_points = {
      rotation * Vector2f(width, height) + pose.translation,
      rotation * Vector2f(width, -height) + pose.translation,
      rotation * Vector2f(-width, -height) + pose.translation,
      rotation * Vector2f(-width, height) + pose.translation};

  const std::vector<Vector2f> side_normals = {
      (rotation * Vector2f(width, 0)).normalized(),
      (rotation * Vector2f(0, -height)).normalized(),
      (rotation * Vector2f(-width, 0)).normalized(),
      (rotation * Vector2f(0, height)).normalized()};

  if (number_of_layers == 1) {
    for (int i = 0; i < vertices_per_layer; i++) {
      Vector2f position;

      if ((4 * i) % vertices_per_layer == 0) {
        // Corner Point
        int corner_index =
            static_cast<int>(floor(i * 4 / vertices_per_layer)) % 4;

        position =
            corner_points[corner_index] +
            scaffold_distance * corner_normals[corner_index].normalized();
      }  // else {
      // Side point
      //         int side_index = floor(i * 4 / vertices_per_layer);
      //         side_index = side_index % 4;
      //
      //         position = corner_points[side_index] +
      //                    (i - vertices_per_layer / 4.0f) /
      //                        (vertices_per_layer / 4.0f) *
      //                        side_directions[side_index] +
      //                    scaffold_distance *
      //                    side_normals[side_index].normalized();
      //       }

      scaffold.AddVertex(position);

      //       vertices_not_transformed_.push_back(position);
      //       graph_.vertices.push_back(Vertex(position + center));
      //
      //       Edge edge;
      //       edge.from_index = i;
      //       if (i == vertices_per_layer - 1) {
      //         edge.to_index = 0;
      //       } else {
      //         edge.to_index = edge.from_index + 1;
      //       }
      //       graph_.edge_vector.push_back(edge);
    }
  }
  return scaffold;
}

Graph GenerateTempCircleScaffold(const Pose2Df& pose) {
  Graph scaffold;
  const Eigen::Rotation2Df rotation(pose.angle);

  // Move in the clockwise direction.
  const std::vector<std::pair<int, int>> multipliers = {
      {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

  for (const auto& multiplier : multipliers) {
    const int x_multiplier = multiplier.first;
    const int y_multiplier = multiplier.second;
    const Vector2f offset((kRobotRadius + kDefaultSafetyMargin) * x_multiplier,
                          (kRobotRadius + kDefaultSafetyMargin) * y_multiplier);
    scaffold.AddVertex(pose.translation + rotation * offset);
  }
  for (size_t i = 0; i < multipliers.size() - 1; ++i) {
    const graph::VertexIndex vi1(i);
    const graph::VertexIndex vi2(i + 1);
    const float weight =
        (scaffold.GetVertex(vi1).position - scaffold.GetVertex(vi2).position)
            .norm();
    scaffold.AddEdge(vi1, vi2, weight);
  }
  const graph::VertexIndex v0(0);
  const graph::VertexIndex vlast(multipliers.size() - 1);
  const float weight =
      (scaffold.GetVertex(v0).position - scaffold.GetVertex(vlast).position)
          .norm();
  scaffold.AddEdge(v0, vlast, weight);
  return scaffold;
}

void ModifyScaffoldPose(const Pose2Df& old_pose, const Pose2Df& new_pose,
                        Graph* graph) {
  const float angle_delta = new_pose.angle - old_pose.angle;
  const Eigen::Rotation2Df rotation(angle_delta);

  for (auto& vertex : *graph->GetMutableVertices()) {
    const Vector2f old_scaffold_frame_position =
        vertex.position - old_pose.translation;
    const Vector2f new_scaffold_frame_position =
        rotation * old_scaffold_frame_position;
    vertex.position = new_pose.translation + new_scaffold_frame_position;
  }
}

// Connects the specified point in the base graph to the specified scaffold
// graph.
void LazyConnectPointToLayer(const GraphIndex scaffold_graph_index,
                             const GraphIndex base_graph_index,
                             const graph::VertexIndex& base_graph_vertex_index,
                             const ObstacleFlag& obstacle_flag,
                             const SafetyMargin& safety_margin,
                             const graph::VertexIndex& ring_lower_vertex_index,
                             const graph::VertexIndex& ring_upper_vertex_index,
                             FastMultiGraph* fast_multi_graph) {
  if (!kProduction) {
    if (ring_lower_vertex_index.index >=
        fast_multi_graph->GetGraph(scaffold_graph_index).GetNumVertices()) {
      LOG(FATAL)
          << "Lower vertex index bound out of range ("
          << ring_lower_vertex_index.index << " vs "
          << fast_multi_graph->GetGraph(scaffold_graph_index).GetNumVertices()
          << ")";
    }
    if (ring_upper_vertex_index.index >=
        fast_multi_graph->GetGraph(scaffold_graph_index).GetNumVertices()) {
      LOG(FATAL)
          << "Lower vertex index bound out of range("
          << ring_upper_vertex_index.index << " vs "
          << fast_multi_graph->GetGraph(scaffold_graph_index).GetNumVertices()
          << ")";
    }
  }

  //   fast_multi_graph->GetMutableGraph(graph_index)
  const graph::VertexIndex nearest_scaffold_vertex_index =
      NearestNeighborInScaffold(
          fast_multi_graph->GetGraph(scaffold_graph_index),
          fast_multi_graph->GetGraph(base_graph_index)
              .GetVertex(base_graph_vertex_index)
              .position,
          ring_lower_vertex_index, ring_upper_vertex_index);
  //             << nearest_scaffold_vertex_index.index;

  const graph::VertexIndex neighbor_a =
      ShiftVertexIndex(nearest_scaffold_vertex_index, 1,
                       ring_lower_vertex_index, ring_upper_vertex_index);

  const graph::VertexIndex neighbor_b =
      ShiftVertexIndex(nearest_scaffold_vertex_index, -1,
                       ring_lower_vertex_index, ring_upper_vertex_index);

  const auto& base_graph_point = fast_multi_graph->GetGraph(base_graph_index)
                                     .GetVertex(base_graph_vertex_index)
                                     .position;
  const auto& scaffold_nearest_point =
      fast_multi_graph->GetGraph(scaffold_graph_index)
          .GetVertex(nearest_scaffold_vertex_index)
          .position;
  const auto& scaffold_a_point =
      fast_multi_graph->GetGraph(scaffold_graph_index)
          .GetVertex(neighbor_a)
          .position;
  const auto& scaffold_b_point =
      fast_multi_graph->GetGraph(scaffold_graph_index)
          .GetVertex(neighbor_b)
          .position;

  const auto weight1 = (base_graph_point - scaffold_nearest_point).norm();
  fast_multi_graph->AddInterGraphEdge(graph::edges::InterGraphEdge(
      true, weight1, base_graph_index, base_graph_vertex_index,
      base_graph_point, scaffold_graph_index, nearest_scaffold_vertex_index,
      scaffold_nearest_point));

  const auto weight2 = (base_graph_point - scaffold_a_point).norm();
  fast_multi_graph->AddInterGraphEdge(graph::edges::InterGraphEdge(
      true, weight2, base_graph_index, base_graph_vertex_index,
      base_graph_point, scaffold_graph_index, neighbor_a, scaffold_a_point));

  const auto weight3 = (base_graph_point - scaffold_b_point).norm();
  fast_multi_graph->AddInterGraphEdge(graph::edges::InterGraphEdge(
      true, weight3, base_graph_index, base_graph_vertex_index,
      base_graph_point, scaffold_graph_index, neighbor_b, scaffold_b_point));
}

void LazyConnectPointToScaffold(
    const GraphIndex scaffold_graph_index, const GraphIndex base_graph_index,
    const graph::VertexIndex& base_graph_vertex_index,
    const ObstacleFlag& obstacle_flag, const SafetyMargin& safety_margin,
    const float distance_threshold, const obstacle::Obstacle* obstacle,
    const float layer_offset, const int num_layers,
    const int vertices_per_layer, FastMultiGraph* fast_multi_graph) {
  const auto& graph_position = fast_multi_graph->GetGraph(base_graph_index)
                                   .GetVertex(base_graph_vertex_index)
                                   .position;
  const float max_radius = obstacle->GetRadius() +
                           safety_margin.GetMargin(obstacle->GetType()) +
                           layer_offset * num_layers + distance_threshold;
  if ((obstacle->GetPose().translation - graph_position).squaredNorm() >
       Sq(max_radius)) {
    return;
  }

  // This will be -1 when inside of the innermost layer.
  const int lower_layer_index = std::max(
      -1,
      std::min(num_layers - 1,
               static_cast<int>(floor(
                   ((obstacle->GetPose().translation - graph_position).norm() -
                   (obstacle->GetRadius() +
                   safety_margin.GetMargin(obstacle->GetType()))) /
                   static_cast<float>(layer_offset)))));


  if (lower_layer_index >= num_layers - 1) {
    int min_index_val =
    std::max(lower_layer_index - 1, 0) * vertices_per_layer + 1;
    int max_index_val = (lower_layer_index + 1) * vertices_per_layer - 1;
    const graph::VertexIndex min_index(min_index_val);
    const graph::VertexIndex max_index(max_index_val);

    //               << " Max index: " << max_index.index;

    LazyConnectPointToLayer(scaffold_graph_index, base_graph_index,
                            base_graph_vertex_index, obstacle_flag,
                            safety_margin, min_index, max_index,
                            fast_multi_graph);
  } else if (lower_layer_index < 0) {
    const graph::VertexIndex min_index((lower_layer_index + 1) *
                                       vertices_per_layer);
    const graph::VertexIndex max_index(
        (lower_layer_index + 2) * vertices_per_layer - 1);

    //               << " Max index: " << max_index.index;

    LazyConnectPointToLayer(scaffold_graph_index, base_graph_index,
                            base_graph_vertex_index, obstacle_flag,
                            safety_margin, min_index, max_index,
                            fast_multi_graph);
  } else {
    const graph::VertexIndex min_index1((lower_layer_index + 1) *
                                        vertices_per_layer);
    const graph::VertexIndex max_index1(
        (lower_layer_index + 2) * vertices_per_layer - 1);

    //               << " Max index: " << max_index1.index;

    LazyConnectPointToLayer(scaffold_graph_index, base_graph_index,
                            base_graph_vertex_index, obstacle_flag,
                            safety_margin, min_index1, max_index1,
                            fast_multi_graph);
    const graph::VertexIndex min_index2((lower_layer_index)*vertices_per_layer);
    const graph::VertexIndex max_index2(
        (lower_layer_index + 1) * vertices_per_layer - 1);

    //               << " Max index: " << max_index2.index;

    LazyConnectPointToLayer(scaffold_graph_index, base_graph_index,
                           base_graph_vertex_index, obstacle_flag,
                           safety_margin, min_index2, max_index2,
                           fast_multi_graph);
  }
}

void LazyInsertScaffolds(const graph::GraphIndex& base_graph_index,
                         const ObstacleFlag& obstacle_flag,
                         const SafetyMargin& safety_margin,
                         const float distance_threshold,
                         const float layer_offset,
                         const int num_layers, const int vertices_per_layer,
                         FastMultiGraph* fast_multi_graph) {
  const Graph& base_graph = fast_multi_graph->GetGraph(base_graph_index);

  // Connects each vertex in the base graph to the scaffold graphs.
  for (size_t base_graph_vertex_index_value = 0;
       base_graph_vertex_index_value < base_graph.GetNumVertices();
       ++base_graph_vertex_index_value) {
    const graph::VertexIndex base_graph_vertex_index(
        base_graph_vertex_index_value);

    auto obs_flag_itr = obstacle_flag.begin();
    for (graph::GraphIndex scaffold_index = 0;
         scaffold_index < fast_multi_graph->GetGraphs().size();
         ++scaffold_index) {
      if (scaffold_index != base_graph_index) {
        LazyConnectPointToScaffold(
            scaffold_index, base_graph_index, base_graph_vertex_index,
            obstacle_flag, safety_margin, distance_threshold, *obs_flag_itr,
            layer_offset, num_layers, vertices_per_layer, fast_multi_graph);
        ++obs_flag_itr;
      }
    }
  }

  auto outer_obs_flag_itr = obstacle_flag.begin();
  for (graph::GraphIndex scaffold_index_outer = 0;
       scaffold_index_outer < fast_multi_graph->GetGraphs().size();
       ++scaffold_index_outer) {
    if (scaffold_index_outer == base_graph_index) {
      continue;
    }
    auto inner_obs_flag_itr = obstacle_flag.begin();
    for (graph::GraphIndex scaffold_index_inner = 0;
         scaffold_index_inner < fast_multi_graph->GetGraphs().size();
         ++scaffold_index_inner) {
      if (scaffold_index_inner == base_graph_index) {
        continue;
      }
      if (scaffold_index_outer != scaffold_index_inner) {
        const auto& outer_position =
            (*outer_obs_flag_itr)->GetPose().translation;
        const auto& inner_position =
            (*inner_obs_flag_itr)->GetPose().translation;
        const float max_distance = (*outer_obs_flag_itr)->GetRadius() +
                                   (*inner_obs_flag_itr)->GetRadius() +
                                   (2 * num_layers * layer_offset) +
                                   distance_threshold;
        if ((outer_position - inner_position).squaredNorm() <
            Sq(max_distance)) {
          for (size_t inner_vertex_index_value = 0;
               inner_vertex_index_value <
               fast_multi_graph->GetGraph(scaffold_index_inner)
                   .GetNumVertices();
               ++inner_vertex_index_value) {
            const graph::VertexIndex inner_vertex_index(
                inner_vertex_index_value);
            LazyConnectPointToScaffold(
                scaffold_index_outer, scaffold_index_inner, inner_vertex_index,
                obstacle_flag, safety_margin, distance_threshold,
                *outer_obs_flag_itr, layer_offset, num_layers,
                vertices_per_layer, fast_multi_graph);
          }
        }
      }
      ++inner_obs_flag_itr;
    }
    ++outer_obs_flag_itr;
  }
}


}  // namespace scaffolding
}  // namespace navigation

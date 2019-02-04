// Copyright 2016 - 2018 kvedder@umass.edu slane@cs.umass.edu
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

#include "navigation/PRM.h"

#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <random>
#include <vector>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Sparse"
#include "graph/graph.h"
#include "graph/vertex.h"
#include "math/poses_2d.h"
#include "navigation/navigation_util.h"
#include "navigation/scaffolding/scaffold.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"

using datastructures::BetterMap;
using geometry::EuclideanDistance;
using Eigen::SparseMatrix;
using Eigen::Vector2f;
using field_dimensions::kHalfFieldLength;
using field_dimensions::kHalfFieldWidth;
using obstacle::Obstacle;
using obstacle::ObstacleType;
using obstacle::SafetyMargin;
using obstacle::ObstacleFlag;
using pose_2d::Pose2Df;
using std::function;
using std::map;
using std::priority_queue;
using std::vector;
using graph::Graph;
using graph::Vertex;
using graph::GraphIndex;
using graph::EdgeIndex;
using graph::Edge;
using graph::edges::InterGraphEdge;
using graph::edges::InterGraphEdgeIndex;
using graph::VertexIndex;
using graph::multi_graph::FastMultiGraph;

namespace navigation {

PRM::PRM(const Graph& initial_graph, const ObstacleFlag& obstacles,
         const SafetyMargin& safety_margin, const bool use_scaffolding,
         const int number_of_layers, const int vertices_per_layer,
         const float layer_offset, const size_t seed)
    : obstacles_(obstacles),
      safety_margin_(safety_margin),
      start_position_(0, 0),
      goal_position_(0, 0),
      start_index_(0),
      goal_index_(0),
      multi_graph_(initial_graph, &base_graph_index_),
      random_(seed),
      abort_search_(false),
      use_scaffolding_(use_scaffolding),
      number_of_layers_(number_of_layers),
      vertices_per_layer_(vertices_per_layer),
      layer_offset_(layer_offset) {
  if (use_scaffolding) {
    for (const obstacle::Obstacle* obstacle : obstacles) {
      if (obstacle->GetType() == obstacle::ObstacleType::ROBOT) {
        multi_graph_.AddAdditionalGraph(scaffolding::GenerateCircleScaffold(
            obstacle->GetPose(),
            obstacle->GetRadius() +
                safety_margin_.GetMargin(obstacle->GetType()),
            number_of_layers, vertices_per_layer, layer_offset));
      }
    }
  }
}

// First element in graph is assumed to be start and second element is assumed
// to be the goal
PRM::PRM(const ObstacleFlag& obstacles, const SafetyMargin& safety_margin,
         const size_t base_graph_vertices, const bool use_scaffolding,
         const int number_of_layers, const int vertices_per_layer,
         const float layer_offset, const size_t seed)
    : obstacles_(obstacles),
      safety_margin_(safety_margin),
      start_position_(0, 0),
      goal_position_(0, 0),
      start_index_(0),
      goal_index_(0),
      random_(seed),
      abort_search_(false),
      use_scaffolding_(use_scaffolding),
      number_of_layers_(number_of_layers),
      vertices_per_layer_(vertices_per_layer),
      layer_offset_(layer_offset) {
  base_graph_index_ = multi_graph_.AddAdditionalGraph(Graph());
  Graph* base_graph = multi_graph_.GetMutableGraph(base_graph_index_);

  // Construct base graph.
  Vector2f sample_point;
  for (size_t i = 0; i < base_graph_vertices; ++i) {
    SamplePoint(&sample_point);

    //     bool sample_point_aborted;
    //     ClosestPointInFreeSpace(obstacles_,
    //                             safety_margin_,
    //                             &sample_point,
    //                             &sample_point_aborted);

    // TODO(slane): Figure out something sensible to do if it has aborted

    base_graph->AddVertex(sample_point);

    vector<std::pair<GraphIndex, VertexIndex>> nearest_vertices =
        NearestNeighbors(sample_point);
    for (const std::pair<GraphIndex, VertexIndex>& near_vertex_index_pair :
         nearest_vertices) {
      // Avoids adding self edge.
      // Avoids adding start or goal
      if (near_vertex_index_pair.second.index != i &&
          near_vertex_index_pair.second.index > 1) {
        const Vector2f& near_vertex =
            base_graph->GetVertex(near_vertex_index_pair.second).position;
        if (CollisionFreePath(obstacles_, safety_margin_, sample_point,
                              near_vertex)) {
          float distance = EuclideanDistance(sample_point, near_vertex);
          base_graph->AddEdge(VertexIndex(i), near_vertex_index_pair.second,
                              distance);
        }
      }
    }
  }

  if (use_scaffolding) {
    for (const obstacle::Obstacle* obstacle : obstacles) {
      if (obstacle->GetType() == obstacle::ObstacleType::ROBOT) {
        multi_graph_.AddAdditionalGraph(scaffolding::GenerateCircleScaffold(
            obstacle->GetPose(),
            obstacle->GetRadius() +
                safety_margin_.GetMargin(obstacle->GetType()),
            number_of_layers, vertices_per_layer, layer_offset));
      }
    }
  }
}

void PRM::Update(const ObstacleFlag& obstacles,
                 const Eigen::Vector2f& current_pose, const Vector2f& goal_pose,
                 logger::Logger* logger) {
  SafetyMargin margin;
  Update(obstacles, margin, current_pose, goal_pose, logger);
}

void PRM::Update(const ObstacleFlag& obstacles,
                 const SafetyMargin& safety_margin,
                 const Eigen::Vector2f& current_pose, const Vector2f& goal_pose,
                 logger::Logger* logger) {
  safety_margin_ = safety_margin;
  start_position_ = current_pose;
  goal_position_ = goal_pose;
  abort_search_ = false;

  // Reset original graphs
  multi_graph_.ResetEdges();

  // Invalidate blocked edges
  InvalidateBlockedEdges();

  if (use_scaffolding_) {
    auto old_obs_itr = obstacles_.begin();
    auto new_obs_itr = obstacles.begin();

    for (graph::GraphIndex i = 1; i < multi_graph_.GetGraphs().size(); ++i) {
      scaffolding::ModifyScaffoldPose((*old_obs_itr)->GetPose(),
                                      (*new_obs_itr)->GetPose(),
                                      multi_graph_.GetMutableGraph(i));
      ++old_obs_itr;
      ++new_obs_itr;
    }

    scaffolding::InsertScaffolds(
        base_graph_index_, obstacles, safety_margin, kScaffoldConnectRadius,
        layer_offset_, number_of_layers_, vertices_per_layer_, &multi_graph_);
  }

  obstacles_ = obstacles;

  // Move the start and goal points so they are not in collision with any
  // obstacles
  //   bool start_aborted;
  //   ClosestPointInFreeSpace(obstacles_,
  //                           safety_margin_,
  //                           &start_position_,
  //                           &start_aborted);
  //
  //   bool goal_aborted;
  //   ClosestPointInFreeSpace(obstacles_,
  //                           safety_margin_,
  //                           &goal_position_,
  //                           &goal_aborted);

  // TODO(slane): Figure out a sensible thing to do when these abort

  // Connect the current position to its nearest neighbors
  bool found_start =
      ClosestFreeVertex(start_position_, &start_graph_, &start_index_);
  bool found_goal =
      ClosestFreeVertex(goal_position_, &goal_graph_, &goal_index_);
  if (!found_start || !found_goal) {
    abort_search_ = true;
  }
}

std::pair<bool, std::vector<Eigen::Vector2f>> PRM::Plan(
    logger::Logger* logger) {
  std::vector<Eigen::Vector2f> plan;
  bool found_plan;
  vector<std::pair<GraphIndex, VertexIndex>> waypoint_list;

  Vector2f one_shot_end_point;
  OneShotMode(obstacles_, safety_margin_, start_position_,
                     goal_position_, &one_shot_end_point);

  // If we can just go from the start to the goal, do that, it will be better
  // than anything PRM gets and it saves us computation.
  found_plan = EuclideanDistance(one_shot_end_point, goal_position_) < kEpsilon;

  if (!(found_plan) && !abort_search_) {
    if (!AStarSearch(start_graph_, start_index_, goal_graph_, goal_index_,
                     &waypoint_list)) {
      return {found_plan, plan};
    }
    found_plan = true;
    plan.push_back(start_position_);
    // A path has been found from the start to the goal
    // Convert the waypoint list to a list of vector2fs
    for (const std::pair<GraphIndex, VertexIndex>& index_pair : waypoint_list) {
      plan.push_back(
          multi_graph_.GetVertex(index_pair.first, index_pair.second).position);
    }
    plan.push_back(goal_position_);

    // Smooth the path
    // TODO(slane): Keeping the smoothing out for comparison at the moment
    // cost = SmoothPath(obstacles_, safety_margin_, plan);

  } else {
    // Either we can go directly from start to goal or we had to use one-shot
    // mode, either way, the path consists of two points, start and the end
    // point for one shot mode.
    plan.push_back(start_position_);
    plan.push_back(one_shot_end_point);
  }

  return {found_plan, plan};
}

const graph::multi_graph::FastMultiGraph& PRM::GetGraph() const {
  return multi_graph_;
}

bool PRM::CompareVertices(const PRM::PriorityQueueVertex& pe1,
                          const PRM::PriorityQueueVertex& pe2) {
  return (pe1.total_distance + pe1.heuristic_value) >
         (pe2.total_distance + pe2.heuristic_value);
}

bool PRM::AStarSearch(
    GraphIndex start_graph, VertexIndex start_index, GraphIndex goal_graph,
    VertexIndex goal_index,
    vector<std::pair<GraphIndex, VertexIndex>>* waypoint_list) const {
  if (!kProduction) {
    if (!waypoint_list->empty()) {
      LOG(ERROR) << "Given non-empty vector to add waypoints to!!!!";
      waypoint_list->clear();
    }
  }

  map<std::pair<GraphIndex, /*Unwrapped VertexIndex*/ size_t>,
      std::pair<GraphIndex, VertexIndex>>
      path_map;
  vector<std::pair<GraphIndex, VertexIndex>> closed_vertices;

  priority_queue<PRM::PriorityQueueVertex, vector<PRM::PriorityQueueVertex>,
                 function<bool(const PRM::PriorityQueueVertex&,
                               const PRM::PriorityQueueVertex&)>>
      pq(CompareVertices);

  // Add the start node as the initial position in the priority queue.
  float heuristic_value = EuclideanDistance(start_position_, goal_position_);
  pq.push(PriorityQueueVertex(0, heuristic_value, start_graph, start_index,
                              start_graph, kStartNodePredecessor));

  while (!pq.empty()) {
    const PriorityQueueVertex top_priorirty_queue_vertex = pq.top();
    pq.pop();
    const GraphIndex& graph_index = top_priorirty_queue_vertex.vertex_graph;
    const VertexIndex& vertex_index = top_priorirty_queue_vertex.vertex_index;

    // Check to see that the current vertex isn't in the closed vertex list.
    if (std::find(closed_vertices.begin(), closed_vertices.end(),
                  std::make_pair(graph_index, vertex_index)) ==
        closed_vertices.end()) {
      closed_vertices.push_back(std::make_pair(graph_index, vertex_index));
      path_map.insert(std::make_pair(
          std::make_pair(graph_index, vertex_index.index),
          std::make_pair(top_priorirty_queue_vertex.prev_vertex_graph,
                         top_priorirty_queue_vertex.prev_vertex_index)));

      // Goal found, begin unwind process.
      if (vertex_index == goal_index && graph_index == goal_graph) {
        std::pair<GraphIndex, VertexIndex> current_index =
            std::make_pair(goal_graph, goal_index);
        while (current_index.second != start_index ||
               current_index.first != start_graph) {
          waypoint_list->push_back(current_index);
          current_index = path_map
                              .find(std::make_pair(current_index.first,
                                                   current_index.second.index))
                              ->second;
        }
        waypoint_list->push_back(current_index);
        std::reverse(waypoint_list->begin(), waypoint_list->end());
        return true;
      }

      std::vector<EdgeIndex> next_graph_edges;
      std::vector<InterGraphEdgeIndex> next_inter_graph_edges;
      multi_graph_.GetVertexEdges(graph_index, vertex_index, &next_graph_edges,
                                  &next_inter_graph_edges);

      // Look at all the edges that are connected to this vertex in the graph.
      for (const EdgeIndex& edge_index : next_graph_edges) {
        const Edge& edge = multi_graph_.GetGraphEdge(graph_index, edge_index);
        if (edge.is_valid) {
          const VertexIndex other_vertex_index =
              (edge.index_1 != vertex_index) ? edge.index_1 : edge.index_2;
          float total_distance =
              top_priorirty_queue_vertex.total_distance + edge.weight;
          float heuristic_value = EuclideanDistance(
              multi_graph_.GetVertex(graph_index, other_vertex_index).position,
              goal_position_);
          pq.push(PriorityQueueVertex(total_distance, heuristic_value,
                                      graph_index, other_vertex_index,
                                      graph_index, vertex_index));
        }
      }

      // Look at all the edges that are connected to this vertex that go between
      // graphs.
      for (const InterGraphEdgeIndex& edge_index : next_inter_graph_edges) {
        const InterGraphEdge& edge = multi_graph_.GetInterGraphEdge(edge_index);
        if (edge.is_valid) {
          const VertexIndex other_vertex_index =
              (edge.graph_1 != graph_index || edge.vertex_1 != vertex_index)
                  ? edge.vertex_1
                  : edge.vertex_2;
          const GraphIndex other_graph_index =
              (edge.graph_1 != graph_index || edge.vertex_1 != vertex_index)
                  ? edge.graph_1
                  : edge.graph_2;
          float total_distance =
              top_priorirty_queue_vertex.total_distance + edge.weight;
          float heuristic_value = EuclideanDistance(
              multi_graph_.GetVertex(other_graph_index, other_vertex_index)
                  .position,
              goal_position_);
          pq.push(PriorityQueueVertex(total_distance, heuristic_value,
                                      other_graph_index, other_vertex_index,
                                      graph_index, vertex_index));
        }
      }
    }
  }

  return false;
}

void PRM::InvalidateBlockedEdges() {
  multi_graph_.InvalidateBlockedEdges(obstacles_, safety_margin_);
}

void PRM::SamplePoint(Vector2f* sample_position) {
  sample_position->x() = static_cast<float>(
      random_.UniformRandom(-kHalfFieldLength, kHalfFieldLength));
  sample_position->y() = static_cast<float>(
      random_.UniformRandom(-kHalfFieldWidth, kHalfFieldWidth));
}

vector<std::pair<GraphIndex, VertexIndex>> PRM::NearestNeighbors(
    const Vector2f& input_position) const {
  vector<std::pair<GraphIndex, VertexIndex>> index_vector;
  GraphIndex graph_index = 0;
  for (const Graph& graph : multi_graph_.GetGraphs()) {
    for (size_t i = 0; i < graph.GetNumVertices(); ++i) {
      const VertexIndex vertex_index(i);
      if (EuclideanDistance(
              input_position,
              multi_graph_.GetVertex(graph_index, vertex_index).position) <=
          kSearchMargin) {
        index_vector.push_back(std::make_pair(graph_index, vertex_index));
      }
    }
    ++graph_index;
  }
  return index_vector;
}

bool PRM::ClosestFreeVertex(const Vector2f& position, GraphIndex* graph_index,
                            VertexIndex* vertex_index) {
  if (!kProduction) {
    CHECK_NOTNULL(graph_index);
    CHECK_NOTNULL(vertex_index);
  }
  vector<std::pair<GraphIndex, VertexIndex>> nearest_vertices =
      NearestNeighbors(position);

  // No nearby vertices.
  if (nearest_vertices.empty()) {
    return false;
  }

  bool found_point = false;
  float closest_vertex_distance_to_free_point =
      std::numeric_limits<float>::max();
  VertexIndex closest_vertex_index(0);
  GraphIndex closest_vertex_graph = 0;

  for (const std::pair<GraphIndex, VertexIndex>& near_vertex_index_pair :
       nearest_vertices) {
    const Vertex& near_vertex = multi_graph_.GetVertex(
        near_vertex_index_pair.first, near_vertex_index_pair.second);
    float distance_to_free_point =
        EuclideanDistance(position, near_vertex.position);
    if ((distance_to_free_point < closest_vertex_distance_to_free_point) &&
        CollisionFreePath(obstacles_, safety_margin_, position,
                          near_vertex.position)) {
      closest_vertex_distance_to_free_point = distance_to_free_point;
      closest_vertex_graph = near_vertex_index_pair.first;
      closest_vertex_index = near_vertex_index_pair.second;
      found_point = true;
    }
  }

  *graph_index = closest_vertex_graph;
  *vertex_index = closest_vertex_index;
  return found_point;
}

}  // namespace navigation

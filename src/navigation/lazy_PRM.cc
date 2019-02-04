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

#include "navigation/lazy_PRM.h"

#include <functional>
#include <queue>
#include <map>
#include <utility>
#include <vector>

#include "constants/includes.h"
#include "graph/edge.h"
#include "graph/graph.h"
#include "graph/graph_util.h"
#include "graph/fastmultigraph.h"
#include "graph/vertex.h"
#include "logging/logger.h"
#include "navigation/navigation_util.h"
#include "navigation/PRM.h"
#include "navigation/scaffolding/scaffold.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"

STANDARD_USINGS;
using geometry::EuclideanDistance;
using graph::Edge;
using graph::EdgeIndex;
using graph::edges::InterGraphEdge;
using graph::edges::InterGraphEdgeIndex;
using graph::Graph;
using graph::GraphIndex;
using graph::VertexIndex;
using logger::Logger;
using obstacle::ObstacleFlag;
using obstacle::SafetyMargin;
using pose_2d::Pose2Df;
using std::function;
using std::map;
using std::pair;
using std::priority_queue;
using std::size_t;

namespace navigation {
LazyPRM::LazyPRM(const graph::Graph& initial_graph,
                 const ObstacleFlag& obstacles,
                 const SafetyMargin& safety_margin, const bool use_scaffolding,
                 const int number_of_layers, const int vertices_per_layer,
                 const float layer_offset, const size_t seed)
    : PRM(initial_graph, obstacles, safety_margin, use_scaffolding,
          number_of_layers, vertices_per_layer, layer_offset, seed) {}

LazyPRM::LazyPRM(const ObstacleFlag& obstacles,
                 const SafetyMargin& safety_margin,
                 const std::size_t base_graph_vertices,
                 const bool use_scaffolding, const int number_of_layers,
                 const int vertices_per_layer, const float layer_offset,
                 const std::size_t seed)
    : PRM(obstacles, safety_margin, base_graph_vertices, use_scaffolding,
          number_of_layers, vertices_per_layer, layer_offset, seed) {}



void LazyPRM::Update(const ObstacleFlag& obstacles,
                     const Vector2f& current_pose,
                     const Vector2f& goal_pose,
                     Logger* logger) {
  SafetyMargin margin;
  Update(obstacles,
         margin,
         current_pose,
         goal_pose,
         logger);
}

void LazyPRM::Update(const ObstacleFlag& obstacles,
                     const SafetyMargin& safety_margin,
                     const Vector2f& current_pose,
                     const Vector2f& goal_pose,
                     Logger* logger) {
  safety_margin_ = safety_margin;
  start_position_ = current_pose;
  goal_position_ = goal_pose;
  abort_search_ = false;

  // Reset original graphs
  multi_graph_.ResetEdges();

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

    scaffolding::LazyInsertScaffolds(
      base_graph_index_, obstacles, safety_margin, kScaffoldConnectRadius,
      layer_offset_, number_of_layers_, vertices_per_layer_, &multi_graph_);
  }

  obstacles_ = obstacles;

  // Connect the current position to its nearest neighbors
  bool found_start =
      ClosestFreeVertex(start_position_, &start_graph_, &start_index_);

  bool found_goal =
      ClosestFreeVertex(goal_position_, &goal_graph_, &goal_index_);

  if (!found_start || !found_goal) {
    abort_search_ = true;
  }
}


std::pair<bool, std::vector<Eigen::Vector2f>> LazyPRM::Plan(
    logger::Logger* logger) {
  // Empty anything in the existing plan
  std::vector<Eigen::Vector2f> plan;
  bool found_plan;
  vector<std::pair<GraphIndex, VertexIndex>> waypoint_list;

  Vector2f one_shot_end_point;
  OneShotMode(obstacles_, safety_margin_, start_position_,
                     goal_position_, &one_shot_end_point);

  // If we can just go from the start to the goal, do that, it will be better
  // than anything PRM gets and it saves us computation.
  //   *found_plan =
  //       EuclideanDistance(one_shot_end_point, goal_position_) < kEpsilon;
  found_plan = false;

  if (!found_plan && !abort_search_ &&
      AStarSearch(start_graph_, start_index_, goal_graph_, goal_index_,
                  &waypoint_list)) {
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
    // SmoothPath(obstacles_, safety_margin_, plan);
  } else {
    // Either we can go directly from start to goal or we had to use one-shot
    // mode, either way, the path consists of two points, start and the end
    // point for one shot mode.
    plan.push_back(start_position_);
    plan.push_back(one_shot_end_point);
  }

  return {found_plan, plan};
}

bool LazyPRM::AStarSearch(
    GraphIndex start_graph, VertexIndex start_index, GraphIndex goal_graph,
    VertexIndex goal_index,
    vector<pair<GraphIndex, VertexIndex>>* waypoint_list) const {
  if (!kProduction) {
    if (!waypoint_list->empty()) {
      LOG(ERROR) << "Given non-empty vector to add waypoints to!!!!";
      waypoint_list->clear();
    }
  }

  map<pair<GraphIndex, /*Unwrapped VertexIndex*/ size_t>,
      pair<GraphIndex, VertexIndex>>
      path_map;
  vector<std::pair<GraphIndex, VertexIndex>> closed_vertices;

  priority_queue<PRM::PriorityQueueVertex, vector<PRM::PriorityQueueVertex>,
                 function<bool(const PRM::PriorityQueueVertex&,
                    const PRM::PriorityQueueVertex&)>>
      pq(PRM::CompareVertices);

  // Add the start node as the initial position in the priority queue.
  float heuristic_value = EuclideanDistance(start_position_, goal_position_);
  pq.push(PriorityQueueVertex(0, heuristic_value, start_graph,
                              start_index, start_graph, kStartNodePredecessor));

  while (!pq.empty()) {
    const PriorityQueueVertex top_priorirty_queue_vertex = pq.top();
    pq.pop();
    const GraphIndex& graph_index = top_priorirty_queue_vertex.vertex_graph;
    const VertexIndex& vertex_index = top_priorirty_queue_vertex.vertex_index;

    // Check to see that the current vertex isn't in the closed vertex list.
    if (std::find(closed_vertices.begin(), closed_vertices.end(),
                  std::make_pair(graph_index, vertex_index)) ==
        closed_vertices.end()) {
      closed_vertices.push_back(std::make_pair(graph_index,
                                                vertex_index));
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

      const Vector2f& current_vertex =
        multi_graph_.GetVertex(graph_index, vertex_index).position;

      // Look at all the edges that are connected to this vertex in the graph.
      for (const EdgeIndex& edge_index : next_graph_edges) {
        const Edge& edge = multi_graph_.GetGraphEdge(graph_index, edge_index);
        const VertexIndex other_vertex_index =
            (edge.index_1 != vertex_index) ? edge.index_1 : edge.index_2;
        const Vector2f& other_vertex =
            multi_graph_.GetVertex(graph_index, other_vertex_index).position;

        if (CollisionFreePath(obstacles_, safety_margin_, current_vertex,
                              other_vertex)) {
          float total_distance =
              top_priorirty_queue_vertex.total_distance + edge.weight;
          float heuristic_value =
              EuclideanDistance(other_vertex, goal_position_);
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
          const Vector2f& other_vertex =
              multi_graph_.GetVertex(other_graph_index,
                                     other_vertex_index).position;

          if (CollisionFreePath(obstacles_, safety_margin_, current_vertex,
                                other_vertex)) {
            float total_distance =
                top_priorirty_queue_vertex.total_distance + edge.weight;
            float heuristic_value =
                EuclideanDistance(other_vertex, goal_position_);
            pq.push(PriorityQueueVertex(total_distance, heuristic_value,
                                        other_graph_index, other_vertex_index,
                                        graph_index, vertex_index));
          }
        }
      }
    }
  }

  return false;
}


};  // namespace navigation

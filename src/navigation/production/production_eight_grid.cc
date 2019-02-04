// Copyright 2018 kvedder@umass.edu
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

#include "navigation/production/production_eight_grid.h"

#include <algorithm>
#include <array>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

STANDARD_USINGS;
#include "util/array_util.h"

namespace navigation {
namespace production {
namespace eight_grid {

ProductionEightGrid::ProductionEightGrid(
    const CollisionGrid& static_collision_grid,
    const CollisionGrid& dynamic_collision_grid)
    : static_collision_grid_(static_collision_grid),
      dynamic_collision_grid_(dynamic_collision_grid),
      distance_between_vertices_(kEightGridSquareSize) {}

ProductionEightGrid::ProductionEightGrid(
    const CollisionGrid& static_collision_grid,
    const CollisionGrid& dynamic_collision_grid,
    const float distance_between_vertices)
    : static_collision_grid_(static_collision_grid),
      dynamic_collision_grid_(dynamic_collision_grid),
      distance_between_vertices_(distance_between_vertices) {}

void ProductionEightGrid::Update(const obstacle::ObstacleFlag& obstacles,
                                 const obstacle::SafetyMargin& safety_margin,
                                 const FreeSpaceVertex& current_pose,
                                 const FreeSpaceVertex& goal_pose,
                                 logger::Logger* logger) {
  navigation::Navigation::Update(obstacles, safety_margin, current_pose,
                                 goal_pose, logger);
}

float ProductionEightGrid::Heuristic(const GridVertex& position,
                                     const GridVertex& goal) const {
  const GridVertex delta = (position - goal).cwiseAbs();
  const int& x_diff = delta.x();
  const int& y_diff = delta.y();

  int smaller, larger;
  if (x_diff < y_diff) {
    smaller = x_diff;
    larger = y_diff;
  } else {
    smaller = y_diff;
    larger = x_diff;
  }
  const float result =
      distance_between_vertices_ * (larger + smaller * (1.0f - kSqrtTwo));
  NP_CHECK_MSG(result >= 0.0f, "Heuristic was " << result);
  return result;
}

GridVertex ProductionEightGrid::FreeSpaceToGridVertex(
    const FreeSpaceVertex& free_space_vector) const {
  return util::FreeSpaceToGridVertex(free_space_vector,
                                     distance_between_vertices_);
}

FreeSpaceVertex ProductionEightGrid::GridVertexToFreeSpace(
    const GridVertex& grid_vertex) const {
  return util::GridVertexToFreeSpace(grid_vertex, distance_between_vertices_);
}

NeighborArray ProductionEightGrid::GetNeighbors(
    const GridVertex& vertex) const {
  return {
      {//       {{GridVertex(vertex.x() + 1, vertex.y() - 1),
       //         distance_between_vertices_ * kSqrtTwo},
       //        {GridVertex(vertex.x() + 1, vertex.y() + 1),
       //         distance_between_vertices_ * kSqrtTwo},
       //        {GridVertex(vertex.x() - 1, vertex.y() - 1),
       //         distance_between_vertices_ * kSqrtTwo},
       //        {GridVertex(vertex.x() - 1, vertex.y() + 1),
       //         distance_between_vertices_ * kSqrtTwo},
       {GridVertex(vertex.x() + 1, vertex.y()), distance_between_vertices_},
       {GridVertex(vertex.x(), vertex.y() - 1), distance_between_vertices_},
       {GridVertex(vertex.x(), vertex.y() + 1), distance_between_vertices_},
       {GridVertex(vertex.x() - 1, vertex.y()), distance_between_vertices_}}};
}

NeighborCollisions ProductionEightGrid::GetNeighborCollisions(
    const NeighborArray& neighbor_array) const {
  NeighborCollisions collision_array =
      array_util::MakeArray<kNumNeighbors>(false);

  for (size_t i = 0; i < kNumNeighbors; ++i) {
    const auto& neighbor = neighbor_array[i].first;
    collision_array[i] =
        (static_collision_grid_.IsColliding(neighbor, obstacles_) ||
         dynamic_collision_grid_.IsColliding(neighbor, obstacles_));
  }
  return collision_array;
}

bool ProductionEightGrid::IsColliding(
    const GridVertex& center, const NeighborArray& neighbor_array,
    const NeighborCollisions& neighbor_collisions, const size_t index) const {
  NP_CHECK(index < kNumNeighbors);
  return neighbor_collisions[index];
}

void VerifyCollisionFreePoint(const FreeSpaceVertex& v,
                              const obstacle::ObstacleFlag& obstacles,
                              const obstacle::SafetyMargin& safety_margin) {
  if (!kProduction) {
    for (const auto* obstacle : obstacles) {
      NP_CHECK_MSG(!obstacle->PointCollision(
                       v, safety_margin.GetMargin(obstacle->GetType())),
                   "Query point " << v.x() << ", " << v.y()
                                  << " is colliding with obstacle centered at "
                                  << obstacle->GetPose().translation.x() << ", "
                                  << obstacle->GetPose().translation.y());
    }
  }
}

GridVertex ProductionEightGrid::FreeSpaceToOpenGridVertex(
    const FreeSpaceVertex& free_space_vector) const {
  const GridVertex closest_point = FreeSpaceToGridVertex(free_space_vector);
  // Controls ordering to be sensible.
  static const std::array<GridVertex, 91> offset_array = {
      {{0, 0},   {-1, 0},  {1, 0},   {0, -1},  {0, 1},   {-1, -1}, {1, -1},
       {-1, 1},  {1, 1},   {-2, 0},  {2, 0},   {0, -2},  {0, 2},   {-2, -1},
       {-2, 1},  {2, -1},  {2, 1},   {-2, -1}, {-2, 1},  {1, 2},   {-1, 2},
       {1, -2},  {-1, -2}, {2, 2},   {2, -2},  {-2, 2},  {-2, -2}, {3, 0},
       {-3, 0},  {0, 3},   {0, 3},   {3, 3},   {-3, 3},  {3, -3},  {-3, -3},
       {4, 0},   {-4, 0},  {0, 4},   {0, 4},   {4, 4},   {-4, 4},  {4, -4},
       {-4, -4}, {5, 0},   {-5, 0},  {0, 5},   {0, 5},   {5, 5},   {-5, 5},
       {5, -5},  {-5, -5}, {6, 0},   {-6, 0},  {0, 6},   {0, 6},   {6, 6},
       {-6, 6},  {6, -6},  {-6, -6}, {7, 0},   {-7, 0},  {0, 7},   {0, 7},
       {7, 7},   {-7, 7},  {7, -7},  {-7, -7}, {8, 0},   {-8, 0},  {0, 8},
       {0, 8},   {8, 8},   {-8, 8},  {8, -8},  {-8, -8}, {9, 0},   {-9, 0},
       {0, 9},   {0, 9},   {9, 9},   {-9, 9},  {9, -9},  {-9, -9}, {9, 3},
       {9, -3},  {-9, 3},  {-9, -3}, {3, 9},   {-3, 9},  {3, -9},  {-3, -9}}};

  for (const GridVertex& offset : offset_array) {
    const GridVertex proposed_position = closest_point + offset;
    const bool proposed_free =
        !static_collision_grid_.IsColliding(proposed_position, obstacles_) &&
        !dynamic_collision_grid_.IsColliding(proposed_position, obstacles_);
    if (proposed_free) {
      return proposed_position;
    }
  }

  if (!kProduction) {
    LOG(ERROR) << "Unable to find free grid vector for free space vector "
               << free_space_vector.x() << ", " << free_space_vector.y();
  }
  return closest_point;
}

bool IsStraightPathClear(const FreeSpaceVertex& start,
                         const FreeSpaceVertex& goal,
                         const obstacle::ObstacleFlag& obstacles,
                         const obstacle::SafetyMargin& safety_margin) {
  for (const auto* obstacle : obstacles) {
    if (obstacle->LineCollision(start, goal,
                                safety_margin.GetMargin(obstacle->GetType()))) {
      return false;
    }
  }
  return true;
}

void ValidatePath(const FreeSpacePath& path,
                  const obstacle::ObstacleFlag& obstacles,
                  const obstacle::SafetyMargin& saftey_margin) {
  if (!kProduction) {
    for (const auto* obstacle : obstacles) {
      for (const auto& p : path) {
        if (obstacle->PointCollision(
                p, saftey_margin.GetMargin(obstacle->GetType()))) {
          LOG(ERROR) << "Point " << p.x() << ", " << p.y() << " collides";
          const auto& obs_position = obstacle->GetPose().translation;
          LOG(ERROR) << "Obstacle at " << obs_position.x() << ", "
                     << obs_position.y() << " of type: " << obstacle->GetType();
          for (const auto& p2 : path) {
            if (p == p2) {
              LOG(ERROR) << p2.x() << ", " << p2.y();
            } else {
              LOG(INFO) << p2.x() << ", " << p2.y();
            }
          }
          LOG(FATAL) << "Quitting!";
        }
      }
    }
  }
}

void DrawGridGoal(logger::Logger* robot_logger, const FreeSpaceVertex& goal) {
  robot_logger->AddLine(goal - Vector2f(10, 10), goal + Vector2f(10, 10), 0, 0,
                        1, 1);
  robot_logger->AddLine(goal - Vector2f(10, -10), goal + Vector2f(10, -10), 0,
                        0, 1, 1);
  robot_logger->AddCircle(goal, 10, 0, 0, 1, 1);
  robot_logger->AddCircle(goal, 15, 0, 0, 1, 1);
}

std::pair<bool, FreeSpacePath> ProductionEightGrid::Plan(
    logger::Logger* logger) {
  return Plan(logger, true);
}

std::pair<bool, FreeSpacePath> ProductionEightGrid::Plan(
    logger::Logger* logger, const bool use_oneshot_mode) {
  static constexpr int kMaxGoalCells = 3;
  if (use_oneshot_mode && IsStraightPathClear(start_position_, goal_position_,
                                              obstacles_, safety_margin_)) {
    logger->LogPrint("Straight line path is free.");
    logger->Pop();
    return {true, {start_position_, goal_position_}};
  }
  const GridVertex start = FreeSpaceToOpenGridVertex(start_position_);
  const GridVertex goal = FreeSpaceToOpenGridVertex(goal_position_);

  DrawGridGoal(logger, GridVertexToFreeSpace(goal));

  // Exit early if a free goal cell is too far away from the free space goal.
  if ((FreeSpaceToGridVertex(goal_position_) - goal).lpNorm<1>() >
      kMaxGoalCells) {
    return {false, {}};
  }

  const FreeSpaceVertex start_free = GridVertexToFreeSpace(start);
  const FreeSpaceVertex goal_free = GridVertexToFreeSpace(goal);
  logger->Push();
  if (start == goal) {
    logger->LogPrint("Start is goal (%f, %f)", start_free.x(), start_free.y());
    logger->Pop();
    return {true, {start_free, goal_free, goal_position_}};
  }
  logger->Pop();
  const auto result = AStarSearch(start, goal, logger);
  //   ValidatePath(result.second, obstacles_, safety_margin_);
  return result;
}

FreeSpacePath ProductionEightGrid::UnwindPath(
    const std::unordered_map<GridVertex, GridVertex, GridHasher>& path_map,
    const GridVertex& start, const GridVertex& goal) const {
  FreeSpacePath grid_path;
  GridVertex current = goal;
  size_t iter = 0;
  for (; current != start && iter < path_map.size(); ++iter) {
    grid_path.push_back(GridVertexToFreeSpace(current));
    // Is in parent map.
    NP_CHECK_MSG(path_map.find(current) != path_map.end(),
                 "Current not in path map!");
    NP_CHECK_MSG(path_map.find(current)->second != current,
                 "Self loop detected");
    NP_CHECK_MSG(
        std::find(grid_path.begin(), grid_path.end(),
                  GridVertexToFreeSpace(path_map.find(current)->second)) ==
            grid_path.end(),
        "Loop detected");
    current = path_map.find(current)->second;
  }

  if (iter >= path_map.size()) {
    return {};
  }
  grid_path.push_back(GridVertexToFreeSpace(start));
  std::reverse(grid_path.begin(), grid_path.end());
  grid_path.push_back(goal_position_);
  return grid_path;
}

std::pair<bool, FreeSpacePath> ProductionEightGrid::AStarSearch(
    const GridVertex& start, const GridVertex& goal, logger::Logger* logger) {
  const size_t kMaxIterations =
      static_cast<size_t>(std::ceil(kFieldLength / distance_between_vertices_) *
                          std::ceil(kFieldWidth / distance_between_vertices_));
  logger->Push();
  std::unordered_set<GridVertex, GridHasher> closed_vertices(5000);
  std::unordered_map<GridVertex, GridVertex, GridHasher> path_map(5000);
  std::priority_queue<PriorityQueueVertex, std::vector<PriorityQueueVertex>>
      priority_queue;
  priority_queue.push({0, 0, start, start});
  size_t iter = 0;
  for (; !priority_queue.empty() && iter < kMaxIterations; ++iter) {
    const PriorityQueueVertex top_priorirty_queue_vertex = priority_queue.top();
    priority_queue.pop();
    const GridVertex& current_vertex =
        top_priorirty_queue_vertex.current_vertex;
    const GridVertex& previous_vertex =
        top_priorirty_queue_vertex.previous_vertex;
    if (!closed_vertices.insert(current_vertex).second) {
      continue;
    }

    path_map.insert({current_vertex, previous_vertex});

    if (current_vertex == goal) {
      logger->Pop();
      const auto unwound_path = UnwindPath(path_map, start, goal);
      if (unwound_path.empty()) {
        logger->LogPrint("Unwind failed even though a path was found.");
        return {false, {}};
      }
      return {true, UnwindPath(path_map, start, goal)};
    }

    const NeighborArray neighbor_array = GetNeighbors(current_vertex);
    const NeighborCollisions neighbor_collisions =
        GetNeighborCollisions(neighbor_array);
    for (size_t i = 0; i < neighbor_array.size(); ++i) {
      if (IsColliding(current_vertex, neighbor_array, neighbor_collisions, i)) {
        continue;
      }
      const GridVertex& next_vertex = neighbor_array[i].first;
      const float distance =
          top_priorirty_queue_vertex.total_distance + neighbor_array[i].second;
      const float heuristic = Heuristic(next_vertex, goal);

      // Throw away states that are already closed, as they are optimal.
      if (closed_vertices.find(next_vertex) != closed_vertices.end()) {
        continue;
      }

      NP_CHECK(heuristic >= 0.0f);
      NP_CHECK(distance >= 0.0f);
      priority_queue.push({distance, heuristic, next_vertex, current_vertex});
    }
  }
  if (iter >= kMaxIterations) {
    logger->LogPrint("Exceeded max search iterations of %d", kMaxIterations);
  } else {
    logger->LogPrint("No path found, queue was exhausted!");
  }
  logger->Pop();
  return {false, {}};
}

}  // namespace eight_grid
}  // namespace production
}  // namespace navigation

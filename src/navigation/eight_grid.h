// Copyright 2017 kvedder@umass.edu
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

#ifndef SRC_NAVIGATION_EIGHT_GRID_H_
#define SRC_NAVIGATION_EIGHT_GRID_H_

#include <gtest/gtest_prod.h>
#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "logging/logger.h"

#include "navigation/navigation.h"

namespace navigation {
namespace eight_grid {

class EightGrid : public Navigation {
 public:
  EightGrid() = default;
  // Constructs an 8 grid with a vertex at (0, 0), and verticies at intervals of
  // the given distances in all cardinal directions.
  EightGrid(const obstacle::ObstacleFlag& obstacles,
            const obstacle::SafetyMargin& safety_margin,
            const float distance_between_vertices);
  ~EightGrid() = default;

  void Update(const obstacle::ObstacleFlag& obstacles,
              const obstacle::SafetyMargin& safety_margin,
              const Eigen::Vector2f& current_pose,
              const Eigen::Vector2f& goal_pose,
              logger::Logger* logger) override;

  std::pair<bool, std::vector<Eigen::Vector2f>> Plan(
      logger::Logger* logger) override;

  std::pair<bool, std::vector<Eigen::Vector2i>> PlanGridIndex(
      logger::Logger* logger);

  struct GridHasher {
    std::size_t operator()(const Eigen::Vector2i& k) const {
      return k.dot(Eigen::Vector2i(1, field_dimensions::kFieldLength));
    }
  };

  const std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                           EightGrid::GridHasher>&
  GetStaticInvalidVertices();

  const std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                           EightGrid::GridHasher>&
  GetDynamicInvalidVertices();

 private:
  FRIEND_TEST(EightGrid, FreeSpaceToGridVertex);
  FRIEND_TEST(EightGrid, VertexToIndex);

  struct PriorityQueueVertex {
    float total_distance;
    float heuristic_value;
    Eigen::Vector2i current_vertex;
    Eigen::Vector2i previous_vertex;

    PriorityQueueVertex() = delete;
    PriorityQueueVertex(float total_distance, float heuristic_value,
                        const Eigen::Vector2i& current_vertex,
                        const Eigen::Vector2i& previous_vertex)
        : total_distance(total_distance),
          heuristic_value(heuristic_value),
          current_vertex(current_vertex),
          previous_vertex(previous_vertex) {}
    ~PriorityQueueVertex() = default;

    bool operator<(const PriorityQueueVertex& other) const {
      return ((total_distance + heuristic_value) >
              (other.total_distance + other.heuristic_value));
    }

    bool operator==(const PriorityQueueVertex& other) const {
      return (total_distance == other.total_distance &&
              heuristic_value == other.heuristic_value &&
              current_vertex == other.current_vertex &&
              previous_vertex == other.previous_vertex);
    }
  };

  float Heuristic(const Eigen::Vector2i& position, const Eigen::Vector2i& goal);

  std::array<std::pair<Eigen::Vector2i, float>, 8> GetNeighbors(
      const Eigen::Vector2i& vertex) const;

  // Returns the location of the nearest grid vertex.
  Eigen::Vector2i FreeSpaceToGridVertex(const Vector2f& free_space_vector);

  // Returns the location of the nearest free grid vertex.
  Eigen::Vector2i FreeSpaceToOpenGridVertex(const Vector2f& free_space_vector);

  Vector2f GridVertexToFreeSpace(const Eigen::Vector2i& in);

  void PrintQueue(
      std::priority_queue<EightGrid::PriorityQueueVertex,
                          std::vector<EightGrid::PriorityQueueVertex>>
          pq) const;

  std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                     EightGrid::GridHasher>
  CalculateInvalidVertices(logger::Logger* logger);

  std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                     EightGrid::GridHasher>
  CalculateInvalidStaticVertices();

  bool AStarSearch(const Eigen::Vector2f& start, const Eigen::Vector2f& goal,
                   std::vector<Eigen::Vector2i>* waypoint_list,
                   logger::Logger* logger);

  int VertexToIndex(const Eigen::Vector2i& vec);
  float distance_between_vertices_;

  int total_vertex_count_;

  std::vector<bool> closed_vertices_;
  std::unordered_map<Eigen::Vector2i, Eigen::Vector2i, GridHasher> path_map_;

  std::priority_queue<PriorityQueueVertex, std::vector<PriorityQueueVertex>>
      priority_queue_;

  std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                     EightGrid::GridHasher>
      static_invalid_vertices_;

  std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                     EightGrid::GridHasher>
      dynamic_invalid_vertices_;
};
}  // namespace eight_grid
}  // namespace navigation

#endif  // SRC_NAVIGATION_EIGHT_GRID_H_

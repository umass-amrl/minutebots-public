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

#ifndef SRC_NAVIGATION_PRODUCTION_PRODUCTION_EIGHT_GRID_H_
#define SRC_NAVIGATION_PRODUCTION_PRODUCTION_EIGHT_GRID_H_

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "navigation/navigation.h"
#include "navigation/production/collision_grid.h"
#include "navigation/production/eight_grid_common_utils.h"

namespace navigation {
namespace production {
namespace eight_grid {

static constexpr size_t kNumNeighbors = 4;
using NeighborArray = std::array<std::pair<GridVertex, float>, kNumNeighbors>;
using NeighborCollisions = std::array<bool, kNumNeighbors>;

class ProductionEightGrid : public Navigation {
 public:
  ProductionEightGrid(const CollisionGrid& static_collision_grid,
                      const CollisionGrid& dynamic_collision_grid);
  ProductionEightGrid(const CollisionGrid& static_collision_grid,
                      const CollisionGrid& dynamic_collision_grid,
                      const float distance_between_vertices);
  ~ProductionEightGrid() = default;

  void Update(const obstacle::ObstacleFlag& obstacles,
              const obstacle::SafetyMargin& safety_margin,
              const FreeSpaceVertex& current_pose,
              const FreeSpaceVertex& goal_pose,
              logger::Logger* logger) override;

  std::pair<bool, std::vector<Eigen::Vector2f>> Plan(
      logger::Logger* logger) override;

  std::pair<bool, std::vector<Eigen::Vector2f>> Plan(
      logger::Logger* logger, const bool use_oneshot_mode);

 private:
  float Heuristic(const GridVertex& position, const GridVertex& goal) const;

  GridVertex FreeSpaceToGridVertex(
      const FreeSpaceVertex& free_space_vector) const;

  Eigen::Vector2f GridVertexToFreeSpace(const GridVertex& grid_vertex) const;

  std::pair<bool, FreeSpacePath> AStarSearch(const GridVertex& start,
                                             const GridVertex& goal,
                                             logger::Logger* logger);

  NeighborArray GetNeighbors(const GridVertex& vertex) const;

  NeighborCollisions GetNeighborCollisions(
      const NeighborArray& neighbor_array) const;

  bool IsColliding(const GridVertex& center,
                   const NeighborArray& neighbor_array,
                   const NeighborCollisions& neighbor_collisions,
                   const size_t index) const;
  FreeSpacePath UnwindPath(
      const std::unordered_map<GridVertex, GridVertex, GridHasher>& path_map,
      const GridVertex& start, const GridVertex& goal) const;

  GridVertex FreeSpaceToOpenGridVertex(
      const FreeSpaceVertex& free_space_vector) const;

  struct PriorityQueueVertex {
    float total_distance;
    float heuristic_value;
    GridVertex current_vertex;
    // Used to insert parent into parent map.
    GridVertex previous_vertex;

    PriorityQueueVertex() = delete;
    PriorityQueueVertex(float total_distance, float heuristic_value,
                        const GridVertex& current_vertex,
                        const GridVertex& previous_vertex)
        : total_distance(total_distance),
          heuristic_value(heuristic_value),
          current_vertex(current_vertex),
          previous_vertex(previous_vertex) {}
    ~PriorityQueueVertex() = default;

    bool operator<(const PriorityQueueVertex& other) const {
      const float f_val = (total_distance + heuristic_value);
      const float other_f_val = (other.total_distance + other.heuristic_value);
      if (f_val == other_f_val) {
        return total_distance < other.total_distance;
      }
      return (f_val > other_f_val);
    }

    bool operator==(const PriorityQueueVertex& other) const {
      return (total_distance == other.total_distance &&
              heuristic_value == other.heuristic_value &&
              current_vertex == other.current_vertex &&
              previous_vertex == other.previous_vertex);
    }
  };

  const CollisionGrid& static_collision_grid_;
  const CollisionGrid& dynamic_collision_grid_;
  const float distance_between_vertices_;
};

}  // namespace eight_grid
}  // namespace production
}  // namespace navigation

#endif  // SRC_NAVIGATION_PRODUCTION_PRODUCTION_EIGHT_GRID_H_

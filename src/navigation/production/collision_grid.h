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

#ifndef SRC_NAVIGATION_PRODUCTION_COLLISION_GRID_H_
#define SRC_NAVIGATION_PRODUCTION_COLLISION_GRID_H_

#include <bitset>
#include <unordered_map>
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "gui/opengl_helpers.h"
#include "logging/logger.h"
#include "navigation/production/eight_grid_common_utils.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"

namespace navigation {
namespace production {
namespace eight_grid {

class CollisionGrid {
 public:
  CollisionGrid();
  explicit CollisionGrid(const float distance_between_verticies);
  ~CollisionGrid() = default;

  void RebuildDynamic(const obstacle::SafetyMargin& margin);
  void RebuildStatic(const obstacle::SafetyMargin& margin);

  bool IsColliding(const GridVertex& vertex,
                   const obstacle::ObstacleFlag& obstacles) const;

  void DrawVertices(logger::Logger* logger,
                    const opengl_helpers::Color4f& color,
                    const obstacle::ObstacleFlag& obstacle_flag) const;

  std::vector<std::vector<bool>> GenerateDenseOccupancyGrid(
      const obstacle::ObstacleFlag& obstacles) const;

 private:
  std::pair<int, int> GetGridIndexBounds(const int bounding_box_min,
                                         const int bounding_box_max);

  void InvalidateObstacle(const obstacle::Obstacle* obstacle,
                          const obstacle::SafetyMargin& margin,
                          const std::bitset<kNumObstacles>& invalidated_bitset);

  void Rebuild(const obstacle::ObstacleFlag& obstacles,
               const obstacle::SafetyMargin& margin);

  const float distance_between_verticies_;
  std::unordered_map<GridVertex, std::bitset<kNumObstacles>, GridHasher>
      colliding_verticies_;
};

}  // namespace eight_grid
}  // namespace production
}  // namespace navigation

#endif  // SRC_NAVIGATION_PRODUCTION_COLLISION_GRID_H_

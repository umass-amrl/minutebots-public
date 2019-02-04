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

#ifndef SRC_NAVIGATION_PRODUCTION_EIGHT_GRID_COMMON_UTILS_H_
#define SRC_NAVIGATION_PRODUCTION_EIGHT_GRID_COMMON_UTILS_H_

#include <vector>

#include "constants/constants.h"

namespace navigation {
namespace production {
namespace eight_grid {

using FreeSpaceVertex = Eigen::Vector2f;
using GridVertex = Eigen::Vector2i;
using GridPath = std::vector<GridVertex>;
using RobotPathIndex = std::size_t;
using FreeSpacePath = std::vector<FreeSpaceVertex>;

struct GridHasher {
  std::size_t operator()(const GridVertex& k) const {
    return k.dot(GridVertex(1, field_dimensions::kFieldLength));
  }
};

namespace util {

GridVertex FreeSpaceToGridVertex(const FreeSpaceVertex& free_space_vector,
                                 const float distance_between_vertices);

FreeSpaceVertex GridVertexToFreeSpace(const GridVertex& grid_vertex,
                                      const float distance_between_vertices);

}  // namespace util
}  // namespace eight_grid
}  // namespace production
}  // namespace navigation

#endif  // SRC_NAVIGATION_PRODUCTION_EIGHT_GRID_COMMON_UTILS_H_

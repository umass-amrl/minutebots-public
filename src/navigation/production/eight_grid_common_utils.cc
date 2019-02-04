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
#include "navigation/production/eight_grid_common_utils.h"

namespace navigation {
namespace production {
namespace eight_grid {
namespace util {

GridVertex FreeSpaceToGridVertex(const FreeSpaceVertex& free_space_vector,
                                 const float distance_between_vertices) {
  const float x_multiple = free_space_vector.x() / distance_between_vertices;
  const float y_multiple = free_space_vector.y() / distance_between_vertices;
  const float x_remaining =
      x_multiple - static_cast<float>(static_cast<int>(x_multiple));
  const float y_remaining =
      y_multiple - static_cast<float>(static_cast<int>(y_multiple));
  const int x_index =
      static_cast<int>(x_multiple) +
      ((fabs(x_remaining) > 0.5) ? math_util::Sign<float>(x_remaining) : 0);
  const int y_index =
      static_cast<int>(y_multiple) +
      ((fabs(y_remaining) > 0.5) ? math_util::Sign<float>(y_remaining) : 0);
  return {x_index, y_index};
}

FreeSpaceVertex GridVertexToFreeSpace(const GridVertex& grid_vertex,
                                      const float distance_between_vertices) {
  return grid_vertex.cast<float>() * distance_between_vertices;
}

}  // namespace util
}  // namespace eight_grid
}  // namespace production
}  // namespace navigation

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

#include "search/robocup_eastar/common_defines.h"

namespace search {
namespace eastar {

std::ostream& operator<<(std::ostream& os, const MovingHaltedSteps& jd) {
  os << "{" << jd.transit_steps << ", " << jd.at_goal_steps << "} ";
  return os;
}

namespace util {
GridVertex FreeSpaceToGridVertex(const FreeSpaceVertex& free_space_vertex) {
  return navigation::production::eight_grid::util::FreeSpaceToGridVertex(
      free_space_vertex, kEightGridSquareSize);
}

FreeSpaceVertex GridVertexToFreeSpace(const GridVertex& grid_vertex) {
  return navigation::production::eight_grid::util::GridVertexToFreeSpace(
      grid_vertex, kEightGridSquareSize);
}

GridPath FreeSpacePathToGridPath(const FreeSpacePath& fsp) {
  GridPath gp(fsp.size());
  for (size_t i = 0; i < fsp.size(); ++i) {
    gp[i] = FreeSpaceToGridVertex(fsp[i]);
  }
  return gp;
}

FreeSpaceDistance GridDistanceToFreeSpaceDistance(const GridDistance& gd) {
  return (gd * kEightGridSquareSize);
}

}  // namespace util

}  // namespace eastar
}  // namespace search

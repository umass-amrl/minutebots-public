// Copyright 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
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

#include "search/robocup_eastar/multiagent_data.h"

namespace search {
namespace eastar {

GridVertex FreeSpaceVertexToGridVertex(const FreeSpaceVertex& rp) {
  return (rp / kEightGridSquareSize).cast<int>();
}

FreeSpaceVertex GridVertexToFreeSpaceVertex(const GridVertex& gp) {
  return (gp.cast<float>() * kEightGridSquareSize);
}

std::ostream& operator<<(std::ostream& os, const RobotInfo& robot_info) {
  os << "ID: " << robot_info.ssl_vision_id << " Current Free Space Position: ("
     << robot_info.position.x() << ", " << robot_info.position.y() << ")";
  return os;
}

}  // namespace eastar
}  // namespace search

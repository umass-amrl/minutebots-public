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

#ifndef SRC_SEARCH_ROBOCUP_EASTAR_PATH_COLLISION_DETECTOR_H_
#define SRC_SEARCH_ROBOCUP_EASTAR_PATH_COLLISION_DETECTOR_H_

#include <limits>
#include <vector>

#include "constants/constants.h"
#include "search/robocup_eastar/individual_planner.h"
#include "search/robocup_eastar/multiagent_data.h"

namespace search {
namespace eastar {

struct CollidingRobot {
  RobotPathIndex path_index;
  CollidingRobot() : path_index(std::numeric_limits<RobotPathIndex>::max()) {}
  explicit CollidingRobot(const RobotPathIndex& path_index)
      : path_index(path_index) {}

  bool operator==(const CollidingRobot& other) const {
    return (path_index == other.path_index);
  }
};

template <size_t kMaxRobotCount>
using CollidingRobotArray = SizedStorage<CollidingRobot, kMaxRobotCount>;

template <size_t kMaxRobotCount>
struct CollisionEvent {
  GridVertex collision_center;
  CollidingRobotArray<kMaxRobotCount> colliding_robots;
  CollisionEvent() : collision_center(0, 0), colliding_robots() {}
  CollisionEvent(const GridVertex& collision_center,
                 const CollidingRobotArray<kMaxRobotCount>& colliding_robots)
      : collision_center(collision_center), colliding_robots(colliding_robots) {
    for (const CollidingRobot& cr : colliding_robots) {
      NP_CHECK(cr.path_index < kMaxRobotCount);
    }
  }
};

template <size_t kMaxRobotCount>
std::ostream& operator<<(std::ostream& os,
                         const CollisionEvent<kMaxRobotCount>& event) {
  os << "Center: " << event.collision_center.x() << ", "
     << event.collision_center.y() << "\n Relevant paths:\n";
  for (const auto& r : event.colliding_robots) {
    os << "\tPath index: " << r.path_index << '\n';
  }
  return os;
}

template <size_t kMaxRobotCount>
std::vector<CollisionEvent<kMaxRobotCount>> GetCollisionEvents(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& data_slice);

bool GridPositionsCollide(const GridVertex& p1, const GridVertex& p2,
                          const GridVertex& p1_prev, const GridVertex& p2_prev);

}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_EASTAR_PATH_COLLISION_DETECTOR_H_

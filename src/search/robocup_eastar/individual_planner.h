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

#ifndef SRC_SEARCH_ROBOCUP_EASTAR_INDIVIDUAL_PLANNER_H_
#define SRC_SEARCH_ROBOCUP_EASTAR_INDIVIDUAL_PLANNER_H_

#include <utility>

#include "navigation/production/collision_grid.h"
#include "navigation/production/production_eight_grid.h"
#include "obstacles/obstacle_flag.h"
#include "search/robocup_eastar/common_defines.h"
#include "search/robocup_eastar/multiagent_data.h"
#include "state/position_velocity_state.h"
#include "state/world_state.h"

namespace search {
namespace eastar {

template <size_t kMaxRobotCount>
using IndividualPlansArray = SizedStorage<GridPath, kMaxRobotCount>;

template <size_t kMaxRobotCount>
struct IndividuallyPlannedDataSlice {
  PositionsDataSlice<kMaxRobotCount> positions;
  IndividualPlansArray<kMaxRobotCount> individual_plans;
  navigation::production::eight_grid::CollisionGrid collision_grid;
  ObstaclesArray<kMaxRobotCount> obstacles;

  IndividuallyPlannedDataSlice() = default;
  IndividuallyPlannedDataSlice(
      const PositionsDataSlice<kMaxRobotCount>& positions,
      const IndividualPlansArray<kMaxRobotCount>& individual_plans,
      const navigation::production::eight_grid::CollisionGrid& collision_grid,
      const ObstaclesArray<kMaxRobotCount>& obstacles)
      : positions(positions),
        individual_plans(individual_plans),
        collision_grid(collision_grid),
        obstacles(obstacles) {}

  void Validate() const {
    positions.Validate();
    NP_CHECK_EQ(positions.our_robots.size(), individual_plans.size());
  }
};

template <size_t kMaxRobotCount>
std::ostream& operator<<(
    std::ostream& os,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  os << "Individually Planned Slice:\n";
  os << "Num our robots: " << slice.positions.our_robots.size()
     << " Num their robots: " << slice.positions.their_robots.size() << '\n';
  os << "Ours: \n";

  for (size_t i = 0; i < slice.individual_plans.size(); ++i) {
    const RobotInfo& our_robot_info = slice.positions.our_robots[i];
    os << "[" << our_robot_info << "]\n";
    for (const auto& e : slice.individual_plans[i]) {
      os << "\t" << e.x() << ", " << e.y() << "\n";
    }
  }
  return os;
}

template <size_t kMaxRobotCount>
IndividuallyPlannedDataSlice<kMaxRobotCount>
GenerateIndividuallyPlannedDataSlice(
    const PositionsDataSlice<kMaxRobotCount>& slice);

}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_EASTAR_INDIVIDUAL_PLANNER_H_

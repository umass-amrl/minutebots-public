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

#ifndef SRC_SEARCH_ROBOCUP_EASTAR_MULTIAGENT_DATA_H_
#define SRC_SEARCH_ROBOCUP_EASTAR_MULTIAGENT_DATA_H_

#include <limits>
#include <vector>

#include "constants/constants.h"
#include "datastructures/dense_array.h"
#include "obstacles/obstacle_flag.h"

#include "search/robocup_eastar/common_defines.h"

namespace search {
namespace eastar {

struct RobotInfo {
  SSLVisionId ssl_vision_id;
  FreeSpaceVertex position;
  RobotInfo()
      : ssl_vision_id(std::numeric_limits<SSLVisionId>::max()),
        position(0, 0) {}
  RobotInfo(const SSLVisionId& ssl_vision_id, const FreeSpaceVertex& position)
      : ssl_vision_id(ssl_vision_id), position(position) {}

  bool operator==(const RobotInfo& other) const {
    return (ssl_vision_id == other.ssl_vision_id) &&
           (position == other.position);
  }
};

std::ostream& operator<<(std::ostream& os, const RobotInfo& robot_info);

template <class T, size_t kMaxRobotCount>
using SizedStorage = datastructures::DenseArray<T, kMaxRobotCount>;

template <size_t kMaxRobotCount>
using RobotInfoArray = SizedStorage<RobotInfo, kMaxRobotCount>;

template <size_t kMaxRobotCount>
using GoalArray = SizedStorage<FreeSpaceVertex, kMaxRobotCount>;

template <size_t kMaxRobotCount>
using ObstaclesArray = SizedStorage<obstacle::ObstacleFlag, kMaxRobotCount>;

// template <size_t kMaxRobotCount>
// using OurRobotInfoArray =
//     datastructures::DenseArray<PlannedRobotInfo, kMaxRobotCount>;
// template <size_t kMaxRobotCount>
// using TheirRobotInfoArray =
//     datastructures::DenseArray<RobotInfo, kMaxRobotCount>;

template <size_t kMaxRobotCount>
struct PositionsDataSlice {
  RobotInfoArray<kMaxRobotCount> our_robots;
  RobotInfoArray<kMaxRobotCount> their_robots;
  GoalArray<kMaxRobotCount> goal_positions;
  ObstaclesArray<kMaxRobotCount> obstacles;
  PositionsDataSlice() = default;

  void Validate() const {
    NP_CHECK_EQ(our_robots.size(), goal_positions.size());
    NP_CHECK_EQ(our_robots.size(), obstacles.size());
  }
};

}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_EASTAR_MULTIAGENT_DATA_H_

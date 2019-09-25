// Copyright 2018 - 2019 kvedder@umass.edu
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

#ifndef SRC_SEARCH_ROBOCUP_EASTAR_COMMON_DEFINES_H_
#define SRC_SEARCH_ROBOCUP_EASTAR_COMMON_DEFINES_H_

#include <memory>
#include <utility>
#include <vector>

#include "datastructures/dense_array.h"
#include "navigation/production/eight_grid_common_utils.h"

namespace search {
namespace eastar {
static constexpr size_t kRoboCupEAStarMaxRobots = 8;
static constexpr bool kOptimizeMemoryUsage = false;
static constexpr bool kUseLazyNeighbors = true;
static constexpr bool kRunNWA = false;

using FreeSpaceVertex = navigation::production::eight_grid::FreeSpaceVertex;
using GridVertex = navigation::production::eight_grid::GridVertex;
using RobotPathIndex = navigation::production::eight_grid::RobotPathIndex;
using GridPath = navigation::production::eight_grid::GridPath;
using FreeSpacePath = navigation::production::eight_grid::FreeSpacePath;
using GridHasher = navigation::production::eight_grid::GridHasher;

using GridDistance = int;
using FreeSpaceDistance = float;

// Counts the number of steps to the goal, and number of steps to stay at the
// goal.
struct MovingHaltedSteps {
  int transit_steps;
  int at_goal_steps;
  MovingHaltedSteps() : transit_steps(0), at_goal_steps(0) {}
  MovingHaltedSteps(const int& transit_steps, const int& at_goal_steps)
      : transit_steps(transit_steps), at_goal_steps(at_goal_steps) {}

  MovingHaltedSteps operator+(const MovingHaltedSteps& other) const {
    return {other.transit_steps + transit_steps,
            other.at_goal_steps + at_goal_steps};
  }

  void operator+=(const MovingHaltedSteps& other) {
    transit_steps += other.transit_steps;
    at_goal_steps += other.at_goal_steps;
  }
};

std::ostream& operator<<(std::ostream& os, const MovingHaltedSteps& jd);

template <size_t kMaxRobotCount>
using JointPosition = datastructures::DenseArray<GridVertex, kMaxRobotCount>;
template <size_t kMaxRobotCount>
using JointDistance = datastructures::DenseArray<GridDistance, kMaxRobotCount>;
template <size_t kMaxRobotCount>
using JointSteps =
    datastructures::DenseArray<MovingHaltedSteps, kMaxRobotCount>;
template <size_t kMaxRobotCount>
using JointIndex = datastructures::DenseArray<size_t, kMaxRobotCount>;
template <size_t kMaxRobotCount>
using JointGridPath = std::vector<JointPosition<kMaxRobotCount>>;

using GridIndex = size_t;

using Heuristic = float;

template <size_t kMaxRobotCount>
struct JointPositionHasher {
  std::size_t operator()(const JointPosition<kMaxRobotCount>& jp) const {
    size_t i = 1;
    for (size_t iter = 0; iter < jp.size(); ++iter) {
      const auto& p = jp.at(iter);
      i *= p.dot(Eigen::Vector2i(197 * (static_cast<int>(iter) + 1), 7));
    }
    return i;
  }
};

template <size_t kMaxRobotCount>
std::ostream& operator<<(std::ostream& os,
                         const JointPosition<kMaxRobotCount>& jp) {
  for (const GridVertex& v : jp) {
    os << "(" << v.x() << ", " << v.y() << ") ";
  }
  return os;
}

template <size_t kMaxRobotCount>
std::ostream& operator<<(std::ostream& os,
                         const JointDistance<kMaxRobotCount>& jd) {
  for (const GridDistance& d : jd) {
    os << d << " ";
  }
  return os;
}

template <size_t kMaxRobotCount>
std::ostream& operator<<(std::ostream& os,
                         const JointSteps<kMaxRobotCount>& js) {
  for (const auto& e : js) {
    os << e << " ";
  }
  return os;
}

template <size_t kMaxRobotCount>
std::ostream& operator<<(std::ostream& os,
                         const JointIndex<kMaxRobotCount>& ji) {
  for (const auto& e : ji) {
    os << e << " ";
  }
  return os;
}

template <size_t kMaxRobotCount>
std::ostream& operator<<(std::ostream& os,
                         const JointGridPath<kMaxRobotCount>& jgp) {
  os << "Joint grid path:";
  for (const auto& e : jgp) {
    os << '\n' << e;
  }
  return os;
}

namespace util {
GridVertex FreeSpaceToGridVertex(const FreeSpaceVertex& free_space_vertex);

FreeSpaceVertex GridVertexToFreeSpace(const GridVertex& grid_vertex);

GridPath FreeSpacePathToGridPath(const FreeSpacePath& fsp);

FreeSpaceDistance GridDistanceToFreeSpaceDistance(const GridDistance& gd);

template <size_t kMaxRobotCount>
JointDistance<kMaxRobotCount> JointStepsToJointDistance(
    const JointSteps<kMaxRobotCount>& js) {
  JointDistance<kMaxRobotCount> jd;
  for (const auto& e : js) {
    jd.push_back(e.transit_steps);
  }
  return jd;
}

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace util
}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_EASTAR_COMMON_DEFINES_H_

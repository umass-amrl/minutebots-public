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

#ifndef SRC_SEARCH_ROBOCUP_LOGREADER_LOGREADER_H_
#define SRC_SEARCH_ROBOCUP_LOGREADER_LOGREADER_H_

#include <algorithm>
#include <string>
#include <vector>

#include "search/robocup_eastar/common_defines.h"
#include "search/robocup_eastar/multiagent_data.h"

namespace search {
namespace logreader {

enum Events {
  SINGLE_LARGE,
  MULTI_NON_INTERACT,
  MULTI_INTERACT,
  ROBOCUP_NO_OBSTACLES,
  ROBOCUP_OBSTACLES,
  UNKNOWN
};

#define ALLOWEVENT(event)     \
  if (#event == event_name) { \
    return event;             \
  }

Events GetEvent(const std::string& event_name) {
  ALLOWEVENT(SINGLE_LARGE);
  ALLOWEVENT(MULTI_NON_INTERACT);
  ALLOWEVENT(MULTI_INTERACT);
  ALLOWEVENT(ROBOCUP_NO_OBSTACLES);
  ALLOWEVENT(ROBOCUP_OBSTACLES);
  LOG(FATAL) << "Unknown event: " << event_name;
  return UNKNOWN;
}

search::eastar::GridVertex GetRandomRobocupGridVertex(const int rng_seed) {
  static constexpr bool kDebug = false;
  static util_random::Random random(rng_seed);
  static const int x_min =
      -std::ceil(field_dimensions::kHalfFieldLength / kEightGridSquareSize);
  static const int y_min =
      -std::ceil(field_dimensions::kHalfFieldWidth / kEightGridSquareSize);
  static const int x_max = -x_min;
  static const int y_max = -y_min;
  search::eastar::GridVertex v(random.RandomInt(x_min, x_max),
                               random.RandomInt(y_min, y_max));
  if (kDebug) {
    std::cout << "GetRandomRobocupGridVertex() Vertex: " << v.x() << ", "
              << v.y() << '\n';
  }
  return v;
}

bool FreeSpaceVerticesCollide(const eastar::FreeSpaceVertex& a,
                              const eastar::FreeSpaceVertex& b) {
  return eastar::FreeSpaceVertex(a - b).squaredNorm() <=
         Sq(2 * kRobotRadius + 13);
}

std::vector<
    search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>>
GetRandomRobocupFieldObstacles(const size_t num_robots, const int rng_seed) {
  static constexpr SSLVisionId kHardcodedID = 5;
  search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>
      slice;

  while (slice.our_robots.size() < search::eastar::kRoboCupEAStarMaxRobots) {
    const search::eastar::RobotInfo robot_info(
        kHardcodedID,
        search::eastar::util::GridVertexToFreeSpace(
            GetRandomRobocupGridVertex(rng_seed)));

    bool skip = false;
    // Pass over existing our robots.
    for (const auto& r : slice.our_robots) {
      if (FreeSpaceVerticesCollide(r.position, robot_info.position)) {
        skip = true;
        break;
      }
    }

    if (skip) {
      continue;
    }

    slice.our_robots.push_back(robot_info);
  }

  while (slice.goal_positions.size() <
         search::eastar::kRoboCupEAStarMaxRobots) {
    const search::eastar::FreeSpaceVertex robot_goal =
        search::eastar::util::GridVertexToFreeSpace(
            GetRandomRobocupGridVertex(rng_seed));
    bool skip = false;
    // Pass over existing our robots.
    for (const auto& r : slice.our_robots) {
      if (FreeSpaceVerticesCollide(r.position, robot_goal)) {
        skip = true;
        break;
      }
    }

    // Pass over existing goals.
    for (const auto& p : slice.goal_positions) {
      if (FreeSpaceVerticesCollide(p, robot_goal)) {
        skip = true;
        break;
      }
    }

    if (skip) {
      continue;
    }

    slice.goal_positions.push_back(robot_goal);
  }

  while (slice.their_robots.size() < search::eastar::kRoboCupEAStarMaxRobots) {
    const search::eastar::RobotInfo robot_info(
        kHardcodedID,
        search::eastar::util::GridVertexToFreeSpace(
            GetRandomRobocupGridVertex(rng_seed)));

    bool skip = false;
    // Pass over existing our robots.
    for (const auto& r : slice.our_robots) {
      if (FreeSpaceVerticesCollide(r.position, robot_info.position)) {
        skip = true;
        break;
      }
    }

    // Pass over existing goals.
    for (const auto& p : slice.goal_positions) {
      if (FreeSpaceVerticesCollide(p, robot_info.position)) {
        skip = true;
        break;
      }
    }

    // Pass over existing their robots.
    for (const auto& r : slice.their_robots) {
      if (FreeSpaceVerticesCollide(r.position, robot_info.position)) {
        skip = true;
        break;
      }
    }

    if (skip) {
      continue;
    }

    slice.their_robots.push_back(robot_info);
  }

  for (size_t i = 0; i < search::eastar::kRoboCupEAStarMaxRobots; ++i) {
    slice.obstacles.push_back(
        obstacle::ObstacleFlag::GetAllRobotsExceptTeam(i));
  }

  slice.our_robots.ForceSetSize(num_robots);
  slice.goal_positions.ForceSetSize(num_robots);
  slice.their_robots.ForceSetSize(num_robots);
  slice.obstacles.ForceSetSize(num_robots);

  for (const eastar::RobotInfo& ri : slice.our_robots) {
    std::cout << "Our team:  SSL Vision ID: " << ri.ssl_vision_id
              << " Position: " << ri.position.x() << ", " << ri.position.y()
              << '\n';
  }

  for (const auto& v : slice.goal_positions) {
    std::cout << "Our team goals:  Position: " << v.x() << ", " << v.y()
              << '\n';
  }

  for (const eastar::RobotInfo& ri : slice.their_robots) {
    std::cout << "Their team:  SSL Vision ID: " << ri.ssl_vision_id
              << " Position: " << ri.position.x() << ", " << ri.position.y()
              << '\n';
  }

  slice.Validate();

  return {slice};
}

std::vector<
    search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>>
GetRandomRobocupFieldNoObstacles(const size_t num_robots, const int rng_seed) {
  search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>
      slice;

  while (slice.our_robots.size() < search::eastar::kRoboCupEAStarMaxRobots) {
    const search::eastar::RobotInfo robot_info(
        5,
        search::eastar::util::GridVertexToFreeSpace(
            GetRandomRobocupGridVertex(rng_seed)));
    if (slice.our_robots.Contains(robot_info)) {
      continue;
    }
    slice.our_robots.push_back(robot_info);
  }

  while (slice.goal_positions.size() <
         search::eastar::kRoboCupEAStarMaxRobots) {
    const search::eastar::FreeSpaceVertex robot_goal =
        search::eastar::util::GridVertexToFreeSpace(
            GetRandomRobocupGridVertex(rng_seed));
    if (slice.goal_positions.Contains(robot_goal)) {
      continue;
    }
    slice.goal_positions.push_back(robot_goal);
  }

  for (size_t i = 0; i < search::eastar::kRoboCupEAStarMaxRobots; ++i) {
    slice.obstacles.push_back(
        obstacle::ObstacleFlag::GetAllRobotsExceptTeam(i));
  }

  slice.our_robots.ForceSetSize(num_robots);
  slice.goal_positions.ForceSetSize(num_robots);
  slice.obstacles.ForceSetSize(num_robots);

  slice.Validate();

  return {slice};
}

std::vector<
    search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>>
GetSingleLargeCollision(const size_t num_robots) {
  search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>
      slice;

  const search::eastar::RobotInfo robot0_info(
      5, search::eastar::util::GridVertexToFreeSpace({-10, 0}));
  const search::eastar::RobotInfo robot1_info(
      7, search::eastar::util::GridVertexToFreeSpace({5, -5}));
  const search::eastar::RobotInfo robot2_info(
      9, search::eastar::util::GridVertexToFreeSpace({0, -10}));
  const search::eastar::RobotInfo robot3_info(
      9, search::eastar::util::GridVertexToFreeSpace({0, 10}));
  const search::eastar::RobotInfo robot4_info(
      9, search::eastar::util::GridVertexToFreeSpace({5, 5}));
  const search::eastar::RobotInfo robot5_info(
      9, search::eastar::util::GridVertexToFreeSpace({10, 0}));
  //   const search::eastar::RobotInfo robot6_info(
  //       9, search::eastar::util::GridVertexToFreeSpace({-5, -5}));

  const auto info_list = {robot0_info,
                          robot1_info,
                          robot2_info,
                          robot3_info,
                          robot4_info,
                          robot5_info};

  CHECK(info_list.size() <= search::eastar::kRoboCupEAStarMaxRobots);
  CHECK(num_robots <= info_list.size());

  for (size_t i = 0; i < std::min(num_robots, info_list.size()); ++i) {
    slice.our_robots.push_back(*(info_list.begin() + i));
  }

  const search::eastar::FreeSpaceVertex robot0_goal =
      search::eastar::util::GridVertexToFreeSpace({10, 0});
  const search::eastar::FreeSpaceVertex robot1_goal =
      search::eastar::util::GridVertexToFreeSpace({-5, 5});
  const search::eastar::FreeSpaceVertex robot2_goal =
      search::eastar::util::GridVertexToFreeSpace({0, 10});
  const search::eastar::FreeSpaceVertex robot3_goal =
      search::eastar::util::GridVertexToFreeSpace({0, -10});
  const search::eastar::FreeSpaceVertex robot4_goal =
      search::eastar::util::GridVertexToFreeSpace({-5, -5});
  const search::eastar::FreeSpaceVertex robot5_goal =
      search::eastar::util::GridVertexToFreeSpace({-10, 0});
  //   const search::eastar::FreeSpaceVertex robot6_goal =
  //       search::eastar::util::GridVertexToFreeSpace({5, 5});

  const auto goal_list = {robot0_goal,
                          robot1_goal,
                          robot2_goal,
                          robot3_goal,
                          robot4_goal,
                          robot5_goal};

  CHECK(goal_list.size() <= search::eastar::kRoboCupEAStarMaxRobots);
  CHECK(num_robots <= goal_list.size());

  for (size_t i = 0; i < std::min(num_robots, goal_list.size()); ++i) {
    slice.goal_positions.push_back(*(goal_list.begin() + i));
  }

  for (size_t i = 0; i < slice.our_robots.size(); ++i) {
    slice.obstacles.push_back(
        obstacle::ObstacleFlag::GetAllRobotsExceptTeam(i));
  }

  for (const eastar::RobotInfo& ri : slice.our_robots) {
    std::cout << "Our team:  SSL Vision ID: " << ri.ssl_vision_id
              << " Position: " << ri.position.x() << ", " << ri.position.y()
              << '\n';
  }

  for (const auto& v : slice.goal_positions) {
    std::cout << "Our team goals:  Position: " << v.x() << ", " << v.y()
              << '\n';
  }

  for (const eastar::RobotInfo& ri : slice.their_robots) {
    std::cout << "Their team:  SSL Vision ID: " << ri.ssl_vision_id
              << " Position: " << ri.position.x() << ", " << ri.position.y()
              << '\n';
  }

  slice.Validate();

  return {slice};
}

std::vector<
    search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>>
GetMultipleSeperateCollisions(const size_t num_robots) {
  search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>
      slice;

  const search::eastar::RobotInfo robot0_info(
      5, search::eastar::util::GridVertexToFreeSpace({-40, 0}));
  const search::eastar::RobotInfo robot1_info(
      7, search::eastar::util::GridVertexToFreeSpace({-30, 10}));
  const search::eastar::RobotInfo robot2_info(
      9, search::eastar::util::GridVertexToFreeSpace({40, 0}));
  const search::eastar::RobotInfo robot3_info(
      9, search::eastar::util::GridVertexToFreeSpace({30, 10}));

  const auto info_list = {robot0_info, robot1_info, robot2_info, robot3_info};

  CHECK(info_list.size() <= search::eastar::kRoboCupEAStarMaxRobots);
  CHECK(num_robots <= info_list.size());

  for (size_t i = 0; i < std::min(num_robots, info_list.size()); ++i) {
    slice.our_robots.push_back(*(info_list.begin() + i));
  }

  const search::eastar::FreeSpaceVertex robot0_goal =
      search::eastar::util::GridVertexToFreeSpace({-20, 0});
  const search::eastar::FreeSpaceVertex robot1_goal =
      search::eastar::util::GridVertexToFreeSpace({-30, -10});
  const search::eastar::FreeSpaceVertex robot2_goal =
      search::eastar::util::GridVertexToFreeSpace({20, 0});
  const search::eastar::FreeSpaceVertex robot3_goal =
      search::eastar::util::GridVertexToFreeSpace({30, -10});

  const auto goal_list = {robot0_goal, robot1_goal, robot2_goal, robot3_goal};

  CHECK(goal_list.size() <= search::eastar::kRoboCupEAStarMaxRobots);
  CHECK(num_robots <= goal_list.size());

  for (size_t i = 0; i < std::min(num_robots, goal_list.size()); ++i) {
    slice.goal_positions.push_back(*(goal_list.begin() + i));
  }

  for (size_t i = 0; i < slice.our_robots.size(); ++i) {
    slice.obstacles.push_back(
        obstacle::ObstacleFlag::GetAllRobotsExceptTeam(i));
  }

  slice.Validate();

  return {slice};
}

std::vector<
    search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>>
GetMultipleInteractingCollisions(const size_t num_robots) {
  search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>
      slice;

  const search::eastar::RobotInfo robot0_info(
      5, search::eastar::util::GridVertexToFreeSpace({-28, 0}));
  const search::eastar::RobotInfo robot1_info(
      7, search::eastar::util::GridVertexToFreeSpace({-20, 8}));
  const search::eastar::RobotInfo robot2_info(
      9, search::eastar::util::GridVertexToFreeSpace({0, 28}));
  const search::eastar::RobotInfo robot3_info(
      9, search::eastar::util::GridVertexToFreeSpace({30, 10}));

  const auto info_list = {robot0_info, robot1_info, robot2_info, robot3_info};

  CHECK(info_list.size() <= search::eastar::kRoboCupEAStarMaxRobots);
  CHECK(num_robots <= info_list.size());

  for (size_t i = 0; i < std::min(num_robots, info_list.size()); ++i) {
    slice.our_robots.push_back(*(info_list.begin() + i));
  }

  const search::eastar::FreeSpaceVertex robot0_goal =
      search::eastar::util::GridVertexToFreeSpace({0, 0});
  const search::eastar::FreeSpaceVertex robot1_goal =
      search::eastar::util::GridVertexToFreeSpace({-20, -10});
  const search::eastar::FreeSpaceVertex robot2_goal =
      search::eastar::util::GridVertexToFreeSpace({0, -10});
  const search::eastar::FreeSpaceVertex robot3_goal =
      search::eastar::util::GridVertexToFreeSpace({30, -10});

  const auto goal_list = {robot0_goal, robot1_goal, robot2_goal, robot3_goal};

  CHECK(goal_list.size() <= search::eastar::kRoboCupEAStarMaxRobots);
  CHECK(num_robots <= goal_list.size());

  NP_CHECK_EQ(info_list.size(), goal_list.size());

  for (size_t i = 0; i < std::min(num_robots, goal_list.size()); ++i) {
    slice.goal_positions.push_back(*(goal_list.begin() + i));
  }

  for (size_t i = 0; i < slice.our_robots.size(); ++i) {
    slice.obstacles.push_back(
        obstacle::ObstacleFlag::GetAllRobotsExceptTeam(i));
  }

  slice.Validate();

  return {slice};
}

std::vector<
    search::eastar::PositionsDataSlice<search::eastar::kRoboCupEAStarMaxRobots>>
GetPositionsData(const size_t num_robots,
                 const std::string& event_name,
                 const int rng_seed) {
  switch (GetEvent(event_name)) {
    case SINGLE_LARGE:
      return GetSingleLargeCollision(num_robots);
    case MULTI_NON_INTERACT:
      return GetMultipleSeperateCollisions(num_robots);
    case MULTI_INTERACT:
      return GetMultipleInteractingCollisions(num_robots);
    case ROBOCUP_NO_OBSTACLES:
      return GetRandomRobocupFieldNoObstacles(num_robots, rng_seed);
    case ROBOCUP_OBSTACLES:
      return GetRandomRobocupFieldObstacles(num_robots, rng_seed);
    default:
      LOG(FATAL) << "Unsupported event name: " << event_name;
  }
  return GetSingleLargeCollision(num_robots);
}

}  // namespace logreader
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_LOGREADER_LOGREADER_H_

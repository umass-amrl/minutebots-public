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
#include <algorithm>

#include "search/robocup_eastar/multiagent_data.h"
#include "search/robocup_eastar/path_collision_detector.h"

namespace search {
namespace eastar {

template <size_t kMaxRobotCount>
size_t GetMaxPathLength(
    const IndividualPlansArray<kMaxRobotCount>& individual_plans) {
  size_t max = 0;
  for (const auto& e : individual_plans) {
    max = std::max(max, e.size());
  }
  return max;
}

inline GridVertex AveragePosition(const GridVertex& p1, const GridVertex& p2) {
  return (p1 + p2) / 2;
}

template <size_t kMaxRobotCount>
std::vector<CollisionEvent<kMaxRobotCount>> GetPairwiseCollisionEvents(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& data_slice) {
  std::vector<CollisionEvent<kMaxRobotCount>> events;

  const size_t max_path_length =
      GetMaxPathLength<kMaxRobotCount>(data_slice.individual_plans);

  for (size_t path_index = 0; path_index < max_path_length; ++path_index) {
    for (size_t p1 = 0; p1 < data_slice.positions.our_robots.size(); ++p1) {
      const GridPath& p1_path = data_slice.individual_plans.at(p1);
      const size_t p1_path_index = std::min(p1_path.size() - 1, path_index);
      for (size_t p2 = p1 + 1; p2 < data_slice.positions.our_robots.size();
           ++p2) {
        const GridPath& p2_path = data_slice.individual_plans.at(p2);
        const size_t p2_path_index = std::min(p2_path.size() - 1, path_index);
        const GridVertex& p1_position = p1_path[p1_path_index];
        const GridVertex& p2_position = p2_path[p2_path_index];
        const GridVertex& p1_prev_position =
            p1_path[std::max(p1_path_index, 1ul) - 1];
        const GridVertex& p2_prev_position =
            p2_path[std::max(p2_path_index, 1ul) - 1];
        if (GridPositionsCollide(p1_position, p2_position, p1_prev_position,
                                 p2_prev_position)) {
          CollidingRobotArray<kMaxRobotCount> colliding_robots;
          colliding_robots.push_back(CollidingRobot(p1));
          colliding_robots.push_back(CollidingRobot(p2));
          events.push_back(
              {AveragePosition(p1_position, p2_position), colliding_robots});
        }
      }
    }
  }
  return events;
}

template <size_t kMaxRobotCount>
CollisionEvent<kMaxRobotCount> MergeCollisionEvents(
    const CollisionEvent<kMaxRobotCount>& e1,
    const CollisionEvent<kMaxRobotCount>& e2) {
  const GridVertex center =
      AveragePosition(e1.collision_center, e2.collision_center);
  CollisionEvent<kMaxRobotCount> result(center, e1.colliding_robots);
  for (const auto& candidate : e2.colliding_robots) {
    bool is_duplicate = false;
    for (const auto& existing : result.colliding_robots) {
      if (existing.path_index == candidate.path_index) {
        is_duplicate = true;
        break;
      }
    }
    if (!is_duplicate) {
      result.colliding_robots.push_back(candidate);
    }
  }
  return result;
}

template <size_t kMaxRobotCount>
std::vector<CollisionEvent<kMaxRobotCount>> MergePairwiseCollisionEvents(
    std::vector<CollisionEvent<kMaxRobotCount>> collision_events) {
  // Distance between event centers, where in or below that distance the two
  // event centers will be merged.
  static constexpr int kGridDistanceMergeRadius = 2;
  std::vector<CollisionEvent<kMaxRobotCount>> merged_events;
  for (size_t i = 0; i < collision_events.size(); ++i) {
    NP_CHECK(i < collision_events.size());
    CollisionEvent<kMaxRobotCount> current_event = collision_events[i];
    // Grab all elements after the ith element.
    for (size_t j = i + 1; j < collision_events.size();) {
      NP_CHECK(j < collision_events.size());
      const CollisionEvent<kMaxRobotCount>& following_event =
          collision_events[j];
      const int grid_radius = GridVertex(following_event.collision_center -
                                         current_event.collision_center)
                                  .lpNorm<1>();
      if (grid_radius <= kGridDistanceMergeRadius) {
        current_event = MergeCollisionEvents(current_event, following_event);
        NP_CHECK(j < collision_events.size());
        collision_events.erase(collision_events.begin() + j);
      } else {
        ++j;
      }
    }
    merged_events.push_back(current_event);
  }
  return merged_events;
}

template <size_t kMaxRobotCount>
std::vector<CollisionEvent<kMaxRobotCount>> GetCollisionEvents(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& data_slice) {
  static constexpr bool kDebug = false;
  const std::vector<CollisionEvent<kMaxRobotCount>> pairwise_events =
      GetPairwiseCollisionEvents<kMaxRobotCount>(data_slice);
  if (kDebug) {
    LOG(INFO) << "Pairwise events: ";
    for (const auto& e : pairwise_events) {
      LOG(INFO) << e;
    }
  }
  const auto result = MergePairwiseCollisionEvents(pairwise_events);

  //   for (const CollisionEvent<kMaxRobotCount>& ce : result) {
  //
  //   }

  return result;
}

bool GridPositionsCollide(const GridVertex& p1, const GridVertex& p2,
                          const GridVertex& p1_prev,
                          const GridVertex& p2_prev) {
  if (p1 == p2) {
    return true;
  }
  if ((p1 == p2_prev) && (p2 == p1_prev)) {
    return true;
  }
  return false;
}

template std::vector<CollisionEvent<kRoboCupEAStarMaxRobots>>
GetCollisionEvents<kRoboCupEAStarMaxRobots>(
    const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>& data_slice);

}  // namespace eastar
}  // namespace search

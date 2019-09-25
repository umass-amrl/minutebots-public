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

#include "search/robocup_eastar/individual_planner.h"

#include <algorithm>

#include "search/robocup_eastar/common_defines.h"

namespace search {
namespace eastar {
// Preps the WorldState for the current obstacle positions, so that we can pass
// in an obstacle flag to the collision grid.
template <size_t kMaxRobotCount>
void PrepWorldStateForObstacleFlagCreation(
    const PositionsDataSlice<kMaxRobotCount>& slice) {
  state::PositionVelocityState pvs;

  auto* our_team_robots = pvs.GetMutableOurTeamRobots();

  NP_CHECK_MSG(slice.our_robots.MaxSize() <= our_team_robots->MaxSize(),
               "World State too small to support all robot positions!: "
                   << slice.our_robots.MaxSize() << " vs "
                   << our_team_robots->MaxSize());

  for (const RobotInfo& i : slice.our_robots) {
    our_team_robots->push_back({i.ssl_vision_id,
                                {0, i.position},
                                {0, 0, 0},
                                {0, i.position},
                                {0, 0, 0},
                                0.0f,
                                1.0f});
  }

  auto* their_team_robots = pvs.GetMutableTheirTeamRobots();
  for (const RobotInfo& i : slice.their_robots) {
    their_team_robots->push_back({i.ssl_vision_id,
                                  {0, i.position},
                                  {0, 0, 0},
                                  {0, i.position},
                                  {0, 0, 0},
                                  0.0f,
                                  1.0f});
  }

  // Creation as a side effect initializes the static array.
  state::WorldState s(&pvs, team::Team::BLUE);
}

template <size_t kMaxRobotCount>
std::pair<IndividualPlansArray<kMaxRobotCount>,
          navigation::production::eight_grid::CollisionGrid>
PlanIndividualPaths(const PositionsDataSlice<kMaxRobotCount>& slice) {
  slice.Validate();
  PrepWorldStateForObstacleFlagCreation<kMaxRobotCount>(slice);

  static const navigation::production::eight_grid::CollisionGrid
      static_collision_grid(kEightGridSquareSize);

  navigation::production::eight_grid::CollisionGrid dynamic_collision_grid(
      kEightGridSquareSize);
  dynamic_collision_grid.RebuildDynamic({});

  navigation::production::eight_grid::ProductionEightGrid individual_planner(
      static_collision_grid, dynamic_collision_grid);

  IndividualPlansArray<kMaxRobotCount> individual_plans;

  for (OurRobotIndex i = 0; i < slice.our_robots.size(); ++i) {
    const FreeSpaceVertex& current_pose = slice.our_robots.at(i).position;
    const FreeSpaceVertex& goal_pose = slice.goal_positions.at(i);
    logger::Logger logger;
    individual_planner.Update(
        slice.obstacles.at(i), {}, current_pose, goal_pose, &logger);
    std::pair<bool, FreeSpacePath> result =
        individual_planner.Plan(&logger, false);
    if (!result.first) {
      LOG(FATAL) << "Could not find individual path!";
    }
    result.second.erase(result.second.end() - 1);
    individual_plans.push_back(util::FreeSpacePathToGridPath(result.second));
  }

  // Set all plans to be the same length.
  {
    size_t max_individual_plan_length = 0;
    for (const auto& plan : individual_plans) {
      max_individual_plan_length =
          std::max(max_individual_plan_length, plan.size());
    }

    for (auto& plan : individual_plans) {
      NP_CHECK(!plan.empty());
      const GridVertex last_position = plan[plan.size() - 1];
      while (plan.size() < max_individual_plan_length) {
        plan.push_back(last_position);
      }
    }
  }

  return {individual_plans, dynamic_collision_grid};
}

template <size_t kMaxRobotCount>
IndividuallyPlannedDataSlice<kMaxRobotCount>
GenerateIndividuallyPlannedDataSlice(
    const PositionsDataSlice<kMaxRobotCount>& slice) {
  const auto result = PlanIndividualPaths(slice);
  return {slice, result.first, result.second, slice.obstacles};
}

template IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>
GenerateIndividuallyPlannedDataSlice(
    const PositionsDataSlice<kRoboCupEAStarMaxRobots>& slice);

}  // namespace eastar
}  // namespace search

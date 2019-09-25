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

#include "search/robocup_eastar/planner_state.h"

namespace search {
namespace eastar {

template <size_t kMaxRobotCount>
StartAndGoalInfo<kMaxRobotCount> ComputeStartAndGoal(
    const IndividualPlansArray<kMaxRobotCount>& individual_plans,
    const SearchWindow<kMaxRobotCount>& search_window) {
  NP_CHECK(!individual_plans.empty());

  JointDistance<kMaxRobotCount> inside_window_start_distances;
  JointPositionAndIndex<kMaxRobotCount> goal;

  // Handle start.
  for (const CollidingRobot& cr : search_window.relevant_paths) {
    const GridPath& individual_plan = individual_plans.at(cr.path_index);
    {
      bool start_found = false;
      for (size_t i = 0; i < individual_plan.size(); ++i) {
        const GridVertex& v = individual_plan.at(i);
        if (search_window.box.Inside(v)) {
          inside_window_start_distances.push_back(i);
          start_found = true;
          break;
        }
      }
      if (!kProduction && !start_found) {
        LOG(INFO) << "Box:";
        LOG(INFO) << search_window.box.upper_left.x() << ", "
                  << search_window.box.upper_left.y();
        LOG(INFO) << search_window.box.lower_right.x() << ", "
                  << search_window.box.lower_right.y();
        LOG(INFO) << "individual_plan:";
        for (const auto& e : individual_plan) {
          LOG(INFO) << "Pos: " << e.x() << ", " << e.y();
        }
        NP_CHECK_MSG(start_found, "No start found for path " << cr.path_index);
      }
    }
    {
      bool goal_found = false;
      for (int i = individual_plan.size() - 1; i >= 0; --i) {
        const GridVertex& v = individual_plan.at(i);
        if (search_window.box.Inside(v)) {
          goal.position.push_back(v);
          goal.relative_index.push_back(i);
          goal_found = true;
          break;
        }
      }
      NP_CHECK(goal_found);
    }
  }

  const GridIndex first_in_window_index =
      inside_window_start_distances.MinElement();
  JointPosition<kMaxRobotCount> first_in_window_position;
  for (const CollidingRobot& cr : search_window.relevant_paths) {
    const GridPath& individual_plan = individual_plans.at(cr.path_index);
    first_in_window_position.push_back(
        individual_plan.at(first_in_window_index));
  }

  return {first_in_window_position, first_in_window_index, goal};
}

template <size_t kMaxRobotCount>
PlannerState<kMaxRobotCount>::PlannerState(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const SearchWindow<kMaxRobotCount>& search_window) {
  UpdateStartAndGoal(slice, search_window);
  const JointPosition<kMaxRobotCount>& start_position =
      this->current_start_goal.first_in_window_position;

  const SearchNode<kMaxRobotCount> start_search_node(
      start_position, start_position,
      JointSteps<kMaxRobotCount>(
          {static_cast<GridDistance>(
               this->current_start_goal.first_in_window_index),
           0},
          start_position.size()),
      0);
  PushSearchNode(this, start_search_node);
  this->previous_goal_node = start_search_node;
}

template <size_t kMaxRobotCount>
void PlannerState<kMaxRobotCount>::UpdateStartAndGoal(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const SearchWindow<kMaxRobotCount>& search_window) {
  this->current_start_goal =
      ComputeStartAndGoal(slice.individual_plans, search_window);
}

template <size_t kMaxRobotCount>
void PlannerState<kMaxRobotCount>::ResetPlanningState() {
  open_list.clear();
  closed_list.clear();
  out_of_window_list.clear();
  // Skip repaired paths.
  new_start_to_old_start_path.clear();
}

template <size_t kMaxRobotCount>
void PlannerState<kMaxRobotCount>::Verify() const {
  if (!kProduction) {
    return;
  }
  LOG(FATAL) << "Verify is not setup yet.";
}

template <size_t kMaxRobotCount>
PlannerState<kMaxRobotCount> PlannerState<kMaxRobotCount>::Merge(
    const PlannerState<kMaxRobotCount>& other,
    const SearchWindow<kMaxRobotCount>& merged_search_window,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) const {
  return {slice, merged_search_window};
}

template <size_t kMaxRobotCount>
void PlannerState<kMaxRobotCount>::AddNewPaths(
    const SearchWindow<kMaxRobotCount>& updated_search_window,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  *this = {slice, updated_search_window};
}

template PlannerState<kRoboCupEAStarMaxRobots>::PlannerState(
    const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>& slice,
    const SearchWindow<kRoboCupEAStarMaxRobots>& search_window);

template void PlannerState<kRoboCupEAStarMaxRobots>::UpdateStartAndGoal(
    const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>& slice,
    const SearchWindow<kRoboCupEAStarMaxRobots>& search_window);

template void PlannerState<kRoboCupEAStarMaxRobots>::ResetPlanningState();

template PlannerState<kRoboCupEAStarMaxRobots>
PlannerState<kRoboCupEAStarMaxRobots>::Merge(
    const PlannerState<kRoboCupEAStarMaxRobots>& other,
    const SearchWindow<kRoboCupEAStarMaxRobots>& merged_search_window,
    const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>& slice) const;

template void PlannerState<kRoboCupEAStarMaxRobots>::AddNewPaths(
    const SearchWindow<kRoboCupEAStarMaxRobots>& updated_search_window,
    const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>& slice);

}  // namespace eastar
}  // namespace search

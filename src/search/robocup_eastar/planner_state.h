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

#ifndef SRC_SEARCH_ROBOCUP_EASTAR_PLANNER_STATE_H_
#define SRC_SEARCH_ROBOCUP_EASTAR_PLANNER_STATE_H_

#include <algorithm>
#include <memory>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#include "constants/constants.h"
#include "datastructures/pointer_comparator.h"
#include "datastructures/transacted_multi_vector_priority_queue.h"
#include "datastructures/transacted_vector_priority_queue.h"
#include "datastructures/vector_priority_queue.h"
#include "search/robocup_eastar/lazy_neighbor_generator.h"
#include "search/robocup_eastar/multiagent_data.h"
#include "search/robocup_eastar/path_collision_detector.h"
#include "search/robocup_eastar/search_node.h"
#include "search/robocup_eastar/search_window.h"

namespace search {
namespace eastar {

template <size_t kMaxRobotCount>
void IndividualPlansToJointPlans(
    IndividualPlansArray<kMaxRobotCount> individual_plans) {
  size_t max_length = 0;
  for (const auto& plan : individual_plans) {
    max_length = std::max(max_length, plan.size());
  }

  // Make all individual plans the same length.
  for (auto& plan : individual_plans) {
    NP_CHECK(!plan.empty());
    const auto last_element = plan[plan.size() - 1];
    while (plan.size() < max_length) {
      plan.push_back(last_element);
    }
  }
}

template <size_t kMaxRobotCount>
using EagerOpenList =
    datastructures::TransactedVectorPriorityQueue<SearchNode<kMaxRobotCount>>;

template <size_t kMaxRobotCount>
using LazyOpenList = datastructures::TransactedVectorPriorityQueue<
    lazy_neighbors::NeighborGenerator<kMaxRobotCount>*,
    datastructures::PointerComparator<
        lazy_neighbors::NeighborGenerator<kMaxRobotCount>>>;

template <size_t kMaxRobotCount>
using OpenList =
    typename std::conditional<kUseLazyNeighbors, LazyOpenList<kMaxRobotCount>,
                              EagerOpenList<kMaxRobotCount>>::type;

template <size_t kMaxRobotCount, typename Dummy = SearchNode<kMaxRobotCount>>
typename std::enable_if<!kUseLazyNeighbors, Dummy>::type GetTop(
    PlannerState<kMaxRobotCount>* planner_state) {
  return planner_state->open_list.top();
}

template <size_t kMaxRobotCount, typename Dummy = SearchNode<kMaxRobotCount>>
typename std::enable_if<kUseLazyNeighbors, Dummy>::type GetTop(
    PlannerState<kMaxRobotCount>* planner_state) {
  return planner_state->open_list.top()->GetMinimum();
}

template <size_t kMaxRobotCount, typename Dummy = void>
typename std::enable_if<!kUseLazyNeighbors, Dummy>::type PopTop(
    PlannerState<kMaxRobotCount>* planner_state) {
  planner_state->open_list.pop();
}

template <size_t kMaxRobotCount, typename Dummy = void>
typename std::enable_if<kUseLazyNeighbors, Dummy>::type PopTop(
    PlannerState<kMaxRobotCount>* planner_state) {
  lazy_neighbors::NeighborGenerator<kMaxRobotCount>* top =
      planner_state->open_list.top();
  planner_state->open_list.pop();
  if (!top->GenerateNext()) {
    delete top;
    return;
  }
  planner_state->open_list.push(top);
}

template <size_t kMaxRobotCount, typename Dummy = void>
typename std::enable_if<!kUseLazyNeighbors, Dummy>::type PushSearchNode(
    PlannerState<kMaxRobotCount>* planner_state,
    const SearchNode<kMaxRobotCount>& sn) {
  planner_state->open_list.push(sn);
}

template <size_t kMaxRobotCount, typename Dummy = void>
typename std::enable_if<kUseLazyNeighbors, Dummy>::type PushSearchNode(
    PlannerState<kMaxRobotCount>* planner_state,
    const SearchNode<kMaxRobotCount>& sn) {
  planner_state->open_list.push(
      new lazy_neighbors::NeighborGenerator<kMaxRobotCount>(
          sn, planner_state->current_start_goal.goal.position));
}

template <size_t kMaxRobotCount>
using ClosedList = std::unordered_map<JointPosition<kMaxRobotCount>,
                                      SearchNode<kMaxRobotCount>,
                                      JointPositionHasher<kMaxRobotCount>>;

template <size_t kMaxRobotCount>
using OutOfWindowList = std::vector<SearchNode<kMaxRobotCount>>;

template <size_t kMaxRobotCount>
struct JointPositionAndIndex {
  // Position in the joint path.
  JointPosition<kMaxRobotCount> position;
  // Negative for positions relative to the end.
  JointIndex<kMaxRobotCount> relative_index;
  JointPositionAndIndex() : position(), relative_index() {}
  JointPositionAndIndex(const JointPosition<kMaxRobotCount>& position,
                        const JointIndex<kMaxRobotCount> relative_index)
      : position(position), relative_index(relative_index) {}
};

template <size_t kMaxRobotCount>
struct StartAndGoalInfo {
  JointPosition<kMaxRobotCount> first_in_window_position;
  GridIndex first_in_window_index;
  JointPositionAndIndex<kMaxRobotCount> goal;
  StartAndGoalInfo()
      : first_in_window_position(), first_in_window_index(0), goal() {}
  StartAndGoalInfo(
      const JointPosition<kMaxRobotCount>& first_in_window_position,
      const GridIndex& first_in_window_index,
      const JointPositionAndIndex<kMaxRobotCount>& goal)
      : first_in_window_position(first_in_window_position),
        first_in_window_index(first_in_window_index),
        goal(goal) {}
};

template <size_t kMaxRobotCount>
struct JointDistancePosition {
  JointPosition<kMaxRobotCount> position;
  JointSteps<kMaxRobotCount> step;
  JointDistancePosition() = default;
  JointDistancePosition(const JointPosition<kMaxRobotCount>& position,
                        const JointSteps<kMaxRobotCount>& step)
      : position(position), step(step) {}
};

template <size_t kMaxRobotCount>
std::ostream& operator<<(std::ostream& os,
                         const JointDistancePosition<kMaxRobotCount>& jp) {
  os << "JointDistancePosition: " << jp.position << " | " << jp.step;
  return os;
}

template <size_t kMaxRobotCount>
using JointStepGridPath = std::vector<JointDistancePosition<kMaxRobotCount>>;

template <size_t kMaxRobotCount>
struct PlannerState {
  StartAndGoalInfo<kMaxRobotCount> current_start_goal;
  OpenList<kMaxRobotCount> open_list;
  ClosedList<kMaxRobotCount> closed_list;
  OutOfWindowList<kMaxRobotCount> out_of_window_list;
  JointStepGridPath<kMaxRobotCount> repaired_paths;
  JointGridPath<kMaxRobotCount> new_start_to_old_start_path;
  SearchNode<kMaxRobotCount> previous_goal_node;

  PlannerState() = default;

  // Constructs the first planner state given a search window and the relevant
  // paths stored in the individual plans.
  PlannerState(const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
               const SearchWindow<kMaxRobotCount>& search_window);

  void UpdateStartAndGoal(
      const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
      const SearchWindow<kMaxRobotCount>& search_window);

  void Verify() const;

  void ResetPlanningState();

  PlannerState<kMaxRobotCount> Merge(
      const PlannerState<kMaxRobotCount>& other,
      const SearchWindow<kMaxRobotCount>& merged_search_window,
      const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) const;

  void AddNewPaths(const SearchWindow<kMaxRobotCount>& updated_search_window,
                   const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice);
};

}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_EASTAR_PLANNER_STATE_H_

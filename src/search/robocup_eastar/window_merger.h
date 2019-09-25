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

#ifndef SRC_SEARCH_ROBOCUP_EASTAR_WINDOW_MERGER_H_
#define SRC_SEARCH_ROBOCUP_EASTAR_WINDOW_MERGER_H_

#include <utility>
#include <vector>

#include "search/robocup_eastar/planner_state.h"
#include "search/robocup_eastar/search_window.h"

namespace search {
namespace eastar {

template <size_t kMaxRobotCount>
std::pair<SearchWindow<kMaxRobotCount>, PlannerState<kMaxRobotCount>>
MergeWindows(SearchWindow<kMaxRobotCount>* s1,
             PlannerState<kMaxRobotCount>* p1,
             SearchWindow<kMaxRobotCount>* s2,
             PlannerState<kMaxRobotCount>* p2,
             const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  NP_CHECK(s1->Intersects(*s2));
  NP_CHECK(s2->Intersects(*s1));

  const SearchWindow<kMaxRobotCount> search_window_result = s1->Merge(*s2);
  const PlannerState<kMaxRobotCount> planner_state_result =
      p1->Merge(*p2, search_window_result, slice);
  return {search_window_result, planner_state_result};
}

template <size_t kMaxRobotCount>
void ReplaceStates(const size_t index_i,
                   const size_t index_j,
                   std::vector<SearchWindow<kMaxRobotCount>>* search_windows,
                   std::vector<PlannerState<kMaxRobotCount>>* planner_states,
                   const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  NP_CHECK(index_i < index_j);
  NP_CHECK_EQ(search_windows->size(), planner_states->size());
  NP_CHECK(index_i < search_windows->size());
  NP_CHECK(index_j < search_windows->size());
  SearchWindow<kMaxRobotCount>& i_window = search_windows->at(index_i);
  PlannerState<kMaxRobotCount>& i_state = planner_states->at(index_i);
  SearchWindow<kMaxRobotCount>& j_window = search_windows->at(index_j);
  PlannerState<kMaxRobotCount>& j_state = planner_states->at(index_j);
  NP_CHECK(i_window.Intersects(j_window));

  const auto merged_result =
      MergeWindows(&i_window, &i_state, &j_window, &j_state, slice);

  search_windows->erase(search_windows->begin() + index_i);
  ClearOpenList(&planner_states->at(index_i));
  planner_states->erase(planner_states->begin() + index_i);

  search_windows->erase(search_windows->begin() + index_j - 1);
  ClearOpenList(&planner_states->at(index_j - 1));
  planner_states->erase(planner_states->begin() + index_j - 1);

  search_windows->push_back(merged_result.first);
  planner_states->push_back(merged_result.second);
}

template <size_t kMaxRobotCount>
void MergeTouchingSearchWindows(
    std::vector<SearchWindow<kMaxRobotCount>>* search_windows,
    std::vector<PlannerState<kMaxRobotCount>>* planner_states,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  NP_CHECK_EQ(search_windows->size(), planner_states->size());

  bool collisions_found = false;
  do {
    collisions_found = false;
    NP_CHECK_EQ(search_windows->size(), planner_states->size());

    for (size_t i = 0; i < search_windows->size(); ++i) {
      for (size_t j = i + 1; j < search_windows->size(); ++j) {
        const SearchWindow<kMaxRobotCount>& i_window = search_windows->at(i);
        const SearchWindow<kMaxRobotCount>& j_window = search_windows->at(j);
        if (!i_window.Intersects(j_window)) {
          continue;
        }

        ReplaceStates(i, j, search_windows, planner_states, slice);

        collisions_found = true;
        break;
      }

      if (collisions_found) {
        break;
      }
    }
  } while (collisions_found);
}

template <size_t kMaxRobotCount, typename Dummy = void>
typename std::enable_if<kUseLazyNeighbors, Dummy>::type ClearOpenList(
    PlannerState<kMaxRobotCount>* planner_state) {
  for (auto*& e : *planner_state->open_list.GetMutableVector()) {
    delete e;
    e = nullptr;
  }
  planner_state->open_list.GetMutableVector()->clear();
}

template <size_t kMaxRobotCount, typename Dummy = void>
typename std::enable_if<!kUseLazyNeighbors, Dummy>::type ClearOpenList(
    PlannerState<kMaxRobotCount>* planner_state) {}

template <size_t kMaxRobotCount>
void RemoveCompletedWindows(
    std::vector<SearchWindow<kMaxRobotCount>>* search_windows,
    std::vector<PlannerState<kMaxRobotCount>>* planner_states,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  for (int j = 0; j < static_cast<int>(planner_states->size()); ++j) {
    PlannerState<kMaxRobotCount>& planner_state = planner_states->at(j);
    const SearchWindow<kMaxRobotCount>& search_window = search_windows->at(j);
    if (planner_state.repaired_paths.empty()) {
      continue;
    }
    if (!planner_state.out_of_window_list.empty()) {
      continue;
    }

    bool is_completed = true;

    const JointPosition<kMaxRobotCount>& repaired_start =
        planner_state.repaired_paths.at(0).position;
    const JointPosition<kMaxRobotCount>& repaired_goal =
        planner_state.repaired_paths.at(planner_state.repaired_paths.size() - 1)
            .position;

    NP_CHECK_EQ(repaired_start.size(), repaired_goal.size());
    for (size_t i = 0; i < search_window.relevant_paths.size(); ++i) {
      const GridVertex& individual_repaired_start = repaired_start.at(i);
      const GridVertex& individual_repaired_goal = repaired_goal.at(i);
      const OurRobotIndex& individual_planned_index =
          search_window.relevant_paths.at(i).path_index;
      const GridVertex& individual_initial_start =
          slice.individual_plans.at(individual_planned_index).at(0);
      const GridVertex& individual_initial_goal =
          slice.individual_plans.at(individual_planned_index)
              .at(slice.individual_plans.at(individual_planned_index).size() -
                  1);
      if (individual_repaired_start != individual_initial_start) {
        is_completed = false;
        break;
      }

      if (individual_repaired_goal != individual_initial_goal) {
        is_completed = false;
        break;
      }
    }

    if (!is_completed) {
      continue;
    }

    std::cout << "Path: ";
    for (const auto& jdp : planner_state.repaired_paths) {
      const auto& joint_position = jdp.position;
      std::cout << joint_position << "; ";
    }
    std::cout << "\n";

    ClearOpenList(&planner_state);
    search_windows->erase(search_windows->begin() + j);
    planner_states->erase(planner_states->begin() + j);
    --j;
  }
}

template <size_t kMaxRobotCount>
using PathIndexArray =
    datastructures::DenseArray<OurRobotIndex, kMaxRobotCount>;

template <size_t kMaxRobotCount>
PathIndexArray<kMaxRobotCount> GetNonWindowPaths(
    const SearchWindow<kMaxRobotCount>& search_window,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  PathIndexArray<kMaxRobotCount> non_window_paths;
  for (size_t i = 0; i < slice.individual_plans.size(); ++i) {
    if (!search_window.relevant_paths.Contains(CollidingRobot(i))) {
      non_window_paths.push_back(i);
    }
  }
  return non_window_paths;
}

template <size_t kMaxRobotCount>
JointPosition<kMaxRobotCount> GetNonWindowJointPosition(
    const size_t path_step,
    const PathIndexArray<kMaxRobotCount>& non_window_paths,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  JointPosition<kMaxRobotCount> result;
  for (const OurRobotIndex& i : non_window_paths) {
    NP_CHECK(slice.individual_plans.size() > i);
    NP_CHECK(slice.individual_plans.at(i).size() > path_step);
    const GridVertex& v = slice.individual_plans.at(i)[path_step];
    result.push_back(v);
  }
  return result;
}

template <size_t kMaxRobotCount>
PathIndexArray<kMaxRobotCount> GetCollidingIndices(
    const JointPosition<kMaxRobotCount>& non_window_position,
    const PathIndexArray<kMaxRobotCount>& non_window_path_indices,
    const JointPosition<kMaxRobotCount>& window_position) {
  NP_CHECK_EQ(non_window_position.size(), non_window_path_indices.size());

  PathIndexArray<kMaxRobotCount> result;
  for (size_t i = 0; i < non_window_position.size(); ++i) {
    const GridVertex& nwv = non_window_position.at(i);
    for (const GridVertex& wv : window_position) {
      if (nwv == wv) {
        result.push_back(non_window_path_indices.at(i));
        continue;
      }
    }
  }
  return result;
}

template <size_t kMaxRobotCount>
PathIndexArray<kMaxRobotCount> GetPlannedPathsCollidingWithIndividualPaths(
    const SearchWindow<kMaxRobotCount>& search_window,
    const PlannerState<kMaxRobotCount>& planner_state,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  const PathIndexArray<kMaxRobotCount> non_window_path_indices =
      GetNonWindowPaths(search_window, slice);

  PathIndexArray<kMaxRobotCount> result;

  for (size_t replanned_path_index = 0;
       replanned_path_index < planner_state.repaired_paths.size();
       ++replanned_path_index) {
    const size_t individually_planned_index =
        replanned_path_index +
        planner_state.current_start_goal.first_in_window_index;
    const JointPosition<kMaxRobotCount>& non_window_position =
        GetNonWindowJointPosition(
            individually_planned_index, non_window_path_indices, slice);
    NP_CHECK(replanned_path_index < planner_state.repaired_paths.size());
    const JointPosition<kMaxRobotCount>& window_position =
        planner_state.repaired_paths[replanned_path_index].position;

    const PathIndexArray<kMaxRobotCount> colliding_indices =
        GetCollidingIndices(
            non_window_position, non_window_path_indices, window_position);
    if (colliding_indices.empty()) {
      continue;
    }
    result.SetUnion(colliding_indices);
  }
  return result;
}

template <size_t kMaxRobotCount>
void AddNewlyCollidingPaths(
    std::vector<SearchWindow<kMaxRobotCount>>* search_windows,
    std::vector<PlannerState<kMaxRobotCount>>* planner_states,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  NP_CHECK_EQ(search_windows->size(), planner_states->size());

  for (size_t i = 0; i < search_windows->size(); ++i) {
    SearchWindow<kMaxRobotCount>& search_window = search_windows->at(i);
    PlannerState<kMaxRobotCount>& planner_state = planner_states->at(i);

    if (planner_state.repaired_paths.empty()) {
      continue;
    }

    const PathIndexArray<kMaxRobotCount> colliding_individually_planned_paths =
        GetPlannedPathsCollidingWithIndividualPaths(
            search_window, planner_state, slice);
    if (colliding_individually_planned_paths.empty()) {
      continue;
    }

    search_window.AddNewPaths(colliding_individually_planned_paths);
    planner_state.AddNewPaths(search_window, slice);
  }
}

}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_EASTAR_WINDOW_MERGER_H_

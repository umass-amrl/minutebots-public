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

#include "search/robocup_eastar/eastar_solver.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "search/robocup_eastar/lazy_neighbor_generator.h"

namespace search {
namespace eastar {

Heuristic Heuristic2d(const GridVertex& p, const GridVertex& goal) {
  return GridVertex(p - goal).lpNorm<1>();
}

template <size_t kMaxRobotCount>
Heuristic HeuristicJoint(const JointPosition<kMaxRobotCount>& p,
                         const JointPosition<kMaxRobotCount>& goal,
                         const float inflation) {
  NP_CHECK(p.size() == goal.size());
  Heuristic h = 0;
  for (size_t i = 0; i < p.size(); ++i) {
    const GridVertex& pv = p.at(i);
    const GridVertex& goalv = goal.at(i);
    h += Heuristic2d(pv, goalv);
  }
  return static_cast<Heuristic>(static_cast<float>(h) * inflation);
}

template <size_t kMaxRobotCount>
bool ShouldTerminateXStar(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const std::vector<SearchWindow<kMaxRobotCount>>& search_windows,
    const std::vector<PlannerState<kMaxRobotCount>>& planner_states) {
  static constexpr bool kDebug = false;
  NP_CHECK(search_windows.size() == planner_states.size());
  if (search_windows.empty()) {
    if (kDebug) {
      LOG(INFO) << "No search windows, exiting";
    }
    return true;
  }

  if (search_windows.size() > 1) {
    if (kDebug) {
      LOG(INFO) << "Search windows have not converged";
    }
    return false;
  }

  static size_t terminate_call_count = 0;
  static constexpr size_t kMaxCallsToXStar = 1000;
  if (++terminate_call_count > kMaxCallsToXStar) {
    if (kDebug) {
      LOG(INFO) << "Triggered early termination of X*, made more than "
                << kMaxCallsToXStar;
    }
    return true;
  }

  for (size_t j = 0; j < planner_states.size(); ++j) {
    const PlannerState<kMaxRobotCount>& planner_state = planner_states.at(j);
    const SearchWindow<kMaxRobotCount>& search_window = search_windows.at(j);
    if (planner_state.repaired_paths.empty()) {
      if (kDebug) {
        LOG(INFO) << "Repaired path is empty";
      }
      return false;
    }
    if (!planner_state.out_of_window_list.empty()) {
      if (kDebug) {
        LOG(INFO) << "Out of window list has states.";
      }
      return false;
    }

    const JointPosition<kMaxRobotCount>& repaired_start =
        planner_state.repaired_paths.at(0).position;
    const JointPosition<kMaxRobotCount>& repaired_goal =
        planner_state.repaired_paths.at(planner_state.repaired_paths.size() - 1)
            .position;
    NP_CHECK(repaired_start.size() == repaired_goal.size());
    for (size_t i = 0; i < search_window.relevant_paths.size(); ++i) {
      const GridVertex& individual_repaired_start = repaired_start.at(i);
      const GridVertex& individual_repaired_goal = repaired_goal.at(i);
      const OurRobotIndex& individual_planned_index =
          search_window.relevant_paths.at(i).path_index;
      const GridPath& individually_planned_path =
          slice.individual_plans.at(individual_planned_index);
      const GridVertex& individual_initial_start =
          individually_planned_path.at(0);
      const GridVertex& individual_initial_goal =
          individually_planned_path.at(individually_planned_path.size() - 1);
      if (individual_repaired_start != individual_initial_start) {
        if (kDebug) {
          LOG(INFO) << "Starts do not line up with full start.";
        }
        return false;
      }

      if (individual_repaired_goal != individual_initial_goal) {
        if (kDebug) {
          LOG(INFO) << "Goals do not line up with full goal.";
        }
        return false;
      }
    }
  }

  if (kDebug) {
    LOG(INFO) << "Not meeting any continue conditions; all "
              << planner_states.size() << " states checked.";
    for (const PlannerState<kMaxRobotCount>& planner_state : planner_states) {
      for (const JointDistancePosition<kMaxRobotCount>& jdp :
           planner_state.repaired_paths) {
        LOG(INFO) << jdp;
      }
      for (size_t i = 0; i < slice.individual_plans.size(); ++i) {
        const GridPath& path = slice.individual_plans.at(i);
        const GridVertex& start = path.at(0);
        const GridVertex& goal = path.at(path.size() - 1);
        LOG(INFO) << "Start: " << start.x() << ", " << start.y()
                  << " Goal: " << goal.x() << ", " << goal.y();
      }
    }
  }

  return true;
}

template <size_t kMaxRobotCount>
void ReintroduceOutOfWindowStates(PlannerState<kMaxRobotCount>* planner_state) {
  for (const SearchNode<kMaxRobotCount>& node :
       planner_state->out_of_window_list) {
    PushSearchNode(planner_state, node);
  }
  planner_state->out_of_window_list.clear();
}

static constexpr size_t kNumNeighbors = 6;

struct Neighbor2d {
  GridVertex neighbor;
  MovingHaltedSteps steps;
  bool in_window;
  Neighbor2d() : neighbor(0, 0), steps(), in_window(true) {}
  Neighbor2d(const GridVertex& neighbor,
             const MovingHaltedSteps& steps,
             const bool in_window)
      : neighbor(neighbor), steps(steps), in_window(in_window) {}
};

using Neighbors2d = datastructures::DenseArray<Neighbor2d, kNumNeighbors>;

// Returns all neighbors for a given grid vertex that are not in collision with
// the world.
template <size_t kMaxRobotCount>
Neighbors2d GetNeighbors2d(
    const GridVertex& p,
    const GridVertex& goal,
    const OurRobotIndex& robot_index,
    const MovingHaltedSteps& dist,
    const GridPath& individual_path,
    const navigation::production::eight_grid::CollisionGrid& collision_grid,
    const SearchWindow<kMaxRobotCount>& search_window) {
  Neighbors2d neighbors_2d;

  if (!search_window.box.Inside(p)) {
    NP_CHECK(dist.transit_steps + 1 < static_cast<int>(individual_path.size()));
    neighbors_2d.push_back(
        {individual_path.at(dist.transit_steps + 1), {1, 0}, true});
    return neighbors_2d;
  }

  const auto obstacle_flag =
      obstacle::ObstacleFlag::GetAllRobotsExceptTeam(robot_index);
  std::array<Neighbor2d, kNumNeighbors - 1> possible_neighbors = {{
      {{p.x() + 1, p.y()}, {1, 0}, true},
      {{p.x() - 1, p.y()}, {1, 0}, true},
      {{p.x(), p.y() + 1}, {1, 0}, true},
      {{p.x(), p.y() - 1}, {1, 0}, true},
      {{p.x(), p.y()}, {1, 0}, true},
  }};
  for (Neighbor2d& possible_neighbor : possible_neighbors) {
    if (collision_grid.IsColliding(possible_neighbor.neighbor, obstacle_flag)) {
      // Skip all vertices that are colliding with an obstacle.
      continue;
    }
    possible_neighbor.in_window =
        search_window.box.Inside(possible_neighbor.neighbor);
    neighbors_2d.push_back(possible_neighbor);
  }

  if (p == goal) {
    neighbors_2d.push_back({p, {0, 1}, true});
  }

  return neighbors_2d;
}

template <size_t kMaxRobotCount>
struct JointNeighbor {
  JointPosition<kMaxRobotCount> neighbor;
  JointSteps<kMaxRobotCount> additional_steps;
  bool in_window;
  JointNeighbor() : neighbor(), additional_steps(), in_window(true) {}
  JointNeighbor(const JointPosition<kMaxRobotCount>& neighbor,
                const JointSteps<kMaxRobotCount>& additional_steps,
                const bool in_window)
      : neighbor(neighbor),
        additional_steps(additional_steps),
        in_window(in_window) {}
};

template <size_t kMaxRobotCount>
std::ostream& operator<<(std::ostream& os,
                         const JointNeighbor<kMaxRobotCount>& neighbor) {
  os << "JointNeighbor: Position: ";
  for (const GridVertex& p : neighbor.neighbor) {
    os << "(" << p.x() << ", " << p.y() << ") ";
  }
  os << "\t| Distance: ";
  for (const GridDistance& d : neighbor.additional_distance) {
    os << d << " ";
  }
  return os;
}

template <size_t kMaxRobotCount>
using NeighborsJoint =
    datastructures::DenseArray<JointNeighbor<kMaxRobotCount>,
                               math_util::ConstexprPow(kNumNeighbors,
                                                       kMaxRobotCount)>;

template <size_t kMaxRobotCount>
void DuplicateExistingContentsNTimes(NeighborsJoint<kMaxRobotCount>* data_array,
                                     const size_t n) {
  if (n <= 1) {
    return;
  }

  auto& underlying_std_array = *data_array->GetMutableUnderlyingArray();
  const size_t orig_data_size = data_array->size();

  JointNeighbor<kMaxRobotCount>* orig_data_begin = &(underlying_std_array[0]);
  JointNeighbor<kMaxRobotCount>* orig_data_end =
      &(underlying_std_array[orig_data_size]);
  auto* new_data_start = &(underlying_std_array[orig_data_size]);
  for (size_t i = 1; i < n; ++i) {
    std::copy(orig_data_begin, orig_data_end, new_data_start);
    new_data_start += orig_data_size;
  }
  data_array->ForceSetSize(orig_data_size * n);
}

template <size_t kMaxRobotCount>
bool IsJointPositionColliding(const JointPosition<kMaxRobotCount>& jp) {
  for (size_t i = 0; i < jp.size(); ++i) {
    const GridVertex& i_pos = jp.at(i);
    for (size_t j = i + 1; j < jp.size(); ++j) {
      const GridVertex& j_pos = jp.at(j);
      if (i_pos == j_pos) {
        return true;
      }
    }
  }
  return false;
}

template <size_t kMaxRobotCount>
NeighborsJoint<kMaxRobotCount> GetNeighborsJoint(
    const JointPosition<kMaxRobotCount>& p,
    const JointSteps<kMaxRobotCount>& current_joint_steps,
    const JointPosition<kMaxRobotCount>& goal,
    const navigation::production::eight_grid::CollisionGrid& collision_grid,
    const IndividualPlansArray<kMaxRobotCount>& individually_planned_paths,
    const SearchWindow<kMaxRobotCount>& search_window) {
  datastructures::DenseArray<Neighbors2d, kMaxRobotCount> individual_neighbors;
  JointPosition<kMaxRobotCount> default_position;

  // Constructs individual_neighbors and default_position.
  for (size_t i = 0; i < search_window.relevant_paths.size(); ++i) {
    const CollidingRobot& cr = search_window.relevant_paths.at(i);
    NP_CHECK_MSG(i < p.size(),
                 "i: " << i << " p.size(): " << p.size() << " p: " << p);
    NP_CHECK(i < goal.size());
    const GridVertex& individual_p = p.at(i);
    const MovingHaltedSteps& individual_current_steps =
        current_joint_steps.at(i);
    const GridPath& individual_path =
        individually_planned_paths.at(cr.path_index);
    const GridVertex& individual_goal = goal.at(i);
    individual_neighbors.push_back(GetNeighbors2d(individual_p,
                                                  individual_goal,
                                                  cr.path_index,
                                                  individual_current_steps,
                                                  individual_path,
                                                  collision_grid,
                                                  search_window));
    default_position.push_back(GridVertex(std::numeric_limits<int>::max(),
                                          std::numeric_limits<int>::max()));
  }

  NeighborsJoint<kMaxRobotCount> joint_neighbors;
  joint_neighbors.push_back(
      {default_position,
       JointSteps<kMaxRobotCount>({}, default_position.size()),
       true});

  for (size_t neighbor_index = 0; neighbor_index < individual_neighbors.size();
       ++neighbor_index) {
    const Neighbors2d& neighbors = individual_neighbors.at(neighbor_index);
    const size_t old_count = joint_neighbors.size();
    DuplicateExistingContentsNTimes(&joint_neighbors, neighbors.size());

    // Iterate over the blocks and assign one of the eight values to
    // each block.
    for (size_t block_index = 0; block_index < neighbors.size();
         ++block_index) {
      for (size_t intrablock_index = 0; intrablock_index < old_count;
           ++intrablock_index) {
        const Neighbor2d& neighbor2d = neighbors.at(block_index);
        JointNeighbor<kMaxRobotCount>* copied_value =
            joint_neighbors.GetMutable(block_index * old_count +
                                       intrablock_index);
        copied_value->neighbor.Set(neighbor_index, neighbor2d.neighbor);
        copied_value->additional_steps.Set(neighbor_index, neighbor2d.steps);
        copied_value->in_window &= neighbor2d.in_window;
      }
    }
  }

  return joint_neighbors;
}

// struct CurrentAndPreviousPosition {
//   bool position_found;
//   GridVertex given_distance_position;
//   GridVertex previous_distance_position;
//   CurrentAndPreviousPosition()
//       : position_found(false),
//         given_distance_position(std::numeric_limits<int>::max(),
//                                 std::numeric_limits<int>::max()),
//         previous_distance_position(std::numeric_limits<int>::max(),
//                                    std::numeric_limits<int>::max()) {}
//   CurrentAndPreviousPosition(const GridVertex given_distance_position,
//                              const GridVertex previous_distance_position)
//       : position_found(true),
//         given_distance_position(given_distance_position),
//         previous_distance_position(previous_distance_position) {}
// };
//
// std::ostream& operator<<(std::ostream& os,
//                          const CurrentAndPreviousPosition& cpp) {
//   os << "CurrentAndPreviousPosition: ";
//   os << "Position Found: " << (cpp.position_found ? "'true'" : "'false'")
//      << " ";
//   os << "Given distance positon: '(" << cpp.given_distance_position.x() << ",
//   "
//      << cpp.given_distance_position.y() << ")' ";
//   os << "Prev distance positon: '(" << cpp.previous_distance_position.x()
//      << ", " << cpp.previous_distance_position.y() << ")' ";
//   return os;
// }

template <size_t kMaxRobotCount>
bool AnyElementsGreaterThan(const JointDistance<kMaxRobotCount>& query_data,
                            const GridDistance& reference_data) {
  for (size_t i = 0; i < query_data.size(); ++i) {
    if (query_data.at(i) > reference_data) {
      return true;
    }
  }
  return false;
}

// Finds the current position and previous position along the path at the given
// time
template <size_t kMaxRobotCount>
bool IsCollidingLookback(const PlannerState<kMaxRobotCount>& planner_state,
                         SearchNode<kMaxRobotCount> top,
                         const JointPosition<kMaxRobotCount>& given_positions,
                         const JointDistance<kMaxRobotCount>& given_distances,
                         const JointDistance<kMaxRobotCount>& start_distances) {
  const JointPosition<kMaxRobotCount>& given_prev_positions =
      top.current_position;
  const JointDistance<kMaxRobotCount>& given_prev_distances =
      top.joint_distance;

  // Distances at which we can halt looking back.
  JointDistance<kMaxRobotCount> halt_distances = given_distances;
  for (size_t i = 0; i < halt_distances.size(); ++i) {
    const auto& e = halt_distances.at(i);
    halt_distances.Set(i, std::max({e - 2, start_distances.at(i), 0}));
  }
  const GridDistance min_halt_distance = halt_distances.MinElement();

  // Collects the first search node info.
  SearchNode<kMaxRobotCount>& n_plus_one_th_search_node = top;
  SearchNode<kMaxRobotCount>& n_th_search_node = top;

  datastructures::DenseArray<bool, kMaxRobotCount> index_result_found(
      false, given_distances.size());

  static constexpr size_t kMaxLookback = 100;
  size_t lookback_iteration = 0;
  for (; lookback_iteration < kMaxLookback &&
         AnyElementsGreaterThan(n_th_search_node.joint_distance,
                                min_halt_distance);
       ++lookback_iteration) {
    for (size_t i = 0; i < given_distances.size(); ++i) {
      for (size_t j = 0; j < given_distances.size(); ++j) {
        if (i == j) {
          continue;
        }

        const auto& nth_distance = n_th_search_node.joint_distance.at(j);
        const auto& nth_position = n_th_search_node.current_position.at(j);
        const auto& given_distance = given_distances.at(i);
        const auto& given_position = given_positions.at(i);

        // Handle occupy same place at same time.
        if ((nth_distance == given_distance) &&
            (nth_position == given_position)) {
          return true;
        }

        // Handle swapping positions.
        if ((n_plus_one_th_search_node.joint_distance.at(j) ==
             given_distance) &&
            (nth_distance == given_prev_distances.at(i))) {
          if ((n_plus_one_th_search_node.current_position.at(j) ==
               given_prev_positions.at(i)) &&
              (nth_position == given_position)) {
            return true;
          }
        }
      }
    }

    // Break out of loops.
    if (n_th_search_node.previous_position ==
        n_th_search_node.current_position) {
      break;
    }

    n_plus_one_th_search_node = n_th_search_node;
    auto find_result =
        planner_state.closed_list.find(n_th_search_node.previous_position);
    NP_CHECK(find_result != planner_state.closed_list.end());
    n_th_search_node = find_result->second;
  }
  NP_CHECK(lookback_iteration < kMaxLookback);
  return false;
}

template <size_t kMaxRobotCount>
bool IsNeighborColliding(
    const JointPosition<kMaxRobotCount>& neighbor,
    const JointPosition<kMaxRobotCount>& top,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice) {
  if (IsJointPositionColliding(neighbor)) {
    return true;
  }

  for (size_t i = 0; i < neighbor.size(); ++i) {
    const GridVertex& i_current_pos = neighbor.at(i);
    const GridVertex& i_prev_pos = top.at(i);
    if (slice.collision_grid.IsColliding(i_current_pos,
                                         slice.obstacles.at(i))) {
      return true;
    }
    for (size_t j = i + 1; j < neighbor.size(); ++j) {
      const GridVertex& j_current_pos = neighbor.at(j);
      const GridVertex& j_prev_pos = top.at(j);

      // X shape.
      if ((i_current_pos == j_prev_pos) && (j_current_pos == i_prev_pos)) {
        return true;
      }
    }
  }
  return false;
}

template <size_t kMaxRobotCount, typename Dummy = void>
typename std::enable_if<!kUseLazyNeighbors, Dummy>::type ExpandState(
    PlannerState<kMaxRobotCount>* planner_state,
    const SearchNode<kMaxRobotCount>& top,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const SearchWindow<kMaxRobotCount>& search_window,
    const float inflation) {
  OpenList<kMaxRobotCount>& open_list = planner_state->open_list;
  ClosedList<kMaxRobotCount>& closed_list = planner_state->closed_list;
  const JointPosition<kMaxRobotCount>& goal =
      planner_state->current_start_goal.goal.position;

  const NeighborsJoint<kMaxRobotCount> neighbors =
      GetNeighborsJoint(top.current_position,
                        top.joint_steps,
                        goal,
                        slice.collision_grid,
                        slice.individual_plans,
                        search_window);

  for (const JointNeighbor<kMaxRobotCount>& neighbor : neighbors) {
    if (IsNeighborColliding(neighbor.neighbor, top.current_position, slice)) {
      continue;
    }

    if (kOptimizeMemoryUsage &&
        closed_list.find(neighbor.neighbor) != closed_list.end()) {
      continue;
    }

    SearchNode<kMaxRobotCount> neighbor_search_node(
        top,
        neighbor.neighbor,
        neighbor.additional_steps,
        HeuristicJoint(neighbor.neighbor,
                       planner_state->current_start_goal.goal.position,
                       inflation));

    if (!neighbor.in_window) {
      planner_state->out_of_window_list.push_back(neighbor_search_node);
      continue;
    }
    open_list.push(neighbor_search_node);
  }
}

template <size_t kMaxRobotCount, typename Dummy = void>
typename std::enable_if<kUseLazyNeighbors, Dummy>::type ExpandState(
    PlannerState<kMaxRobotCount>* planner_state,
    const SearchNode<kMaxRobotCount>& top,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const SearchWindow<kMaxRobotCount>& search_window,
    const float inflation) {
  OpenList<kMaxRobotCount>& open_list = planner_state->open_list;
  const JointPosition<kMaxRobotCount>& goal =
      planner_state->current_start_goal.goal.position;
  open_list.push(new lazy_neighbors::NeighborGenerator<kMaxRobotCount>(
      top, goal, inflation, &slice, &search_window));
}

template <size_t kMaxRobotCount, typename Dummy = bool>
typename std::enable_if<!kUseLazyNeighbors, Dummy>::type RejectTop(
    PlannerState<kMaxRobotCount>* planner_state,
    const SearchNode<kMaxRobotCount>& top,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const SearchWindow<kMaxRobotCount>& search_window) {
  return false;
}

template <size_t kMaxRobotCount, typename Dummy = bool>
typename std::enable_if<kUseLazyNeighbors, Dummy>::type RejectTop(
    PlannerState<kMaxRobotCount>* planner_state,
    const SearchNode<kMaxRobotCount>& top,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const SearchWindow<kMaxRobotCount>& search_window) {
  if (IsNeighborColliding(top.current_position, top.previous_position, slice)) {
    return true;
  }

  for (size_t i = 0; i < top.current_position.size(); ++i) {
    const GridVertex& v = top.current_position.at(i);
    const RobotPathIndex v_i = search_window.relevant_paths.at(i).path_index;
    const MovingHaltedSteps& v_steps = top.joint_steps.at(i);
    const GridPath& individual_path = slice.individual_plans.at(v_i);
    if (slice.collision_grid.IsColliding(v, slice.obstacles.at(i))) {
      return true;
    }

    const bool not_in_window = !search_window.box.Inside(v);
    const bool too_many_steps_along_path =
        (individual_path.size() <= static_cast<size_t>(v_steps.transit_steps));
    if (not_in_window && (too_many_steps_along_path ||
                          (v != individual_path.at(v_steps.transit_steps)))) {
      planner_state->out_of_window_list.push_back(top);
      return true;
    }
  }
  return false;
}

template <size_t kMaxRobotCount>
size_t AStarRepair(PlannerState<kMaxRobotCount>* planner_state,
                   const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
                   const SearchWindow<kMaxRobotCount>& search_window,
                   const GridDistance f_value_cap,
                   const float inflation) {
  size_t total_expansions = 0;
  OpenList<kMaxRobotCount>& open_list = planner_state->open_list;
  ClosedList<kMaxRobotCount>& closed_list = planner_state->closed_list;
  while (!open_list.empty() &&
         GetTop(planner_state).GetFValue() < f_value_cap) {
    const SearchNode<kMaxRobotCount> top = GetTop(planner_state);
    PopTop(planner_state);

    if (RejectTop(planner_state, top, slice, search_window)) {
      continue;
    }

    const auto find_result = closed_list.find(top.current_position);
    if (find_result != closed_list.end() &&
        find_result->second.transit_distance <= top.transit_distance) {
      continue;
    }
    ++total_expansions;

    auto top_insert_result = closed_list.insert({top.current_position, top});
    if (!top_insert_result.second) {
      top_insert_result.first->second = top;
    }

    ExpandState(planner_state, top, slice, search_window, inflation);
  }
  return total_expansions;
}

template <size_t kMaxRobotCount>
void VerifySearchNode(const SearchNode<kMaxRobotCount>& node,
                      const PlannerState<kMaxRobotCount>& planner_state,
                      const SearchWindow<kMaxRobotCount>& search_window) {
  if (kProduction) {
    return;
  }

  const JointPosition<kMaxRobotCount>& goal =
      planner_state.current_start_goal.goal.position;

  NP_CHECK(node.current_position.size() == goal.size());
  NP_CHECK(node.current_position.size() == search_window.relevant_paths.size());
}

template <size_t kMaxRobotCount>
JointStepGridPath<kMaxRobotCount> AStarContinue(
    PlannerState<kMaxRobotCount>* planner_state,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const SearchWindow<kMaxRobotCount>& search_window,
    size_t* expansions,
    const float inflation) {
  OpenList<kMaxRobotCount>& open_list = planner_state->open_list;
  ClosedList<kMaxRobotCount>& closed_list = planner_state->closed_list;
  const JointPosition<kMaxRobotCount>& goal =
      planner_state->current_start_goal.goal.position;
  NP_CHECK(!open_list.empty());
  static constexpr size_t kMaxContinueIterations = 100000000;
  size_t i = 0;
  for (; !open_list.empty() && i < kMaxContinueIterations; ++i) {
    const SearchNode<kMaxRobotCount> top = GetTop(planner_state);
    VerifySearchNode(top, *planner_state, search_window);

    if (RejectTop(planner_state, top, slice, search_window)) {
      PopTop(planner_state);
      continue;
    }

    if (top.current_position == goal) {
      return UnwindPath(top, *planner_state);
    }

    auto top_insert_result = closed_list.insert({top.current_position, top});
    PopTop(planner_state);
    if (!top_insert_result.second) {
      continue;
    }
    ++(*expansions);

    ExpandState(planner_state, top, slice, search_window, inflation);
  }
  if (i < kMaxContinueIterations) {
    LOG(ERROR) << "Overran search max!";
  }
  LOG(ERROR) << "No path found!";
  return {};
}

// template <size_t kMaxRobotCount>
// void CostExtendOpenClosedList(PlannerState<kMaxRobotCount>* planner_state)
// {
//   if (planner_state->new_start_to_old_start_path.empty()) {
//     return;
//   }
//   const JointPosition<kMaxRobotCount>& new_start =
//       planner_state->new_start_to_old_start_path[0];
//   const FreeSpaceDistance distance_between_starts =
//       util::GridDistanceToFreeSpaceDistance(
//           planner_state->new_start_to_old_start_path.size() *
//           new_start.size());
//
//   // Maximum distance increase is the distance to travel from the old start
//   to
//   // the new start.
//   for (SearchNode<kMaxRobotCount>& node :
//        *(planner_state->open_list.GetMutableVector())) {
//     node.distance += distance_between_starts;
//   }
//
//   planner_state->open_list.RebuildHeap();
//
//   for (std::pair<const JointPosition<kMaxRobotCount>,
//                  SearchNode<kMaxRobotCount>>& e :
//                  planner_state->closed_list)
//                  {
//     e.second.distance += distance_between_starts;
//   }
// }

template <size_t kMaxRobotCount>
void FixParentOfOldStart(PlannerState<kMaxRobotCount>* planner_state) {
  const JointGridPath<kMaxRobotCount>& path =
      planner_state->new_start_to_old_start_path;
  if (path.empty()) {
    return;
  }

  NP_CHECK(path[0] ==
           planner_state->current_start_goal.first_in_window_position);

  // If the size is 1, then the start is the goal, and no work to be done.
  if (path.size() == 1) {
    return;
  }

  const JointPosition<kMaxRobotCount>& old_start = path[path.size() - 1];
  const JointPosition<kMaxRobotCount>& old_start_pred = path[path.size() - 2];

  auto find_result = planner_state->closed_list.find(old_start);
  NP_CHECK(find_result != planner_state->closed_list.end());
  find_result->second.previous_position = old_start_pred;
}

template <size_t kMaxRobotCount>
JointSteps<kMaxRobotCount> JointPositionToJointStepsHack(
    const JointPosition<kMaxRobotCount>& p1,
    const JointPosition<kMaxRobotCount>& p2) {
  NP_CHECK(p1.size() == p2.size());
  JointSteps<kMaxRobotCount> js;
  for (size_t i = 0; i < p1.size(); ++i) {
    // std::min(GridVertex(p2.at(i) - p1.at(i)).lpNorm<1>(), 1)
    js.push_back({1, 0});
  }
  return js;
}

template <size_t kMaxRobotCount>
void AddPathNodesBetweenStarts(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const SearchWindow<kMaxRobotCount>& search_window,
    PlannerState<kMaxRobotCount>* planner_state) {
  const JointPosition<kMaxRobotCount>& goal =
      planner_state->current_start_goal.goal.position;
  const JointGridPath<kMaxRobotCount>& path =
      planner_state->new_start_to_old_start_path;

  JointSteps<kMaxRobotCount> running_joint_steps(
      {static_cast<GridDistance>(
           planner_state->current_start_goal.first_in_window_index),
       0},
      goal.size());

  for (size_t i = 0; i < path.size(); ++i) {
    const JointPosition<kMaxRobotCount>& current_position = path[i];
    const JointPosition<kMaxRobotCount>& previous_position =
        path[((i > 0) ? i - 1 : 0)];
    NP_CHECK(current_position.size() == previous_position.size());

    const SearchNode<kMaxRobotCount> current_node(
        current_position, previous_position, running_joint_steps, 0);
    PushSearchNode(planner_state, current_node);

    running_joint_steps +=
        JointPositionToJointStepsHack(current_position, previous_position);
  }
}

template <size_t kMaxRobotCount, typename Dummy = void>
typename std::enable_if<!kUseLazyNeighbors, Dummy>::type SetOpenListGoals(
    OpenList<kMaxRobotCount>* open_list,
    const JointPosition<kMaxRobotCount>& goal,
    const float inflation) {
  NP_CHECK(goal.size() <= kMaxRobotCount);

  for (SearchNode<kMaxRobotCount>& node : *(open_list->GetMutableVector())) {
    node.heuristic = HeuristicJoint(node.current_position, goal, inflation);
    node.UpdateAtGoalSteps(goal);
  }
}

template <size_t kMaxRobotCount, typename Dummy = void>
typename std::enable_if<kUseLazyNeighbors, Dummy>::type SetOpenListGoals(
    OpenList<kMaxRobotCount>* open_list,
    const JointPosition<kMaxRobotCount>& goal,
    const float inflation) {
  NP_CHECK(goal.size() <= kMaxRobotCount);
  for (lazy_neighbors::NeighborGenerator<kMaxRobotCount>* gen :
       *(open_list->GetMutableVector())) {
    gen->UpdateGoal(goal, inflation);
  }
}

template <size_t kMaxRobotCount>
void SetNewGoal(PlannerState<kMaxRobotCount>* planner_state,
                const float inflation) {
  //   for (auto& node_vec : *(planner_state->open_list.GetMutableVectors())) {
  //     for (SearchNode<kMaxRobotCount>& node : node_vec) {
  //       node.heuristic =
  //           HeuristicJoint(node.current_position,
  //                          planner_state->current_start_goal.goal.position);
  //       node.UpdateAtGoalSteps(planner_state->current_start_goal.goal.position);
  //     }
  //   }

  SetOpenListGoals(&(planner_state->open_list),
                   planner_state->current_start_goal.goal.position,
                   inflation);

  planner_state->open_list.Rebuild();

  for (auto& e : planner_state->closed_list) {
    SearchNode<kMaxRobotCount>& e_node = e.second;
    e_node.heuristic = HeuristicJoint(
        e.first, planner_state->current_start_goal.goal.position, inflation);
    e_node.UpdateAtGoalSteps(planner_state->current_start_goal.goal.position);
  }
}

template <size_t kMaxRobotCount>
JointStepGridPath<kMaxRobotCount> UnwindPath(
    const SearchNode<kMaxRobotCount>& goal_node,
    const PlannerState<kMaxRobotCount>& planner_state) {
  const JointPosition<kMaxRobotCount>& start =
      planner_state.current_start_goal.first_in_window_position;

  JointStepGridPath<kMaxRobotCount> path = {
      {goal_node.current_position, goal_node.joint_steps}};

  JointPosition<kMaxRobotCount> current = goal_node.previous_position;
  SearchNode<kMaxRobotCount> current_node;
  //       planner_state.current_start_goal.goal.position;
  static constexpr size_t kMaxUnwindIter = 1000;
  size_t i = 0;
  for (; current != start && i < kMaxUnwindIter; ++i) {
    const auto find_result = planner_state.closed_list.find(current);
    NP_CHECK(find_result != planner_state.closed_list.end());
    current_node = find_result->second;
    NP_CHECK_MSG(current != current_node.previous_position,
                 "Self loop detected: " << current);
    path.push_back({current, current_node.joint_steps});
    current = current_node.previous_position;
    //     LOG(INFO) << current_node;
  }

  NP_CHECK_MSG(i < kMaxUnwindIter, "Unwind seems to be stuck in a loop!");
  NP_CHECK(current == start);

  const auto find_result = planner_state.closed_list.find(current);
  NP_CHECK(find_result != planner_state.closed_list.end());
  current_node = find_result->second;
  NP_CHECK(current == current_node.current_position);

  path.push_back({start, current_node.joint_steps});
  std::reverse(path.begin(), path.end());
  return path;
}

template <size_t kMaxRobotCount>
void VerifyPlannerState(const PlannerState<kMaxRobotCount>& planner_state,
                        const SearchWindow<kMaxRobotCount>& search_window) {
  NP_CHECK(!IsJointPositionColliding(
      planner_state.current_start_goal.first_in_window_position));
  NP_CHECK(!IsJointPositionColliding(
      planner_state.current_start_goal.goal.position));
  NP_CHECK_EQ(planner_state.current_start_goal.goal.position.size(),
              search_window.relevant_paths.size());
  for (const GridVertex& v : planner_state.current_start_goal.goal.position) {
    NP_CHECK(search_window.box.Inside(v));
  }
}

template <size_t kMaxRobotCount>
void RunPhase11(const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
                SearchWindow<kMaxRobotCount>* search_window,
                PlannerState<kMaxRobotCount>* planner_state,
                const float inflation) {
  static constexpr bool kDebug = true;
  ReintroduceOutOfWindowStates(planner_state);
  const size_t phase11_expansions =
      AStarRepair<kMaxRobotCount>(planner_state,
                                  slice,
                                  *search_window,
                                  planner_state->previous_goal_node.GetFValue(),
                                  inflation);
  if (kDebug) {
    std::cout << "Phase 1.1 expansions: " << phase11_expansions << '\n';
  }
}

template <size_t kMaxRobotCount>
void RunPhase12(const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
                SearchWindow<kMaxRobotCount>* search_window,
                PlannerState<kMaxRobotCount>* planner_state,
                const float inflation) {
  static constexpr bool kDebug = true;
  AddPathNodesBetweenStarts(slice, *search_window, planner_state);
  FixParentOfOldStart(planner_state);

  planner_state->new_start_to_old_start_path.clear();
  const size_t phase12_expansions =
      AStarRepair<kMaxRobotCount>(planner_state,
                                  slice,
                                  *search_window,
                                  planner_state->previous_goal_node.GetFValue(),
                                  inflation);
  if (kDebug) {
    std::cout << "Phase 1.2 expansions: " << phase12_expansions << '\n';
  }
}

template <size_t kMaxRobotCount>
void RunPhase2(const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
               SearchWindow<kMaxRobotCount>* search_window,
               PlannerState<kMaxRobotCount>* planner_state,
               const float inflation) {
  static constexpr bool kDebug = true;
  SetNewGoal(planner_state, inflation);
  const auto find_result = planner_state->closed_list.find(
      planner_state->current_start_goal.goal.position);
  if (find_result != planner_state->closed_list.end()) {
    planner_state->repaired_paths =
        UnwindPath(find_result->second, *planner_state);
    if (kDebug) {
      std::cout << "Phase 2 expansions: " << 0 << '\n';
    }
    return;
  }

  size_t phase2_expansions = 0;
  planner_state->repaired_paths = AStarContinue(
      planner_state, slice, *search_window, &phase2_expansions, inflation);
  if (kDebug) {
    std::cout << "Phase 2 expansions: " << phase2_expansions << '\n';
  }
}

template <size_t kMaxRobotCount>
void LogPlannedPath(const PlannerState<kMaxRobotCount>& planner_state) {
  LOG(INFO) << "Planned path:";
  for (const JointDistancePosition<kMaxRobotCount>& jdp :
       planner_state.repaired_paths) {
    LOG(INFO) << jdp;
  }
}

template <size_t kMaxRobotCount>
JointGridPath<kMaxRobotCount> ExtractJointGridPathBetween(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const SearchWindow<kMaxRobotCount>& search_window,
    const GridIndex& start_idx,
    const GridIndex& end_idx) {
  NP_CHECK(end_idx >= start_idx);
  const size_t longest_span = end_idx - start_idx;

  JointDistance<kMaxRobotCount> path_distances(
      0, search_window.relevant_paths.size());
  JointGridPath<kMaxRobotCount> extract_path;

  for (size_t traversal_count = 0; traversal_count < longest_span + 1;
       ++traversal_count) {
    JointPosition<kMaxRobotCount> jp;
    for (size_t i = 0; i < search_window.relevant_paths.size(); ++i) {
      const CollidingRobot& cr = search_window.relevant_paths.at(i);
      const GridPath& individual_path =
          slice.individual_plans.at(cr.path_index);
      const size_t& min_index = start_idx;
      const size_t& max_index = end_idx;
      if (min_index + traversal_count <= max_index) {
        jp.push_back(individual_path.at(min_index + traversal_count));
        (*path_distances.GetMutable(i)) += 1;
      } else {
        jp.push_back(individual_path.at(max_index));
      }
    }
    extract_path.push_back(jp);
  }

  return extract_path;
}

template <size_t kMaxRobotCount>
void PrepareForNextIteration(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    SearchWindow<kMaxRobotCount>* search_window,
    PlannerState<kMaxRobotCount>* planner_state) {
  if (kRunNWA) {
    search_window->Expand();
    planner_state->ResetPlanningState();
    planner_state->out_of_window_list.clear();
    planner_state->UpdateStartAndGoal(slice, *search_window);
    const auto new_start =
        planner_state->current_start_goal.first_in_window_position;
    const SearchNode<kMaxRobotCount> new_start_node(
        new_start,
        new_start,
        {{static_cast<int>(
              planner_state->current_start_goal.first_in_window_index),
          0},
         planner_state->current_start_goal.goal.position.size()},
        0);

    PushSearchNode(planner_state, new_start_node);
    return;
  }

  NP_CHECK(!slice.individual_plans.empty());
  const StartAndGoalInfo<kMaxRobotCount> previous_start_goal =
      planner_state->current_start_goal;
  NP_CHECK(GetTop(planner_state).current_position ==
           previous_start_goal.goal.position);
  planner_state->previous_goal_node = GetTop(planner_state);

  search_window->Expand();

  planner_state->UpdateStartAndGoal(slice, *search_window);
  const StartAndGoalInfo<kMaxRobotCount> current_start_goal =
      planner_state->current_start_goal;

  const GridIndex& start_path_begin_idx =
      current_start_goal.first_in_window_index;
  const GridIndex& start_path_end_idx =
      previous_start_goal.first_in_window_index;

  planner_state->new_start_to_old_start_path = ExtractJointGridPathBetween(
      slice, *search_window, start_path_begin_idx, start_path_end_idx);

  if (!kProduction) {
    const auto& start_path = planner_state->new_start_to_old_start_path;
    NP_CHECK(!start_path.empty());
    NP_CHECK_MSG(start_path[start_path.size() - 1] ==
                     previous_start_goal.first_in_window_position,
                 start_path[start_path.size() - 1]
                     << " vs " << previous_start_goal.first_in_window_position);

    NP_CHECK_MSG(
        planner_state->closed_list.find(previous_start_goal.goal.position) ==
            planner_state->closed_list.end(),
        "Old goal cannot be re-explored as it's in the closed list already!");
  }
}

template <size_t kMaxRobotCount>
void LogOpenListOperations(PlannerState<kMaxRobotCount>* planner_state) {
  LOG(INFO) << "Push/Pops: " << planner_state->open_list.GetPushPopsCounts();
  LOG(INFO) << "Reorders: " << planner_state->open_list.GetReorderCounts();
  planner_state->open_list.ResetCounts();
}

// Incoming data will be in the form of individual paths for each robot.
// Inside this function, we will convert to JointPosition<>s on an as needed
// basis. So, for instance, for the start and goal computation, we will take
// in all the individual paths as well as a list of relevant indices, and it
// will be this functions job to return the relevant positions stuck together
// into a JointPosition<>.
template <size_t kMaxRobotCount>
void RunExpandingAStar(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    SearchWindow<kMaxRobotCount>* search_window,
    PlannerState<kMaxRobotCount>* planner_state,
    const float inflation) {
  static constexpr bool kDebug = false;
  VerifyPlannerState(*planner_state, *search_window);

  if (kDebug) {
    LOG(INFO) << "Start: "
              << planner_state->current_start_goal.first_in_window_position;
    LOG(INFO) << "Goal: " << planner_state->current_start_goal.goal.position;
  }

  if (!kRunNWA) {
    RunPhase11(slice, search_window, planner_state, inflation);
    RunPhase12(slice, search_window, planner_state, inflation);
  }

  RunPhase2(slice, search_window, planner_state, inflation);

  const JointDistancePosition<kMaxRobotCount> start =
      planner_state->repaired_paths.at(0);
  const JointDistancePosition<kMaxRobotCount> end =
      planner_state->repaired_paths.at(planner_state->repaired_paths.size() -
                                       1);
  NP_CHECK_EQ(start.step.size(), end.step.size());

  size_t individual_solition_cost = 0;
  for (size_t i = 0; i < start.step.size(); ++i) {
    individual_solition_cost +=
        planner_state->current_start_goal.goal.relative_index.at(i) -
        planner_state->current_start_goal.first_in_window_index;
  }
  std::cout << "Individual solution: " << individual_solition_cost << '\n';

  size_t joint_solition_cost = 0;
  for (size_t i = 0; i < start.step.size(); ++i) {
    const MovingHaltedSteps& start_step = start.step.at(i);
    const MovingHaltedSteps& end_step = end.step.at(i);
    joint_solition_cost += end_step.transit_steps - start_step.transit_steps;
  }
  std::cout << "Joint solution: " << joint_solition_cost << '\n';

  //   LogPlannedPath(*planner_state);

  //   LogOpenListOperations(planner_state);

  PrepareForNextIteration<kMaxRobotCount>(slice, search_window, planner_state);
}

template bool ShouldTerminateXStar(
    const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>& slice,
    const std::vector<SearchWindow<kRoboCupEAStarMaxRobots>>& search_windows,
    const std::vector<PlannerState<kRoboCupEAStarMaxRobots>>& planner_states);

template void RunExpandingAStar(
    const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>& slice,
    SearchWindow<kRoboCupEAStarMaxRobots>* search_window,
    PlannerState<kRoboCupEAStarMaxRobots>* planner_state,
    const float inflation);

}  // namespace eastar
}  // namespace search

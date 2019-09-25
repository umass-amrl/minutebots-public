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

#include "search/fourgrid/fourgrid_solver.h"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "constants/constants.h"
#include "util/array_util.h"

namespace search {
namespace fourgrid {
namespace solver {
namespace expandingastar {

namespace util {

template <size_t kRobotCount>
std::string SearchNodeToString(const SearchNode<kRobotCount>& node) {
  std::stringstream ss;
  ss << "F Value: (" << node.GetCost() << " + " << node.heuristic << " = "
     << node.GetFValue() << ") | ";
  for (const auto& cp : node.path) {
    ss << "{" << search::fourgrid::util::JointPositionToString(cp.first)
       << " c: " << cp.second << "} => ";
  }
  return ss.str();
}

template <size_t kRobotCount>
void DumpOpenList(OpenList<kRobotCount> open_list) {
  while (!open_list.empty()) {
    const auto top = open_list.top();
    open_list.pop();
    LOG(INFO) << SearchNodeToString(top);
  }
}

}  // namespace util

using ::search::fourgrid::util::JointPositionToString;

template <size_t kRobotCount>
float Heuristic(const JointPosition<kRobotCount>& current,
                const JointPosition<kRobotCount>& goal) {
  float cost = 0;
  for (size_t i = 0; i < kRobotCount; ++i) {
    const Eigen::Vector2i d = (current[i] - goal[i]);
    cost += d.lpNorm<1>();
  }
  return cost;
}

template <size_t kRobotCount>
SearchNode<kRobotCount>::SearchNode() : path(), heuristic(0) {}

template <size_t kRobotCount>
SearchNode<kRobotCount>::SearchNode(
    const SearchNode& prev, const JointPosition<kRobotCount>& additional_step,
    const float& additional_cost, const JointPosition<kRobotCount>& goal)
    : path(prev.path), heuristic(Heuristic(additional_step, goal)) {
  path.push_back({additional_step, GetCost() + additional_cost});
}

template <size_t kRobotCount>
SearchNode<kRobotCount>::SearchNode(const CostPath<kRobotCount>& path,
                                    const float& heuristic)
    : path(path), heuristic(heuristic) {}

template <size_t kRobotCount>
bool SearchNode<kRobotCount>::operator<(const SearchNode& other) const {
  if ((GetCost() + heuristic) == (other.GetCost() + other.heuristic)) {
    if (heuristic == other.heuristic) {
      return path.size() > other.path.size();
    } else {
      return heuristic > other.heuristic;
    }
  }
  return (GetCost() + heuristic) > (other.GetCost() + other.heuristic);
}

template <size_t kRobotCount>
bool SearchNode<kRobotCount>::operator==(const SearchNode& other) const {
  return (path == other.path) && (GetCost() == other.GetCost()) &&
         (heuristic == other.heuristic);
}

template <size_t kRobotCount>
float SearchNode<kRobotCount>::GetCost() const {
  NP_CHECK(!path.empty());
  return path[path.size() - 1].second;
}

template <size_t kRobotCount>
const JointPosition<kRobotCount>& SearchNode<kRobotCount>::LastPathPosition()
    const {
  NP_CHECK(!path.empty());
  return path[path.size() - 1].first;
}

template <size_t kRobotCount>
float SearchNode<kRobotCount>::GetFValue() const {
  return (GetCost() + heuristic);
}

template <size_t kRobotCount>
CostPath<kRobotCount> ExtractClosedListPath(
    const ClosedList<kRobotCount>& closed_list,
    const FourGrid<kRobotCount>& four_grid,
    const JointPosition<kRobotCount>& start,
    const JointPosition<kRobotCount>& goal) {
  CostPath<kRobotCount> path;
  NP_CHECK(closed_list.find(goal) != closed_list.end());
  float current_cost = closed_list.find(goal)->second.GetCost();
  JointPosition<kRobotCount> current = goal;
  while (current != start) {
    const std::pair<JointPosition<kRobotCount>, int> position(current,
                                                              current_cost);

    // Loop checking.
    if (!kProduction && !path.empty()) {
      const auto find_result =
          std::find(path.begin(), path.end() - 1, position);
      if (find_result != path.end() - 1) {
        LOG(ERROR) << "Query position: "
                   << JointPositionToString(position.first) << " "
                   << position.second;
        LOG(WARNING) << "Path:";
        for (const auto& p : path) {
          if (p == position) {
            LOG(ERROR) << JointPositionToString(p.first) << " " << p.second;
          } else {
            LOG(WARNING) << JointPositionToString(p.first) << " " << p.second;
          }
        }
        LOG(FATAL) << "Loop detected!";
      }
    }

    path.push_back(position);

    float minimum_cost = std::numeric_limits<float>::max();
    JointPosition<kRobotCount> minumum_cost_next_position = current;

    LOG(INFO) << "QUERY: " << JointPositionToString(current);
    for (const auto& edge : four_grid.GetBackwardEdges(current)) {
      const auto find_result = closed_list.find(edge.from);
      if (find_result == closed_list.end()) {
        LOG(INFO) << "Reject " << JointPositionToString(edge.from);
        continue;
      }
      LOG(INFO) << "Accept " << JointPositionToString(edge.from)
                << " cost: " << find_result->second.GetCost();
      if (find_result->second.GetCost() < minimum_cost) {
        minumum_cost_next_position = find_result->first;
        minimum_cost = find_result->second.GetCost();
      }
    }
    if (minumum_cost_next_position == current) {
      LOG(FATAL) << "Cannot find next step! Position: "
                 << search::fourgrid::util::JointPositionToString(
                        minumum_cost_next_position);
    }

    current = minumum_cost_next_position;
    current_cost = minimum_cost;
  }

  path.push_back({start, 0.0f});
  std::reverse(path.begin(), path.end());
  return path;
}

template <size_t kRobotCount>
void FourGridSolver<kRobotCount>::AStarRepair(
    OpenList<kRobotCount>* open_list, ClosedList<kRobotCount>* closed_list,
    const FourGrid<kRobotCount>& four_grid,
    const JointPosition<kRobotCount>& goal,
    const SearchNode<kRobotCount>& prev_search_top,
    size_t* total_expansions) const {
  while (!open_list->empty() &&
         open_list->top().GetFValue() < prev_search_top.GetFValue()) {
    const SearchNode<kRobotCount> top = open_list->top();
    open_list->pop();
    //     LOG(INFO) << "Top: " << JointPositionToString(top.LastPathPosition())
    //               << " cost: " << top.GetCost() << " F: " << top.GetFValue();

    const auto find_result = closed_list->find(top.LastPathPosition());
    if (find_result != closed_list->end() &&
        find_result->second.GetCost() <= top.GetCost()) {
      continue;
    }
    ++(*total_expansions);

    (*closed_list)[top.LastPathPosition()] = top;

    for (const auto& edge : four_grid.GetEdges(top.LastPathPosition())) {
      open_list->push(SearchNode<kRobotCount>(top, edge.to, edge.cost, goal));
    }
  }
}

template <size_t kRobotCount>
void VerifyNodePathAgainstClosedList(const ClosedList<kRobotCount>& closed_list,
                                     const FourGrid<kRobotCount>& four_grid,
                                     const JointPosition<kRobotCount>& start,
                                     const JointPosition<kRobotCount>& goal,
                                     const SearchNode<kRobotCount>& top) {
  const auto closed_path =
      ExtractClosedListPath(closed_list, four_grid, start, goal);
  NP_CHECK(!closed_path.empty());
  NP_CHECK(!top.path.empty());
  NP_CHECK_EQ(closed_path[closed_path.size() - 1].second, top.GetCost());
  NP_CHECK_EQ(closed_path[0].second, 0);
  NP_CHECK_EQ(top.path[0].second, 0);
  //   if (closed_path != top.path) {
  //     LOG(WARNING) << "Closed list path and node path differ. They may both "
  //                     "be legitimate paths of equal cost, however.";
  //     LOG(WARNING) << "Closed Result:";
  //     for (const auto& joint_position : closed_path) {
  //       LOG(WARNING) << search::fourgrid::util::JointPositionToString(
  //                           joint_position.first)
  //                    << "cost: " << joint_position.second;
  //     }
  //     LOG(WARNING) << "Node Result:";
  //     for (const auto& joint_position : top.path) {
  //       LOG(WARNING) << search::fourgrid::util::JointPositionToString(
  //                           joint_position.first)
  //                    << "cost: " << joint_position.second;
  //     }
  //   }
}

template <size_t kRobotCount>
CostPath<kRobotCount> FourGridSolver<kRobotCount>::ZipIndividualPlans(
    const std::array<CostPath<1>, kRobotCount>& individual_plans) const {
  size_t max_path_length = 0;
  for (size_t i = 0; i < kRobotCount; ++i) {
    max_path_length = std::max(max_path_length, individual_plans[i].size());
  }

  CostPath<kRobotCount> final_plan;

  for (size_t step = 0; step < max_path_length; ++step) {
    JointPosition<kRobotCount> position = array_util::MakeArray<kRobotCount>(
        Eigen::Vector2i(kIntMaxHack, kIntMaxHack));
    int cost = 0;
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (step < individual_plans[i].size()) {
        position[i] = individual_plans[i][step].first[0];
        cost += individual_plans[i][step].second;
      } else {
        position[i] = individual_plans[i][individual_plans.size() - 1].first[0];
        cost += individual_plans[i][individual_plans.size() - 1].second;
      }
    }
    final_plan.push_back({position, cost});
  }
  return final_plan;
}

template <size_t kRobotCount>
CostPath<kRobotCount> FourGridSolver<kRobotCount>::AStarContinue(
    OpenList<kRobotCount>* open_list, ClosedList<kRobotCount>* closed_list,
    const FourGrid<kRobotCount>& four_grid,
    const JointPosition<kRobotCount>& start,
    const JointPosition<kRobotCount>& goal, size_t* total_expansions) const {
  NP_CHECK(!open_list->empty());
  while (!open_list->empty()) {
    const SearchNode<kRobotCount> top = open_list->top();
    //     LOG(INFO) << "Top: " << JointPositionToString(top.LastPathPosition())
    //               << " cost: " << top.GetCost() << " F: " << top.GetFValue();
    //     LOG(INFO) << "path:";
    //     for (const auto& p : top.path) {
    //       LOG(INFO) << JointPositionToString(p.first) << " " << p.second;
    //     }
    if (!closed_list->insert({top.LastPathPosition(), top}).second) {
      open_list->pop();
      continue;
    }
    ++(*total_expansions);

    if (top.LastPathPosition() == goal) {
      return top.path;
    }
    open_list->pop();

    for (const auto& edge : four_grid.GetEdges(top.LastPathPosition())) {
      open_list->push(SearchNode<kRobotCount>(top, edge.to, edge.cost, goal));
    }
  }
  LOG(ERROR) << "No path found!";
  return {};
}

CostPath<1> IndividualAStar(OpenList<1>* open_list, ClosedList<1>* closed_list,
                            const FourGrid<1>& four_grid,
                            const JointPosition<1>& start,
                            const JointPosition<1>& goal,
                            size_t* total_expansions) {
  NP_CHECK(!open_list->empty());
  while (!open_list->empty()) {
    const SearchNode<1> top = open_list->top();
    open_list->pop();

    if (!closed_list->insert({top.LastPathPosition(), top}).second) {
      continue;
    }
    ++(*total_expansions);

    if (top.LastPathPosition() == goal) {
      open_list->push(top);
      return top.path;
    }

    for (const auto& edge : four_grid.GetEdges(top.LastPathPosition())) {
      open_list->push(SearchNode<1>(top, edge.to, edge.cost, goal));
    }
  }
  LOG(ERROR) << "No path found!";
  return {};
}

template <size_t kRobotCount>
std::array<CostPath<1>, kRobotCount>
FourGridSolver<kRobotCount>::SolveIndividualAStar(
    const std::array<FourGrid<1>, kRobotCount>& four_grids,
    const std::array<JointPosition<1>, kRobotCount>& starts,
    const std::array<JointPosition<1>, kRobotCount>& goals) {
  std::array<CostPath<1>, kRobotCount> result;
  for (size_t i = 0; i < kRobotCount; ++i) {
    const auto& four_grid = four_grids[i];
    const auto& start = starts[i];
    const auto& goal = goals[i];

    LOG(INFO) << "Search " << i;
    OpenList<1> open_list;
    ClosedList<1> closed_list;
    open_list.push({{{start, 0.0f}}, Heuristic(start, goal)});
    size_t search_expansions = 0;
    result[i] = IndividualAStar(&open_list, &closed_list, four_grid, start,
                                goal, &search_expansions);
    LOG(INFO) << "Search total: " << search_expansions;
  }
  return result;
}

template <size_t kRobotCount>
SolveResult<kRobotCount> FourGridSolver<kRobotCount>::SolveAStar(
    const std::vector<FourGrid<kRobotCount>>& four_grids,
    const std::vector<JointPosition<kRobotCount>>& starts,
    const std::vector<JointPosition<kRobotCount>>& goals) {
  LOG(INFO) << "A*";
  NP_CHECK(!four_grids.empty());
  NP_CHECK_EQ(starts.size(), goals.size());
  NP_CHECK_EQ(four_grids.size(), goals.size());

  size_t total_expansions = 0;
  size_t total_push_pops = 0;
  CostPath<kRobotCount> result;
  SolveInstrumentation instrumentation;
  for (size_t i = 0; i < four_grids.size(); ++i) {
    const auto phase_start = GetMonotonicTime();
    const FourGrid<kRobotCount>& four_grid = four_grids[i];
    const JointPosition<kRobotCount>& start = starts[i];
    const JointPosition<kRobotCount>& goal = goals[i];

    LOG(INFO) << "Search " << i;
    OpenList<kRobotCount> open_list;
    ClosedList<kRobotCount> closed_list;
    open_list.push({{{start, 0.0f}}, Heuristic(start, goal)});
    //     LOG(INFO) << "Start: "
    //               << search::fourgrid::util::JointPositionToString(start);
    //     LOG(INFO) << "Goal: "
    //               << search::fourgrid::util::JointPositionToString(goal);
    size_t search_expansions = 0;
    result = AStarContinue(&open_list, &closed_list, four_grid, start, goal,
                           &search_expansions);

    //     LOG(INFO) << "Intermediate Result:";
    //     for (const auto& joint_position : result) {
    //       LOG(INFO) << search::fourgrid::util::JointPositionToString(
    //                        joint_position.first)
    //                 << "cost: " << joint_position.second;
    //     }
    const auto phase_end = GetMonotonicTime();
    LOG(INFO) << "Search total: " << search_expansions;
    LOG(INFO) << "Push/Pop counts: " << open_list.GetPushPopsCounts();
    LOG(INFO) << "Reorder counts: " << open_list.GetReorderCounts();
    LOG(INFO) << "Iteration time (ms): " << (phase_end - phase_start) * 1000;
    instrumentation.state_expansions.push_back(search_expansions);
    instrumentation.queue_push_pops.push_back(open_list.GetPushPopsCounts());
    instrumentation.queue_reorders.push_back(open_list.GetReorderCounts());
    total_expansions += search_expansions;
    total_push_pops += open_list.GetPushPopsCounts();
  }
  LOG(INFO) << "Total expansions: " << total_expansions;
  LOG(INFO) << "Total push/pops: " << total_push_pops;
  return {result, instrumentation};
}

template <size_t kRobotCount>
CostPath<kRobotCount> CostExtendResult(
    const CostPath<kRobotCount>& cost_path,
    const CostPath<kRobotCount>& old_result) {
  if (cost_path.empty()) {
    return old_result;
  }
  NP_CHECK_MSG(
      cost_path[cost_path.size() - 1].first == old_result[0].first,
      "Result path should have its start match with the end of the extension");
  const auto cost_between_starts =
      ((cost_path.empty()) ? 0.0f : cost_path[cost_path.size() - 1].second);
  CostPath<kRobotCount> new_result = old_result;
  for (auto& pair : new_result) {
    pair.second += cost_between_starts;
  }
  new_result.insert(new_result.begin(), cost_path.begin(), cost_path.end() - 1);
  return new_result;
}

template <size_t kRobotCount>
void FourGridSolver<kRobotCount>::CostExtendOpenClosedList(
    OpenList<kRobotCount>* open_list, ClosedList<kRobotCount>* closed_list,
    const CostPath<kRobotCount>& cost_path) const {
  const auto cost_between_starts =
      ((cost_path.empty()) ? 0.0f : cost_path[cost_path.size() - 1].second);
  for (auto& node : *(open_list->GetMutableVector())) {
    for (auto& p : node.path) {
      p.second += cost_between_starts;
    }
    if (!cost_path.empty()) {
      if (cost_path[cost_path.size() - 1] != node.path[0]) {
        LOG(INFO) << "Cost path";
        for (const auto& p : cost_path) {
          LOG(INFO) << JointPositionToString(p.first);
        }
        LOG(INFO) << "Node path";
        for (const auto& p : node.path) {
          LOG(INFO) << JointPositionToString(p.first);
        }
      }
      NP_CHECK(cost_path[cost_path.size() - 1] == node.path[0]);
      node.path.insert(node.path.begin(), cost_path.begin(),
                       cost_path.end() - 1);
    }
  }
  open_list->RebuildHeap();
  for (auto& pair : *(closed_list)) {
    auto& node = pair.second;
    for (auto& p : node.path) {
      p.second += cost_between_starts;
    }
    if (!cost_path.empty()) {
      NP_CHECK(cost_path[cost_path.size() - 1] == node.path[0]);
      node.path.insert(node.path.begin(), cost_path.begin(),
                       cost_path.end() - 1);
    }
  }
}

template <size_t kRobotCount>
void FourGridSolver<kRobotCount>::AddPathNodesBetweenStarts(
    OpenList<kRobotCount>* open_list,
    const CostPath<kRobotCount>& extension) const {
  NP_CHECK(extension[0].second == 0.0f);
  CostPath<kRobotCount> path_so_far;
  for (const auto& pair : extension) {
    const JointPosition<kRobotCount>& position = pair.first;
    const auto cost = pair.second;
    path_so_far.push_back({position, cost});
    // Zero heuristic ensures that this position will be expanded.
    open_list->push({path_so_far, 0.0f});
  }
}

template <size_t kRobotCount>
void FourGridSolver<kRobotCount>::AddPathNodesBetweenGoals(
    OpenList<kRobotCount>* open_list, const CostPath<kRobotCount>& prev_path,
    const CostPath<kRobotCount>& extension) const {
  if (prev_path.empty()) {
    return;
  }
  if (extension.empty()) {
    return;
  }
  NP_CHECK(extension[0].second == 0.0f);

  //   LOG(INFO) << "Extension: ";
  //   for (const auto& p : extension) {
  //     LOG(INFO) << JointPositionToString(p.first);
  //   }

  const auto& new_goal = extension[extension.size() - 1].first;
  const auto& last_position = prev_path[prev_path.size() - 1];
  const auto& top_last_position = last_position.first;
  const auto& prev_path_end_cost = last_position.second;
  NP_CHECK_MSG(
      top_last_position == extension[0].first,
      "Extension does not pickup where the previous path left off (ends at "
          << JointPositionToString(top_last_position) << ", whereas starts at "
          << JointPositionToString(extension[0].first) << ")");

  CostPath<kRobotCount> path_so_far = prev_path;
  std::vector<SearchNode<kRobotCount>> node_list;
  for (const auto& pair : extension) {
    const JointPosition<kRobotCount>& position = pair.first;
    path_so_far.push_back({position, pair.second + prev_path_end_cost});
    const SearchNode<kRobotCount> search_node(path_so_far,
                                              Heuristic(position, new_goal));
    //     LOG(INFO) << "Position: "
    //               << JointPositionToString(search_node.LastPathPosition())
    //               << " cost: " << search_node.GetCost()
    //               << " F: " << search_node.GetFValue();
    node_list.push_back(search_node);
  }

  // Reverses the list such that the shorter nodes are expanded first.
  std::reverse(node_list.begin(), node_list.end());
  for (const auto& search_node : node_list) {
    open_list->push(search_node);
  }
}

template <size_t kRobotCount>
void FourGridSolver<kRobotCount>::SetNewGoal(
    OpenList<kRobotCount>* open_list, ClosedList<kRobotCount>* closed_list,
    const JointPosition<kRobotCount>& new_goal) const {
  for (auto& node : *(open_list->GetMutableVector())) {
    NP_CHECK(!node.path.empty());
    node.heuristic = Heuristic(node.path[node.path.size() - 1].first, new_goal);
  }
  for (auto& pair : (*closed_list)) {
    NP_CHECK(!pair.second.path.empty());
    pair.second.heuristic = Heuristic(
        pair.second.path[pair.second.path.size() - 1].first, new_goal);
  }
  open_list->RebuildHeap();
}

template <size_t kRobotCount>
void VerifyStartConnection(const JointPosition<kRobotCount>& prev_start,
                           const JointPosition<kRobotCount>& start,
                           const CostPath<kRobotCount>& connection_path) {
  constexpr bool kDebug = false;
  if (connection_path.empty()) {
    NP_CHECK(prev_start == start);
    return;
  }
  NP_CHECK(connection_path.size() > 1);
  NP_CHECK(connection_path[0].first == start);
  NP_CHECK_EQ(connection_path[0].second, 0);
  NP_CHECK(connection_path[connection_path.size() - 1].first == prev_start);
  if (kDebug) {
    LOG(INFO) << "Connection path:";
    for (const auto& p : connection_path) {
      LOG(INFO) << ::search::fourgrid::util::JointPositionToString(p.first)
                << " cost: " << p.second;
    }
  }
}

template <size_t kRobotCount>
void VerifyGoalConnection(const JointPosition<kRobotCount>& prev_goal,
                          const JointPosition<kRobotCount>& goal,
                          const CostPath<kRobotCount>& connection_path) {
  constexpr bool kDebug = false;
  if (connection_path.empty()) {
    NP_CHECK(prev_goal == goal);
    return;
  }
  NP_CHECK(connection_path.size() > 1);
  NP_CHECK(connection_path[0].first == prev_goal);
  NP_CHECK_EQ(connection_path[0].second, 0);
  NP_CHECK(connection_path[connection_path.size() - 1].first == goal);
  if (kDebug) {
    LOG(INFO) << "Connection path:";
    for (const auto& p : connection_path) {
      LOG(INFO) << ::search::fourgrid::util::JointPositionToString(p.first)
                << " cost: " << p.second;
    }
  }
}

template <size_t kRobotCount>
void FourGridSolver<kRobotCount>::AllowOldGoalToBeExploredAgain(
    ClosedList<kRobotCount>* closed_list,
    const JointPosition<kRobotCount>& old_goal) const {
  const auto find_result = closed_list->find(old_goal);
  if (find_result != closed_list->end()) {
    closed_list->erase(find_result);
  }
}

template <size_t kRobotCount>
void VerifyTopOfOpenListIsThePreviousPath(
    const CostPath<kRobotCount>& result,
    const OpenList<kRobotCount>& prev_open_list) {
  NP_CHECK(!result.empty());
  NP_CHECK(result == prev_open_list.top().path);
}

template <size_t kRobotCount>
SolveResult<kRobotCount> FourGridSolver<kRobotCount>::SolveExpandingAStar(
    const std::vector<FourGrid<kRobotCount>>& four_grids,
    const std::vector<JointPosition<kRobotCount>>& starts,
    const std::vector<JointPosition<kRobotCount>>& goals,
    const std::vector<CostPath<kRobotCount>>& start_connection_paths,
    const std::vector<CostPath<kRobotCount>>& goal_connection_paths) {
  LOG(INFO) << "EA*";
  NP_CHECK(!four_grids.empty());
  NP_CHECK_EQ(starts.size(), goals.size());
  NP_CHECK_EQ(four_grids.size(), goals.size());
  NP_CHECK_EQ(start_connection_paths.size(), goals.size());

  CostPath<kRobotCount> result = {{starts[0], 0.0f}};
  SolveInstrumentation instrumentation;

  OpenList<kRobotCount> prev_open_list;
  ClosedList<kRobotCount> prev_closed_list;
  JointPosition<kRobotCount> prev_start = starts[0];
  JointPosition<kRobotCount> prev_goal = goals[0];
  prev_open_list.push({{{starts[0], 0.0f}}, Heuristic(starts[0], goals[0])});

  size_t total_expansions = 0;
  size_t total_push_pops = 0;
  for (size_t i = 0; i < four_grids.size(); ++i) {
    const auto phase_start = GetMonotonicTime();
    const FourGrid<kRobotCount>& four_grid = four_grids[i];
    const JointPosition<kRobotCount>& start = starts[i];
    const JointPosition<kRobotCount>& goal = goals[i];
    const CostPath<kRobotCount>& cost_path_starts = start_connection_paths[i];
    const CostPath<kRobotCount>& cost_path_goals = goal_connection_paths[i];

    OpenList<kRobotCount> open_list = prev_open_list;
    ClosedList<kRobotCount> closed_list = prev_closed_list;

    VerifyTopOfOpenListIsThePreviousPath(result, open_list);
    VerifyStartConnection(prev_start, start, cost_path_starts);
    VerifyGoalConnection(prev_goal, goal, cost_path_goals);

    //     LOG(INFO) << "================";
    LOG(INFO) << "Search " << i;
    //     LOG(INFO) << "Start: " << JointPositionToString(start);
    //     LOG(INFO) << "Goal: " << JointPositionToString(goal);
    //     LOG(INFO) << "================";

    AllowOldGoalToBeExploredAgain(&closed_list, prev_goal);

    size_t phase11_expansions = 0;
    // Phase 1.1
    AStarRepair(&open_list, &closed_list, four_grid, prev_goal,
                prev_open_list.top(), &phase11_expansions);

    // Phase 1.2
    CostExtendOpenClosedList(&open_list, &closed_list, cost_path_starts);
    CostExtendOpenClosedList(&prev_open_list, &prev_closed_list,
                             cost_path_starts);

    AddPathNodesBetweenGoals(&open_list,
                             CostExtendResult(cost_path_starts, result),
                             cost_path_goals);
    //     LOG(INFO) << "Add path between starts";
    AddPathNodesBetweenStarts(&open_list, cost_path_starts);
    //     LOG(INFO) << "Repair";
    size_t phase12_expansions = 0;
    AStarRepair(&open_list, &closed_list, four_grid, prev_goal,
                prev_open_list.top(), &phase12_expansions);
    //     LOG(INFO) << "Set new goal";
    SetNewGoal(&open_list, &closed_list, goal);

    size_t phase2_expansions = 0;
    if (closed_list.find(goal) != closed_list.end()) {
      //       LOG(INFO) << "Extract closed list";
      result = closed_list.find(goal)->second.path;
    } else {
      // Phase 2
      //       LOG(INFO) << "AStar Continue";
      result = AStarContinue(&open_list, &closed_list, four_grid, start, goal,
                             &phase2_expansions);
    }

    //     LOG(INFO) << "Intermediate Result:";
    //     for (const auto& joint_position : result) {
    //       LOG(INFO) << search::fourgrid::util::JointPositionToString(
    //                        joint_position.first)
    //                 << "cost: " << joint_position.second;
    //     }
    const auto phase_end = GetMonotonicTime();
    LOG(INFO) << "Phase 1.1: " << phase11_expansions;
    LOG(INFO) << "Phase 1.2: " << phase12_expansions;
    LOG(INFO) << "Phase 2: " << phase2_expansions;
    LOG(INFO) << "Search total: "
              << (phase11_expansions + phase12_expansions + phase2_expansions);
    LOG(INFO) << "Push/Pop counts: " << open_list.GetPushPopsCounts();
    LOG(INFO) << "Reorder counts: " << open_list.GetReorderCounts();
    instrumentation.state_expansions.push_back(
        (phase11_expansions + phase12_expansions + phase2_expansions));
    instrumentation.queue_push_pops.push_back(open_list.GetPushPopsCounts());
    instrumentation.queue_reorders.push_back(open_list.GetReorderCounts());
    LOG(INFO) << "Iteration time (ms): " << (phase_end - phase_start) * 1000;

    total_expansions +=
        (phase11_expansions + phase12_expansions + phase2_expansions);
    total_push_pops += open_list.GetPushPopsCounts();
    open_list.ResetCounts();

    NP_CHECK(result.empty() ||
             !::search::fourgrid::util::CostPathCollides(result).first);

    prev_open_list = open_list;
    prev_closed_list = closed_list;
    prev_start = start;
    prev_goal = goal;
  }
  LOG(INFO) << "Total expansions: " << total_expansions;
  LOG(INFO) << "Total push/pops: " << total_push_pops;
  return {result, instrumentation};
}

}  // namespace expandingastar
}  // namespace solver
}  // namespace fourgrid
}  // namespace search

template struct search::fourgrid::solver::expandingastar::SearchNode<1>;
template class search::fourgrid::solver::expandingastar::FourGridSolver<1>;

template struct search::fourgrid::solver::expandingastar::SearchNode<2>;
template class search::fourgrid::solver::expandingastar::FourGridSolver<2>;

template struct search::fourgrid::solver::expandingastar::SearchNode<3>;
template class search::fourgrid::solver::expandingastar::FourGridSolver<3>;

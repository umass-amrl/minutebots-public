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

#include "search/fourgrid/dstar_fourgrid_solver.h"

#include <algorithm>
#include <limits>
#include <string>

namespace search {
namespace fourgrid {
namespace solver {
namespace dstar {

std::string KeyToString(const Key& key) {
  std::stringstream ss;
  ss << "[" << key.k1 << "; " << key.k2 << "]";
  return ss.str();
}

float Heuristic2dOctal(const Eigen::Vector2i& position,
                       const Eigen::Vector2i& goal) {
  const int x_diff = std::abs(position.x() - goal.x());
  const int y_diff = std::abs(position.y() - goal.y());

  int smaller, larger;
  if (x_diff < y_diff) {
    smaller = x_diff;
    larger = y_diff;
  } else {
    smaller = y_diff;
    larger = x_diff;
  }

  return larger + smaller * (1 - kSqrtTwo);
}

float Heuristic2dManhattan(const Eigen::Vector2i& position,
                           const Eigen::Vector2i& goal) {
  const Eigen::Vector2i d = (position - goal);
  return d.lpNorm<1>();
}

template <size_t kRobotCount>
float Heuristic(const JointPosition<kRobotCount>& current,
                const JointPosition<kRobotCount>& goal) {
  float cost = 0;
  for (size_t i = 0; i < kRobotCount; ++i) {
    cost += Heuristic2dManhattan(current[i], goal[i]);
  }
  return cost;
}

Key::Key(const float k1, const float k2) : k1(k1), k2(k2) {}

bool Key::operator<(const Key& other) const {
  if (k1 == other.k1) {
    return k2 < other.k2;
  }
  return k1 < other.k1;
}

template <size_t kRobotCount>
bool SearchNode<kRobotCount>::operator==(
    const SearchNode<kRobotCount>& other) const {
  return (position == other.position);
}

template <size_t kRobotCount>
bool SearchNode<kRobotCount>::operator<(
    const SearchNode<kRobotCount>& other) const {
  return other.key < key;
}

template <size_t kRobotCount>
float DStarFourGridSolver<kRobotCount>::G(
    const JointPosition<kRobotCount>& s) const {
  const auto find_result = g_map.find(s);
  if (find_result == g_map.end()) {
    return std::numeric_limits<float>::max();
  }
  return find_result->second;
}

template <size_t kRobotCount>
void DStarFourGridSolver<kRobotCount>::SetG(const JointPosition<kRobotCount>& s,
                                            const float val) {
  g_map[s] = val;
}

template <size_t kRobotCount>
void DStarFourGridSolver<kRobotCount>::DeleteG(
    const JointPosition<kRobotCount>& s) {
  const auto find_result = g_map.find(s);
  NP_CHECK(find_result != g_map.end());
  if (find_result != g_map.end()) {
    g_map.erase(find_result);
  }
}

template <size_t kRobotCount>
float DStarFourGridSolver<kRobotCount>::RHS(
    const JointPosition<kRobotCount>& s) const {
  const auto find_result = rhs_map.find(s);
  if (find_result == rhs_map.end()) {
    return std::numeric_limits<float>::max();
  }
  return find_result->second;
}

template <size_t kRobotCount>
void DStarFourGridSolver<kRobotCount>::SetRHS(
    const JointPosition<kRobotCount>& s, const float val) {
  rhs_map[s] = val;
}

template <size_t kRobotCount>
bool DStarFourGridSolver<kRobotCount>::IsDisabled(
    const FourGridDirectedEdge<kRobotCount>& edge) const {
  return (disabled_edges.find(edge) != disabled_edges.end());
}

template <size_t kRobotCount>
bool DStarFourGridSolver<kRobotCount>::DisableEdge(
    const FourGridDirectedEdge<kRobotCount>& edge) {
  return disabled_edges.insert(edge).second;
}

template <size_t kRobotCount>
std::vector<FourGridDirectedEdge<kRobotCount>> DStarFourGridSolver<
    kRobotCount>::Successors(const JointPosition<kRobotCount>& position) const {
  const auto unfiltered_edges = fourgrid.GetEdges(position);
  std::vector<FourGridDirectedEdge<kRobotCount>> filtered_edges;
  for (const auto& edge : unfiltered_edges) {
    if (!IsDisabled(edge)) {
      filtered_edges.push_back(edge);
    }
  }
  return filtered_edges;
}

template <size_t kRobotCount>
std::vector<FourGridDirectedEdge<kRobotCount>>
DStarFourGridSolver<kRobotCount>::Predecessors(
    const JointPosition<kRobotCount>& position) const {
  const auto unfiltered_edges = fourgrid.GetBackwardEdges(position);
  std::vector<FourGridDirectedEdge<kRobotCount>> filtered_edges;
  for (const auto& edge : unfiltered_edges) {
    if (!IsDisabled(edge)) {
      filtered_edges.push_back(edge);
    }
  }
  return filtered_edges;
}

template <size_t kRobotCount>
Key DStarFourGridSolver<kRobotCount>::CalculateKey(
    const JointPosition<kRobotCount>& s) const {
  return {std::min(G(s), RHS(s)) + Heuristic(start, s) + km,
          std::min(G(s), RHS(s))};
}

template <size_t kRobotCount>
void DStarFourGridSolver<kRobotCount>::Initialize() {
  disabled_edges.clear();
  U.clear();
  km = 0;
  rhs_map.clear();
  g_map.clear();
  SetRHS(goal, 0);
  U.push({goal, {Heuristic(start, goal), 0}});
}

template <size_t kRobotCount>
void DStarFourGridSolver<kRobotCount>::UpdateVertex(
    const JointPosition<kRobotCount>& u) {
  if (G(u) != RHS(u)) {
    U.UpdateOrPush({u, CalculateKey(u)});
  } else {
    U.RemoveIfFound({u, CalculateKey(u)});
  }
}

template <size_t kRobotCount>
bool IsAlreadyExpanded(std::vector<JointPosition<kRobotCount>>* expanded_list,
                       const JointPosition<kRobotCount>& u) {
  if (std::find(expanded_list->begin(), expanded_list->end(), u) !=
      expanded_list->end()) {
    return true;
  }
  expanded_list->push_back(u);
  return false;
}

template <size_t kRobotCount>
size_t DStarFourGridSolver<kRobotCount>::ComputeShortestPath() {
  NP_CHECK(!U.empty());
  size_t expansions = 0;
  std::vector<JointPosition<kRobotCount>> expanded_already;
  while ((U.top().key < CalculateKey(start) || RHS(start) > G(start))) {
    const JointPosition<kRobotCount> u = U.top().position;
    const auto kold = U.top().key;
    const auto knew = CalculateKey(u);
    if (kold < knew) {
      NP_CHECK(U.Update({u, knew}));
    } else if (G(u) > RHS(u)) {
      ++expansions;
      NP_CHECK(!IsAlreadyExpanded(&expanded_already, u));
      SetG(u, RHS(u));
      NP_CHECK(U.RemoveIfFound({u, knew}));
      for (const auto& edge : Predecessors(u)) {
        const auto& s = edge.from;
        if (s != goal) {
          SetRHS(s, std::min(RHS(s), edge.cost + G(u)));
        }
        UpdateVertex(s);
      }
    } else {
      ++expansions;
      NP_CHECK(!IsAlreadyExpanded(&expanded_already, u));
      const auto gold = G(u);
      DeleteG(u);
      auto back_edges = Predecessors(u);
      back_edges.push_back({u, u, 0});
      for (const auto& edge : back_edges) {
        const auto& s = edge.from;
        if (RHS(s) == (edge.cost + gold) && (s != goal)) {
          float min_rhs = std::numeric_limits<float>::max();
          for (const auto& succ : Successors(edge.from)) {
            min_rhs = std::min(min_rhs, succ.cost + G(succ.to));
          }
          NP_CHECK(min_rhs >= 0.0f);
          SetRHS(s, min_rhs);
        }
        UpdateVertex(s);
      }
    }
  }
  return expansions;
}

template <size_t kRobotCount>
CostPath<kRobotCount> DStarFourGridSolver<kRobotCount>::ExtractPath() const {
  CostPath<kRobotCount> path;
  JointPosition<kRobotCount> current_position = start;
  while (current_position != goal) {
    path.push_back({current_position, G(current_position)});
    JointPosition<kRobotCount> next_position = current_position;
    float next_cost = std::numeric_limits<float>::max();
    NP_CHECK(!Successors(current_position).empty());
    for (const auto& edge : Successors(current_position)) {
      const float g = G(edge.to);
      if (g < next_cost) {
        next_cost = g;
        next_position = edge.to;
      }
    }
    current_position = next_position;
  }
  NP_CHECK_EQ(G(goal), 0.0f);
  path.push_back({goal, G(goal)});
  return path;
}

template <size_t kRobotCount>
bool CollidingPosition(const JointPosition<kRobotCount>& p) {
  for (size_t i = 0; i < kRobotCount; ++i) {
    for (size_t j = 0; j < kRobotCount; ++j) {
      if (i != j) {
        if (p[i] == p[j]) {
          return true;
        }
      }
    }
  }
  return false;
}

template <size_t kRobotCount>
bool CollidingEdge(const JointPosition<kRobotCount>& p1,
                   const JointPosition<kRobotCount>& p2) {
  static_assert(kRobotCount <= 2, "Need to write better colliding edge check!");
  if (kRobotCount == 2) {
    return (p1[0] == p2[1] && p1[1] == p2[0]);
  }
  return false;
}

template <size_t kRobotCount>
std::pair<bool, FourGridDirectedEdge<kRobotCount>> DStarFourGridSolver<
    kRobotCount>::CollidingPath(const CostPath<kRobotCount>& cost_path) const {
  NP_CHECK(!cost_path.empty());
  // If the begin and end collides, we're sunk.
  NP_CHECK(!CollidingPosition(cost_path[0].first));
  NP_CHECK(!CollidingPosition(cost_path[cost_path.size() - 1].first));
  for (size_t i = 0; i < cost_path.size() - 1; ++i) {
    const auto& current_position = cost_path[i].first;
    const auto& next_position = cost_path[i + 1].first;
    const auto cost = cost_path[i].second - cost_path[i + 1].second;
    if (CollidingPosition(next_position) ||
        CollidingEdge(current_position, next_position)) {
      return {true, {current_position, next_position, cost}};
    }
  }
  return {false, {cost_path[0].first, cost_path[0].first, 0}};
}

template <size_t kRobotCount>
CostPath<kRobotCount> DStarFourGridSolver<kRobotCount>::SolveDStarOneShot(
    const JointPosition<kRobotCount>& given_start,
    const JointPosition<kRobotCount>& given_goal,
    const FourGrid<kRobotCount>& no_collide_graph,
    const FourGrid<kRobotCount>& collide_graph) {
  fourgrid = no_collide_graph;
  start = given_start;
  goal = given_goal;
  JointPosition<kRobotCount> last = start;
  Initialize();
  const size_t initial_expansions = ComputeShortestPath();
  LOG(INFO) << "Initial expansions: " << initial_expansions;

  fourgrid = collide_graph;
  NP_CHECK_EQ(Heuristic(last, start), 0.0f);
  last = start;

  for (const auto& unique_edge :
       no_collide_graph.GetUniqueEdges(collide_graph)) {
    const auto& cold = unique_edge.cost;
    const auto& u = unique_edge.from;
    const auto& v = unique_edge.to;
    // if (cold > c(u, v)); Never happens, skipped.
    if (RHS(u) == (cold + G(v)) && u != goal) {
      float min_rhs = std::numeric_limits<float>::max();
      for (const auto& edge : Successors(u)) {
        const auto& sp = edge.to;
        min_rhs = std::min(min_rhs, edge.cost + G(sp));
      }
      NP_CHECK(min_rhs >= 0.0f);
      SetRHS(u, min_rhs);
    }
    UpdateVertex(u);
  }
  const size_t repair_expansions = ComputeShortestPath();
  LOG(INFO) << "Repair expansions: " << repair_expansions;
  LOG(INFO) << "Total expansions: " << (repair_expansions + initial_expansions);
  return ExtractPath();
}

template <size_t kRobotCount>
CostPath<kRobotCount> DStarFourGridSolver<kRobotCount>::SolveDStarIterative(
    const JointPosition<kRobotCount>& given_start,
    const JointPosition<kRobotCount>& given_goal,
    const FourGrid<kRobotCount>& no_collide_graph) {
  fourgrid = no_collide_graph;
  start = given_start;
  goal = given_goal;
  JointPosition<kRobotCount> last = start;
  Initialize();
  const size_t initial_expansions = ComputeShortestPath();
  LOG(INFO) << "Initial expansions: " << initial_expansions;
  km += Heuristic(last, start);
  last = start;

  auto result_path = ExtractPath();
  auto collide_result = CollidingPath(result_path);
  size_t repair_expansions = 0;
  while (collide_result.first) {
    const auto& edge = collide_result.second;
    const auto& cold = edge.cost;
    const auto& u = edge.from;
    const auto& v = edge.to;
    NP_CHECK(DisableEdge(edge));

    // if (cold > c(u, v)); Never happens, skipped.
    if (RHS(u) == (cold + G(v)) && u != goal) {
      float min_rhs = std::numeric_limits<float>::max();
      for (const auto& succ : Successors(u)) {
        const auto& sp = succ.to;
        min_rhs = std::min(min_rhs, succ.cost + G(sp));
      }
      SetRHS(u, min_rhs);
    }
    UpdateVertex(u);

    repair_expansions += ComputeShortestPath();
    result_path = ExtractPath();
    collide_result = CollidingPath(result_path);
  }

  LOG(INFO) << "Repair expansions: " << repair_expansions;
  return ExtractPath();
}

}  // namespace dstar
}  // namespace solver
}  // namespace fourgrid
}  // namespace search

template struct search::fourgrid::solver::dstar::SearchNode<1>;
template class search::fourgrid::solver::dstar::DStarFourGridSolver<1>;

template struct search::fourgrid::solver::dstar::SearchNode<2>;
template class search::fourgrid::solver::dstar::DStarFourGridSolver<2>;

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

#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "datastructures/vector_priority_queue.h"
#include "graph/general/general_graph.h"

#ifndef SRC_SEARCH_EXPANDING_ASTAR_EXPANDING_ASTAR_SOLVER_H_
#define SRC_SEARCH_EXPANDING_ASTAR_EXPANDING_ASTAR_SOLVER_H_

template <size_t kRobotCount>
using JointPosition = std::array<Eigen::Vector2f, kRobotCount>;
template <size_t kRobotCount>
using JointDistance = std::array<float, kRobotCount>;
template <size_t kRobotCount>
using GeneralGraph = ::graph::general::GeneralGraph<JointPosition<kRobotCount>,
                                                    JointDistance<kRobotCount>>;
template <size_t kRobotCount>
using PositionAndWeightedSelfLoops =
    std::pair<JointPosition<kRobotCount>, size_t>;

namespace search {
namespace eastar {

template <size_t kRobotCount>
struct PositionAndWeightedSelfLoopsHasher {
  std::size_t operator()(
      const PositionAndWeightedSelfLoops<kRobotCount>& jp) const {
    size_t i = 1;
    for (const auto& p : jp.first) {
      i *= p.dot(Eigen::Vector2f(7, 13));
    }
    return i * jp.second;
  }
};

template <size_t kRobotCount>
struct Iteration {
  std::vector<bool> vertex_enabled;
  std::vector<JointPosition<kRobotCount>> path_cur_prev_starts;
  std::vector<JointDistance<kRobotCount>> dist_cur_prev_starts;
  JointDistance<kRobotCount> start_time;
  JointPosition<kRobotCount> start;
  JointPosition<kRobotCount> goal;

  Iteration() = delete;
  Iteration(const std::vector<bool>& vertex_states,
            const std::vector<JointPosition<kRobotCount>>&
                path_between_prev_and_current_starts,
            const std::vector<JointDistance<kRobotCount>>&
                distance_between_prev_and_current_starts,
            const JointDistance<kRobotCount>& start_time,
            const JointPosition<kRobotCount>& start,
            const JointPosition<kRobotCount>& goal);

  JointDistance<kRobotCount> DistanceStartEndDelta() const;
};

template <size_t kRobotCount>
struct SearchNode {
  std::vector<JointPosition<kRobotCount>> path;
  std::vector<JointDistance<kRobotCount>> distances;
  float heuristic;
  size_t weighted_self_loops;

  SearchNode();
  SearchNode(const std::vector<JointPosition<kRobotCount>>& path,
             const std::vector<JointDistance<kRobotCount>>& distances,
             const float heuristic);
  SearchNode(const JointPosition<kRobotCount>& start_position,
             const JointDistance<kRobotCount>& start_distance,
             const JointPosition<kRobotCount>& goal);
  SearchNode(const SearchNode& prev,
             const JointPosition<kRobotCount>& additional_step,
             const JointDistance<kRobotCount>& additional_distance,
             const JointPosition<kRobotCount>& goal);

  const JointPosition<kRobotCount>& LastPathPosition() const;
  const JointDistance<kRobotCount>& LastDistance() const;
  const float LastDistanceSum() const;

  bool operator<(const SearchNode& other) const;
  bool operator==(const SearchNode& other) const;
};

template <size_t kRobotCount>
using OpenList = datastructures::VectorPriorityQueue<SearchNode<kRobotCount>>;
template <size_t kRobotCount>
using ClosedList =
    std::unordered_map<PositionAndWeightedSelfLoops<kRobotCount>,
                       SearchNode<kRobotCount>,
                       PositionAndWeightedSelfLoopsHasher<kRobotCount>>;
template <size_t kRobotCount>
using OutOfWindowNodes = std::vector<SearchNode<kRobotCount>>;
template <size_t kRobotCount>
using PathReturnType = std::pair<std::vector<JointPosition<kRobotCount>>,
                                 std::vector<JointDistance<kRobotCount>>>;

template <size_t kRobotCount>
class ExpandingAStarSolver {
 private:
  GeneralGraph<kRobotCount> gg_;

  std::vector<SearchNode<kRobotCount>> GetNeighbors(
      const std::vector<bool>& vertex_enabled,
      const SearchNode<kRobotCount>& node,
      const JointPosition<kRobotCount>& goal,
      OutOfWindowNodes<kRobotCount>* own);

  std::vector<std::pair<JointPosition<kRobotCount>, size_t>>
  GetNeighborPositionsAndLevel(
      const std::vector<bool>& vertex_enabled,
      const JointPosition<kRobotCount>& current_position,
      const size_t& current_level);

  bool PathsCollide(const SearchNode<kRobotCount>& node,
                    const JointDistance<kRobotCount>& start_time);

  void AddOWNStates(OpenList<kRobotCount>* open_list,
                    OutOfWindowNodes<kRobotCount>* own,
                    const JointDistance<kRobotCount>& start_time,
                    size_t* collision_checks);

  void DistanceExtendOpenClosedLists(const Iteration<kRobotCount>& iteration,
                                     OpenList<kRobotCount>* open_list,
                                     ClosedList<kRobotCount>* closed_list);

  void AddPathBetweenStarts(const Iteration<kRobotCount>& iteration,
                            OpenList<kRobotCount>* open_list);

  void ChangeOpenListGoal(const Iteration<kRobotCount>& iteration,
                          OpenList<kRobotCount>* open_list);

  void AStarRepair(const std::vector<bool>& vertex_enabled,
                   const float minimum_f_value,
                   const JointDistance<kRobotCount>& start_time,
                   const JointPosition<kRobotCount>& goal,
                   OpenList<kRobotCount>* open_list,
                   ClosedList<kRobotCount>* closed_list,
                   OutOfWindowNodes<kRobotCount>* own, size_t* expansions,
                   size_t* collision_checks);

  void VerifyOpenList(const OpenList<kRobotCount>& ol,
                      const JointDistance<kRobotCount>& start);

  std::pair<bool, PathReturnType<kRobotCount>> ExtractPossiblePathInClosedList(
      const ClosedList<kRobotCount>& closed_list,
      const Iteration<kRobotCount>& iteration,
      const float maximum_existing_f_value);

  PathReturnType<kRobotCount> AStarContinued(
      const Iteration<kRobotCount>& iteration, OpenList<kRobotCount>* open_list,
      ClosedList<kRobotCount>* closed_list, OutOfWindowNodes<kRobotCount>* own,
      size_t* expansions, size_t* collision_checks);

 public:
  ExpandingAStarSolver() = delete;
  explicit ExpandingAStarSolver(const GeneralGraph<kRobotCount>& gg);

  PathReturnType<kRobotCount> SolveExpandingAStar(
      const std::vector<Iteration<kRobotCount>>& iterations);
  PathReturnType<kRobotCount> SolveAStar(
      const std::vector<Iteration<kRobotCount>>& iterations);
};

}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_EXPANDING_ASTAR_EXPANDING_ASTAR_SOLVER_H_

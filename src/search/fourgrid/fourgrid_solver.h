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

#ifndef SRC_SEARCH_FOURGRID_FOURGRID_SOLVER_H_
#define SRC_SEARCH_FOURGRID_FOURGRID_SOLVER_H_

#include <unordered_map>
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "datastructures/transacted_vector_priority_queue.h"
#include "datastructures/vector_priority_queue.h"
#include "search/fourgrid/fourgrid.h"

namespace search {
namespace fourgrid {
namespace solver {
namespace expandingastar {

template <size_t kRobotCount>
struct SearchNode {
  CostPath<kRobotCount> path;
  float heuristic;

  SearchNode();
  SearchNode(const CostPath<kRobotCount>& path, const float& heuristic);
  SearchNode(const SearchNode& prev,
             const JointPosition<kRobotCount>& additional_step,
             const float& additional_cost,
             const JointPosition<kRobotCount>& goal);

  const JointPosition<kRobotCount>& LastPathPosition() const;

  float GetCost() const;
  float GetFValue() const;

  bool operator<(const SearchNode& other) const;
  bool operator==(const SearchNode& other) const;
};

struct SolveInstrumentation {
  std::vector<size_t> state_expansions;
  std::vector<size_t> queue_push_pops;
  std::vector<size_t> queue_reorders;
};

template <size_t kRobotCount>
using OpenList =
    datastructures::TransactedVectorPriorityQueue<SearchNode<kRobotCount>>;
template <size_t kRobotCount>
using ClosedList =
    std::unordered_map<JointPosition<kRobotCount>, SearchNode<kRobotCount>,
                       JointPositionHasher<kRobotCount>>;
template <size_t kRobotCount>
using SolveResult = std::pair<CostPath<kRobotCount>, SolveInstrumentation>;

template <size_t kRobotCount>
class FourGridSolver {
 private:
  CostPath<kRobotCount> AStarContinue(OpenList<kRobotCount>* open_list,
                                      ClosedList<kRobotCount>* closed_list,
                                      const FourGrid<kRobotCount>& four_grid,
                                      const JointPosition<kRobotCount>& start,
                                      const JointPosition<kRobotCount>& goal,
                                      size_t* total_expansions) const;

  void AStarRepair(OpenList<kRobotCount>* open_list,
                   ClosedList<kRobotCount>* closed_list,
                   const FourGrid<kRobotCount>& four_grid,
                   const JointPosition<kRobotCount>& goal,
                   const SearchNode<kRobotCount>& prev_search_top,
                   size_t* total_expansions) const;

  FRIEND_TEST(FourGridSolver, CostExtendOpenClosedListEmptyPath);
  FRIEND_TEST(FourGridSolver, CostExtendOpenClosedListRealPath);
  void CostExtendOpenClosedList(OpenList<kRobotCount>* open_list,
                                ClosedList<kRobotCount>* closed_list,
                                const CostPath<kRobotCount>& cost_path) const;

  FRIEND_TEST(FourGridSolver, AddPathNodesBetweenStarts);
  void AddPathNodesBetweenStarts(OpenList<kRobotCount>* open_list,
                                 const CostPath<kRobotCount>& extension) const;

  FRIEND_TEST(FourGridSolver, AddPathNodesBetweenGoals);
  void AddPathNodesBetweenGoals(OpenList<kRobotCount>* open_list,
                                const CostPath<kRobotCount>& prev_path,
                                const CostPath<kRobotCount>& extension) const;

  FRIEND_TEST(FourGridSolver, SetNewGoal);
  void SetNewGoal(OpenList<kRobotCount>* open_list,
                  ClosedList<kRobotCount>* closed_list,
                  const JointPosition<kRobotCount>& new_goal) const;

  void AllowOldGoalToBeExploredAgain(
      ClosedList<kRobotCount>* closed_list,
      const JointPosition<kRobotCount>& old_goal) const;

 public:
  FourGridSolver() = default;

  std::array<CostPath<1>, kRobotCount> SolveIndividualAStar(
      const std::array<FourGrid<1>, kRobotCount>& four_grids,
      const std::array<JointPosition<1>, kRobotCount>& starts,
      const std::array<JointPosition<1>, kRobotCount>& goals);

  FRIEND_TEST(FourGridSolver, ZipPlans);
  CostPath<kRobotCount> ZipIndividualPlans(
      const std::array<CostPath<1>, kRobotCount>& individual_plans) const;

  SolveResult<kRobotCount> SolveAStar(
      const std::vector<FourGrid<kRobotCount>>& four_grids,
      const std::vector<JointPosition<kRobotCount>>& starts,
      const std::vector<JointPosition<kRobotCount>>& goals);

  SolveResult<kRobotCount> SolveExpandingAStar(
      const std::vector<FourGrid<kRobotCount>>& four_grids,
      const std::vector<JointPosition<kRobotCount>>& starts,
      const std::vector<JointPosition<kRobotCount>>& goals,
      const std::vector<CostPath<kRobotCount>>& start_connection_paths,
      const std::vector<CostPath<kRobotCount>>& goal_connection_paths);
};

}  // namespace expandingastar
}  // namespace solver
}  // namespace fourgrid
}  // namespace search

#endif  // SRC_SEARCH_FOURGRID_FOURGRID_SOLVER_H_

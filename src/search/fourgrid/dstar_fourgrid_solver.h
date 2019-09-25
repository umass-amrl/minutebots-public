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

#ifndef SRC_SEARCH_FOURGRID_DSTAR_FOURGRID_SOLVER_H_
#define SRC_SEARCH_FOURGRID_DSTAR_FOURGRID_SOLVER_H_

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "datastructures/vector_priority_queue.h"
#include "search/fourgrid/fourgrid.h"

namespace search {
namespace fourgrid {
namespace solver {
namespace dstar {

template <size_t kRobotCount>
using RHSMap = std::unordered_map<JointPosition<kRobotCount>, float,
                                  JointPositionHasher<kRobotCount>>;
template <size_t kRobotCount>
using GMap = std::unordered_map<JointPosition<kRobotCount>, float,
                                JointPositionHasher<kRobotCount>>;

template <size_t kRobotCount>
using DisabledEdgeSet =
    std::unordered_set<FourGridDirectedEdge<kRobotCount>,
                       FourGridDirectedEdgeHasher<kRobotCount>>;
struct Key {
  float k1, k2;
  Key() = delete;
  Key(const float k1, const float k2);

  bool operator<(const Key& other) const;
};

template <size_t kRobotCount>
struct SearchNode {
  JointPosition<kRobotCount> position;
  Key key;

  bool operator==(const SearchNode<kRobotCount>& other) const;
  bool operator<(const SearchNode<kRobotCount>& other) const;
};

template <size_t kRobotCount>
using OpenList = datastructures::VectorPriorityQueue<SearchNode<kRobotCount>>;

template <size_t kRobotCount>
class DStarFourGridSolver {
 private:
  OpenList<kRobotCount> U;
  float km;
  RHSMap<kRobotCount> rhs_map;
  GMap<kRobotCount> g_map;
  DisabledEdgeSet<kRobotCount> disabled_edges;
  JointPosition<kRobotCount> start;
  JointPosition<kRobotCount> goal;
  FourGrid<kRobotCount> fourgrid;

  float G(const JointPosition<kRobotCount>& s) const;
  void SetG(const JointPosition<kRobotCount>& s, const float val);
  void DeleteG(const JointPosition<kRobotCount>& s);
  float RHS(const JointPosition<kRobotCount>& s) const;
  void SetRHS(const JointPosition<kRobotCount>& s, const float val);

  bool IsDisabled(const FourGridDirectedEdge<kRobotCount>& edge) const;
  bool DisableEdge(const FourGridDirectedEdge<kRobotCount>& edge);

  std::vector<FourGridDirectedEdge<kRobotCount>> Successors(
      const JointPosition<kRobotCount>& position) const;
  std::vector<FourGridDirectedEdge<kRobotCount>> Predecessors(
      const JointPosition<kRobotCount>& position) const;

  Key CalculateKey(const JointPosition<kRobotCount>& s) const;

  void Initialize();

  void UpdateVertex(const JointPosition<kRobotCount>& s);

  size_t ComputeShortestPath();

  CostPath<kRobotCount> ExtractPath() const;

  FRIEND_TEST(DStarFourGridSolver, CollidingPath);
  std::pair<bool, FourGridDirectedEdge<kRobotCount>> CollidingPath(
      const CostPath<kRobotCount>& cost_path) const;

 public:
  DStarFourGridSolver() = default;

  CostPath<kRobotCount> SolveDStarOneShot(
      const JointPosition<kRobotCount>& given_start,
      const JointPosition<kRobotCount>& given_goal,
      const FourGrid<kRobotCount>& no_collide_graph,
      const FourGrid<kRobotCount>& collide_graph);

  CostPath<kRobotCount> SolveDStarIterative(
      const JointPosition<kRobotCount>& given_start,
      const JointPosition<kRobotCount>& given_goal,
      const FourGrid<kRobotCount>& no_collide_graph);
};

}  // namespace dstar
}  // namespace solver
}  // namespace fourgrid
}  // namespace search
#endif  // SRC_SEARCH_FOURGRID_DSTAR_FOURGRID_SOLVER_H_

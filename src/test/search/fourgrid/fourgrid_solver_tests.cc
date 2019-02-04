// Copyright 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
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
#include "search/fourgrid/dstar_fourgrid_solver.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

namespace search {
namespace fourgrid {
namespace solver {

namespace dstar {
static constexpr size_t kRobotCount = 2;
TEST(DStarFourGridSolver, CollidingPath) {
  DStarFourGridSolver<kRobotCount> solver;
  const JointPosition<kRobotCount> p0 = {{{-1, 0}, {2, 0}}};
  const JointPosition<kRobotCount> p1 = {{{0, 0}, {1, 0}}};
  const JointPosition<kRobotCount> p2 = {{{1, 0}, {0, 0}}};
  const CostPath<kRobotCount> cost_path = {{p0, 0}, {p1, 0}, {p2, 0}};
  const auto result = solver.CollidingPath(cost_path);
  const FourGridDirectedEdge<kRobotCount> expected_result = {p1, p2, 0};
  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, expected_result);
}

}  // namespace dstar

namespace expandingastar {
TEST(FourGridSolver, OpenListOrdering) {
  static constexpr size_t kRobotCount = 1;
  OpenList<kRobotCount> open_list;

  const JointPosition<kRobotCount> p1 = {{{0, 0}}};
  const JointPosition<kRobotCount> p2 = {{{-1, 0}}};

  const CostPath<kRobotCount> path1 = {{p1, 0.0f}};
  const CostPath<kRobotCount> path2 = {{p1, 0.0f}, {p2, 1.0f}};

  const SearchNode<kRobotCount> n1(path1, 0);
  const SearchNode<kRobotCount> n2(path2, 0);

  open_list.push(n1);
  open_list.push(n2);

  EXPECT_EQ(open_list.top().GetFValue(), 0);
  open_list.pop();
  EXPECT_EQ(open_list.top().GetFValue(), 1);
}

TEST(FourGridSolver, CostExtendOpenClosedListEmptyPath) {
  static constexpr size_t kRobotCount = 1;
  FourGridSolver<kRobotCount> fgs;

  OpenList<kRobotCount> open_list;
  open_list.push({{{{{{0, 0}}}, 0.0f}}, 0.0f});
  ClosedList<kRobotCount> closed_list;
  CostPath<kRobotCount> cost_path;
  fgs.CostExtendOpenClosedList(&open_list, &closed_list, cost_path);
  size_t i = 0;
  for (; !open_list.empty(); ++i) {
    const auto top = open_list.top();
    open_list.pop();
    EXPECT_EQ(top.path.size(), 1ul);
    EXPECT_EQ(top.GetCost(), 0.0f);
    EXPECT_EQ(top.GetFValue(), 0.0f);
  }
  EXPECT_EQ(i, 1ul);
  EXPECT_TRUE(closed_list.empty());
}

TEST(FourGridSolver, CostExtendOpenClosedListRealPath) {
  static constexpr size_t kRobotCount = 1;
  FourGridSolver<kRobotCount> fgs;
  OpenList<kRobotCount> open_list;
  const JointPosition<kRobotCount> window1_start = {{{0, 0}}};
  const JointPosition<kRobotCount> window2_start = {{{-1, 0}}};
  open_list.push({{{window1_start, 0}}, 0});
  ClosedList<kRobotCount> closed_list;
  closed_list.insert({window1_start, {{{window1_start, 0}}, 0}});
  CostPath<kRobotCount> cost_path = {{window2_start, 0.0f},
                                     {window1_start, 1.0f}};
  fgs.CostExtendOpenClosedList(&open_list, &closed_list, cost_path);
  size_t i = 0;
  for (; !open_list.empty(); ++i) {
    const auto top = open_list.top();
    open_list.pop();
    EXPECT_EQ(top.path.size(), 2ul);
    EXPECT_EQ(top.GetCost(), 1.0f);
    EXPECT_EQ(top.GetFValue(), 1.0f);
  }
  EXPECT_EQ(i, 1ul);

  for (const auto& pair : closed_list) {
    EXPECT_EQ(pair.second.path.size(), 2ul);
    EXPECT_TRUE(pair.second.path[0].first == window2_start);
    EXPECT_TRUE(pair.second.path[1].first == window1_start);
    EXPECT_EQ(pair.second.GetCost(), 1.0f);
    EXPECT_EQ(pair.second.GetFValue(), 1.0f);
  }
  EXPECT_EQ(closed_list.size(), 1ul);
}

TEST(FourGridSolver, AddPathNodesBetweenStarts) {
  static constexpr size_t kRobotCount = 1;
  FourGridSolver<kRobotCount> fgs;
  OpenList<kRobotCount> open_list;
  const JointPosition<kRobotCount> window1_start = {{{0, 0}}};
  const JointPosition<kRobotCount> window2_start = {{{-1, 0}}};
  const JointPosition<kRobotCount> window3_start = {{{-2, 0}}};
  const JointPosition<kRobotCount> window4_start = {{{-3, 0}}};
  ClosedList<kRobotCount> closed_list;
  closed_list.insert({window1_start, {{{window1_start, 0}}, 0}});
  CostPath<kRobotCount> cost_path = {{window4_start, 0.0f},
                                     {window3_start, 1.0f},
                                     {window2_start, 2.0f},
                                     {window1_start, 3.0f}};
  fgs.AddPathNodesBetweenStarts(&open_list, cost_path);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(open_list.top().GetCost(), static_cast<float>(i));
    EXPECT_EQ(open_list.top().GetFValue(), static_cast<float>(i));
    ASSERT_EQ(open_list.top().path.size(), i + 1);
    EXPECT_EQ(open_list.top().path[i].first, cost_path[i].first);
    open_list.pop();
  }
}

TEST(FourGridSolver, AddPathNodesBetweenGoals) {
  static constexpr size_t kRobotCount = 1;
  FourGridSolver<kRobotCount> fgs;
  OpenList<kRobotCount> open_list;
  const JointPosition<kRobotCount> window1_start = {{{0, 0}}};
  const JointPosition<kRobotCount> window2_start = {{{-1, 0}}};
  const JointPosition<kRobotCount> window3_start = {{{-2, 0}}};
  const JointPosition<kRobotCount> window4_start = {{{-3, 0}}};
  ClosedList<kRobotCount> closed_list;
  closed_list.insert({window1_start, {{{window1_start, 0}}, 0}});
  const CostPath<kRobotCount> cost_path1 = {{window4_start, 0.0f},
                                            {window3_start, 1.0f},
                                            {window2_start, 2.0f},
                                            {window1_start, 3.0f}};

  const JointPosition<kRobotCount> window1_goal = {{{0, 0}}};
  const JointPosition<kRobotCount> window2_goal = {{{1, 0}}};
  const JointPosition<kRobotCount> window3_goal = {{{2, 0}}};
  const JointPosition<kRobotCount> window4_goal = {{{3, 0}}};

  open_list.push({cost_path1, 0});

  const CostPath<kRobotCount> cost_path2 = {{window1_goal, 0.0f},
                                            {window2_goal, 1.0f},
                                            {window3_goal, 2.0f},
                                            {window4_goal, 3.0f}};

  fgs.AddPathNodesBetweenGoals(&open_list, cost_path1, cost_path2);

  open_list.pop();
  NP_CHECK_EQ(open_list.size(), 4);
  for (size_t i = 0; i < 4; ++i) {
    ASSERT_EQ(open_list.top().path.size(), (4 - i) + cost_path1.size());
    open_list.pop();
  }
}

TEST(FourGridSolver, SetNewGoal) {
  static constexpr size_t kRobotCount = 1;
  FourGridSolver<kRobotCount> fgs;
  OpenList<kRobotCount> open_list;
  const JointPosition<kRobotCount> start = {{{0, 0}}};
  const JointPosition<kRobotCount> new_goal = {{{1, 0}}};
  open_list.push({{{start, 0}}, 0});
  ClosedList<kRobotCount> closed_list;
  fgs.SetNewGoal(&open_list, &closed_list, new_goal);
  ASSERT_TRUE(closed_list.empty());
  ASSERT_FALSE(open_list.empty());
  EXPECT_EQ(open_list.top().GetCost(), 0);
  EXPECT_EQ(open_list.top().heuristic, 1);
  EXPECT_EQ(open_list.top().GetFValue(), 1);
  open_list.pop();
  EXPECT_TRUE(open_list.empty());
}

TEST(FourGridSolver, ZipPlans) {
  static constexpr size_t kRobotCount = 2;
  FourGridSolver<kRobotCount> fgs;
  const CostPath<1> p1 = {{{{{0, 0}}}, 0}, {{{{1, 0}}}, 1}};
  const CostPath<1> p2 = {{{{{0, 0}}}, 0}, {{{{1, 0}}}, 1}, {{{{2, 0}}}, 2}};
  const CostPath<kRobotCount> result = fgs.ZipIndividualPlans({{p1, p2}});
  const CostPath<kRobotCount> expected = {{{{{0, 0}, {0, 0}}}, 0},
                                          {{{{1, 0}, {1, 0}}}, 2},
                                          {{{{1, 0}, {2, 0}}}, 3}};
  EXPECT_EQ(expected, result);
}

}  // namespace expandingastar
}  // namespace solver
}  // namespace fourgrid
}  // namespace search

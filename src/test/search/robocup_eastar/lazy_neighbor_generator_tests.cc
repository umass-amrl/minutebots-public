// // Copyright 2018 kvedder@umass.edu
// // College of Information and Computer Sciences,
// // University of Massachusetts Amherst
// //
// //
// // This software is free: you can redistribute it and/or modify
// // it under the terms of the GNU Lesser General Public License Version 3,
// // as published by the Free Software Foundation.
// //
// // This software is distributed in the hope that it will be useful,
// // but WITHOUT ANY WARRANTY; without even the implied warranty of
// // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// // GNU Lesser General Public License for more details.
// //
// // You should have received a copy of the GNU Lesser General Public License
// // Version 3 in the file COPYING that came with this distribution.
// // If not, see <http://www.gnu.org/licenses/>.
// // ========================================================================
//
// #include "search/robocup_eastar/individual_planner.h"
// #include "search/robocup_eastar/lazy_neighbor_generator.h"
// #include
// "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
//
// namespace search {
// namespace eastar {
// namespace lazy_neighbors {
//
// TEST(LazyNeighborGenerator, SingleRobot) {
//   SearchNode<1> parent;
//   parent.current_position.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   JointPosition<1> goal_position;
//   goal_position.push_back({10, 0});
//   IndividuallyPlannedDataSlice<1> slice;
//   SearchWindow<1> search_window;
//   NeighborGenerator<1> generator(parent, goal_position, 1.0f, &slice,
//                                  &search_window);
//
//   auto min = generator.GetMinimum();
//   ASSERT_EQ(min.current_position.size(), 1ul);
//   ASSERT_EQ(min.current_position.at(0), GridVertex(1, 0));
//   size_t i = 0;
//   Heuristic prev_heuristic = 0;
//   for (; generator.GenerateNext(); ++i) {
//     min = generator.GetMinimum();
//     ASSERT_LE(prev_heuristic, min.heuristic);
//     prev_heuristic = min.heuristic;
//     ASSERT_EQ(min.current_position.size(), 1ul);
//   }
//   ASSERT_EQ(i, 5ul);
// }
//
// TEST(LazyNeighborGenerator, DoubleRobot) {
//   SearchNode<2> parent;
//   parent.current_position.push_back({0, 0});
//   parent.current_position.push_back({0, 0});
//   parent.previous_position.push_back({0, 0});
//   parent.previous_position.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   JointPosition<2> goal_position;
//   goal_position.push_back({10, 0});
//   goal_position.push_back({10, 0});
//   IndividuallyPlannedDataSlice<2> slice;
//   SearchWindow<2> search_window;
//   NeighborGenerator<2> generator(parent, goal_position, 1.0f, &slice,
//                                  &search_window);
//
//   auto min = generator.GetMinimum();
//   ASSERT_EQ(min.current_position.size(), 2ul);
//   ASSERT_EQ(min.current_position.at(0), GridVertex(1, 0));
//   ASSERT_EQ(min.current_position.at(1), GridVertex(1, 0));
//   size_t i = 0;
//   Heuristic prev_heuristic = 0;
//   GridDistance prev_grid_distance = 0;
//   for (; generator.GenerateNext(); ++i) {
//     min = generator.GetMinimum();
//     ASSERT_LE(prev_heuristic, min.heuristic);
//     ASSERT_LE(prev_grid_distance, min.transit_distance);
//     prev_heuristic = min.heuristic;
//     prev_grid_distance = min.transit_distance;
//     ASSERT_EQ(min.current_position.size(), 2ul);
//   }
//   ASSERT_EQ(i, 25ul);
// }
//
// TEST(LazyNeighborGenerator, DoubleRobotOffCenterHeuristic) {
//   SearchNode<2> parent;
//   parent.current_position.push_back({0, 0});
//   parent.current_position.push_back({0, 0});
//   parent.previous_position.push_back({0, 0});
//   parent.previous_position.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   JointPosition<2> goal_position;
//   goal_position.push_back({10, 0});
//   goal_position.push_back({10, 3});
//   IndividuallyPlannedDataSlice<2> slice;
//   SearchWindow<2> search_window;
//   NeighborGenerator<2> generator(parent, goal_position, 1.0f, &slice,
//                                  &search_window);
//
//   auto min = generator.GetMinimum();
//   ASSERT_EQ(min.current_position.size(), 2ul);
//   ASSERT_EQ(min.current_position.at(0), GridVertex(1, 0));
//   ASSERT_EQ(min.current_position.at(1), GridVertex(1, 0));
//   size_t i = 0;
//   Heuristic prev_heuristic = 0;
//   GridDistance prev_grid_distance = 0;
//   for (; generator.GenerateNext(); ++i) {
//     min = generator.GetMinimum();
//     ASSERT_LE(prev_heuristic, min.heuristic);
//     ASSERT_LE(prev_grid_distance, min.transit_distance);
//     prev_heuristic = min.heuristic;
//     prev_grid_distance = min.transit_distance;
//     ASSERT_EQ(min.current_position.size(), 2ul);
//   }
//   ASSERT_EQ(i, 25ul);
// }
//
// TEST(LazyNeighborGenerator, DoubleRobotNegative) {
//   SearchNode<2> parent;
//   parent.current_position.push_back({0, 0});
//   parent.current_position.push_back({0, 0});
//   parent.previous_position.push_back({0, 0});
//   parent.previous_position.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   JointPosition<2> goal_position;
//   goal_position.push_back({-10, 0});
//   goal_position.push_back({-10, 0});
//   IndividuallyPlannedDataSlice<2> slice;
//   SearchWindow<2> search_window;
//   NeighborGenerator<2> generator(parent, goal_position, 1.0f, &slice,
//                                  &search_window);
//
//   auto min = generator.GetMinimum();
//   ASSERT_EQ(min.current_position.size(), 2ul);
//   ASSERT_EQ(min.current_position.at(0), GridVertex(-1, 0))
//       << min.current_position.at(0).transpose() << " vs "
//       << GridVertex(-1, 0).transpose();
//   ASSERT_EQ(min.current_position.at(1), GridVertex(-1, 0))
//       << min.current_position.at(1).transpose() << " vs "
//       << GridVertex(-1, 0).transpose();
//   size_t i = 0;
//   Heuristic prev_heuristic = 0;
//   GridDistance prev_grid_distance = 0;
//   for (; generator.GenerateNext(); ++i) {
//     min = generator.GetMinimum();
//     ASSERT_LE(prev_heuristic, min.heuristic);
//     ASSERT_LE(prev_grid_distance, min.transit_distance);
//     prev_heuristic = min.heuristic;
//     prev_grid_distance = min.transit_distance;
//     ASSERT_EQ(min.current_position.size(), 2ul);
//   }
//   ASSERT_EQ(i, 25ul);
// }
//
// TEST(LazyNeighborGenerator, DoubleRobotGoal) {
//   SearchNode<2> parent;
//   parent.current_position.push_back({0, 0});
//   parent.current_position.push_back({0, 0});
//   parent.previous_position.push_back({0, 0});
//   parent.previous_position.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   JointPosition<2> goal_position;
//   goal_position.push_back({0, 0});
//   goal_position.push_back({0, 0});
//   IndividuallyPlannedDataSlice<2> slice;
//   SearchWindow<2> search_window;
//   NeighborGenerator<2> generator(parent, goal_position, 1.0f, &slice,
//                                  &search_window);
//
//   SearchNode<2> min = generator.GetMinimum();
//   ASSERT_EQ(min.current_position.size(), 2ul);
//   ASSERT_EQ(min.current_position.at(0), GridVertex(0, 0));
//   ASSERT_EQ(min.current_position.at(1), GridVertex(0, 0));
//   size_t i = 0;
//   auto prev_f = 0;
//   for (; generator.GenerateNext(); ++i) {
//     min = generator.GetMinimum();
//     ASSERT_LE(prev_f, min.heuristic + min.transit_distance);
//     prev_f = min.heuristic + min.transit_distance;
//     ASSERT_EQ(min.current_position.size(), 2ul);
//   }
//   ASSERT_EQ(i, 36ul);
// }
//
// TEST(LazyNeighborGenerator, SingleElementGenerator) {
//   SearchNode<2> parent;
//   parent.current_position.push_back({0, 0});
//   parent.current_position.push_back({0, 0});
//   parent.previous_position.push_back({0, 0});
//   parent.previous_position.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   parent.joint_steps.push_back({0, 0});
//   NeighborGenerator<2> generator(parent);
//
//   auto min = generator.GetMinimum();
//   ASSERT_EQ(min, parent);
//   ASSERT_FALSE(generator.GenerateNext());
//   min = generator.GetMinimum();
//   ASSERT_EQ(min, parent);
// }
//
// }  // namespace lazy_neighbors
// }  // namespace eastar
// }  // namespace search

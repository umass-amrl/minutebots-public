// // Copyright 2016 - 2017 kvedder@umass.edu
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
// #include <glog/logging.h>
// #include <eigen3/Eigen/Core>
// #include <iostream>
// #include <vector>
//
// #include "constants/constants.h"
// #include "graph/graph_util.h"
// #include "math/geometry.h"
// #include "math/poses_2d.h"
// #include "navigation/PRM.h"
// #include "navigation/navigation_util.h"
// #include "navigation/scaffolding/scaffold.h"
// #include "obstacles/ball_obstacle.h"
// #include "obstacles/circle_obstacle.h"
// #include "obstacles/obstacle.h"
// #include "obstacles/obstacle_flag.h"
// #include "obstacles/obstacle_type.h"
// #include "obstacles/safety_margin.h"
// #include "state/soccer_state.h"
// #include "state/world_robot.h"
// #include "state/world_state.h"
// #include
// "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
// #include "util/timer.h"
//
// using state::WorldRobot;
// using state::PositionVelocityState;
// using std::cout;
// using std::endl;
// using std::vector;
// using pose_2d::Pose2Df;
// using Eigen::Vector2f;
// using Eigen::Vector3f;
// using obstacle::Obstacle;
// using obstacle::CircleObstacle;
// using obstacle::ObstacleType;
// using obstacle::ObstacleFlag;
// using obstacle::BallObstacle;
// using obstacle::SafetyMargin;
// using state::SoccerState;
// using state::WorldState;
// using geometry::EuclideanDistance;
// using graph::Graph;
// using graph::VertexIndex;
// using graph::GraphIndex;
//
// namespace navigation {
//
// TEST(ScaffoldTests, TempDoesSomethingSane) {
//   pose_2d::Pose2Df old_pose(M_PI / 4, 0, 0);
//   pose_2d::Pose2Df new_pose(0, 200, 200);
//   auto scaffold = scaffolding::GenerateTempCircleScaffold(old_pose);
//
//   for (const auto& vertex : scaffold.GetVertices()) {
//     LOG(INFO) << vertex.position.x() << ", " << vertex.position.y();
//   }
//
//   LOG(INFO) << graph::util::GraphToTextProto(scaffold);
//
//   scaffolding::ModifyScaffoldPose(old_pose, new_pose, &scaffold);
//
//   LOG(INFO) << graph::util::GraphToTextProto(scaffold);
// }
//
// TEST(GraphTests, DumpGraphTest) {
//   Graph graph;
//
//   for (size_t i = 0; i < 10; ++i) {
//     graph.AddVertex(Vector2f(i, i));
//   }
//
//   graph.AddEdge(VertexIndex(0), VertexIndex(1), 1.414);
//   graph.AddEdge(VertexIndex(2), VertexIndex(1), 1.414);
//
//   const auto str = graph::util::GraphToTextProto(graph);
//
//   LOG(INFO) << str;
//
//   Graph graph2 = graph::util::TextProtoToGraph(str);
//
//   EXPECT_EQ(10u, graph2.GetVertices().size());
//   EXPECT_EQ(2u, graph2.GetEdges().size());
//
//   for (const auto& edge : graph2.GetEdges()) {
//     LOG(INFO) << edge.index_1.index << " -> " << edge.index_2.index;
//   }
//
//   const auto str2 = graph::util::GraphToTextProto(graph2);
//
//   LOG(INFO) << str2;
//
//   EXPECT_EQ(str, str2);
// }
//
// TEST(GraphTests, GraphToFileTest) {
//   Graph graph;
//
//   for (size_t i = 0; i < 10; ++i) {
//     graph.AddVertex(Vector2f(i, i));
//   }
//
//   graph.AddEdge(VertexIndex(0), VertexIndex(1), 1.414);
//   graph.AddEdge(VertexIndex(2), VertexIndex(1), 1.414);
//
//   const auto temp_graph_location = "TestGraph";
//
//   graph::util::WriteGraphToProtoFile(temp_graph_location, graph);
//
//   Graph graph2 =
//   graph::util::ReadGraphFromTextProtoFile(temp_graph_location);
//
//   EXPECT_EQ(graph::util::GraphToTextProto(graph),
//             graph::util::GraphToTextProto(graph2));
// }
//
// TEST(PRMPrimitiveTests, AddEdgeToGraphTest) {
//   Graph graph;
//
//   Vector2f point1(100, 0);
//   Vector2f point2(200, 0);
//
//   // Create points for line collision test
//   auto p1_vertex = graph.AddVertex(point1);
//   auto p2_vertex = graph.AddVertex(point2);
//
//   graph.AddEdge(p1_vertex, p2_vertex, 100);
//
//   PRM prm(graph);
//
//   vector<std::pair<GraphIndex, VertexIndex>> path;
//   prm.AStarSearch(0, p1_vertex, 0, p2_vertex, &path);
//   ASSERT_EQ(path.size(), 2u);
//   path.clear();
//   prm.AStarSearch(0, p2_vertex, 0, p1_vertex, &path);
//   ASSERT_EQ(path.size(), 2u);
// }
//
// // Tests to see that the FurthestFreeVertex function finds the path closest
// to
// // the goal.
// TEST(PRMPrimitiveTests, ClosestFreeVertex) {
//   Graph graph;
//   Vector2f start(0, 0);
//   Vector2f point1(100, 0);
//   Vector2f point2(200, 0);
//   Vector2f point3(0, 500);
//   Vector2f goal(600, 0);
//
//   // Create points for finding furthest connected graph.
//   graph.AddVertex(point1);
//   graph.AddVertex(point2);
//   graph.AddVertex(point3);
//   graph.AddVertex(goal);
//
//   PRM prm(graph);
//
//   GraphIndex graph_index = 0;
//   VertexIndex closest_free(7);
//   const VertexIndex kKnownClosestIndex(0);
//   ASSERT_TRUE(prm.ClosestFreeVertex(start, &graph_index, &closest_free));
//   ASSERT_EQ(closest_free, kKnownClosestIndex);
// }
//
// TEST(PRMPrimitiveTests, CollisionFreePath) {
//   PositionVelocityState pvs;
//   WorldState world_state(pvs, team::Team::BLUE);
//   SoccerState soccer_state(world_state);
//   PositionVelocityState::RobotPositionVelocity world_robot(
//       0, Pose2Df(0, Vector2f(400, 0)), Pose2Df(0, Vector2f(0, 0)),
//       Pose2Df(0, Vector2f(400, 0)), 0, 1);
//   pvs.GetMutableOurTeamRobots()->InsertBack(world_robot);
//   ObstacleFlag flags = ObstacleFlag::GetAll(world_state, soccer_state, 0);
//
//   Vector2f point1(100, 1000);
//   Vector2f point2(150, 1000);
//
//   SafetyMargin margin;
//
//   ASSERT_TRUE(CollisionFreePath(flags, margin, point1, point2));
// }
// TEST(PRMPrimitiveTests, CollisionFreePathObstacleBlock) {
//   // This is the obstacle for which we will check the collision of.
//   CircleObstacle obstacle_circle(Pose2Df(0, Vector2f(150, 0)),
//                                  ObstacleType::ROBOT, 100);
//   PositionVelocityState pvs;
//   WorldState world_state(pvs, team::Team::BLUE);
//   SoccerState soccer_state(world_state);
//   PositionVelocityState::RobotPositionVelocity world_robot(
//       0, Pose2Df(0, Vector2f(150, 0)), Pose2Df(0, Vector2f(0, 0)),
//       Pose2Df(0, Vector2f(150, 0)), 0, 0);
//   pvs.GetMutableOurTeamRobots()->InsertBack(world_robot);
//   ObstacleFlag flags = ObstacleFlag::GetAll(world_state, soccer_state, 0);
//
//   Vector2f point1(100, 0);
//   Vector2f point2(200, 0);
//
//   SafetyMargin margin;
//
//   // TODO(kvedder): Re-enable after obstacle rewrite is done.
//   //   ASSERT_FALSE(CollisionFreePath(flags, margin, point1, point2));
// }
//
// TEST(IntegrationTests, AStarSearch) {
//   LOG(INFO)
//       << "Image of this graph available at: http://i.imgur.com/NlTXynm.png";
//
//   Graph graph;
//
//   Vector2f point0(0, 0);
//   Vector2f point1(0, 1);
//   Vector2f point2(1, 1);
//   Vector2f point3(0, 1);
//   Vector2f point4(1, 2);
//   Vector2f point5(3, 2);
//   Vector2f point6(1, 0.5);
//
//   graph.AddVertex(point0);
//   graph.AddVertex(point1);
//   graph.AddVertex(point2);
//   graph.AddVertex(point3);
//   graph.AddVertex(point4);
//   graph.AddVertex(point5);
//   graph.AddVertex(point6);
//
//   graph.AddEdge(VertexIndex(0), VertexIndex(3),
//                 EuclideanDistance(point0, point3));
//
//   graph.AddEdge(VertexIndex(3), VertexIndex(4),
//                 EuclideanDistance(point3, point4));
//   graph.AddEdge(VertexIndex(4), VertexIndex(2),
//                 EuclideanDistance(point4, point2));
//   graph.AddEdge(VertexIndex(0), VertexIndex(1),
//                 EuclideanDistance(point0, point1));
//   graph.AddEdge(VertexIndex(1), VertexIndex(6),
//                 EuclideanDistance(point1, point6));
//   graph.AddEdge(VertexIndex(6), VertexIndex(2),
//                 EuclideanDistance(point6, point2));
//   graph.AddEdge(VertexIndex(0), VertexIndex(5),
//                 EuclideanDistance(point0, point5));
//
//   PRM prm_obj(graph);
//   vector<std::pair<GraphIndex, VertexIndex>> path;
//   prm_obj.AStarSearch(0, VertexIndex(0), 0, VertexIndex(2), &path);
//
//   ASSERT_TRUE(!path.empty());
//   ASSERT_EQ(path[0].second, VertexIndex(0));
//   ASSERT_EQ(path[1].second, VertexIndex(1));
//   ASSERT_EQ(path[2].second, VertexIndex(6));
//   ASSERT_EQ(path[3].second, VertexIndex(2));
// }
//
// TEST(IntegrationTests, AStarSearchV2) {
//   LOG(INFO) << "Image of this graph available at: "
//             << "https://github.com/umass-amrl/minutebots/blob/master/"
//                "documentation/images/prm_shortest_path_finding_test.png";
//   Graph graph;
//   Vector2f point0(0, 0);
//
//   Vector2f point1(1, 1);
//   Vector2f point2(1, 0);
//   Vector2f point3(1, -1);
//
//   Vector2f point4(2, 1);
//   Vector2f point5(2, 0);
//   Vector2f point6(2, -1);
//
//   Vector2f point7(3, 0);
//
//   graph.AddVertex(point0);
//   graph.AddVertex(point1);
//   graph.AddVertex(point2);
//   graph.AddVertex(point3);
//   graph.AddVertex(point4);
//   graph.AddVertex(point5);
//   graph.AddVertex(point6);
//   graph.AddVertex(point7);
//
//   graph.AddEdge(VertexIndex(0), VertexIndex(1),
//                 EuclideanDistance(point0, point1));
//   graph.AddEdge(VertexIndex(0), VertexIndex(2),
//                 EuclideanDistance(point0, point2));
//   graph.AddEdge(VertexIndex(0), VertexIndex(3),
//                 EuclideanDistance(point0, point3));
//
//   graph.AddEdge(VertexIndex(1), VertexIndex(4),
//                 EuclideanDistance(point1, point4));
//   graph.AddEdge(VertexIndex(2), VertexIndex(5),
//                 EuclideanDistance(point2, point5));
//   graph.AddEdge(VertexIndex(3), VertexIndex(6),
//                 EuclideanDistance(point3, point6));
//
//   graph.AddEdge(VertexIndex(4), VertexIndex(7),
//                 EuclideanDistance(point4, point7));
//   graph.AddEdge(VertexIndex(5), VertexIndex(7),
//                 EuclideanDistance(point5, point7));
//   graph.AddEdge(VertexIndex(6), VertexIndex(7),
//                 EuclideanDistance(point6, point7));
//
//   PRM prm_obj(graph);
//   vector<std::pair<GraphIndex, VertexIndex>> path;
//   prm_obj.AStarSearch(0, VertexIndex(0), 0, VertexIndex(7), &path);
//
//   for (auto path_pos : path) {
//     LOG(INFO) << "Path Position: " << path_pos.second.index;
//   }
//
//   ASSERT_TRUE(!path.empty());
//   ASSERT_EQ(path[0].second, VertexIndex(0));
//   ASSERT_EQ(path[1].second, VertexIndex(2));
//   ASSERT_EQ(path[2].second, VertexIndex(5));
//   ASSERT_EQ(path[3].second, VertexIndex(7));
// }
//
// TEST(IntegrationTests, OneShotTest) {
//   LOG(INFO)
//       << "Image of this graph available at: http://i.imgur.com/ICY7lFI.png";
//
//   Graph graph;
//   Vector2f start_point(0, 0);
//
//   Vector2f point0(0, 0);
//   Vector2f point1(1000, 0);
//
//   Vector2f obstacle_position(2000, 0);
//
//   Vector2f point2(3000, 0);
//   Vector2f point3(4000, 0);
//
//   Vector2f end_point(4000, 0);
//
//   graph.AddVertex(point0);
//   graph.AddVertex(point1);
//   graph.AddVertex(point2);
//   graph.AddVertex(point3);
//
//   graph.AddEdge(VertexIndex(0), VertexIndex(1),
//                 EuclideanDistance(point0, point1));
//   graph.AddEdge(VertexIndex(2), VertexIndex(3),
//                 EuclideanDistance(point2, point3));
//
//   PRM prm_obj(graph);
//
//   // This is the robot that we will use as a start position for the search.
//   Pose2Df robot_pose(0, start_point);
//
//   // This is the obstacle blocking the path from the start to the goal node.
//   Pose2Df obstacle_pose(0, obstacle_position);
//   CircleObstacle obstacle(obstacle_pose, ObstacleType::ROBOT, kRobotRadius);
//
//   PositionVelocityState pvs;
//   WorldState world_state(pvs, team::Team::BLUE);
//   SoccerState soccer_state(world_state);
//   PositionVelocityState::RobotPositionVelocity world_robot(
//       0, Pose2Df(0, Vector2f(0, 0)), Pose2Df(0, Vector2f(0, 0)),
//       Pose2Df(0, Vector2f(0, 0)), 0, 0);
//   pvs.GetMutableOurTeamRobots()->InsertBack(world_robot);
//   ObstacleFlag flags = ObstacleFlag::GetAll(world_state, soccer_state, 0);
//
//   vector<Vector2f> waypoints;
//   prm_obj.Update(flags, robot_pose, end_point);
//   prm_obj.Plan(&waypoints);
//
//   // TODO(kvedder): Re-enable after obstacle rewrite.
//   //   ASSERT_EQ(static_cast<int>(waypoints.size()), 2);
//
//   Vector2f reference_intersect_point;
//   float reference_intersect_length;
//   obstacle.FurthestFreePointOnLine(
//       start_point, end_point, &reference_intersect_point,
//       &reference_intersect_length, kDefaultSafetyMargin);
//   LOG(INFO) << "Ref intersect len: " << reference_intersect_length << "\n";
//   //   ASSERT_EQ(waypoints[1].x(), reference_intersect_point.x());
//   //   ASSERT_EQ(waypoints[1].y(), reference_intersect_point.y());
// }
//
// // TEST(IntegrationTests, RoboCupField) {
// //   PositionVelocityState pvs;
// //   WorldState world_state(pvs, team::Team::BLUE);
// //
// //   Pose2Df current_pose;
// //   for (int i = 0; i < 6; ++i) {
// //     if (i == 0) {
// //       current_pose.angle = 0;
// //       current_pose.translation.x() = 1000 * (i - 2);
// //       current_pose.translation.y() = 1000;
// //     } else {
// //       Pose2Df robot_pose(0, Vector2f(1000 * (i - 2), 1000));
// //       PositionVelocityState::RobotPositionVelocity world_robot(
// //           0, Pose2Df(0, Vector2f(150 * i, 0)), Pose2Df(0, Vector2f(150,
// 0)),
// //           Pose2Df(0, Vector2f(150 * i, 0)), 0, 0);
// //       pvs.GetMutableTheirTeamRobots()->InsertBack(world_robot);
// //     }
// //   }
// //
// //   ObstacleFlag flag = ObstacleFlag::GetStaticObstacles(world_state);
// //   SafetyMargin margin;
// //   PRM prm(flag, margin, 2000);
// //
// //   Vector2f goal_pos(4000, 1000);
// //   const int num_iterations = 5;
// //
// //   vector<Vector2f> waypoints;
// //   for (int loop_iteration = 0; loop_iteration < num_iterations;
// //        ++loop_iteration) {
// //     double loop_start = GetMonotonicTime();
// //     prm.Update(ObstacleFlag::GetAll(world_state), current_pose, goal_pos);
// //     double update_time = GetMonotonicTime();
// //     prm.Plan(&waypoints);
// //     double plan_time = GetMonotonicTime();
// //     // Test to see if the path found isn't empty.
// //     ASSERT_GT((int)waypoints.size(), 0);
// //     LOG(INFO) << "Path Size: " << waypoints.size();
// //     LOG(INFO) << "Total PRM Runtime: " << plan_time - loop_start;
// //     LOG(INFO) << "Update Time: " << update_time - loop_start;
// //     LOG(INFO) << "Planning time: " << plan_time - update_time;
// //     waypoints.clear();
// //   }
// // }
// }  // namespace navigation

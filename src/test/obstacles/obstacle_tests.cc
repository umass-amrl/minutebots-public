// Copyright 2016 - 2018 kvedder@umass.edu, slane@cs.umass.edu
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
#include <glog/logging.h>
#include <stdio.h>
#include <string.h>
#include <bitset>
#include "eigen3/Eigen/Core"

#include "constants/constants.h"
#include "graph/edge.h"
#include "logging/logger.h"
#include "math/geometry.h"
#include "math/poses_2d.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/rectangle_obstacle.h"
#include "obstacles/robot_obstacle.h"
#include "obstacles/safety_margin.h"
#include "state/soccer_state.h"
#include "state/team.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

STANDARD_USINGS;
using team::Team;
using geometry::LineLineIntersection;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using obstacle::CircleObstacle;
using obstacle::RobotObstacle;
using obstacle::RectangleObstacle;
using obstacle::SafetyMargin;
using state::SoccerState;
using state::WorldState;
using state::WorldRobot;
using state::PositionVelocityState;
using geometry::EuclideanDistance;
using graph::Edge;
using logger::Logger;
using logger::NetLogger;

namespace obstacle {

TEST(ObstaclePrimitiveTests, BasicCircleLineCollisionCheckLineSegmentBounds) {
  Vector2f position(1000, 0);
  Pose2Df obstacle_pose(0, position);
  CircleObstacle random_circle(obstacle_pose, ObstacleType::ROBOT, 100);
  Vector2f point1(100, 0);
  Vector2f point2(200, 0);
  ASSERT_FALSE(random_circle.LineCollision(point1, point2, 0));
}

TEST(ObstaclePrimitiveTests, TangentCheck) {
  Vector2f position(500, 1);
  Pose2Df obstacle_pose(0, position);
  CircleObstacle circle(obstacle_pose, ObstacleType::ROBOT, kRobotRadius);
  Vector2f point1(2000, 0);

  Vector2f left_tangent;
  Vector2f right_tangent;
  SafetyMargin margin;
  geometry::GetTangentPoints(
      point1, position,
      kRobotRadius + margin.GetMargin(ObstacleType::ROBOT) + 1, &right_tangent,
      &left_tangent);

  ASSERT_FALSE(circle.PointCollision(right_tangent,
                                     margin.GetMargin(ObstacleType::ROBOT)));
  ASSERT_FALSE(circle.LineCollision(point1, right_tangent,
                                    margin.GetMargin(ObstacleType::ROBOT)));
  ASSERT_FALSE(circle.PointCollision(left_tangent,
                                     margin.GetMargin(ObstacleType::ROBOT)));
  ASSERT_FALSE(circle.LineCollision(point1, left_tangent,
                                    margin.GetMargin(ObstacleType::ROBOT)));
}

TEST(ObstaclePrimitiveTests, BasicCircleLineCollisionCheckColinearCenter) {
  Vector2f position(150, 0);
  Pose2Df obstacle_pose(0, position);
  CircleObstacle random_circle(obstacle_pose, ObstacleType::ROBOT, 100);
  Vector2f point1(50, 0);
  Vector2f point2(100, 0);

  ASSERT_TRUE(random_circle.LineCollision(point1, point2, 0));
}

TEST(ObstaclePrimitiveTests, BasicCircleLineCollisionCheckRadialIntersect) {
  Vector2f position(150, 99);
  Pose2Df obstacle_pose(0, position);
  CircleObstacle random_circle(obstacle_pose, ObstacleType::ROBOT, 100);
  Vector2f point1(100, 0);
  Vector2f point2(200, 0);

  ASSERT_TRUE(random_circle.LineCollision(point1, point2, 0));
}

TEST(ObstaclePrimitiveTests, BasicCircleFarthestFreePoint) {
  CircleObstacle obstacle_circle(Pose2Df(0, Vector2f(0, 0)),
                                 ObstacleType::ROBOT, 100);
  Vector2f start_point(-200, 0);
  Vector2f end_point(200, 0);

  Vector2f intersect_point;
  float intersect_distance;
  ASSERT_TRUE(obstacle_circle.FurthestFreePointOnLine(
      start_point, end_point, &intersect_point, &intersect_distance, 0));
  ASSERT_TRUE(fabs(intersect_distance - 100.0f) < 0.01);
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionContactMass) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(20, 20)),
                                       ObstacleType::STATIC, 10, 10);
  Vector2f start_point(20, 100);
  Vector2f end_point(20, -100);

  const float margin = 5;

  ASSERT_TRUE(obstacle_rectangle.LineCollision(start_point, end_point, margin));
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionContactMargin) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(20, 20)),
                                       ObstacleType::STATIC, 10, 10);

  const std::vector<float> start_point_x1 = {14.9, 14.99, 10};
  const std::vector<float> start_point_x2 = {14.9, 14.99, 15};

  const float margin = 5;

  for (size_t i = 0; i < start_point_x1.size(); ++i) {
    Vector2f start_point(start_point_x1[i], 100);
    Vector2f end_point(start_point_x2[i], -100);

    LOG(INFO) << "i: " << i;

    EXPECT_TRUE(
        obstacle_rectangle.LineCollision(start_point, end_point, margin));
  }

  const std::vector<float> start_point_y1 = {14.9, 14.99, 10};
  const std::vector<float> start_point_y2 = {14.9, 14.99, 15};

  for (size_t i = 0; i < start_point_y1.size(); ++i) {
    Vector2f start_point(100, start_point_y1[i]);
    Vector2f end_point(-100, start_point_y2[i]);

    LOG(INFO) << "i: " << i;

    EXPECT_TRUE(
        obstacle_rectangle.LineCollision(start_point, end_point, margin));
  }
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionContactCornerMargin) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(20, 20)),
                                       ObstacleType::STATIC, 10, 10);

  Vector2f start_point(12, 12);
  Vector2f end_point(0, 0);

  const float margin = 5;

  ASSERT_TRUE(obstacle_rectangle.LineCollision(start_point, end_point, margin));
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionContactCornerMargin2) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(20, 20)),
                                       ObstacleType::STATIC, 10, 10);

  Vector2f start_point(9, 14);
  Vector2f end_point(16, 9);

  const float margin = 5;

  ASSERT_TRUE(obstacle_rectangle.LineCollision(start_point, end_point, margin));
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionNoContactCornerMargin) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(20, 20)),
                                       ObstacleType::STATIC, 10, 10);

  Vector2f start_point(12, 12);
  Vector2f end_point(0, 0);

  const float margin = 0;

  ASSERT_FALSE(
      obstacle_rectangle.LineCollision(start_point, end_point, margin));
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionNoContactNoMargin) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(20, 20)),
                                       ObstacleType::STATIC, 10, 10);
  Vector2f start_point(12, 100);
  Vector2f end_point(12, -100);

  const std::vector<float> start_point_x1 = {14.9, 14.99, 10};
  const std::vector<float> start_point_x2 = {14.9, 14.99, 15};

  const float margin = 0;

  for (size_t i = 0; i < start_point_x1.size(); ++i) {
    Vector2f start_point(start_point_x1[i], 100);
    Vector2f end_point(start_point_x2[i], -100);

    LOG(INFO) << "i: " << i;

    ASSERT_FALSE(
        obstacle_rectangle.LineCollision(start_point, end_point, margin));
  }
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionEdge) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 10, 10);
  Vector2f start_point(-200, 5);
  Vector2f end_point(200, 5);

  ASSERT_TRUE(obstacle_rectangle.LineCollision(start_point, end_point, 0));
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionNoContact) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 10, 10);
  Vector2f start_point(-200, 6);
  Vector2f end_point(200, 6);

  ASSERT_FALSE(obstacle_rectangle.LineCollision(start_point, end_point, 0));
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionTopDown) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 10, 10);
  Vector2f start_point(0, 200);
  Vector2f end_point(0, -200);

  ASSERT_TRUE(obstacle_rectangle.LineCollision(start_point, end_point, 0));
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionDiag) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 10, 10);
  Vector2f start_point(200, 200);
  Vector2f end_point(-200, -200);

  ASSERT_TRUE(obstacle_rectangle.LineCollision(start_point, end_point, 0));
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionOffCenterDiag1) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 10, 10);
  Vector2f start_point(-5, 0);
  Vector2f end_point(0, 5);

  ASSERT_TRUE(
      obstacle_rectangle.LineCollision(start_point, end_point, kRobotRadius));
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineCollisionOffCenterDiag2) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 500)),
                                       ObstacleType::STATIC, 100, 1000);
  Vector2f start_point(-50, 5);
  Vector2f end_point(-45, 0);

  ASSERT_TRUE(obstacle_rectangle.LineCollision(start_point, end_point, 0));
}

TEST(ObstaclePrimitiveTests, BasicRectanglePointCollision) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 10, 10);
  Vector2f query_point(0, 0);

  ASSERT_TRUE(obstacle_rectangle.PointCollision(query_point, 0));
}

TEST(ObstaclePrimitiveTests, BasicRectangleFarthestFreePoint) {
  vector<const Obstacle*> dynamic_obstacles;

  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 10, 10);
  dynamic_obstacles.push_back(&obstacle_rectangle);

  Vector2f start_point(-400, 0);
  Vector2f end_point(200, 0);

  Vector2f intersect_point;
  float intersect_distance;

  ASSERT_TRUE(obstacle_rectangle.FurthestFreePointOnLine(
      start_point, end_point, &intersect_point, &intersect_distance, 0));
  LOG(INFO) << "Intersect point: " << intersect_point.x() << ", "
            << intersect_point.y() << "\n";
  EXPECT_LE(fabs(intersect_distance - 395.0f), 0.002f);
  EXPECT_LE(fabs(intersect_point.x() + 5.0f), 0.002f);
  EXPECT_LE(fabs(intersect_point.y() - 0.0f), 0.002f);
  EXPECT_FALSE(obstacle_rectangle.PointCollision(intersect_point, 0));
}

TEST(ObstaclePrimitiveTests, BasicRectangleFarthestFreePointWithMargin) {
  vector<const Obstacle*> dynamic_obstacles;

  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 10, 10);
  dynamic_obstacles.push_back(&obstacle_rectangle);

  // Horizontal line through (0, 0).
  Vector2f start_point(-400, 0);
  Vector2f end_point(200, 0);

  Vector2f intersect_point;
  float intersect_distance;

  const float margin = 5;

  ASSERT_TRUE(obstacle_rectangle.FurthestFreePointOnLine(
      start_point, end_point, &intersect_point, &intersect_distance, margin));
  LOG(INFO) << "Intersect point: " << intersect_point.x() << ", "
            << intersect_point.y() << "\n";
  LOG(INFO) << "Intersect Distance: " << intersect_distance;
  EXPECT_LE(fabs(intersect_distance - 390.0f), 0.002f);
  EXPECT_LE(fabs(intersect_point.x() + 10.0f), 0.002f);
  EXPECT_LE(fabs(intersect_point.y() - 0.0f), 0.002f);
  EXPECT_FALSE(obstacle_rectangle.PointCollision(intersect_point, margin));
}

TEST(ObstaclePrimitiveTests, BasicRectangleFarthestFreePointWithMargin2) {
  vector<const Obstacle*> dynamic_obstacles;

  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 17, 10);
  dynamic_obstacles.push_back(&obstacle_rectangle);

  // Horizontal line through (0, 0).
  Vector2f start_point(-400, 0);
  Vector2f end_point(200, 0);

  Vector2f intersect_point;
  float intersect_distance;

  const float margin = 5;

  ASSERT_TRUE(obstacle_rectangle.FurthestFreePointOnLine(
      start_point, end_point, &intersect_point, &intersect_distance, margin));
  LOG(INFO) << "Intersect point: " << intersect_point.x() << ", "
            << intersect_point.y() << "\n";
  LOG(INFO) << "Intersect Distance: " << intersect_distance;
  EXPECT_LE(fabs(intersect_distance - 386.5f), 0.002f);
  EXPECT_LE(fabs(intersect_point.x() + 13.5f), 0.002f);
  EXPECT_LE(fabs(intersect_point.y() - 0.0f), 0.002f);
  EXPECT_FALSE(obstacle_rectangle.PointCollision(intersect_point, margin));
}

TEST(ObstaclePrimitiveTests, BasicRectangleNearestFreePoint) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 10, 10);

  Vector2f check_point(4, 4);
  Vector2f result = obstacle_rectangle.NearestFreePoint(check_point, 0);
  LOG(INFO) << "Result: " << result.x() << ", " << result.y();
  ASSERT_FALSE(obstacle_rectangle.PointCollision(result, 0));
}

TEST(ObstaclePrimitiveTests, BasicRectangleNearestFreePointWithMargin) {
  RectangleObstacle obstacle_rectangle(Pose2Df(0, Vector2f(0, 0)),
                                       ObstacleType::STATIC, 10, 10);

  const float margin = 5;
  Vector2f check_point(9, 9);
  Vector2f result = obstacle_rectangle.NearestFreePoint(check_point, margin);
  LOG(INFO) << "Result: " << result.x() << ", " << result.y();
  ASSERT_FALSE(obstacle_rectangle.PointCollision(result, margin));
}

TEST(ObstaclePrimitiveTests, TangentLineCollisionTest) {
  Vector2f position(200, 0);
  CircleObstacle obstacle(Pose2Df(0, 0, 0), ObstacleType::ROBOT, 100);
  float margin = 20;

  Vector2f point_1;
  Vector2f point_2;

  obstacle.GetTangents(position, margin, &point_1, &point_2);

  ASSERT_FALSE(obstacle.LineCollision(position, point_1, margin));
  ASSERT_FALSE(obstacle.LineCollision(position, point_2, margin));
}

TEST(ObstaclePrimitiveTests, BasicRectangleLineDistanceNoMargin) {
  RectangleObstacle obstacle(Pose2Df(0, Vector2f(0, 0)), ObstacleType::STATIC,
                             5, 5);
  float margin = 0;

  Vector2f point_1(-5, -20);
  Vector2f point_2(-5, 20);

  const float distance = obstacle.LineDistance(point_1, point_2, margin);

  LOG(INFO) << "Distance: " << distance;
}

TEST(ObstaclePrimitiveTests, TangentLineRectangleNoMargin) {
  Vector2f position(0, -10);
  RectangleObstacle obstacle(Pose2Df(0, Vector2f(0, 0)), ObstacleType::STATIC,
                             5, 5);
  float margin = 0;

  Vector2f point_1;
  Vector2f point_2;

  obstacle.GetTangents(position, margin, &point_1, &point_2);

  LOG(INFO) << "TANGENTS WORKS TEST:";
  LOG(INFO) << "Left: " << point_2.x() << ", " << point_2.y();
  LOG(INFO) << "Right: " << point_1.x() << ", " << point_1.y();

  EXPECT_FALSE(obstacle.LineCollision(position, point_2, margin));
  EXPECT_FALSE(obstacle.LineCollision(position, point_1, margin));
}

TEST(ObstaclePrimitiveTests, TangentLineRectangleMargin) {
  Vector2f position(0, -10);
  RectangleObstacle obstacle(Pose2Df(0, Vector2f(0, 0)), ObstacleType::STATIC,
                             5, 5);
  float margin = 2.5;

  Vector2f point_1;
  Vector2f point_2;

  obstacle.GetTangents(position, margin, &point_1, &point_2);

  LOG(INFO) << "TANGENTS WORKS TEST:";
  LOG(INFO) << "Left: " << point_2.x() << ", " << point_2.y();
  LOG(INFO) << "Right: " << point_1.x() << ", " << point_1.y();

  EXPECT_FALSE(obstacle.LineCollision(position, point_2, margin));
  EXPECT_FALSE(obstacle.LineCollision(position, point_1, margin));
}

TEST(ObstaclePrimitiveTests, BiTangentLineCollisionTest) {
  Vector2f start(-200, 0);
  Vector2f goal(-200, 0);
  CircleObstacle obstacle(Pose2Df(0, 0, 0), ObstacleType::ROBOT, 100);
  float margin = 20;

  Vector2f start_tangent_left;
  Vector2f start_tangent_right;

  obstacle.GetTangents(start, margin, &start_tangent_left,
                       &start_tangent_right);

  Vector2f goal_tangent_left;
  Vector2f goal_tangent_right;

  obstacle.GetTangents(goal, margin, &goal_tangent_left, &goal_tangent_right);

  Vector2f bi_tangent_left =
      LineLineIntersection(start, start_tangent_left, goal, goal_tangent_right);

  Vector2f bi_tangent_right =
      LineLineIntersection(start, start_tangent_right, goal, goal_tangent_left);

  ASSERT_FALSE(obstacle.LineCollision(start, bi_tangent_left, margin));
  ASSERT_FALSE(obstacle.LineCollision(start, bi_tangent_right, margin));
  ASSERT_FALSE(obstacle.LineCollision(bi_tangent_left, goal, margin));
  ASSERT_FALSE(obstacle.LineCollision(bi_tangent_right, goal, margin));
  ASSERT_FALSE(obstacle.PointCollision(bi_tangent_left, margin));
  ASSERT_FALSE(obstacle.PointCollision(bi_tangent_right, margin));
}

const bool kStaticTest = false;
TEST(StaticObstacleTests, TestStaticObstacles) {
  if (kStaticTest) {
    PositionVelocityState pvs;
    WorldState world_state(&pvs, Team::BLUE);
    logger::NetLogger n_logger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);

    ObstacleFlag obstacles = ObstacleFlag::GetStaticObstacles();

    for (const auto& obstacle : obstacles) {
      obstacle->DrawObstacle(&n_logger);
    }

    n_logger.LogPrint("Hello world");

    n_logger.SetMessageTime(200);
    n_logger.SendData();
  }
}

TEST(ObstacleFlagTests, IterTestAllExceptTeam) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);
  SoccerState soccer_state(world_state, Team::BLUE);
  logger::NetLogger logger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);
  const auto& obstacles = world_state.GetAllObstacles();

  LOG(INFO) << pvs.GetOurTeamRobots().GetElementCount();
  for (int i = 0; i < static_cast<int>(kMaxTeamRobots); ++i) {
    PositionVelocityState::RobotPositionVelocity r_our(
        i, Pose2Df(0, Vector2f(i * 100, 0)), Pose2Df(0, Vector2f(i * 100, 0)),
        Pose2Df(0, Vector2f(i * 100, 0)), {0, 0, 0}, 0, 1);

    PositionVelocityState::RobotPositionVelocity r_their(
        i, Pose2Df(0, Vector2f(i * -100 - 100, 0)),
        Pose2Df(0, Vector2f(i * -100 - 100, 0)),
        Pose2Df(0, Vector2f(i * -100 - 100, 0)), {0, 0, 0}, 0, 1);
    pvs.GetMutableOurTeamRobots()->InsertBack(r_our);
    pvs.GetMutableTheirTeamRobots()->InsertBack(r_their);
  }
  LOG(INFO) << pvs.GetOurTeamRobots().GetElementCount();

  for (size_t i = 0; i < pvs.GetOurTeamRobots().GetElementCount(); ++i) {
    const auto& our_robot = pvs.GetOurTeamRobots().Get(i);
    LOG(INFO) << "Our: "
              << "(" << our_robot.position.translation.x() << ", "
              << our_robot.position.translation.y() << ")"
              << " e: " << ((our_robot.confidence > 0) ? "TRUE" : "FALSE");
  }

  for (size_t i = 0; i < pvs.GetTheirTeamRobots().GetElementCount(); ++i) {
    const auto& their_robot = pvs.GetTheirTeamRobots().Get(i);
    LOG(INFO) << "Their: "
              << "(" << their_robot.position.translation.x() << ", "
              << their_robot.position.translation.y() << ")"
              << " e: " << ((their_robot.confidence > 0) ? "TRUE" : "FALSE");
  }

  pvs.SetTime(GetWallTime());

  LOG(INFO) << "Looped";

  world_state.UpdateState(&logger);

  LOG(INFO) << "Updated world state";

  auto all = ObstacleFlag::GetAllExceptTeam(world_state, soccer_state, 0, 0);

  LOG(INFO) << "Got obstacle flag";

  size_t total_count = 0;
  size_t robot_count = 0;
  size_t ball_count = 0;
  size_t rules_count = 0;
  size_t static_count = 0;
  LOG(INFO) << "================";
  size_t i = 0;
  for (const Obstacle* obstacle : all) {
    total_count++;
    switch (obstacle->GetType()) {
      case obstacle::ROBOT: {
        robot_count++;
      } break;
      case obstacle::RULE: {
        rules_count++;
      } break;
      case obstacle::BALL: {
        ball_count++;
      } break;
      case obstacle::STATIC: {
        static_count++;
      } break;
      case obstacle::NUM_OBSTACLE_TYPES: {
        // Noop();
      } break;
    }
    ++i;
  }
  LOG(INFO) << "================";
  EXPECT_EQ(obstacles.size(), static_cast<size_t>(kNumObstacles));
  EXPECT_EQ(total_count, static_cast<size_t>(kNumObstacles) -
                             static_cast<size_t>(kNumRulesObstacles) + 2 -
                             (kNumBallObstacles - 1));
  EXPECT_EQ(robot_count, static_cast<size_t>(kMaxTeamRobots * kNumTeams) - 1);
  EXPECT_EQ(ball_count, static_cast<size_t>(1));
  EXPECT_EQ(static_count, static_cast<size_t>(kNumStaticObstacles));
}

TEST(ObstacleFlagTests, IterTestAssignment) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);
  SoccerState soccer_state(world_state, Team::BLUE);
  logger::NetLogger logger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);
  const auto& obstacles = world_state.GetAllObstacles();

  LOG(INFO) << pvs.GetOurTeamRobots().GetElementCount();
  for (int i = 0; i < static_cast<int>(kMaxTeamRobots); ++i) {
    PositionVelocityState::RobotPositionVelocity r_our(
        i, Pose2Df(0, Vector2f(i * 100, 0)), Pose2Df(0, Vector2f(i * 100, 0)),
        Pose2Df(0, Vector2f(i * 100, 0)), {0, 0, 0}, 0, 1);

    PositionVelocityState::RobotPositionVelocity r_their(
        i, Pose2Df(0, Vector2f(i * -100 - 100, 0)),
        Pose2Df(0, Vector2f(i * -100 - 100, 0)),
        Pose2Df(0, Vector2f(i * -100 - 100, 0)), {0, 0, 0}, 0, 1);
    pvs.GetMutableOurTeamRobots()->InsertBack(r_our);
    pvs.GetMutableTheirTeamRobots()->InsertBack(r_their);
  }
  LOG(INFO) << pvs.GetOurTeamRobots().GetElementCount();

  for (size_t i = 0; i < pvs.GetOurTeamRobots().GetElementCount(); ++i) {
    const auto& our_robot = pvs.GetOurTeamRobots().Get(i);
    LOG(INFO) << "Our: "
              << "(" << our_robot.position.translation.x() << ", "
              << our_robot.position.translation.y() << ")"
              << " e: " << ((our_robot.confidence > 0) ? "TRUE" : "FALSE");
  }

  for (size_t i = 0; i < pvs.GetTheirTeamRobots().GetElementCount(); ++i) {
    const auto& their_robot = pvs.GetTheirTeamRobots().Get(i);
    LOG(INFO) << "Our: "
              << "(" << their_robot.position.translation.x() << ", "
              << their_robot.position.translation.y() << ")"
              << " e: " << ((their_robot.confidence > 0) ? "TRUE" : "FALSE");
  }

  pvs.SetTime(GetWallTime());

  world_state.UpdateState(&logger);

  auto all = ObstacleFlag::GetAll(world_state, soccer_state, 0);
  ObstacleFlag default_constructed_obstacle_flag;
  default_constructed_obstacle_flag = all;
  size_t total_count = 0;
  size_t robot_count = 0;
  size_t ball_count = 0;
  size_t rules_count = 0;
  size_t static_count = 0;
  LOG(INFO) << "================";
  size_t i = 0;
  for (const Obstacle* obstacle : default_constructed_obstacle_flag) {
    total_count++;
    switch (obstacle->GetType()) {
      case obstacle::ROBOT: {
        robot_count++;
      } break;
      case obstacle::RULE: {
        rules_count++;
      } break;
      case obstacle::BALL: {
        ball_count++;
      } break;
      case obstacle::STATIC: {
        static_count++;
      } break;
      case obstacle::NUM_OBSTACLE_TYPES: {
        // Noop();
      } break;
    }
    ++i;
  }
  LOG(INFO) << "================";
  EXPECT_EQ(obstacles.size(), static_cast<size_t>(kNumObstacles));
  EXPECT_EQ(total_count,
            static_cast<size_t>(kNumObstacles - kNumRulesObstacles + 1) + 2 -
                (kNumBallObstacles - 1));
  EXPECT_EQ(robot_count, static_cast<size_t>(kMaxTeamRobots * kNumTeams));
  EXPECT_EQ(ball_count, static_cast<size_t>(1));
  EXPECT_EQ(static_count, static_cast<size_t>(kNumStaticObstacles));
}

TEST(ObstacleFlagTests, IterTestAll) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);
  SoccerState soccer_state(world_state, Team::BLUE);
  logger::NetLogger logger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);
  const auto& obstacles = world_state.GetAllObstacles();

  LOG(INFO) << pvs.GetOurTeamRobots().GetElementCount();
  for (int i = 0; i < static_cast<int>(kMaxTeamRobots); ++i) {
    PositionVelocityState::RobotPositionVelocity r_our(
        i, Pose2Df(0, Vector2f(i * 100, 0)), Pose2Df(0, Vector2f(i * 100, 0)),
        Pose2Df(0, Vector2f(i * 100, 0)), {0, 0, 0}, 0, 1);

    PositionVelocityState::RobotPositionVelocity r_their(
        i, Pose2Df(0, Vector2f(i * -100 - 100, 0)),
        Pose2Df(0, Vector2f(i * -100 - 100, 0)),
        Pose2Df(0, Vector2f(i * -100 - 100, 0)), {0, 0, 0}, 0, 1);
    pvs.GetMutableOurTeamRobots()->InsertBack(r_our);
    pvs.GetMutableTheirTeamRobots()->InsertBack(r_their);
  }
  LOG(INFO) << pvs.GetOurTeamRobots().GetElementCount();

  for (size_t i = 0; i < pvs.GetOurTeamRobots().GetElementCount(); ++i) {
    const auto& our_robot = pvs.GetOurTeamRobots().Get(i);
    LOG(INFO) << "Our: "
              << "(" << our_robot.position.translation.x() << ", "
              << our_robot.position.translation.y() << ")"
              << " e: " << ((our_robot.confidence > 0) ? "TRUE" : "FALSE");
  }

  for (size_t i = 0; i < pvs.GetTheirTeamRobots().GetElementCount(); ++i) {
    const auto& their_robot = pvs.GetTheirTeamRobots().Get(i);
    LOG(INFO) << "Our: "
              << "(" << their_robot.position.translation.x() << ", "
              << their_robot.position.translation.y() << ")"
              << " e: " << ((their_robot.confidence > 0) ? "TRUE" : "FALSE");
  }
  pvs.SetTime(GetWallTime());

  world_state.UpdateState(&logger);

  auto all = ObstacleFlag::GetAll(world_state, soccer_state, 0);
  size_t total_count = 0;
  size_t robot_count = 0;
  size_t ball_count = 0;
  size_t rules_count = 0;
  size_t static_count = 0;
  LOG(INFO) << "================";
  size_t i = 0;
  for (const Obstacle* obstacle : all) {
    total_count++;
    switch (obstacle->GetType()) {
      case obstacle::ROBOT: {
        robot_count++;
      } break;
      case obstacle::RULE: {
        rules_count++;
      } break;
      case obstacle::BALL: {
        ball_count++;
      } break;
      case obstacle::STATIC: {
        static_count++;
      } break;
      case obstacle::NUM_OBSTACLE_TYPES: {
        // Noop();
      } break;
    }
    ++i;
  }
  LOG(INFO) << "================";
  EXPECT_EQ(obstacles.size(), static_cast<size_t>(kNumObstacles));
  EXPECT_EQ(total_count,
            static_cast<size_t>(kNumObstacles - kNumRulesObstacles + 2) + 1 -
                (kNumBallObstacles - 1));
  EXPECT_EQ(robot_count, static_cast<size_t>(kMaxTeamRobots * kNumTeams));
  EXPECT_EQ(ball_count, static_cast<size_t>(1));
  EXPECT_EQ(static_count, static_cast<size_t>(kNumStaticObstacles));
}

TEST(ObstacleFlagTests, IterTestAllSomeUpdated) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);
  SoccerState soccer_state(world_state, Team::BLUE);
  logger::NetLogger logger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);
  const auto& obstacles = world_state.GetAllObstacles();

  LOG(INFO) << pvs.GetOurTeamRobots().GetElementCount();
  for (int i = 0; i < static_cast<int>(kMaxTeamRobots - 4); ++i) {
    PositionVelocityState::RobotPositionVelocity r_our(
        i, Pose2Df(0, Vector2f(i * 100, 0)), Pose2Df(0, Vector2f(i * 100, 0)),
        Pose2Df(0, Vector2f(i * 100, 0)), {0, 0, 0}, 0, 1);

    PositionVelocityState::RobotPositionVelocity r_their(
        i, Pose2Df(0, Vector2f(i * -100 - 100, 0)),
        Pose2Df(0, Vector2f(i * -100 - 100, 0)),
        Pose2Df(0, Vector2f(i * -100 - 100, 0)), {0, 0, 0}, 0, 1);
    pvs.GetMutableOurTeamRobots()->InsertBack(r_our);
    pvs.GetMutableTheirTeamRobots()->InsertBack(r_their);
  }
  LOG(INFO) << pvs.GetOurTeamRobots().GetElementCount();

  for (size_t i = 0; i < pvs.GetOurTeamRobots().GetElementCount(); ++i) {
    const auto& our_robot = pvs.GetOurTeamRobots().Get(i);
    LOG(INFO) << "Our: "
              << "(" << our_robot.position.translation.x() << ", "
              << our_robot.position.translation.y() << ")"
              << " e: " << ((our_robot.confidence > 0) ? "TRUE" : "FALSE");
  }

  for (size_t i = 0; i < pvs.GetTheirTeamRobots().GetElementCount(); ++i) {
    const auto& their_robot = pvs.GetTheirTeamRobots().Get(i);
    LOG(INFO) << "Our: "
              << "(" << their_robot.position.translation.x() << ", "
              << their_robot.position.translation.y() << ")"
              << " e: " << ((their_robot.confidence > 0) ? "TRUE" : "FALSE");
  }

  pvs.SetTime(GetWallTime());

  world_state.UpdateState(&logger);

  auto all = ObstacleFlag::GetAll(world_state, soccer_state, 0);
  size_t total_count = 0;
  size_t robot_count = 0;
  size_t ball_count = 0;
  size_t rules_count = 0;
  size_t static_count = 0;
  LOG(INFO) << "================";
  size_t i = 0;
  for (const Obstacle* obstacle : all) {
    total_count++;
    switch (obstacle->GetType()) {
      case obstacle::ROBOT: {
        robot_count++;
      } break;
      case obstacle::RULE: {
        rules_count++;
      } break;
      case obstacle::BALL: {
        ball_count++;
      } break;
      case obstacle::STATIC: {
        static_count++;
      } break;
      case obstacle::NUM_OBSTACLE_TYPES: {
        // Noop();
      } break;
    }
    ++i;
  }
  LOG(INFO) << "================";
  EXPECT_EQ(obstacles.size(), static_cast<size_t>(kNumObstacles));
  EXPECT_EQ(total_count,
            static_cast<size_t>(kNumObstacles - 7 - kNumRulesObstacles) + 2 -
                (kNumBallObstacles - 1));
  EXPECT_EQ(robot_count, static_cast<size_t>(kMaxTeamRobots * kNumTeams - 8));
  EXPECT_EQ(ball_count, static_cast<size_t>(1));
  EXPECT_EQ(static_count, static_cast<size_t>(kNumStaticObstacles));
}

// TEST(ScaffoldPrimitiveTests, RayCastTests) {
//   {
//     // Outside Test
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     Vector2f point(-600, 0);
//     Vector2f direction(1, 0);
//     int number_of_intersections;
//     int first_intersect_index;
//     int last_intersect_index;
//     scaffold.CastRay(point, direction, &number_of_intersections,
//                      &first_intersect_index, &last_intersect_index);
//
//     ASSERT_EQ(number_of_intersections, 2);
//   }
//   {
//     // Inside Test
//     // Outside Test
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     Vector2f point(-100, 0);
//     Vector2f direction(1, 0);
//     int number_of_intersections;
//     int first_intersect_index;
//     int last_intersect_index;
//     scaffold.CastRay(point, direction, &number_of_intersections,
//                      &first_intersect_index, &last_intersect_index);
//
//     ASSERT_EQ(number_of_intersections, 1);
//   }
//   {
//     // Outside Test
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 3, 16, 100);
//
//     Vector2f point(-600, 0);
//     Vector2f direction(1, 0);
//     int number_of_intersections;
//     int first_intersect_index;
//     int last_intersect_index;
//     scaffold.CastRay(point, direction, &number_of_intersections,
//                      &first_intersect_index, &last_intersect_index);
//
//     // ASSERT_EQ(number_of_intersections, 2);
//   }
//   {
//     // Inside Test
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 3, 16, 100);
//
//     Vector2f point(-100, 0);
//     Vector2f direction(1, 0);
//     int number_of_intersections;
//     int first_intersect_index;
//     int last_intersect_index;
//     scaffold.CastRay(point, direction, &number_of_intersections,
//                      &first_intersect_index, &last_intersect_index);
//
//     // ASSERT_EQ(number_of_intersections, 1);
//   }
// }
//
// TEST(ScaffoldPrimitiveTests, ScaffoldConnectTests) {
//   {
//     Vector2f vertex1(-600, 50);
//     Vector2f vertex2(600, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::OUTSIDE;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::OUTSIDE;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 4);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//   {
//     Vector2f vertex1(600, 200);
//     Vector2f vertex2(600, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::OUTSIDE;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::OUTSIDE;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_TRUE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 0);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//   {
//     Vector2f vertex1(-600, 50);
//     Vector2f vertex2(0, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::OUTSIDE;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::INSIDE;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 2);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::INSIDE);
//   }
//   {
//     Vector2f vertex1(-600, 50);
//     Vector2f vertex2(600, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::OUTSIDE;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::NC;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 4);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//   {
//     Vector2f vertex1(600, 50);
//     Vector2f vertex2(600, 200);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::OUTSIDE;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::NC;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_TRUE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 0);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//   {
//     Vector2f vertex1(-600, 50);
//     Vector2f vertex2(0, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::OUTSIDE;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::NC;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 2);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::INSIDE);
//   }
//   {
//     Vector2f vertex1(0, 50);
//     Vector2f vertex2(600, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::INSIDE;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::OUTSIDE;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 2);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::INSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//   {
//     Vector2f vertex1(0, 50);
//     Vector2f vertex2(0, -50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::INSIDE;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::INSIDE;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 0);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::INSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::INSIDE);
//   }
//   {
//     Vector2f vertex1(0, 50);
//     Vector2f vertex2(600, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::INSIDE;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::NC;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 2);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::INSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//   {
//     Vector2f vertex1(0, 50);
//     Vector2f vertex2(0, 25);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::INSIDE;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::NC;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 0);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::INSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::INSIDE);
//   }
//   {
//     Vector2f vertex1(-600, 50);
//     Vector2f vertex2(600, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::NC;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::OUTSIDE;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 4);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//   {
//     Vector2f vertex1(600, 50);
//     Vector2f vertex2(600, 250);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::NC;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::OUTSIDE;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_TRUE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 0);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//   {
//     Vector2f vertex1(-600, 50);
//     Vector2f vertex2(0, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::NC;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::INSIDE;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 2);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::INSIDE);
//   }
//   {
//     Vector2f vertex1(0, 50);
//     Vector2f vertex2(600, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::NC;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::OUTSIDE;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 2);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::INSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//   {
//     Vector2f vertex1(0, 50);
//     Vector2f vertex2(10, 25);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::NC;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::INSIDE;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 0);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::INSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::INSIDE);
//   }
//   {
//     Vector2f vertex1(0, 50);
//     Vector2f vertex2(600, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::NC;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::NC;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 2);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::INSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//
//   {
//     Vector2f vertex1(0, 50);
//     Vector2f vertex2(10, 25);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::NC;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::NC;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 0);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::INSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::INSIDE);
//   }
//
//   {
//     Vector2f vertex1(-600, 50);
//     Vector2f vertex2(600, 50);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::NC;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::NC;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_FALSE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 4);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
//   {
//     Vector2f vertex1(600, 50);
//     Vector2f vertex2(600, 250);
//
//     ScaffoldIndicator indicator_vertex1 = ScaffoldIndicator::NC;
//     ScaffoldIndicator indicator_vertex2 = ScaffoldIndicator::NC;
//
//     Pose2Df robot_pose2(0, Vector2f(0, 0));
//     CircleObstacle circle(robot_pose2, ObstacleType::ROBOT, 90);
//     CircleScaffold scaffold(&circle, 4, 16, 100);
//
//     vector<Edge> edges_to_add;
//
//     bool is_safe =
//         scaffold.ScaffoldConnect(vertex1, 1, vertex2, 2, &indicator_vertex1,
//                                  &indicator_vertex2, &edges_to_add);
//
//     ASSERT_TRUE(is_safe);
//     ASSERT_EQ(static_cast<int>(edges_to_add.size()), 0);
//     ASSERT_EQ(indicator_vertex1, ScaffoldIndicator::OUTSIDE);
//     ASSERT_EQ(indicator_vertex2, ScaffoldIndicator::OUTSIDE);
//   }
// }
}  // namespace obstacle

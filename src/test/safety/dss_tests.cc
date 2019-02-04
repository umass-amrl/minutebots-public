// Copyright 2017 - 2018 kvedder@umass.edu
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

#include "safety/dss.h"
#include "constants/constants.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using team::Team;
using safety::DSS;
using state::SoccerState;
using state::WorldState;
using state::WorldRobot;
using state::SharedRobotState;
using state::PositionVelocityState;

namespace safety {

TEST(DSSPrimitiveTests, HelloWorld) { ASSERT_TRUE(true); }

// Check to see that all of the generated new velocities are less than or equal
// to the max accel in magnitude.
TEST(DSSPrimitiveTests, RandomVectorGeneratorTest) {
  constexpr float kEpsilon = 0.001;
  const float control_period = 0.1;

  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);
  // Junk data just to get something started.
  DSS dss(world_state, nullptr);

  vector<Vector2f> new_velocities;
  Vector2f initial_velocity(100, 100);
  dss.GenerateNewVelocities(initial_velocity, &new_velocities, 100,
                            control_period,
                            {kMaxRobotAcceleration, kMaxRobotVelocity});
  for (const Vector2f& new_velocity : new_velocities) {
    // Check acceleration cap.
    EXPECT_LE((initial_velocity - new_velocity).norm(),
              kMaxRobotAcceleration * control_period + kEpsilon);
    // Check velocity cap.
    EXPECT_LE(new_velocity.norm(), kMaxRobotVelocity + kEpsilon);
  }
}

TEST(DSSPrimitiveTests, CostTest) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);
  // Junk data just to get something started.
  DSS dss(world_state, nullptr);
  Vector2f first_vector(1, 0);
  Vector2f second_vector(0, 0);
  Vector2f third_vector(0.5, 0);
  ASSERT_EQ(dss.Cost(first_vector, first_vector), 0);
  ASSERT_EQ(dss.Cost(first_vector, second_vector), 1);
  ASSERT_EQ(dss.Cost(third_vector, second_vector), 0.25);
}

TEST(DSSPrimitiveTests, PillBoxTest) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);
  DSS dss(world_state, nullptr);

  logger::Logger logger;

  LOG(INFO) << "Testing non-intersecting!\n";
  // Tests non-intersecting pillboxes.
  Vector2f first_start(1, 0);
  Vector2f first_velocity(4, 0);
  Vector2f second_start(1, kRobotRadius * 4);
  Vector2f second_velocity(4, 0);
  EXPECT_FALSE(dss.PillBoxCollide(
      &logger, first_start, first_velocity, first_velocity,
      {kMaxRobotAcceleration, kMaxRobotVelocity}, second_start, second_velocity,
      second_velocity, {kMaxRobotAcceleration, kMaxRobotVelocity}, kRobotRadius,
      0.001, 0, kDSSSafetyMargin));

  LOG(INFO) << "Testing end circles!\n";
  // Tests intersection of end circles
  first_start = Vector2f(0, 0);
  first_velocity = Vector2f(0, 1);
  second_start = Vector2f(1, 0);
  second_velocity = Vector2f(-1, 1);
  EXPECT_TRUE(dss.PillBoxCollide(
      &logger, first_start, first_velocity, first_velocity,
      {kMaxRobotAcceleration, kMaxRobotVelocity}, second_start, second_velocity,
      second_velocity, {kMaxRobotAcceleration, kMaxRobotVelocity}, 0.001, 1, 0,
      kDSSSafetyMargin));
}

TEST(DSSPrimitiveTests, PillBoxTestDecelCollide) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);
  SoccerState soccer_state(world_state);
  DSS dss(world_state, &soccer_state);

  logger::Logger logger;

  LOG(INFO) << "Testing non-intersecting!\n";
  // Tests non-intersecting pillboxes.
  Vector2f first_start(0, 0);
  Vector2f first_velocity(0, 0);
  Vector2f second_start(250, 0);
  Vector2f second_velocity(-1000, 0);
  ASSERT_TRUE(dss.PillBoxCollide(
      &logger, first_start, first_velocity, first_velocity,
      {kMaxRobotAcceleration, kMaxRobotVelocity}, second_start, second_velocity,
      second_velocity, {kMaxRobotAcceleration, kMaxRobotVelocity}, 90,
      1.0 / 60.0, 0, kDSSSafetyMargin));

  // We're 90mm * 2 + 70mm = 0.25m away going 1m/s.
  // We going to go 1m/s * 1/60s = 1/60m or ~ 0.01666m during the control period
  // We then want to stop, which means 0m^2/s^ = 1m^2/s^2 + 2 * -7m/s^2 * dist
  // Therefore 1m^2/s^2 / 14 m/s^2 = dist = 1/14 m or 0.07m.
  // This means we've traveled 0.086m and thus have a 0.17m buffer between
  // yourself and the other robot, which is smaller than 90mm * 2 radius.
  // Thus, the robots should crash.
}

TEST(DSSIntegrationTests, FullLoopTestNoCollision) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);

  PositionVelocityState::RobotPositionVelocity world_robot1(
      0, Pose2Df(0, Vector2f(0, 0)), Pose2Df(0, Vector2f(0, 0)),
      Pose2Df(0, Vector2f(0, 0)), {0, 0, 0}, 0, 1);
  PositionVelocityState::RobotPositionVelocity world_robot2(
      1, Pose2Df(0, Vector2f(kRobotRadius * 3, 0)), Pose2Df(0, Vector2f(0, 1)),
       Pose2Df(0, Vector2f(0, 0)), {0, 0, 0}, 0, 1);

  pvs.GetMutableOurTeamRobots()->InsertBack(world_robot1);
  pvs.GetMutableOurTeamRobots()->InsertBack(world_robot2);

  LOG(INFO) << "Setting up SoccerState\n";

  SoccerState soccer_state(world_state);

  LOG(INFO) << "SoccerState setup!\n";

  LOG(INFO) << "Grabbing shared state...\n";
  SharedRobotState* shared_state_0 =
      soccer_state.GetMutableSharedState()->GetSharedState(0);
  shared_state_0->ssl_vision_id = 0;
  shared_state_0->velocity_x = 0;
  shared_state_0->velocity_y = 1;

  SharedRobotState* shared_state_1 =
      soccer_state.GetMutableSharedState()->GetSharedState(1);
  shared_state_1->ssl_vision_id = 1;
  shared_state_1->velocity_x = 0;
  shared_state_1->velocity_y = 1;

  LOG(INFO) << "Grabbed shared state...\n";

  DSS dss(world_state, &soccer_state);

  LOG(INFO) << "Robot 0 shared state: " << shared_state_0->velocity_x << ", "
            << shared_state_0->velocity_y << "\n";

  LOG(INFO) << "Robot 1 shared state: " << shared_state_1->velocity_x << ", "
            << shared_state_1->velocity_y << "\n";

  dss.MakeSafe({kMaxRobotAcceleration, kMaxRobotVelocity},
               {kMaxRobotAcceleration, kMaxRobotVelocity}, 1.0 / 60.0);

  LOG(INFO) << "Robot 0 shared state: " << shared_state_0->velocity_x << ", "
            << shared_state_0->velocity_y << "\n";

  LOG(INFO) << "Robot 1 shared state: " << shared_state_1->velocity_x << ", "
            << shared_state_1->velocity_y << "\n";

  ASSERT_EQ(shared_state_0->velocity_x, 0);
  ASSERT_EQ(shared_state_1->velocity_x, 0);
  ASSERT_EQ(shared_state_0->velocity_y, 1);
  ASSERT_EQ(shared_state_1->velocity_y, 1);
}

TEST(DSSIntegrationTests, FullLoopTestCollision) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);

  PositionVelocityState::RobotPositionVelocity world_robot1(
      2, Pose2Df(0, Vector2f(0, 0)), Pose2Df(0, Vector2f(0, 0)),
      Pose2Df(0, Vector2f(0, 0)), {0, 0, 0}, 0, 1);
  PositionVelocityState::RobotPositionVelocity world_robot2(
      3, Pose2Df(0, Vector2f(kRobotRadius * 3, 0)), Pose2Df(0, Vector2f(0, 0)),
      Pose2Df(kRobotRadius * 3, Vector2f(0, 0)), {0, 0, 0}, 0, 1);

  pvs.GetMutableOurTeamRobots()->InsertBack(world_robot1);
  pvs.GetMutableOurTeamRobots()->InsertBack(world_robot2);

  SoccerState soccer_state(world_state);

  SharedRobotState* shared_state_0 =
      soccer_state.GetMutableSharedState()->GetSharedState(0);
  shared_state_0->ssl_vision_id = 0;
  shared_state_0->velocity_x = 0;
  shared_state_0->velocity_y = 1;

  SharedRobotState* shared_state_1 =
      soccer_state.GetMutableSharedState()->GetSharedState(1);
  shared_state_1->ssl_vision_id = 1;
  shared_state_1->velocity_x = -3 * kRobotRadius;
  shared_state_1->velocity_y = 0;

  DSS dss(world_state, &soccer_state);

  LOG(INFO) << "Robot 0 shared state: " << shared_state_0->velocity_x << ", "
            << shared_state_0->velocity_y << "\n";

  LOG(INFO) << "Robot 1 shared state: " << shared_state_1->velocity_x << ", "
            << shared_state_1->velocity_y << "\n";

  dss.MakeSafe({kMaxRobotAcceleration, kMaxRobotVelocity},
               {kMaxRobotAcceleration, kMaxRobotVelocity}, 1.0);

  LOG(INFO) << "Robot 0 shared state: " << shared_state_0->velocity_x << ", "
            << shared_state_0->velocity_y << "\n";

  LOG(INFO) << "Robot 1 shared state: " << shared_state_1->velocity_x << ", "
            << shared_state_1->velocity_y << "\n";
  // Check to see if safe moves failed, reverted to 0, 0
  if (shared_state_0->velocity_x == 0 && shared_state_0->velocity_y == 0) {
    // Second robot needs to have their move changed.
    ASSERT_TRUE(shared_state_1->velocity_x != -3 * kRobotRadius ||
                shared_state_1->velocity_y != 0);
  } else {
    // Second robot should not have their move changed.
    ASSERT_EQ(shared_state_1->velocity_x, -3 * kRobotRadius);
    ASSERT_EQ(shared_state_1->velocity_y, 0);
  }
}

// Tests to see if robot 1 will avoid the static robot 0.
TEST(DSSIntegrationTests, FullLoopTestStaticObstacle) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);

  PositionVelocityState::RobotPositionVelocity world_robot1(
      0, Pose2Df(0, Vector2f(0, 0)), Pose2Df(0, Vector2f(0, 0)),
      Pose2Df(0, Vector2f(0, 0)), {0, 0, 0}, 0, 1);
  PositionVelocityState::RobotPositionVelocity world_robot2(
      1, Pose2Df(0, Vector2f(kRobotRadius * 3, 0)), Pose2Df(0, Vector2f(0, 0)),
      Pose2Df(kRobotRadius * 3, Vector2f(0, 0)), {0, 0, 0}, 0, 1);

  pvs.GetMutableOurTeamRobots()->InsertBack(world_robot1);
  pvs.GetMutableOurTeamRobots()->InsertBack(world_robot2);

  SoccerState soccer_state(world_state);

  SharedRobotState* shared_state_0 =
      soccer_state.GetMutableSharedState()->GetSharedState(0);
  shared_state_0->ssl_vision_id = 0;
  shared_state_0->velocity_x = 0;
  shared_state_0->velocity_y = 0;

  SharedRobotState* shared_state_1 =
      soccer_state.GetMutableSharedState()->GetSharedState(1);
  shared_state_1->ssl_vision_id = 1;
  shared_state_1->velocity_x = -1.5 * kRobotRadius;
  shared_state_1->velocity_y = 0;

  DSS dss(world_state, &soccer_state);

  LOG(INFO) << "Robot 0 shared state: " << shared_state_0->velocity_x << ", "
            << shared_state_0->velocity_y << "\n";

  LOG(INFO) << "Robot 1 shared state: " << shared_state_1->velocity_x << ", "
            << shared_state_1->velocity_y << "\n";

  dss.MakeSafe({kMaxRobotAcceleration, kMaxRobotVelocity},
               {kMaxRobotAcceleration, kMaxRobotVelocity}, 1.0);

  LOG(INFO) << "Robot 0 shared state: " << shared_state_0->velocity_x << ", "
            << shared_state_0->velocity_y << "\n";

  LOG(INFO) << "Robot 1 shared state: " << shared_state_1->velocity_x << ", "
            << shared_state_1->velocity_y << "\n";
  // Check to see if safe moves failed, reverted to 0, 0
  ASSERT_TRUE(shared_state_1->velocity_x != -3 * kRobotRadius ||
              shared_state_1->velocity_y != 0);
}

// Tests to see if robot 1 will not try to swerve from a static obstacle far
// away
TEST(DSSIntegrationTests, FullLoopTestStaticObstacleFasterNoCrash) {
  PositionVelocityState pvs;
  WorldState world_state(&pvs, Team::BLUE);

  PositionVelocityState::RobotPositionVelocity world_robot1(
      0, Pose2Df(0, Vector2f(0, 0)), Pose2Df(0, Vector2f(0, 0)),
      Pose2Df(0, Vector2f(0, 0)), {0, 0, 0}, 0, 1);
  PositionVelocityState::RobotPositionVelocity world_robot2(
      1, Pose2Df(0, Vector2f(kRobotRadius * 5, 0)), Pose2Df(0, Vector2f(0, 0)),
      Pose2Df(kRobotRadius * 5, Vector2f(0, 0)), {0, 0, 0}, 0, 1);

  pvs.GetMutableOurTeamRobots()->InsertBack(world_robot1);
  pvs.GetMutableOurTeamRobots()->InsertBack(world_robot2);

  SoccerState soccer_state(world_state);

  SharedRobotState* shared_state_0 =
      soccer_state.GetMutableSharedState()->GetSharedState(0);
  shared_state_0->ssl_vision_id = 0;
  shared_state_0->velocity_x = 0;
  shared_state_0->velocity_y = 0;

  SharedRobotState* shared_state_1 =
      soccer_state.GetMutableSharedState()->GetSharedState(1);
  shared_state_1->ssl_vision_id = 1;
  // 1 m/s.
  shared_state_1->velocity_x = -1000;
  shared_state_1->velocity_y = 0;

  DSS dss(world_state, &soccer_state);

  LOG(INFO) << "Robot 0 shared state: " << shared_state_0->velocity_x << ", "
            << shared_state_0->velocity_y << "\n";

  LOG(INFO) << "Robot 1 shared state: " << shared_state_1->velocity_x << ", "
            << shared_state_1->velocity_y << "\n";

  dss.MakeSafe({kMaxRobotAcceleration, kMaxRobotVelocity},
               {kMaxRobotAcceleration, kMaxRobotVelocity}, 1.0 / 60.0);

  // We're 90mm * 5 = 0.45m away going 1m/s.
  // We going to go 1m.s * 1/60s = 1/60m or ~ 0.01666m during the control period
  // We then want to stop, which means 0m^2/s^ = 1m^2/s^2 + 2 * -7m/s^2 * dist
  // 1m^2/s^2 / 14 m/s^2 = 1/14 m or 0.07m.
  // This means we've traveled 0.086m and thus have a 0.37m buffer between
  // yourself and the other robot.
  // Thus, the robots should not crash.

  LOG(INFO) << "Robot 0 shared state: " << shared_state_0->velocity_x << ", "
            << shared_state_0->velocity_y << "\n";

  LOG(INFO) << "Robot 1 shared state: " << shared_state_1->velocity_x << ", "
            << shared_state_1->velocity_y << "\n";
  // Check to see that the robots do not change course.
  ASSERT_EQ(shared_state_0->velocity_x, 0);
  ASSERT_EQ(shared_state_1->velocity_x, -1000);
  ASSERT_EQ(shared_state_0->velocity_y, 0);
  ASSERT_EQ(shared_state_1->velocity_y, 0);
}

}  // namespace safety

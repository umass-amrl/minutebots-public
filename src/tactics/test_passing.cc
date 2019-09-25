// Copyright 2018 - 2019 jaholtz@cs.umass.edu
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

#include "tactics/test_passing.h"
#include <algorithm>
#include <cmath>
#include <iomanip>  // std::setprecision
#include <memory>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "obstacles/obstacle_flag.h"
#include "state/shared_state.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/ball_interception.h"
#include "tactics/kick.h"
#include "tactics/primary_attacker.h"
#include "tactics/eight_grid_navigation.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using geometry::Angle;
using geometry::RayIntersect;
using state::WorldState;
using state::WorldRobot;
using state::SoccerRobot;
using state::SharedState;
using state::SharedRobotState;
using std::cos;
using std::sin;
using std::endl;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestPassTarget;
using offense::GetBestChipTarget;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;

namespace tactics {

  TestPassing::TestPassing(const WorldState& world_state,
    TacticArray* tactic_list, SharedState* shared_state,
    OurRobotIndex our_robot_index, state::SoccerState* soccer_state)
    : StateMachineTactic("TestPassing", world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      aim_(std::bind(&TestPassing::Aim, this), "Aim"),
      kick_(std::bind(&TestPassing::Kick, this), "Kick"),
      finish_(std::bind(&TestPassing::Finish, this),
             "Finish"),
      thresholds_kick_timeout_(40, 0.0, 100.0, "kick_timeout", this),
      complete_(false),
      kick_count_(0),
      aim_count_(0) {
  state_ = aim_;
}

void TestPassing::Init() {
}

void TestPassing::Reset() {
  complete_ = false;
  state_ = aim_;
  kick_count_ = 0;
  aim_count_ = 0;
}

void TestPassing::SetGoal(const Pose2Df& pose) {}

const Pose2Df TestPassing::GetGoal() { return robot_goal_pose_; }

bool TestPassing::IsComplete() { return complete_; }

float TestPassing::GetCost() {
  return 0;
}

void TestPassing::GetTarget(float* target_angle) {
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;
  OurRobotIndex receiving_robot = 42;
  vector<Vector2f> world_robot_positions;
  for (const auto& robot : world_state_.GetOurRobots()) {
    if (world_state_.GetOurRobotIndex(robot.ssl_vision_id) !=
        static_cast<int>(our_robot_index_)) {
      world_robot_positions.push_back(robot.position.translation);
    }
  }
  for (const auto& robot : world_state_.GetTheirRobots()) {
    world_robot_positions.push_back(robot.position.translation);
  }
  complete_ = false;
  bool should_pass = true;
  // If no good aim option for shooting on the goal, pass the ball.
  if (should_pass) {
    zone::FieldZone field_zone(zone::FULL_FIELD);
    *target_angle = GetBestPassTarget(
        world_state_,
        soccer_state_,
        our_robot_index_,
        world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id,
        field_zone,
        &receiving_robot);
    if (receiving_robot != 42) {
      shared_state_->SetPass(receiving_robot,
          world_state_.GetOurRobotPosition
              (receiving_robot).position.translation);
    }
  }
}

void TestPassing::Aim() {
  Pose2Df current_pose =
  world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  GetTarget(&target_angle_);

  // Set the movement goal as a position behind the ball with respect
  // to the target, facing the ball.
  robot_goal_pose_.translation.x() =
      ball_pose.x() - (kRotationRadius)*cos(target_angle_);
  robot_goal_pose_.translation.y() =
      ball_pose.y() - (kRotationRadius)*sin(target_angle_);
  robot_goal_pose_.angle = target_angle_;

  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  // Run the controller with the calculated goal and margins.
  Pose2Df target_pose;
  target_pose.translation = robot_goal_pose_.translation;
  target_pose.angle = target_angle_;
  controller->SetLocationThreshold(5);
  controller->SetGoal(target_pose);
  controller->Run();
}

void TestPassing::Kick() {
  GetTarget(&target_angle_);
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  kick_count_++;
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  shared_state_->SetPassShot();
  state->flat_kick_set = true;
  state->flat_kick = 2.0;
  state->velocity_x = current_velocity.translation.x() +
                      (1 * kMaxRobotAcceleration) * kTransmitPeriodSeconds;
}

// Set complete = true
// Do nothing
void TestPassing::Finish() {
  complete_ = true;
  Tactic* controller = (*tactic_list_)[TacticIndex::HALT].get();
  controller->Run();
}

void TestPassing::Transition() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

  const Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  const Vector2f ball_pose = world_state_.GetBallPosition().position;

  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;

  if (state_ == aim_) {
    aim_count_++;
    // SET POTENTIAL TRANSITION
    potential_state_ = "WAIT";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;

    PrimaryAttacker* attacker = static_cast<PrimaryAttacker*>(
      (*tactic_list_)[TacticIndex::PRIMARY_ATTACKER].get());

    const bool should_kick =
        attacker->ShouldKick(robot_logger,
                             target_angle_,
                             ball_pose,
                             ball_vel,
                             current_pose,
                             current_velocity,
                             false,
                             false,
                             true);
    SetTransition(should_kick);
    if (should_kick && aim_count_ > 40) {
      state_ = kick_;
    }
  } else if (state_ == kick_) {  // wait until ball kicked
    // SET POTENTIAL TRANSITION
    potential_state_ = "POSTKICK";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;

    const bool timed_out = kick_count_ > thresholds_kick_timeout_;
    SetTransition(timed_out);
    const Vector2f kick_direction = geometry::Heading(target_angle_);
    robot_logger->LogPrint("Ball vel: %f", ball_vel.dot(kick_direction));
    const bool kicked = ball_vel.dot(kick_direction) > 500;
    if (timed_out || kicked) {
      state_ = finish_;
    }
  } else if (state_ == finish_) {
    Reset();
  }
}

}  // namespace tactics

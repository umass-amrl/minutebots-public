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

#include "tactics/receiver.h"
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
#include "tactics/catch.h"
#include "tactics/deflection.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/kick.h"
#include "tactics/ntoc_controller.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using geometry::Angle;
using geometry::RayIntersect;
using geometry::ProjectPointOntoLine;
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
using tactics::EightGridNavigation;
using tactics::NTOC_Controller;
using tactics::Deflection;
using tactics::Catch;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestPassTarget;
using offense::GetBestChipTarget;
using offense::GetTargetEvaluated;
using offense::GetKickAngle;
using offense::GetFinalBallSpeed;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;

namespace tactics {

Receiver::Receiver(const WorldState& world_state, TacticArray* tactic_list,
                   SharedState* shared_state, OurRobotIndex our_robot_index,
                   state::SoccerState* soccer_state)
    : StateMachineTactic("TestReceiver", world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      setup_(std::bind(&Receiver::Setup, this), "Setup"),
      wait_(std::bind(&Receiver::Wait, this), "Wait"),
      catch_(std::bind(&Receiver::Catch, this), "Catch"),
      deflect_(std::bind(&Receiver::Deflect, this), "Deflect"),
      pass_failed_(std::bind(&Receiver::Finish, this), "PassFailed"),
      finish_(std::bind(&Receiver::Finish, this), "Finish"),
      thresholds_receive_velocity_(800, 0.0, 5000.0, "catch_velocity", this),
      thresholds_toward_target_(80, 0.0, 100.0, "toward_target", this),
      thresholds_toward_robot_(95, 0.0, 100.0, "toward_robot", this),
      thresholds_deflection_angle_(offense::kDeflectionAngle, 0.0, 3.14,
                                   "deflection_angle", this),
      thresholds_min_deflection_angle_(DegToRad(10.0), 0.0, 3.14,
                                       "min_deflection_angle",
                                       this),
      thresholds_setup_time_(999999, 0.0, 999999, "setup_time", this),
      thresholds_setup_distance_(1000.0, 0.0, kFieldLength,
                                 "setup_distance", this),
      complete_(false),
      last_target_(offense::kNoPassRobot),
      wait_count_(0),
      fail_count_(0) {
  state_ = setup_;
}

void Receiver::Init() {}

void Receiver::Reset() {
  wait_count_ = 0;
  fail_count_ = 0;
  complete_ = false;
  state_ = setup_;
  // Reset the other controllers being used by this tactic
  // when this tactic resets.
  Tactic* controller = (*tactic_list_)[TacticIndex::DEFLECTION].get();
  controller->Reset();
  controller = (*tactic_list_)[TacticIndex::CATCH].get();
  controller->Reset();
}

void Receiver::SetGoal(const Pose2Df& pose) { robot_goal_pose = pose; }

const Pose2Df Receiver::GetGoal() {
  if (state_ == wait_ || state_ == setup_) {
    return robot_goal_pose;
  } else if (state_ == catch_) {
    Tactic* controller = (*tactic_list_)[TacticIndex::CATCH].get();
    return controller->GetGoal();
  } else if (state_ == deflect_) {
    Tactic* controller = (*tactic_list_)[TacticIndex::DEFLECTION].get();
    return controller->GetGoal();
  }
  return robot_goal_pose;
}

bool Receiver::BadTiming() {
  const Vector2f robot_position =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  const Vector2f robot_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  const Vector2f ball_dir = ball_pose + ball_vel;
  Vector2f projected_point;
  ProjectPointOntoLine(robot_position, ball_pose, ball_dir, &projected_point);
  // Time for ball to travel to receive position
  const float ball_time =
      offense::GetBallTravelTime(ball_vel.norm(), ball_pose, projected_point);

  float robot_time = offense::GetNtocTime({0, robot_position}, robot_velocity,
                                          projected_point);
  const Vector2f ball_to_robot = robot_position - ball_pose;
  // So we don't transition out of catch
  if (ball_vel.dot(ball_to_robot) / ball_vel.norm() > .98) {
    return false;
  }
  Deflection* controller =
      static_cast<Deflection*>((*tactic_list_)[TacticIndex::DEFLECTION].get());
  if (!controller->SettingUp()) {
    return false;
  }
  return ball_time - robot_time < -.4;
}

bool Receiver::IsComplete() { return complete_; }

float Receiver::GetCost() { return 0; }

// Move to the setup position
void Receiver::Setup() {
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  // Move to the wait positions
  float target_angle;
  OurRobotIndex target_robot;
  Vector2f target_position;
  GetTargetEvaluated(robot_goal_pose.translation, world_state_, soccer_state_,
                     our_robot_index_, false, &target_position, &target_angle,
                     &target_robot, &last_target_);
  // Set the pass speed as either a pass or a kick
  float kick_speed = offense::kKickSpeed;
  if (target_robot != offense::kNoPassRobot) {
    kick_speed = offense::kPassSpeed;
  }
  // Get the speed of the ball at intercept position
  const float final_ball_speed = GetFinalBallSpeed(ball_vel.norm(), ball_pose,
                                                   robot_goal_pose.translation);
  const Vector2f final_ball_vel = final_ball_speed * ball_vel.normalized();
  // Adjust the kick angle for the ball velocity.
  target_angle = GetKickAngle(final_ball_vel, robot_goal_pose.translation,
                              kick_speed, robot_logger, Heading(target_angle));
  const Vector2f target_vector = geometry::Heading(target_angle);
  // Determine if we should set up for a deflection, or a catch.
  float deflection_angle = 0;
  if (final_ball_vel.norm() > 0) {
    deflection_angle =
        acos(target_vector.dot(-final_ball_vel) / (final_ball_vel.norm()));
  } else {
    deflection_angle = acos(target_vector.dot(-final_ball_vel));
  }
  deflection_angle = AngleMod(deflection_angle);
  deflection_angle = fabs(deflection_angle);
  // Deflect or Catch?
  if (deflection_angle > offense::kDeflectionAngle) {
    const Vector2f neg_ball_vel = -ball_vel;
    robot_goal_pose.angle = Angle(neg_ball_vel);
  } else {
    robot_goal_pose.angle = target_angle;
  }
  robot_logger->LogPrint("Deflection Angle: %f", RadToDeg(deflection_angle));
  Tactic* controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  controller->SetGoal(robot_goal_pose);
  controller->Run();
}

void Receiver::Wait() {
  // TODO(jaholtz) always rotating to target angle, is that correct?
  const Vector2f target_translation = shared_state_->GetPassLocation();
  Pose2Df target_pose;
  float target_angle;
  OurRobotIndex target_robot;
  Vector2f target_position;
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  GetTargetEvaluated(ball_pose, world_state_, soccer_state_, our_robot_index_,
                     false, &target_position, &target_angle, &target_robot,
                     &last_target_);
  float kick_speed = offense::kKickSpeed;
  if (target_robot != offense::kNoPassRobot) {
    kick_speed = offense::kPassSpeed;
  }
  const Vector2f robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  target_angle = GetKickAngle(ball_vel, robot_pose, kick_speed, robot_logger,
                              Heading(target_angle));
  target_pose.translation = target_translation;
  target_pose.angle = target_angle;
  NTOC_Controller* ntoc =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  ntoc->TurnOnAngleRelaxation();

  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  // Run the controller with the calculated goal and margins.
  controller->SetObstacles(obstacle::ObstacleFlag::GetAllExceptTeam(
      world_state_, *soccer_state_, our_robot_index_, our_robot_index_));
  controller->SetGoal(target_pose);
  controller->Run();
}

// Run the catch controller
void Receiver::Catch() {
  Tactic* controller = (*tactic_list_)[TacticIndex::CATCH].get();
  controller->Run();
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->dribbler_set = true;
  state->dribbler_spin = -1;
}

// Run the deflection controller
void Receiver::Deflect() {
  Tactic* controller = (*tactic_list_)[TacticIndex::DEFLECTION].get();
  controller->Run();
}

// Set complete = true, clear the pass target
// Do nothing
void Receiver::Finish() {
  complete_ = true;
  shared_state_->ClearPass();
  Tactic* controller = (*tactic_list_)[TacticIndex::HALT].get();
  controller->Run();
}

bool Receiver::PassGood() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Vector2f current_ball_pose = world_state_.GetBallPosition().position;
  const Vector2f current_ball_velocity =
      world_state_.GetBallPosition().velocity;
  Vector2f ball_to_target_displace =
      shared_state_->GetPassLocation() - current_ball_pose;
  Vector2f ball_to_robot_displace =
      current_pose.translation - current_ball_pose;
  ball_to_robot_displace.normalize();
  ball_to_target_displace.normalize();

  const float relative_ball_vel =
      ball_to_robot_displace.dot(current_ball_velocity);
  const float relative_percent =
      (relative_ball_vel / current_ball_velocity.norm()) * 100;
  const float relative_target_vel =
      ball_to_target_displace.dot(current_ball_velocity);
  const float relative_target_percent =
      (relative_target_vel / current_ball_velocity.norm()) * 100;

  // SET POTENTIAL TRANSITION
  potential_state_ = "RECEIVE";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;

  const bool speed_check =
      fabs(current_ball_velocity.norm()) > thresholds_receive_velocity_;

  AddBlock(true);
  const bool towards_robot = relative_percent > thresholds_toward_target_;
  and_clause_ = false;
  const bool towards_target =
      relative_target_percent > thresholds_toward_target_;

  const bool should_receive = speed_check && (towards_robot || towards_target);
  SetTransition(should_receive);
  return should_receive;
}

bool Receiver::ShouldReceive() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Vector2f current_ball_pose = world_state_.GetBallPosition().position;
  const Vector2f current_ball_velocity =
      world_state_.GetBallPosition().velocity;
  Vector2f ball_to_robot_displace =
      current_pose.translation - current_ball_pose;
  ball_to_robot_displace.normalize();

  const float relative_ball_vel =
      ball_to_robot_displace.dot(current_ball_velocity);
  const float relative_percent =
      (relative_ball_vel / current_ball_velocity.norm()) * 100;

  // SET POTENTIAL TRANSITION
  potential_state_ = "RECEIVE";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;

  const bool speed_check =
      fabs(current_ball_velocity.norm()) > thresholds_receive_velocity_;

  const bool towards_robot = relative_percent > thresholds_toward_robot_;

  const bool should_receive = speed_check && towards_robot;
  SetTransition(should_receive);
  return should_receive;
}

bool Receiver::IsSetup() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;

  // Locations
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  Vector2f ball_to_target_displace =
      shared_state_->GetPassLocation() - ball_pose;
  Vector2f bot_to_target_displace =
      shared_state_->GetPassLocation() - current_pose.translation;
  Vector2f ball_to_robot_displace = current_pose.translation - ball_pose;
  ball_to_robot_displace.normalize();
  ball_to_target_displace.normalize();

  // Relative Velocities
  const float relative_ball_vel = ball_to_robot_displace.dot(ball_vel);
  const float relative_percent = (relative_ball_vel / ball_vel.norm()) * 100;
  const float relative_target_vel = ball_to_target_displace.dot(ball_vel);
  const float relative_target_percent =
      (relative_target_vel / ball_vel.norm()) * 100;

  // Setup complete conditions
  // SET POTENTIAL TRANSITION
  potential_state_ = "WAIT";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  const float percent_diff = relative_target_percent - relative_percent;
  const bool towards_robot = percent_diff < thresholds_toward_robot_;
  const bool within_setup =
      bot_to_target_displace.norm() < thresholds_setup_distance_;
  if (towards_robot && within_setup) {
    wait_count_++;
  }
  const bool time_setup = wait_count_ > thresholds_setup_time_;
  // If sufficiently close to target, and ball is coming towards us
  const bool setup_complete = within_setup && time_setup;
  SetTransition(setup_complete);
  return setup_complete;
}

bool Receiver::ShouldDeflect() {
  // Get world state values
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;

  // Calculate target angle
  float target_angle;
  OurRobotIndex target_robot;
  Vector2f target_position;
  GetTargetEvaluated(current_pose.translation, world_state_, soccer_state_,
                     our_robot_index_, false, &target_position, &target_angle,
                     &target_robot, &last_target_);
  // Set the pass speed as either a pass or a kick
  float kick_speed = offense::kKickSpeed;
  if (target_robot != offense::kNoPassRobot) {
    kick_speed = offense::kPassSpeed;
  }
  // Get the speed of the ball at intercept position
  const float final_ball_speed =
      GetFinalBallSpeed(ball_vel.norm(), ball_pose, current_pose.translation);
  const Vector2f final_ball_vel = final_ball_speed * ball_vel.normalized();
  // Adjust the kick angle for the ball velocity.
  target_angle = GetKickAngle(final_ball_vel, current_pose.translation,
                              kick_speed, robot_logger, Heading(target_angle));
  const Vector2f target_vector = geometry::Heading(target_angle);
  // Determine if we should set up for a deflection, or a catch.
  float deflection_angle = 0;
  if (final_ball_vel.norm() > 0) {
    deflection_angle =
        acos(target_vector.dot(-final_ball_vel) / (final_ball_vel.norm()));
  } else {
    deflection_angle = acos(target_vector.dot(-final_ball_vel));
  }
  deflection_angle = AngleMod(deflection_angle);
  deflection_angle = fabs(deflection_angle);
  // Deflect or Catch?
  robot_logger->LogPrint("Should Deflect Deflection Angle: %f",
                         RadToDeg(deflection_angle));
  // SET POTENTIAL TRANSITION
  potential_state_ = "DEFLECTION";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  bool should_deflect = deflection_angle < thresholds_deflection_angle_ &&
                        deflection_angle > thresholds_min_deflection_angle_;

  Deflection* controller =
      static_cast<Deflection*>((*tactic_list_)[TacticIndex::DEFLECTION].get());
  if (controller->BadTiming()) {
    should_deflect = false;
  }
  SetTransition(should_deflect);

  return should_deflect;
}

// TODO(jaholtz) Needs RepairableParam upgrade
void Receiver::Transition() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  if (kDebug) {
    robot_logger->LogPrint("Goal: %f,%f,%f", robot_goal_pose.translation.x(),
                           robot_goal_pose.translation.y(),
                           robot_goal_pose.angle);
    robot_logger->LogPrint("Position: %f,%f,%f", current_pose.translation.x(),
                           current_pose.translation.y(), current_pose.angle);
  }
  // Setup moves to the pass target location
  // Only transitions to wait if the pass is more towards the robot than the
  // pass location
  if (state_ == setup_) {
    state_ = wait_;
  }
  if (state_ == wait_ || state_ == catch_) {
    if (ShouldDeflect()) {
      state_ = deflect_;
    } else {
      state_ = catch_;
    }
  } else if (state_ == deflect_) {
    Deflection* controller = static_cast<Deflection*>(
        (*tactic_list_)[TacticIndex::DEFLECTION].get());
    //     // Force to catch if the deflection is impossible.
    if (controller->BadDeflection()) {
      robot_logger->LogPrint("BadDeflection");
      state_ = catch_;
    }
    if (controller->IsComplete()) {
      state_ = finish_;
    }
  }

  if (state_ == catch_) {
    // TODO(jaholtz) catch tactic needs to
    // handle completion conditions properly.
    if (!PassGood()) {
      state_ = finish_;
    }
  } else if (state_ == finish_) {
    Reset();
  }
}

}  // namespace tactics

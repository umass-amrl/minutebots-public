// Copyright 2017 - 2019 srabiee@cs.umass.edu, jaholtz@cs.umass.edu
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

#include <algorithm>
#include <cmath>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/defense_evaluation.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "safety/dss_helpers.h"
#include "state/shared_state.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/coercive_attacker.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/ntoc_controller.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using safety::CalculateTimeToRest;
using safety::CalculateDeltaPosition;
using state::WorldState;
using state::WorldRobot;
using state::SharedState;
using std::unique_ptr;
using tactics::TacticIndex;
using obstacle::ObstacleType;
using obstacle::ObstacleFlag;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::PassEvaluator;
using offense::PassEvaluatorUnlimited;
using offense::GetTargetEvaluated;
using offense::GetKickAngle;
using defense::DefenseEvaluator;
using geometry::Angle;
using geometry::Cross;
using geometry::GetTangentPoints;
using geometry::LineLineIntersection;
using geometry::RayIntersect;
using math_util::AngleDiff;
using zone::ZoneType;
using zone::FieldZone;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using tactics::NTOC_Controller;

namespace tactics {
CoerciveAttacker::CoerciveAttacker(const string& machine_name,
                                   const WorldState& world_state,
                                   TacticArray* tactic_list,
                                   SharedState* shared_state,
                                   OurRobotIndex our_robot_index,
                                   state::SoccerState* soccer_state)
    : StateMachineTactic(machine_name, world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      navigate_(std::bind(&CoerciveAttacker::Navigate, this), "Navigate"),
      receive_(std::bind(&CoerciveAttacker::Receive, this), "Receive"),
      wait_(std::bind(&CoerciveAttacker::Wait, this), "Wait"),
      thresholds_setup_time_(999999, 0.0, 999999,  "setup_time", this),
      is_complete_(false),
      last_target_(offense::kNoPassRobot),
      attacker_index_(42),
      last_cost_(1.0),
      last_goal_({0, 0, 0}) {
  state_ = navigate_;
}

void CoerciveAttacker::Init() {
  state_ = navigate_;
  Tactic* controller = (*tactic_list_)[TacticIndex::RECEIVER].get();
  controller->Reset();
}

void CoerciveAttacker::Reset() {
  attacker_index_ = 42;
  state_ = navigate_;
  Tactic* controller = (*tactic_list_)[TacticIndex::RECEIVER].get();
  controller->Reset();
  controller = (*tactic_list_)[TacticIndex::KICK].get();
  controller->Reset();
  controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  controller->Reset();
}

void CoerciveAttacker::SetGoal(const Pose2Df& pose) {
  evaluated_target_ = pose;
}

bool CoerciveAttacker::IsComplete() { return is_complete_; }

float CoerciveAttacker::GetCost() { return 0; }

void CoerciveAttacker::Receive() {
  Tactic* controller = (*tactic_list_)[TacticIndex::RECEIVER].get();
  controller->Run();
}

float CoerciveAttacker::GetFacing() {
  const Vector2f current_trans =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  // Calculate a kick target
  Vector2f target_position = {0, 0};
  float target_angle = 0;
  OurRobotIndex target_robot = offense::kNoPassRobot;
  GetTargetEvaluated(evaluated_target_.translation, world_state_, soccer_state_,
                     our_robot_index_, false, &target_position, &target_angle,
                     &target_robot, &last_target_);
  float kick_speed = offense::kKickSpeed;
  if (target_robot != offense::kNoPassRobot) {
    kick_speed = offense::kPassSpeed;
  }
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  Vector2f ball_dir = evaluated_target_.translation - ball_pose;
  // TODO(jaholtz) calculate this more accuratley for moving ball kicks
  const Vector2f ball_vel_at_intercept = kick_speed * ball_dir;
  // Correct the kick target based on ball motion
  target_angle = GetKickAngle(ball_vel_at_intercept, current_trans, kick_speed,
                              robot_logger, Heading(target_angle));
  evaluated_target_.angle = target_angle;
  const Vector2f target_vector = geometry::Heading(target_angle);
  // Calculate deflection angle
  float deflection_angle = 0;
  if (ball_dir.norm() > 0) {
    deflection_angle = acos(target_vector.dot(-ball_dir) / (ball_dir.norm()));
  } else {
    deflection_angle = acos(target_vector.dot(-ball_dir));
  }
  deflection_angle = AngleMod(deflection_angle);
  deflection_angle = fabs(deflection_angle);
  robot_logger->LogPrint("Deflection Angle: %f", RadToDeg(deflection_angle));
  // If deflection angle too high, face the ball, otherwise rotate to deflect
  if (deflection_angle > offense::kDeflectionAngle) {
    ball_dir = -ball_dir;
    target_angle = Angle(ball_dir);
  } else {
    const Vector2f target_vector = Heading(target_angle);
    const Vector2f offset = (kRobotRadius + 42.0) * -target_vector;
    evaluated_target_.translation += offset;
  }
  return target_angle;
}

void CoerciveAttacker::Wait() {
  const Vector2f current_trans =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  const float target_angle = GetFacing();
  const Pose2Df wait_goal = {target_angle, current_trans};
  EightGridNavigation* navigator = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());
  // Run the controller with the calculated goal and margins.
  navigator->SetObstacles(obstacle::ObstacleFlag::GetAllExceptTeam(
      world_state_, *soccer_state_, our_robot_index_, our_robot_index_));
  navigator->SetGoal(wait_goal);
  navigator->Run();
}

void CoerciveAttacker::Navigate() {
  const Vector2f ball_velocity = world_state_.GetBallPosition().velocity;
  // Wait for the support positions to stabalize before moving?
  if (ball_velocity.norm() < kTooFastThreshold) {
    NTOC_Controller* controller =
        static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
    controller->TurnOnAngleRelaxation();
    EightGridNavigation* navigator = static_cast<EightGridNavigation*>(
        (*tactic_list_)[TacticIndex::EIGHT_GRID].get());
    // Run the controller with the calculated goal and margins.
    navigator->SetObstacles(
        obstacle::ObstacleFlag::GetAllExceptTeam(
            world_state_, *soccer_state_, our_robot_index_, our_robot_index_) |
        obstacle::ObstacleFlag::GetLargeBall());

    const float target_angle = GetFacing();
    evaluated_target_.angle = target_angle;
    navigator->SetGoal(evaluated_target_);
    navigator->Run();
  } else {
    (*tactic_list_)[TacticIndex::HALT].get()->Run();
  }
}

void CoerciveAttacker::GetPrimaryAttacker() {
  // Find the attacker_index
  size_t our_robots_num = world_state_.GetOurRobots().GetElementCount();
  for (size_t k = 0; k < our_robots_num; k++) {
    if (soccer_state_->GetRobotByOurRobotIndex(k).current_tactic_ ==
            PRIMARY_ATTACKER ||
        soccer_state_->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::DIRECT_FREE_KICKER ||
        soccer_state_->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::KICKOFF ||
        soccer_state_->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::PENALTY_KICK ||
        soccer_state_->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::DIRECT_FREE_KICKER ||
        soccer_state_->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::INDIRECT_FREE_KICKER) {
      attacker_index_ = k;
      break;
    }
  }
}

Vector2f CoerciveAttacker::GetStoppingPose() {
  // Time to reach receive position
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  MotionModel model(kDefaultRobotAcceleration, kDefaultRobotVelocity);
  const float stopping_time =
      CalculateTimeToRest(current_velocity.translation, model);
  const Vector2f stopping_accel =
      kMaxRobotAcceleration * current_velocity.translation.normalized();
  const Vector2f neg_stopping_accel = -stopping_accel;
  const Vector2f stopping_dist = CalculateDeltaPosition(
      current_velocity.translation, neg_stopping_accel, stopping_time);

  Vector2f stopping_pose = current_pose.translation + stopping_dist;
  if (current_velocity.translation.norm() < kEpsilon) {
    stopping_pose = current_pose.translation;
  }
  return stopping_pose;
}

float CoerciveAttacker::TimeToReceive(const Vector2f stopping_pose) {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  const Pose2Df wait_target_ = {current_pose.angle, stopping_pose};
  float time_to_receive_no_stop =
      offense::GetNtocTime(current_pose, current_velocity.translation,
                           evaluated_target_.translation);

  return time_to_receive_no_stop;
}

bool CoerciveAttacker::ShouldNavigate() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  // Time to reach receive position
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  const Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  // Place we will stop if we stop now
  Vector2f stopping_pose = GetStoppingPose();

  // Ball time to receive after kick
  const float time_to_receive = TimeToReceive(stopping_pose);

  // Distance from our stopping point to the target
  const Vector2f stop_to_target = evaluated_target_.translation - stopping_pose;

  // Attacker time to ball
  float time_to_ball = 0.0;
  Vector2f ball_kick_pose = ball_pose;
  if (attacker_index_ != offense::kNoPassRobot) {
    time_to_ball =
        soccer_state_->GetRobotByOurRobotIndex(attacker_index_).GetCost();
    ball_kick_pose = soccer_state_->GetRobotByOurRobotIndex(attacker_index_)
                         .GetGoal()
                         .translation;
  }

  // Kick returns zero cost, but does not take zero time (adjustment here).
  if (time_to_ball - 0.0 < kEpsilon) {
    time_to_ball += .3;
  }

  // Estimated speed of a pass
  const float pass_speed = offense::kPassSpeed + 1000;

  // Time for ball to travel to receive position
  const float time_after_kick = offense::GetBallTravelTime(
      pass_speed, ball_kick_pose, evaluated_target_.translation);

  // Estimated amount of time we will arrive at the receive position
  // before the ball.
  const float time_diff = (time_to_ball + time_after_kick) - time_to_receive;
  // Won't move if we aren't the pass target.
  const bool pass_target = shared_state_->IsPass() &&
                           shared_state_->GetPassTarget() == our_robot_index_;
  // Unless we are too far away from the target.
  const bool can_move =
      pass_target || stop_to_target.norm() >= offense::kTimingAdjustDistance;

  // Log this stuff
  robot_logger->LogPrintPush("Pass Ahead Calculations");
  robot_logger->LogPrint("Move Possible %s", can_move ? "true" : "false");
  robot_logger->AddCircle(stopping_pose, kRobotRadius, 1.0, 1.0, 1.0, 1.0);
  robot_logger->LogPrint("Distance from Receive: %f", stop_to_target.norm());
  robot_logger->LogPrint("Time to reach receive position: %f", time_to_receive);
  robot_logger->LogPrint("Time Attacker to reach ball: %f", time_to_ball);
  robot_logger->LogPrint("Ball Travel Time after Kick: %f", time_after_kick);
  robot_logger->LogPrint("Total Time: %f", time_to_ball + time_after_kick);
  robot_logger->LogPrint("Time Buffer: %f", time_diff);
  robot_logger->LogPrint("Move Threshold: %f",
                         static_cast<float>(thresholds_setup_time_));
  robot_logger->Pop();
  zone::FieldZone field_zone(zone::FULL_FIELD);
  bool is_valid_intercept = field_zone.IsInZone(
      evaluated_target_.translation.cast<float>(), kDefaultSafetyMargin);
  if (!is_valid_intercept) {
    return false;
  }
  // SRTR Boilerplate
  if (pass_target || stop_to_target.norm() >= offense::kTimingAdjustDistance) {
    // SET POTENTIAL TRANSITION
    potential_state_ = "NAVIGATE";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;
    const bool should_move = time_diff < thresholds_setup_time_;
    SetTransition(should_move);
    return should_move;
  }
  return false;
}

void CoerciveAttacker::Transition() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  GetPrimaryAttacker();
  // Time to reach receive position
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;

  // Displaying goal as white line
  robot_logger->AddLine(current_pose.translation, evaluated_target_.translation,
                        1.0, 1.0, 1.0, 1.0);
  if (true) {
    state_ = navigate_;
  } else {
    state_ = wait_;
  }
}

}  // namespace tactics

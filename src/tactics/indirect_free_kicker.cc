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

#include "tactics/indirect_free_kicker.h"
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
#include "navigation/navigation_util.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"
#include "safety/dss2.h"
#include "state/shared_state.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/ball_interception.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/kick.h"
#include "tactics/navigate_to_catch.h"
#include "tactics/navigate_to_intercept.h"
#include "tactics/three_kick.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using geometry::Angle;
using geometry::RayIntersect;
using geometry::EuclideanDistance;
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
using tactics::InterceptionController;
using tactics::ThreeKick;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestPassTarget;
using offense::GetBestChipTarget;
using offense::GetTarget;
using offense::GetTargetEvaluated;
using obstacle::ObstacleFlag;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;
using navigation::CollisionFreePath;
using navigation::ProjectToSafety;

namespace tactics {

IndirectFreeKicker::IndirectFreeKicker(const string& machine_name,
                                       const WorldState& world_state,
                                       TacticArray* tactic_list,
                                       SharedState* shared_state,
                                       OurRobotIndex our_robot_index,
                                       state::SoccerState* soccer_state)
    : StateMachineTactic(machine_name, world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      target_angle_(0),
      is_complete_(false),
      last_target_(offense::kNoPassRobot),
      setup_(std::bind(&IndirectFreeKicker::Setup, this), "Setup"),
      aim_(std::bind(&IndirectFreeKicker::Aim, this), "Aim"),
      kick_(std::bind(&IndirectFreeKicker::Kicking, this), "Kick"),
      post_kick_(std::bind(&IndirectFreeKicker::PostKick, this), "PostKick"),
      thresholds_angle_(offense::kThreshAngle, 0.0, 360.0, "angle", this),
      thresholds_distance_(offense::kThreshDistance,
                           0.0, 9000.0, "distance", this),
      thresholds_y_prime_vel_(offense::kThreshYVel,
                              0.0, 5000.0,  "y_prime_vel", this),
      thresholds_relative_vel_(offense::kThreshRelVel,
                               0.0, 5000.0, "relative_vel", this),
      thresholds_align_(offense::kThreshAlign, 0.0, 9000.0, "align", this),
      thresholds_angular_vel_(offense::kAngularVel,
                              0.0, 5000.0, "angular_vel", this),
      thresholds_kick_timeout_(40, 0.0, 5000.0, "kick_timeout", this),
      thresholds_ball_velocity_(100, 0.0, 5000.0, "ball_velocity", this),
      is_goalie_(false) {
  state_ = setup_;
}

void IndirectFreeKicker::Init() {}

const Pose2Df IndirectFreeKicker::GetGoal() {
  const Vector2f ball = world_state_.GetBallPosition().position;
  return {0, ball};
}

void IndirectFreeKicker::Reset() {
  aim_count_ = 0;
  state_ = setup_;
  is_complete_ = false;
  is_goalie_ = false;
  Tactic* controller = (*tactic_list_)[TacticIndex::KICK].get();
  controller->Reset();
  controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  controller->Reset();
  controller = (*tactic_list_)[TacticIndex::THREE_KICK].get();
  controller->Reset();
}

void IndirectFreeKicker::SetGoal(const Pose2Df& pose) {}

bool IndirectFreeKicker::IsComplete() { return is_complete_; }

bool IndirectFreeKicker::ShouldKick(
    logger::Logger* logger, const float target_angle,
    const Vector2f current_ball_pose, const Vector2f current_ball_velocity,
    const Pose2Df current_robot_pose, const Pose2Df current_robot_velocity,
    const bool is_currenlty_kicking, const bool has_timed_out,
    const bool debug) {
  logger->LogPrintPush("ShouldKick");
  const Eigen::Rotation2Df robot_to_world_rotation(current_robot_pose.angle);
  const Eigen::Rotation2Df world_to_target_rotation(-target_angle);
  const Eigen::Rotation2Df robot_to_target_rotation =
      world_to_target_rotation * robot_to_world_rotation;

  const Vector2f current_velocity_world =
      robot_to_world_rotation * current_robot_velocity.translation;

  const Vector2f robot_prime_vel =
      robot_to_target_rotation * current_robot_velocity.translation;

  const Vector2f robot_to_ball_displace =
      current_ball_pose - current_robot_pose.translation;
  Vector2f desired_norm(cos(target_angle), sin(target_angle));
  Vector2f collision_point =
      current_robot_pose.translation + kInterceptionRadius * desired_norm;
  const float robot_heading = current_robot_pose.angle;
  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  float y_dist = y_dir.dot(robot_to_ball_displace);
  float robot_y_prime_vel = robot_prime_vel.y();
  float ball_y_prime_vel =
      (world_to_target_rotation * current_ball_velocity).y();
  float rel_y_prime_vel = robot_y_prime_vel - ball_y_prime_vel;

  const Vector2f relative_velocity_vector =
      current_velocity_world - current_ball_velocity;

  // The boolean logic of "should_kick"

  // SET POTENTIAL TRANSITION
  potential_state_ = "KICKING";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  double angle_diff =
      RadToDeg(fabs(AngleDiff(target_angle, current_robot_pose.angle)));
  const bool is_at_angle = angle_diff < thresholds_angle_;

  double angular_vel = RadToDeg(fabs(current_robot_velocity.angle));
  const bool is_rotation_at_rest = angular_vel < thresholds_angular_vel_;

  double radial_dist = (collision_point - current_ball_pose).norm();
  const bool is_at_radial_dist = radial_dist < thresholds_distance_;

  const bool is_y_prime_at_relative_rest =
      fabs(rel_y_prime_vel) < thresholds_y_prime_vel_;

  double relative_vel = (relative_velocity_vector).norm();
  const bool is_at_relative_rest = relative_vel < thresholds_relative_vel_;

  const bool is_in_alignment = fabs(y_dist) < thresholds_align_;

  if (!is_at_angle) {
    logger->LogPrint("Not At Angle. Angle Diff: %f, Thresh: %f", angle_diff,
                     static_cast<float>(thresholds_angle_));
  }
  if (!is_rotation_at_rest) {
    logger->LogPrint("Not Rotation At Rest. Angular Vel: %f, Thresh: %f",
                     angular_vel, static_cast<float>(thresholds_angular_vel_));
  }
  if (!is_at_radial_dist) {
    logger->LogPrint("Not At Radial Dist. Distance %f, Thresh: %f", radial_dist,
                     static_cast<float>(thresholds_distance_));
  }
  if (!is_y_prime_at_relative_rest) {
    logger->LogPrint("Not at Relative Rest Y. Y Rel. Vel: %f, Thresh: %f",
                     fabs(rel_y_prime_vel),
                     static_cast<float>(thresholds_y_prime_vel_));
  }
  if (!is_at_relative_rest) {
    logger->LogPrint("Not at Relative Rest. Relative Vel: %f, Thresh: %f",
                     relative_vel,
                     static_cast<float>(thresholds_relative_vel_));
  }
  if (!is_in_alignment) {
    logger->LogPrint("Not in alignment. Y dist %f, Thresh: %f", y_dist,
                     static_cast<float>(thresholds_align_));
  }

  const bool should_kick =
      (is_at_angle && is_at_radial_dist && is_at_relative_rest &&
       is_in_alignment && is_rotation_at_rest && is_y_prime_at_relative_rest) ||
      (!has_timed_out && is_currenlty_kicking);

  if (should_kick) {
    logger->LogPrint("Kick Conditions Met");
  }

  // SET SHOULD TRANSITION
  SetTransition(should_kick);
  logger->Pop();
  return should_kick;
}

// Cost is currently the ntoc time to the ball.
float IndirectFreeKicker::GetCost() {
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Vector2f ball_pose = world_state_.GetBallPosition().position;

  MotionModel motion_model(kMaxRobotAcceleration, kMaxRobotVelocity);
  ControlSequence2D linear_control;

  Vector2f ntoc_position;
  Vector2f ntoc_velocity;

  ntoc::TransformCoordinatesForNTOC(current_pose, current_velocity.translation,
                                    ball_pose, &ntoc_position, &ntoc_velocity);

  ntoc_velocity.x() = 0;
  ntoc_velocity.y() = 0;

  return (NTOC2D(ntoc_position, ntoc_velocity, motion_model, &linear_control));
}

void IndirectFreeKicker::SetIsGoalie() { is_goalie_ = true; }

void IndirectFreeKicker::Setup() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  OurRobotIndex target_robot;
  Vector2f target_position;
  GetTargetEvaluated(ball_pose, world_state_, soccer_state_, our_robot_index_,
                     true, &chip_, &chip_distance_, &target_position,
                     &target_angle_, &target_robot, &last_target_);
  const Vector2f ball_displacement = ball_pose - current_pose.translation;
  if (target_robot != 42 && ball_displacement.norm() < 5 * kRobotRadius) {
    shared_state_->SetPass(target_robot, target_position);
  } else {
    shared_state_->ClearPass();
  }
  const Vector2f best_position = soccer_state_->best_pass_position_;
  robot_logger->AddCircle(best_position, kRobotRadius, 1.0, 0.0, 1.0, 1.0);
  // Set the movement goal as a position behind the ball with respect
  // to the target, facing the ball.
  Vector2f desired_norm(cos(target_angle_), sin(target_angle_));
  Pose2Df goal_pose;
  goal_pose.translation = ball_pose - (kRotationRadius + 100) * desired_norm;

  if (!is_goalie_) {
    goal_pose.translation =
        ProjectToSafety(goal_pose.translation, current_pose.translation,
                        kAttackerFieldMargin, robot_logger);
  }
  goal_pose.angle = target_angle_;

  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  // Run the controller with the calculated goal and margins.
  controller->SetObstacles(
      obstacle::ObstacleFlag::GetAllExceptTeam(
          world_state_, *soccer_state_, our_robot_index_, our_robot_index_) |
      obstacle::ObstacleFlag::GetMediumBall());
  controller->SetGoal(goal_pose);
  controller->Run();
}

void IndirectFreeKicker::Aim() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;

  ball_start = ball_pose;
  OurRobotIndex target_robot;
  Vector2f target_position;
  GetTargetEvaluated(ball_pose, world_state_, soccer_state_, our_robot_index_,
                     true, &chip_, &chip_distance_, &target_position,
                     &target_angle_, &target_robot, &last_target_);
  const Vector2f ball_displacement = ball_pose - current_pose.translation;
  if (target_robot != 42 && ball_displacement.norm() < 5 * kRobotRadius) {
    shared_state_->SetPass(target_robot, target_position);
  } else {
    shared_state_->ClearPass();
  }
  // Set the movement goal as a position behind the ball with respect
  // to the target, facing the ball.
  Vector2f desired_norm(cos(target_angle_), sin(target_angle_));
  Pose2Df goal_pose;
  const float smaller_radius = kBallRadius + kRobotRadius;
  goal_pose.translation = ball_pose - smaller_radius * desired_norm;

  if (!is_goalie_) {
    goal_pose.translation =
        ProjectToSafety(goal_pose.translation, current_pose.translation,
                        kAttackerFieldMargin, robot_logger);
  }
  goal_pose.angle = target_angle_;

  robot_logger->LogPrint("IDFK, Goal Pose: %f, %f, %f",
                         goal_pose.translation.x(), goal_pose.translation.y(),
                         goal_pose.angle);

  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetTheirDefenseArea());
  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::NTOC].get());

  // Run the controller with the calculated goal and margins.
  controller->SetObstacles(obstacle::ObstacleFlag::GetAllExceptTeam(
      world_state_, *soccer_state_, our_robot_index_, our_robot_index_));
  controller->SetGoal(goal_pose);
  controller->Run();
}

void IndirectFreeKicker::Kicking() {
  ThreeKick* controller =
      static_cast<ThreeKick*>((*tactic_list_)[TacticIndex::THREE_KICK].get());
  controller->SetPassOnly(true);
  controller->SetChip(chip_, chip_distance_);
  controller->SetGoal({target_angle_, 0, 0});
  controller->Run();
}

void IndirectFreeKicker::PostKick() {
  Tactic* controller = (*tactic_list_)[TacticIndex::HALT].get();
  controller->Run();
}

void IndirectFreeKicker::Transition() {
  // Setup
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  Vector2f filtered_ball_pose =
      world_state_.GetBallPosition().filtered_position;
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  const Vector2f ball_dir = filtered_ball_pose - ball_start;
  float ball_angle = Angle(ball_dir);
  ball_angle = RadToDeg(ball_angle);
  Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);
  if (state_ == setup_) {
    EightGridNavigation* controller = static_cast<EightGridNavigation*>(
        (*tactic_list_)[TacticIndex::EIGHT_GRID].get());
    if (controller->IsComplete()) {
      state_ = aim_;
    }
  }
  if (state_ == aim_) {
    aim_count_++;
    is_complete_ = false;
    const bool should_kick =
        ShouldKick(robot_logger, target_angle_, ball_pose, ball_vel,
                   current_pose, current_velocity, false, false, true);
    // Decide whether to pass or receive (or keep waiting)
    if (should_kick) {
      state_ = kick_;
    }
  }
  if (state_ == kick_) {
    Tactic* controller = (*tactic_list_)[TacticIndex::THREE_KICK].get();
    if (controller->IsComplete()) {
      state_ = post_kick_;
      is_complete_ = true;
    }
  }

  if (state_ == post_kick_) {
    Reset();
  }
}

}  // namespace tactics

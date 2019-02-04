// Copyright 2018 jaholtz@cs.umass.edu
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

#include "tactics/kickoff_kicker.h"
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
using tactics::Kick;
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

KickoffKicker::KickoffKicker(const string& machine_name,
                             const WorldState& world_state,
                             TacticArray* tactic_list,
                             SharedState* shared_state,
                             OurRobotIndex our_robot_index,
                             state::SoccerState* soccer_state)
    : StateMachineTactic(machine_name, world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      target_angle_(0),
      target_robot_(offense::kNoPassRobot),
      target_position_(kTheirGoalCenter),
      is_complete_(false),
      last_target_(offense::kNoPassRobot),
      first_(true),
      preparation_(std::bind(&KickoffKicker::Preparation, this), "Prep"),
      setup_(std::bind(&KickoffKicker::Setup, this), "Setup"),
      kick_(std::bind(&KickoffKicker::Kick, this), "Kick"),
      post_kick_(std::bind(&KickoffKicker::PostKick, this), "PostKick"),
      thresholds_angle_(offense::kThreshAngle, "angle", this),
      thresholds_distance_(offense::kThreshDistance, "distance", this),
      thresholds_y_prime_vel_(offense::kThreshYVel, "y_prime_vel", this),
      thresholds_relative_vel_(offense::kThreshRelVel, "relative_vel", this),
      thresholds_align_(offense::kThreshAlign, "align", this),
      thresholds_angular_vel_(offense::kAngularVel, "angular_vel", this),
      thresholds_kick_timeout_(40, "kick_timeout", this),
      thresholds_ball_velocity_(100, "ball_velocity", this) {
  state_ = preparation_;
}

void KickoffKicker::Init() {}

void KickoffKicker::Reset() {
  state_ = preparation_;
  is_complete_ = false;
  first_ = true;
  target_angle_ = 0;
  target_robot_ = offense::kNoPassRobot;
  target_position_ = kTheirGoalCenter;
}

void KickoffKicker::SetGoal(const Pose2Df& pose) {}

bool KickoffKicker::IsComplete() { return is_complete_; }

bool KickoffKicker::ShouldKick(logger::Logger* logger, const float target_angle,
                               const Vector2f current_ball_pose,
                               const Vector2f current_ball_velocity,
                               const Pose2Df current_robot_pose,
                               const Pose2Df current_robot_velocity,
                               const bool is_currenlty_kicking,
                               const bool has_timed_out, const bool debug) {
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

  // SET SHOULD TRANSITION
  SetTransition(should_kick);
  logger->Pop();
  return should_kick;
}

// Cost is currently the ntoc time to the ball.
float KickoffKicker::GetCost() {
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

void KickoffKicker::Preparation() {
  is_complete_ = false;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  const Vector2f offset = {550, 0};
  // Sit away from the ball waiting for the kickoff to start.
  const Vector2f goal_translation = ball_pose - offset;
  const Pose2Df goal_pose = {0, goal_translation};
  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  // Run the controller with the calculated goal and margins.
  controller->SetObstacles(obstacle::ObstacleFlag::GetAllExceptTeam(
      world_state_, *soccer_state_, our_robot_index_, our_robot_index_));
  controller->SetGoal(goal_pose);
  controller->Run();
}

void KickoffKicker::Setup() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  if (first_) {
    first_ = false;
    GetTargetEvaluated(ball_pose, world_state_, soccer_state_, our_robot_index_,
                       false, &target_position_, &target_angle_, &target_robot_,
                       &last_target_);
    if (target_robot_ != offense::kNoPassRobot) {
      shared_state_->SetPass(target_robot_, target_position_);
    } else {
      shared_state_->ClearPass();
    }
  }
  // Set the movement goal as a position behind the ball with respect
  // to the target, facing the ball.
  Vector2f desired_norm(cos(target_angle_), sin(target_angle_));
  Pose2Df goal_pose;
  const float smaller_radius = kBallRadius + kRobotRadius;
  goal_pose.translation = ball_pose - smaller_radius * desired_norm;
  goal_pose.translation =
      ProjectToSafety(goal_pose.translation, current_pose.translation,
                      kAttackerFieldMargin, robot_logger);
  goal_pose.angle = target_angle_;

  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());
  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  // Run the controller with the calculated goal and margins.
  controller->SetObstacles(obstacle::ObstacleFlag::GetAllExceptTeam(
      world_state_, *soccer_state_, our_robot_index_, our_robot_index_));

  controller->SetGoal(goal_pose);
  controller->Run();
}

void KickoffKicker::Kick() {
  class ::Kick* controller = static_cast<class ::Kick*>(
      (*tactic_list_)[TacticIndex::THREE_KICK].get());
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());
  controller->SetGoal({target_angle_, 0, 0});
  controller->SetPassOnly(shared_state_->IsPass());
  controller->Run();
}

void KickoffKicker::PostKick() {
  Tactic* controller = (*tactic_list_)[TacticIndex::HALT].get();
  controller->Run();
}

void KickoffKicker::Transition() {
  // Setup
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;

  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  Vector2f ball_vel = world_state_.GetBallPosition().velocity;

  Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);
  if (state_ == preparation_) {
    state::RefereeState ref_state = soccer_state_->GetRefereeState();
    if (ref_state.IsNormalStart() && !is_complete_) {
      state_ = setup_;
    }
  } else if (state_ != kick_) {
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
}

}  // namespace tactics

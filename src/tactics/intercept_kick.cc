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

#include "tactics/intercept_kick.h"
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
#include "tactics/deflection.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/kick.h"
#include "tactics/navigate_to_catch.h"
#include "tactics/navigate_to_intercept.h"
#include "tactics/ntoc_controller.h"
#include "tactics/receiver.h"
#include "tactics/stox_pivot.h"
#include "tactics/three_kick.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using geometry::Angle;
using geometry::RayIntersect;
using geometry::EuclideanDistance;
using geometry::ProjectPointOntoLine;
using geometry::CheckLineLineIntersection;
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
using tactics::NTOC_Controller;
using tactics::TacticIndex;
using tactics::InterceptionController;
using tactics::Kick;
using tactics::ThreeKick;
using tactics::Receiver;
using tactics::Deflection;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestPassTarget;
using offense::GetBestChipTarget;
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
using math_util::SolveQuadratic;

namespace tactics {

PrimaryAttacker::PrimaryAttacker(const string& machine_name,
                                 const WorldState& world_state,
                                 TacticArray* tactic_list,
                                 SharedState* shared_state,
                                 OurRobotIndex our_robot_index,
                                 state::SoccerState* soccer_state)
    : StateMachineTactic(machine_name, world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      target_angle_(0),
      last_target_(offense::kNoPassRobot),
      ball_intercept_point_(Vector2f(0, 0)),
      set_kick_goal(false),
      is_complete_(false),
      aim_count_(0),
      kMaxAimIterations(3),
      start_(std::bind(&PrimaryAttacker::Start, this), "Start"),
      intercept_(std::bind(&PrimaryAttacker::Intercept, this), "Intercept"),
      kick_(std::bind(&PrimaryAttacker::Kick, this), "Kick"),
      post_kick_(std::bind(&PrimaryAttacker::PostKick, this), "PostKick"),
      thresholds_angle_(1.5, "angle", this),
      thresholds_distance_(kRotationRadius, "distance", this),
      thresholds_y_prime_vel_(200, "y_prime_vel", this),
      thresholds_relative_vel_(400, "relative_vel", this),
      thresholds_align_(18, "align", this),
      thresholds_angular_vel_(50, "angular_vel", this),
      thresholds_kick_timeout_(40, "kick_timeout", this),
      thresholds_lower_catch_velocity_(1500, "catch_velocity", this),
      thresholds_could_score_speed_(1000, "could_score_speed", this),
      thresholds_ball_velocity_(100, "ball_velocity", this),
      thresholds_catch_radius_(6 * kRobotRadius, "catch_radius", this),
      thresholds_kick_percent_(85, "kick_percent", this),
      thresholds_kick_speed_(1000, "kick_speed", this),
      thresholds_follow_through_(10, "kick_follow_through", this),
      kick_count_(0),
      prep_count_(0),
      last_target_score_(10.0),
      last_target_robot_(42),
      last_target_position_({0, 0}) {
  state_ = start_;
}

void PrimaryAttacker::Init() {
  intercept_solution_.isInitialized = false;
  catch_solution_.isInitialized = false;
}

void PrimaryAttacker::Reset() {
  state_ = start_;
  is_complete_ = false;
  kick_count_ = 0;
  aim_count_ = 0;
  last_target_score_ = 10.0;
  last_target_robot_ = 42;
  last_target_position_ = {0, 0};
  intercept_solution_.isInitialized = false;
  catch_solution_.isInitialized = false;
  // Reset the receiver when this tactic resets, otherwise the states
  // can hang between calls.
  Tactic* controller = (*tactic_list_)[TacticIndex::RECEIVER].get();
  controller->Reset();
  controller = (*tactic_list_)[TacticIndex::KICK].get();
  controller->Reset();
  controller = (*tactic_list_)[TacticIndex::THREE_KICK].get();
  controller->Reset();
}

void PrimaryAttacker::SetGoal(const Pose2Df& pose) {}

bool PrimaryAttacker::IsComplete() { return is_complete_; }

bool PrimaryAttacker::ShouldKick(logger::Logger* logger,
                                 const float target_angle,
                                 const Vector2f current_ball_pose,
                                 const Vector2f current_ball_velocity,
                                 const Pose2Df current_robot_pose,
                                 const Pose2Df current_robot_velocity,
                                 const bool is_currenlty_kicking,
                                 const bool has_timed_out, const bool debug) {
  const bool kDebug = false;
  if (kDebug) {
    logger->LogPrintPush("ShouldKick");
  }
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

  if (kDebug) {
    if (!is_at_angle) {
      logger->LogPrint("Not At Angle. Angle Diff: %f, Thresh: %f", angle_diff,
                       static_cast<float>(thresholds_angle_));
    }
    if (!is_rotation_at_rest) {
      logger->LogPrint("Not Rotation At Rest. Angular Vel: %f, Thresh: %f",
                       angular_vel,
                       static_cast<float>(thresholds_angular_vel_));
    }
    if (!is_at_radial_dist) {
      logger->LogPrint("Not At Radial Dist. Distance %f, Thresh: %f",
                       radial_dist, static_cast<float>(thresholds_distance_));
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
  }

  const bool should_kick =
      (is_at_angle && is_at_radial_dist && is_at_relative_rest &&
       is_in_alignment && is_rotation_at_rest && is_y_prime_at_relative_rest) ||
      (!has_timed_out && is_currenlty_kicking);

  // SET SHOULD TRANSITION
  SetTransition(should_kick);
  if (kDebug) {
    logger->Pop();
  }
  return should_kick;
}

void PrimaryAttacker::GetSolution(Vector2f current_robot_trans,
                                  const Vector2f& current_velocity_world,
                                  const Vector2f& current_ball_pose,
                                  const Vector2f& current_ball_velocity) {
  InterceptionController* controller = static_cast<InterceptionController*>(
      (*tactic_list_)[TacticIndex::BALL_INTERCEPTION].get());
  controller->match_velocity_ = true;
  controller->SetSolution(intercept_solution_);

  Vector2f ball_to_goal;
  if (!intercept_solution_.isInitialized) {
    // if we don't have an intercept solution, our target angle defaults to the
    // angle between the ball and the goal
    ball_to_goal = kTheirGoalCenter - current_ball_pose;
  } else {
    ball_to_goal = kTheirGoalCenter - ball_intercept_point_;
  }

  Vector2f desired_norm = ball_to_goal.normalized();
  controller->SetOffset(kInterceptionRadius * desired_norm);
  controller->RunDontCommand();
  robot_interception_point_ = controller->final_robot_pos_.cast<float>();
  robot_interception_vel_ = controller->final_ball_vel_.cast<float>();
  ball_intercept_point_ = controller->final_ball_pos_.cast<float>();
  ball_interception_vel_ = controller->final_ball_vel_.cast<float>();
  intercept_solution_ = controller->GetSolution();
}

void PrimaryAttacker::Start() { }

void PrimaryAttacker::Intercept() {
  Vector2f ball_pose = world_state_.GetBallPosition().position;

  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  Vector2f ball_vel = world_state_.GetBallPosition().velocity;

  Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);

  Vector2f current_velocity_world =
      robot_to_world_rotation * current_velocity.translation;

  GetSolution(current_pose.translation, current_velocity_world, ball_pose,
              ball_vel);
  controller->SetSolution(intercept_solution_);
  controller->SetAngle(target_angle_);
  controller->Command();
}

void PrimaryAttacker::Kick() {
  logger::Logger* robot_logger =
    soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrint("Shooting");
  class ::Kick* controller =
      static_cast<class ::Kick*>((*tactic_list_)[TacticIndex::KICK].get());
  controller->SetGoal({target_angle_, 0, 0});
  controller->SetPassOnly(shared_state_->IsPass());
  controller->Run();
}

void PrimaryAttacker::PostKick() {
  kick_follow_through_count_++;
  logger::Logger* robot_logger =
    soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrint("In PostKick: followed through for %d out of %f steps",
    kick_follow_through_count_,
    static_cast<float>(thresholds_follow_through_));
}

void PrimaryAttacker::Transition() {
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

  Vector2f current_velocity_world =
      robot_to_world_rotation * current_velocity.translation;

  State last_state = state_;
  if (state_ == start_) {
    state_ = intercept_;
  } else if (state_ == intercept_) {
    const bool should_kick =
        ShouldKick(robot_logger, target_angle_, ball_pose, ball_vel,
                  current_pose, current_velocity, false, false, true);
    // Decide whether to pass or receive (or keep waiting)
    if (should_kick) {
      state_ = kick_;
      kick_solution_.isInitialized = false;
    }
  } else if (state_ == kick_) {
    Tactic* controller = (*tactic_list_)[TacticIndex::KICK].get();
    if (controller->IsComplete()) {
      state_ = post_kick_;
      is_complete_ = true;
      kick_follow_through_count_ = 0;
    }
  } else if (state_ == post_kick_) {
    if (kick_follow_through_count_ + 1 > thresholds_follow_through_) {
      Reset();
    }
  }
  if (state_ != last_state) {
    aim_count_ = 0;
  }
}

}  // namespace tactics

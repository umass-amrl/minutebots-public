// Copyright 2017 - 2018 dbalaban@cs.umass.edu
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

#include "tactics/ball_interception.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/angular_planner.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "state_estimation/default_motion_model.h"

STANDARD_USINGS;
using datastructures::OptionalValueMutable;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using Eigen::Matrix2f;
using Eigen::Rotation2Df;
using math_util::AngleMod;
using math_util::AngleDiff;
using state::SharedRobotState;
using state::WorldState;
using std::atan2;
using std::cos;
using std::endl;
using std::map;
using std::max;
using std::min;
using std::sin;
using std::abs;
using std::vector;
using std::unique_ptr;
using tactics::TacticIndex;
using state::SharedState;
using tsocs::BallInterception;
using ntoc::ControlPhase1D;
using ntoc::ControlSequence1D;
using ntoc::ControlPhase2D;
using motion::CalculateAngularWaypoint;
using motion::MotionModel;

namespace tactics {

InterceptionController::InterceptionController(
                                   const WorldState& world_state,
                                   TacticArray* tactic_list,
                                   SharedState* shared_state,
                                   OurRobotIndex our_robot_index,
                                   state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state),
      params_(SolutionParameters(1, 2, 3, 4, 5)),
      motion_model_(motion::MotionModel(kDefaultRobotAcceleration,
                                        kDefaultRobotVelocity)),
      offset_(Vector2f(0, 0)),
      relax_angle_(true) {
  params_.isInitialized = false;
}

void InterceptionController::RunDontCommand() {
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  if (kDebug) {
    logger->LogPrintPush("Running Ball Interception RunDontCommand");
    logger->LogPrint("Current World Time: %f", world_state_.world_time_);
    logger->LogPrint("Match Velocity: %s", match_velocity_ ? "True" : "False");
  }

  Pose2Df current_velocity =
    world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Pose2Df current_pose =
    world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pos = world_state_.GetBallPosition().position;
  Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  Vector2f collision_point = current_pose.translation + offset_;

  if (kDebug) {
    logger->LogPrint("Collision Point: (%f, %f)", collision_point.x(),
                        collision_point.y());
    logger->LogPrint("Ball Pos: (%f, %f)", ball_pos.x(), ball_pos.y());
    logger->LogPrint("Distance to ball: %f",
                        (ball_pos - collision_point).norm());
    logger->LogPrint("Current Pose (%.5f, %.5f, %.5f deg)",
                        current_pose.translation.x(),
                        current_pose.translation.y(),
                        RadToDeg(current_pose.angle));
    logger->LogPrint("Current Velocity (%.5f, %.5f, %.5fdeg/s)",
                        current_velocity.translation.x(),
                        current_velocity.translation.y(),
                        RadToDeg(current_velocity.angle));
    logger->LogPrint("Current Ball Pose (%.5f, %.5f)", ball_pos.x(),
                        ball_pos.y());
    logger->LogPrint("Current Ball Velocity (%.5f, %.5f)", ball_vel.x(),
                        ball_vel.y(), current_velocity.angle);
  }
  Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);

  Vector2f current_velocity_world =
      robot_to_world_rotation * current_velocity.translation;

  logger->AddLine(
      current_pose.translation.x(), current_pose.translation.y(),
      current_pose.translation.x() + current_velocity_world.x() / 10.0,
      current_pose.translation.y() + current_velocity_world.y() / 10.0, 0, 1, 0,
      1);
  BallInterception intercept(collision_point.cast<double>(),
                             current_velocity_world.cast<double>(),
                             ball_pos.cast<double>(), ball_vel.cast<double>(),
                             motion_model_.a_max,
                             kBallAcceleration);
  intercept.match_velocity_ = match_velocity_;
  intercept.GetInterceptSolution(&params_, logger);
  // update variables that tell other tactics about the solution we found
  interception_time_ = params_.T;
  intercept.GetState(&final_robot_pos_,
                     &final_robot_vel_,
                     &final_ball_pos_,
                     &final_ball_vel_,
                     params_.T,
                     params_);
  if (kDebug) {
    logger->LogPrint("Robot pose at interception: %f, %f",
                        final_robot_pos_.x(), final_robot_pos_.y());
    logger->LogPrint("Ball pose at interception: %f, %f",
                        final_ball_pos_.x(), final_ball_pos_.y());
    logger->LogPrint("Robot vel at interception: %f, %f",
                        final_robot_vel_.x(), final_robot_vel_.y());
    logger->LogPrint("Ball vel at interception: %f, %f",
                        final_ball_vel_.x(), final_ball_vel_.y());
    logger->Pop();
  }
}

void InterceptionController::Command() {
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  logger->LogPrintPush("Running Ball Interception Command");
  Pose2Df current_velocity =
    world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Pose2Df current_pose =
    world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pos = world_state_.GetBallPosition().position;
  Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  Vector2f collision_point = current_pose.translation + offset_;
  Vector2f current_velocity_world =
    Rotation2Df(current_pose.angle) * current_velocity.translation;
  Vector2d expected_robot_pos;
  Vector2d expected_robot_vel;
  Vector2d expected_ball_pos;
  Vector2d expected_ball_vel;
  BallInterception intercept(collision_point.cast<double>(),
                             current_velocity_world.cast<double>(),
                             ball_pos.cast<double>(), ball_vel.cast<double>(),
                             motion_model_.a_max,
                             kBallAcceleration);
  intercept.GetState(&expected_robot_pos,
                     &expected_robot_vel,
                     &expected_ball_pos,
                     &expected_ball_vel,
                     kTransmitPeriodSeconds,
                     params_);
  vector<Vector2f> path_points = intercept.GetPath(20, params_);

  // subtract the offset from path_points so the path starts at the robot,
  // not at the desired collision point of the robot
  for (unsigned int i = 0; i < path_points.size(); i++) {
    path_points[i] -= offset_;
  }
  logger->AddPoints(path_points, 1, 1, 0, 1);
  logger->AddCircle(final_ball_pos_.cast<float>(), 0.9 * kBallRadius, 0.0,
                        0.0, 1.0, 1.0);
  logger->AddCircle(final_robot_pos_.cast<float>(), 1.1 * kBallRadius, 0.0,
                        1.0, 0.0, 1.0);

  logger->LogPrint("target angle: %f deg", RadToDeg(target_angle));
  logger->LogPrint("current angle: %f deg", RadToDeg(current_pose.angle));

  ControlSequence1D rotational_control;
  float delta_angle = AngleDiff(target_angle, current_pose.angle);
  if (relax_angle_ &&
      params_.T - kAnglePlannerDelta >
      kAnglePlannerLookahead*kTransmitPeriodSeconds) {
    delta_angle = CalculateAngularWaypoint(target_angle,
                                           current_pose.angle,
                                           params_.T,
                                           kAnglePlannerDelta,
                                           kAnglePlannerLookahead*
                                               kTransmitPeriodSeconds,
                                           kMaxRobotRotVel,
                                           logger);
  }

  TimeOptimalControlAnyFinal1D(0.0,
                               current_velocity.angle,
                               delta_angle,
                               0.0,
                               0.0,
                               kMaxRobotRotAccel,
                               kMaxRobotRotVel,
                               &rotational_control);

  logger->LogPrint("delta angle: %f", delta_angle);
  rotational_control.LogSequence(logger, false);

  const double angle_accel =
      GetAccelToPreservePosition(rotational_control,
                                 kTransmitPeriodSeconds,
                                 current_velocity.angle);

  const Pose2Df desired_velocity_world(
      current_velocity.angle + angle_accel * kTransmitPeriodSeconds,
      expected_robot_vel.cast<float>());

  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  logger->LogPrint("Expected World Velocity: (%.5f, %.5f, %.5f deg/s)",
                       expected_robot_vel.x(),
                       expected_robot_vel.y(),
                       RadToDeg(desired_velocity_world.angle));
  state->our_robot_index = our_robot_index_;
  state->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;

  // TODO(slane) find a better way to do this
  Vector2f translation_accel =
      (desired_velocity_world.translation - current_velocity_world) /
      kTransmitPeriodSeconds;


  state->acceleration_command.translation = translation_accel;
  state->acceleration_command.angle = angle_accel;

  logger->AddLine(
      current_pose.translation.x(), current_pose.translation.y(),
      current_pose.translation.x() + expected_robot_vel.x() / 10.0,
      current_pose.translation.y() + expected_robot_vel.y() / 10.0, 1, 0, 0, 1);
  logger->Pop();

  relax_angle_ = true;
}

void InterceptionController::Run() {
  RunDontCommand();
  Command();
}

double InterceptionController::GetInterceptionTime() {
  return interception_time_;
}

Vector2d InterceptionController::GetInterceptionPoint() {
  return final_robot_pos_;
}

Vector2d InterceptionController::GetInterceptionVelocity() {
  return final_robot_vel_;
}

void InterceptionController::Reset() {
  params_.isInitialized = false;
}

void InterceptionController::Init() {
  params_.isInitialized = false;
}

void InterceptionController::SetGoal(const pose_2d::Pose2Df& pose) {
  target_angle = pose.angle;
}

void InterceptionController::SetAngle(float angle) { target_angle = angle; }

void InterceptionController::SetMotionModel(MotionModel motion_model) {
  motion_model_ = motion_model;
}

void InterceptionController::SetOffset(Vector2f offset) {
  offset_ = offset;
}

SolutionParameters InterceptionController::GetSolution() {
  return params_;
}

void InterceptionController::SetSolution(SolutionParameters params) {
  params_ = params;
}

void InterceptionController::TurnOnAngleRelaxation() {
  relax_angle_ = true;
}

void InterceptionController::TurnOffAngleRelaxation() {
  relax_angle_ = false;
}


}  // namespace tactics

// Copyright 2017 - 2019 dbalaban@cs.umass.edu, slane@cs.umass.edu
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

#include "tactics/ntoc_controller.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>
#include <iomanip>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/angular_planner.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "state_estimation/default_motion_model.h"
#include "tactics/pd_controller.h"

STANDARD_USINGS;
using datastructures::OptionalValueMutable;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Matrix2f;
using Eigen::Rotation2Df;
using geometry::EuclideanDistance;
using geometry::SafeVectorNorm;
using math_util::AngleMod;
using motion::CalculateAngularWaypoint;
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
using tactics::PDController;
using state::SharedState;
using ntoc::ControlPhase1D;
using ntoc::ControlSequence1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using motion_model::DefaultMotionModel;

extern ScopedFile ntoc_data_fid;

namespace tactics {

NTOC_Controller::NTOC_Controller(const WorldState& world_state,
                                 TacticArray* tactic_list,
                                 SharedState* shared_state,
                                 OurRobotIndex our_robot_index,
                                 state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state),
      motion_model_(kDefaultRobotAcceleration, kDefaultRobotVelocity),
      relax_angle_(false),
      translation_complete_(false),
      rotation_complete_(false),
      position_and_trajectory_file_(),
      use_pd_(kDefaultUsePD_) {}

bool NTOC_Controller::HasNans() {
  logger::Logger* the_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  const Pose2Df& current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  const Pose2Df& current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  if (!(std::isnan(current_pose.angle) ||
        std::isnan(current_pose.translation.x()) ||
        std::isnan(current_pose.translation.y()) ||
        std::isnan(current_velocity.angle) ||
        std::isnan(current_velocity.translation.x()) ||
        std::isnan(current_velocity.translation.y()) ||
        std::isnan(goal_.translation.x()) ||
        std::isnan(goal_.translation.y()) ||
        world_state_.GetOurRobotPosition(our_robot_index_).confidence == 0)) {
    return false;
  }
  the_logger->LogPrint("NTOC Pased nan values");

  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);

  state->our_robot_index = our_robot_index_;
  state->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;
  state->velocity_x = 0.0;
  state->velocity_y = 0.0;
  state->velocity_r = 0.0;
  return true;
}

void NTOC_Controller::LogPositionTrajectoryToFile(
    const ntoc::ControlSequence2D& linear_control,
    const ntoc::ControlSequence1D& rotational_control,
    const pose_2d::Pose2Df& accel_command) {
  if (!kLogPositionTrajectory) {
    return;
  }

  if (!position_and_trajectory_file_.is_open()) {
    position_and_trajectory_file_.open(
        "position_and_trajectory_file_robot_" +
            std::to_string(world_state_.GetOurRobotPosition(our_robot_index_)
                               .ssl_vision_id) +
            ".txt",
        std::ios::out);
  }

  if (!position_and_trajectory_file_) {
    return;
  }

  const pose_2d::Pose2Df vision_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

  const pose_2d::Pose2Df filtered_not_fp_position =
      world_state_.GetOurRobotPosition(our_robot_index_).filtered_position;

  const pose_2d::Pose2Df filtered_not_fp_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).filtered_velocity;

  const pose_2d::Pose2Df current_position =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Pose2Df& current_velocity_robot_frame =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  const Pose2Df current_velocity_world_frame = {
      current_velocity_robot_frame.angle,
      Rotation2Df(
          world_state_.GetOurRobotPosition(our_robot_index_).position.angle) *
          current_velocity_robot_frame.translation};
  const pose_2d::Pose2Df goal_position = goal_;

  position_and_trajectory_file_
      << std::setprecision(20) << world_state_.GetWorldTime() << ", "
      << world_state_.GetOurRobotPosition(our_robot_index_).observed_time
      << ", " << GetMonotonicTime() << ", " << vision_pose.translation.x()
      << ", " << vision_pose.translation.y() << ", " << vision_pose.angle
      << ", " << filtered_not_fp_position.translation.x() << ", "
      << filtered_not_fp_position.translation.y() << ", "
      << filtered_not_fp_position.angle << ", "
      << filtered_not_fp_velocity.translation.x() << ", "
      << filtered_not_fp_velocity.translation.y() << ", "
      << filtered_not_fp_velocity.angle << ", "
      << current_position.translation.x() << ", "
      << current_position.translation.y() << ", " << current_position.angle
      << ", " << current_velocity_world_frame.translation.x() << ", "
      << current_velocity_world_frame.translation.y() << ", "
      << current_velocity_world_frame.angle << ", "
      << goal_position.translation.x() << ", " << goal_position.translation.y()
      << ", " << goal_position.angle << ", " << accel_command.translation.x()
      << ", " << accel_command.translation.y() << ", " << accel_command.angle
      << ", "
      << "#" << linear_control.num_phases << ", "
      << "#" << rotational_control.num_phases << ", ";

  for (size_t i = 0; i < linear_control.num_phases; ++i) {
    const ControlPhase2D& phase = linear_control.phases[i];
    position_and_trajectory_file_ << phase.acceleration.x() << ", "
                                  << phase.acceleration.y() << ", "
                                  << phase.duration << ", ";
  }

  for (size_t i = 0; i < rotational_control.num_phases; ++i) {
    const ControlPhase1D& phase = rotational_control.phases[i];
    position_and_trajectory_file_ << phase.acceleration << ", "
                                  << phase.duration << ", ";
  }
  position_and_trajectory_file_ << '\n';
}

void NTOC_Controller::Run() {
  static const float kPDPositionThreshold_ =
    configuration_reader::CONFIG_ntoc_pd_position_threshold;
  static const float kPDVelocityThreshold_ =
    configuration_reader::CONFIG_ntoc_pd_velocity_threshold;
  logger::Logger* the_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  the_logger->LogPrint("NTOC Controller");
  the_logger->Push();

  if (kDebug_) {
    the_logger->LogPrint("Current World Time: %f", world_state_.world_time_);
  }

  if (HasNans()) {
    return;
  }

  const Pose2Df& current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  const float current_speed = SafeVectorNorm(current_velocity.translation);

  const Pose2Df& current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;

  const Vector2f current_velocity_world =
      Rotation2Df(current_pose.angle) * current_velocity.translation;

  the_logger->LogPrint("Goal Pose (%.5f, %.5f, %.5f deg)",
                       goal_.translation.x(), goal_.translation.y(),
                       RadToDeg(goal_.angle));
  the_logger->LogPrint(
      "Current Pose (%.5f, %.5f, %.5f deg)", current_pose.translation.x(),
      current_pose.translation.y(), RadToDeg(current_pose.angle));
  the_logger->LogPrint("Current Velocity World: %f, %f, %f deg/s",
                       current_velocity_world.x(), current_velocity_world.y(),
                       RadToDeg(current_velocity.angle));

  the_logger->AddCircle(goal_.translation, kDistanceThreshold, 1, 0, 0, 1);

  float goal_distance =
      EuclideanDistance(current_pose.translation, goal_.translation);
  if (kDebug_) {
    the_logger->LogPrint("Goal Distance: %0.1f", goal_distance);

    the_logger->LogPrint("Current Velocity (%.5f, %.5f, %.5f deg/s)",
                         current_velocity.translation.x(),
                         current_velocity.translation.y(),
                         RadToDeg(current_velocity.angle));
    the_logger->LogPrint("Current Speed: %.5f",
                         current_velocity.translation.norm());
  }
  //   if (false) {
  if (goal_distance <= kPDPositionThreshold_ &&
      current_speed <= kPDVelocityThreshold_ && use_pd_) {
    PDController* pd_controller = static_cast<PDController*>(
        (*tactic_list_)[TacticIndex::PD_CONTROLLER].get());
    pd_controller->SetGoal(goal_);
    pd_controller->Run();
    LogPositionTrajectoryToFile(
        ControlSequence2D(), ControlSequence1D(),
        shared_state_->GetSharedState(our_robot_index_)->acceleration_command);
    Reset();
    return;
  }

  const Vector2f ntoc_goal = current_pose.translation - goal_.translation;

  if (kDebug_) {
    the_logger->LogPrint("Displacement World Frame (%.5f, %.5f)",
                         -ntoc_goal.x(), -ntoc_goal.y());
  }

  if (!soccer_state_->IsNormalPlay()) {
    motion_model_.v_max = std::min(1050.0f, motion_model_.v_max);
  }

  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);

  state->our_robot_index = our_robot_index_;
  state->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;

  float total_time;
  Pose2Df accel(0, 0, 0);

  ControlSequence2D linear_control;
  if (!translation_complete_) {
    total_time = NTOC2D(ntoc_goal, current_velocity_world, motion_model_,
                        &linear_control, &(state->max_velocity.translation.x()),
                        &(state->max_velocity.translation.y()));

    if (kDebug_) {
      linear_control.LogSequence(the_logger);
    }

    const vector<Vector2f> path_points =
        GetPath(current_pose.translation, current_velocity_world,
                linear_control, 20, total_time);
    the_logger->AddPoints(path_points, 1, 1, 0, 1);
    accel.translation = GetAccelToPreservePosition(
        linear_control, kTransmitPeriodSeconds, current_velocity_world);

    if (total_time <= kTransmitPeriodSeconds) {
      state->should_stop_linear = true;
      the_logger->LogPrint("Commanding a stop for translation");
    }
  } else {
    the_logger->LogPrint("NTOC for translation set to complete");
    state->should_stop_linear = true;
  }
  the_logger->LogPrint("Expected NTOC time: %f\n", total_time);

  if (collect_ntoc_data) {
    fprintf(ntoc_data_fid, "\tEXPECTED TIME TO FINISH: %f\n", total_time);
  }

  ControlSequence1D rotational_control;
  if (!rotation_complete_) {
    float delta_angle = AngleDiff(goal_.angle, current_pose.angle);

    float max_rotational_velocity = kMaxRobotRotVel;

    if (current_speed >= kRotSpeedLimitThreshold) {
      max_rotational_velocity = kLowerRobotRotVel;
    }

    if (relax_angle_ &&
        total_time - kAnglePlannerDelta >
            kAnglePlannerLookahead * kTransmitPeriodSeconds) {
      delta_angle = CalculateAngularWaypoint(
          goal_.angle, current_pose.angle, total_time, kAnglePlannerDelta,
          kAnglePlannerLookahead * kTransmitPeriodSeconds,
          max_rotational_velocity, the_logger);
    }

    float angular_time = TimeOptimalControlAnyFinal1D(
        0.0, current_velocity.angle, delta_angle, 0.0, 0.0, kMaxRobotRotAccel,
        max_rotational_velocity, &rotational_control);
    if (kDebug_) {
      rotational_control.LogSequence(the_logger, false);
    }

    if (angular_time <= kTransmitPeriodSeconds &&
        total_time - kAnglePlannerDelta <=
            kAnglePlannerLookahead * kTransmitPeriodSeconds) {
      state->should_stop_angular = true;
      the_logger->LogPrint("Commanding a stop for rotation");
    } else {
      accel.angle = GetAccelToPreservePosition(
          rotational_control, kTransmitPeriodSeconds, current_velocity.angle);
    }
    state->max_velocity.angle = GetMaxVelocity(
        current_velocity.angle, max_rotational_velocity, rotational_control);
  } else {
    the_logger->LogPrint("1D TOC for Rotation set to complete");
    state->should_stop_angular = true;
  }

  LogPositionTrajectoryToFile(linear_control, rotational_control, accel);

  if (kDebug_) {
    the_logger->LogPrint("Applying Acceleration (%.5f, %.5f, %.5f deg/s^2)",
                         accel.translation.x(), accel.translation.y(),
                         RadToDeg(accel.angle));
    the_logger->LogPrint("Change in speed: %.5f", accel.translation.norm());
  }

  state->acceleration_command = accel;

  the_logger->Pop();

  Reset();
}

void NTOC_Controller::Reset() {
  motion_model_.a_max = kDefaultRobotAcceleration;
  motion_model_.v_max = kDefaultRobotVelocity;

  // Reset angular relaxation
  relax_angle_ = false;
  translation_complete_ = false;
  rotation_complete_ = false;
  use_pd_ = kDefaultUsePD_;
  static_cast<PDController*>((*tactic_list_)[TacticIndex::PD_CONTROLLER].get())
      ->Reset();
}

void NTOC_Controller::Init() {}

void NTOC_Controller::SetGoal(const pose_2d::Pose2Df& pose) { goal_ = pose; }

void NTOC_Controller::SetMotionModel(MotionModel motion_model) {
  motion_model_ = motion_model;
}

void NTOC_Controller::TurnOffAngleRelaxation() { relax_angle_ = false; }

void NTOC_Controller::TurnOnAngleRelaxation() { relax_angle_ = true; }

void NTOC_Controller::SetTranslationComplete() {
  translation_complete_ = true;
  PDController* pd_controller = static_cast<PDController*>(
      (*tactic_list_)[TacticIndex::PD_CONTROLLER].get());
  pd_controller->SetTranslationComplete();
}

void NTOC_Controller::SetRotationComplete() {
  rotation_complete_ = true;
  PDController* pd_controller = static_cast<PDController*>(
      (*tactic_list_)[TacticIndex::PD_CONTROLLER].get());
  pd_controller->SetRotationComplete();
}

void NTOC_Controller::TurnOffPDController() { use_pd_ = false; }

void NTOC_Controller::TurnOnPDController() { use_pd_ = true; }

}  // namespace tactics

// Copyright 2017 - 2019 dbalaban@cs.umass.edu
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

#include "tactics/tsocs_controller.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"

STANDARD_USINGS;
using datastructures::OptionalValueMutable;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using Eigen::Matrix2f;
using math_util::AngleMod;
using geometry::EuclideanDistance;
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
using tsocs::Tsocs;
using state::SharedState;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using motion::MotionModel;
using math_util::Sign;

extern ScopedFile tsocs_data_fid;

namespace tactics {

TSOCSController::TSOCSController(const WorldState& world_state,
                                   TacticArray* tactic_list,
                                   SharedState* shared_state,
                                   OurRobotIndex our_robot_index,
                                   state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state),
      goal_pos_(0, Vector2f(0, 0)),
      goal_vel_(0, Vector2f(0, 0)),
      params_(SolutionParameters(1, 1, 1, 1, 1, 1)),
      motion_model_(kDefaultRobotAcceleration, kDefaultRobotVelocity) {
  params_.isInitialized = false;
}

void TSOCSController::GetPathPoints(vector<Vector2f>* points) {
  *points = tsocs.GetPath(20, params_);
}

bool TSOCSController::HasNans() {
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
        std::isnan(goal_pos_.translation.x()) ||
        std::isnan(goal_pos_.translation.y()) ||
        std::isnan(goal_vel_.translation.x()) ||
        std::isnan(goal_vel_.translation.y()) ||
        world_state_.GetOurRobotPosition(our_robot_index_).confidence == 0)) {
    return false;
  }
  the_logger->LogPrint("Received nan values");
  the_logger->LogPrint("Params: a=%f,b=%f,c=%f,d=%f,T=%f", params_.a, params_.b,
                       params_.c, params_.d, params_.T);

  if (std::isnan(current_pose.angle)) {
    the_logger->LogPrint("Current pose angle is nan");
  }
  if (std::isnan(current_pose.translation.x()) ||
      std::isnan(current_pose.translation.y())) {
    the_logger->LogPrint("Current pose is (%f, %f)",
                         current_pose.translation.x(),
                         current_pose.translation.y());
  }
  if (std::isnan(current_velocity.angle) ||
      std::isnan(current_velocity.translation.x()) ||
      std::isnan(current_velocity.translation.y())) {
    the_logger->LogPrint(
        "Current velocity is (%f, %f, %f)", current_velocity.translation.x(),
        current_velocity.translation.y(), current_velocity.angle);
  }
  if (world_state_.GetOurRobotPosition(our_robot_index_).confidence == 0) {
    the_logger->LogPrint("Confidence = 0");
  }

  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);

  state->our_robot_index = our_robot_index_;
  state->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;
  state->velocity_x = 0.0;
  state->velocity_y = 0.0;
  state->velocity_r = 0.0;

  return true;
}

void TSOCSController::Run() {
  logger::Logger* the_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  the_logger->LogPrint("TSOCS Controller");
  the_logger->Push();

  if (kDebug_) {
    the_logger->LogPrint("Current World Time: %f", world_state_.world_time_);
    the_logger->LogPrint("Expected Total Time to Complettion: %f", params_.T);
  }

  const Pose2Df& current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  const Pose2Df& current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;

  if (HasNans()) {
    return;
  }

  the_logger->LogPrint("Running TSOCS", "");
  the_logger->LogPrint("Goal Pose (%.5f, %.5f, %.5f)",
                       goal_pos_.translation.x(),
                       goal_pos_.translation.y(),
                       goal_pos_.angle);
  the_logger->LogPrint("Goal Vel (%.5f, %.5f, %.5f)",
                       goal_vel_.translation.x(),
                       goal_vel_.translation.y(),
                       goal_vel_.angle);
  the_logger->AddCircle(goal_pos_.translation, kDistanceThreshold, 1, 0, 0, 1);

  const Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);
  const Eigen::Rotation2Df world_to_robot_rotation(-(current_pose.angle
    + current_velocity.angle * kControlPeriodRotation / 2.0f));
  const Vector2f current_velocity_world =
      robot_to_world_rotation * current_velocity.translation;

  if (kDebug_) {
    the_logger->LogPrint("Current Pose (%.5f, %.5f, %.5f)",
                         current_pose.translation.x(),
                         current_pose.translation.y(), current_pose.angle);

    the_logger->LogPrint(
        "Goal Distance: %0.1f",
        EuclideanDistance(current_pose.translation, goal_pos_.translation));

    the_logger->LogPrint("Current Velocity (%.5f, %.5f, %.5f)",
                         current_velocity_world.x(), current_velocity_world.y(),
                         RadToDeg(current_velocity.angle));
    the_logger->LogPrint("Current Speed: %.5f",
                         current_velocity.translation.norm());
  }

  the_logger->LogPrint("TSOCS");
  the_logger->Push();
  const auto start_clock = GetMonotonicTime();
  tsocs = Tsocs(current_pose.translation.cast<double>(),
                current_velocity_world.cast<double>(),
                goal_pos_.translation.cast<double>(),
                goal_vel_.translation.cast<double>(),
                motion_model_.a_max);
  const bool success = tsocs.GetSolution(&params_, the_logger);
  const auto end_clock = GetMonotonicTime();
  const double sol_time = 1000 * (end_clock - start_clock);
  the_logger->Pop();
  the_logger->LogPrint("Copmuting time: %f ms", sol_time);
  if (!success) {
    the_logger->LogPrint("Finding Solution Failed");
  }
  if (kCollectTSOCSData) {
    fprintf(tsocs_data_fid, "\tCURRENT STATE: %f, %f, %f, %f\n",
      current_pose.translation.cast<double>().x(),
      current_pose.translation.cast<double>().y(),
      current_velocity_world.cast<double>().x(),
      current_velocity_world.cast<double>().y());

    const Pose2Df observed_pose =
        world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;
    fprintf(tsocs_data_fid, "\tOBSERVED POSE: %f, %f\n",
      observed_pose.translation.cast<double>().x(),
      observed_pose.translation.cast<double>().y());
    fprintf(tsocs_data_fid, "\tOBSERVATION TIME: %f\n",
            world_state_.GetOurRobotPosition(our_robot_index_).observed_time);
    fprintf(tsocs_data_fid, "\tCURRENT TIME: %f\n", world_state_.world_time_);
    fprintf(tsocs_data_fid, "\tTIME TAKEN TO SOLVE (MS): %f\n", sol_time);
    fprintf(tsocs_data_fid,
      "\tPARAMS: %f %f %f %f %f\n", params_.a, params_.b, params_.c, params_.d,
      params_.T);
    fprintf(tsocs_data_fid, "\tlog_10(cost): %f\n", log10(params_.cost));
    Vector2d xfinal;
    Vector2d vfinal;
    tsocs.GetState(&xfinal, &vfinal, params_.T, params_);
    fprintf(tsocs_data_fid, "\tExpected X dist: %f\n",
      static_cast<double>
      ((goal_pos_.translation.cast<double>() - xfinal).norm()));
    fprintf(tsocs_data_fid, "\tExpected V dist: %f\n",
      static_cast<double>
      ((goal_vel_.translation.cast<double>() - vfinal).norm()));
    fprintf(tsocs_data_fid, "\tTRAJECTORY:\n");
    double total_time = 0;
    const double dt = 1.0 / kTransmitFrequency;
    while (total_time - dt < params_.T) {
      Eigen::Vector2d x_cur(0, 0);
      Eigen::Vector2d v_cur(0, 0);
      tsocs.GetState(&x_cur, &v_cur, total_time, params_);
      fprintf(tsocs_data_fid,
        "\t\t%f %f %f %f\n", x_cur.x(), x_cur.y(), v_cur.x(), v_cur.y());
      total_time += dt;
    }
  }

  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  if (params_.T < 1.0 / kTransmitFrequency) {
    finished_ = true;
    the_logger->LogPrint("Expecting negative time next timestep (%f), finished",
      params_.T);
    state->acceleration_command = Pose2Df({0, 0, 0});
    return;
  } else {
    if (kDebug_) {
      Vector2d xfinal, vfinal;
      tsocs.GetState(&xfinal, &vfinal, params_.T, params_);
      the_logger->LogPrint("Expected X dist: %f",
        (goal_pos_.translation.cast<double>() - xfinal).norm());
      the_logger->LogPrint("Expected V dist: %f",
        (goal_vel_.translation.cast<double>() - vfinal).norm());
      the_logger->LogPrint("Final Pos: (%f, %f)", xfinal.x(), xfinal.y());
      the_logger->LogPrint("Final Vel: (%f, %f)", vfinal.x(), vfinal.y());
    }
    Vector2d xt, vt;
    tsocs.GetState(&xt, &vt, kControlPeriodTranslation, params_);

    the_logger->LogPrint("Commanding World Velocity: (%f, %f)", vt.x(), vt.y());

    vector<Vector2f> path_points =
        tsocs.GetPath(20, params_);
    the_logger->AddPoints(path_points, 1, 1, 0, 1);

    Pose2Df desired_velocity;
    desired_velocity.translation = world_to_robot_rotation * vt.cast<float>();
    // Compute angular velocity that will get us to 0 orientation
    ControlSequence1D rotational_control;
    double angular_time =
      TimeOptimalControlAnyFinal1D(current_pose.angle, current_velocity.angle,
      goal_pos_.angle, goal_vel_.angle,
      0.0, kMaxRobotRotAccel, kMaxRobotRotVel,
      &rotational_control);
    double angular_accel;
    if (angular_time >= kControlPeriodRotation) {
      angular_accel = GetAverageAccel(rotational_control,
                                      kControlPeriodRotation);
    } else {
      angular_accel = GetAccelToPreservePosition(rotational_control,
                                                 kControlPeriodRotation,
                                                 current_velocity.angle);
    }

    state->acceleration_command.translation =
      tsocs.GetAccelVector(0, params_).cast<float>();
    state->acceleration_command.angle = angular_accel;
    the_logger->LogPrint("Commanding acceleration: (%.5f, %.5f, %.5f deg/s^2 )",
                        state->acceleration_command.translation.x(),
                        state->acceleration_command.translation.y(),
                        RadToDeg(state->acceleration_command.angle));

    finished_ = false;
  }

  the_logger->Pop();
}

void TSOCSController::Reset() {
  params_.isInitialized = false;

  // Reset motion model to default
  motion_model_.a_max = kDefaultRobotAcceleration;
  motion_model_.v_max = kDefaultRobotVelocity;
}

void TSOCSController::Init() {
  params_.isInitialized = false;
}

void TSOCSController::SetGoal(const pose_2d::Pose2Df& pose) {
  goal_pos_ = pose;
  goal_vel_ = pose_2d::Pose2Df(0.0, Vector2f(0.0, 0.0));
  params_.isInitialized = false;
}

void TSOCSController::SetGoal(const pose_2d::Pose2Df& pose,
                               const pose_2d::Pose2Df& vel) {
  goal_pos_ = pose;
  goal_vel_ = vel;
}

void TSOCSController::SetMotionModel(MotionModel motion_model) {
  motion_model_ = motion_model;
}

}  // namespace tactics

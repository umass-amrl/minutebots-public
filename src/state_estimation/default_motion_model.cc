// Copyright 2018 slane@cs.umass.edu
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

#include "state_estimation/default_motion_model.h"


#include "state_estimation/motion_model.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using logger::Logger;
using pose_2d::Pose2Df;

namespace motion_model {

DefaultMotionModel::DefaultMotionModel()
    : MotionModel() {}

DefaultMotionModel::DefaultMotionModel(SSLVisionId id, bool is_ours)
    : MotionModel(id, is_ours) {}

void DefaultMotionModel::UpdateInternalParams() {}


void DefaultMotionModel::Predict(double delta_t,
                                 const Vector6d& current_state,
                                 Vector6d* next_state,
                                 Matrix6d* jacobian,
                                 Logger* logger) const {
  Matrix6d velocity_jacobian = MatrixXd::Identity(6, 6);
  velocity_jacobian.bottomRightCorner(3, 3) = MatrixXd::Identity(3, 3);

  Vector6d state(current_state);

  const double x = state(0);
  const double y = state(1);
  const double theta = state(2);
  const double v_x = state(3);
  const double v_y = state(4);
  const double omega = state(5);

  const double cosine = cos(theta);
  const double sine = sin(theta);
  const double cosine_time = cos(omega * delta_t + theta);
  const double sine_time = sin(omega * delta_t + theta);

  const double cosine_diff = cosine_time - cosine;
  const double sine_diff = sine_time - sine;

  if (is_ours_) {
    if (fabs(omega) > kAngularVelocityThreshold) {
      state(0) = x + (v_x * sine_diff + v_y * cosine_diff) / omega;
      state(1) = y + (-v_x * cosine_diff + v_y * sine_diff) / omega;

      velocity_jacobian(0, 2) = (v_x * cosine_diff - v_y * sine_diff) / omega;
      velocity_jacobian(0, 3) = sine_diff / omega;
      velocity_jacobian(0, 4) = cosine_diff / omega;
      velocity_jacobian(0, 5) =
          (v_x * sine_diff - v_y * cosine_diff) / Sq(omega) +
          (delta_t / omega) * (v_x * cosine_time - v_y * sine_time);

      velocity_jacobian(1, 2) = (v_x * sine_diff + v_y * cosine_diff) / omega;
      velocity_jacobian(1, 3) = -cosine_diff / omega;
      velocity_jacobian(1, 4) = sine_diff / omega;
      velocity_jacobian(1, 5) =
          (v_x * cosine_diff - v_y * sine_diff) / Sq(omega) +
          (delta_t / omega) * (v_x * sine_time + v_y * cosine_time);
    } else {
      state(0) = x + delta_t * (v_x * cosine - v_y * sine);
      state(1) = y + delta_t * (v_x * sine + v_y * cosine);

      velocity_jacobian(0, 2) = -delta_t * (v_x * sine + v_y * cosine);
      velocity_jacobian(0, 3) = delta_t * cosine;
      velocity_jacobian(0, 4) = -delta_t * sine;

      velocity_jacobian(1, 2) = delta_t * (v_x * cosine - v_y * sine);
      velocity_jacobian(1, 3) = delta_t * sine;
      velocity_jacobian(1, 4) = delta_t * cosine;
    }
  } else {
    state(0) = x + delta_t * v_x;
    state(1) = y + delta_t * v_y;
    velocity_jacobian(0, 3) = delta_t;
    velocity_jacobian(1, 4) = delta_t;
  }

  state(2) = AngleMod(omega * delta_t + theta);
  velocity_jacobian(2, 5) = delta_t;

  *next_state = state;
  *jacobian = velocity_jacobian;
}

void DefaultMotionModel::Predict(double delta_t,
                                 const Vector6d& current_state,
                                 const pose_2d::Pose2Dd command,
                                 Vector6d* next_state,
                                 Matrix6d* jacobian,
                                 Logger* logger) const {
  // velocity expected is weighted sum of command and current velocity
  // for perfect command actuation use kAlpha_ = 1.0
  if (is_ours_) {
    Vector3d expected_velocity;
    expected_velocity(0) =
        (1.0 - kAlpha_) * current_state(3) + kAlpha_ * command.translation.x();
    expected_velocity(1) =
        (1.0 - kAlpha_) * current_state(4) + kAlpha_ * command.translation.y();
    expected_velocity(2) =
        (1.0 - kAlpha_) * current_state(5) + kAlpha_ * command.angle;

    // the state in which the motion model should calculate is the current
    // position with the average velocity between time steps
    Vector6d calculation_state(current_state);
    //   calculation_state(3) = (expected_velocity(0) + current_state(3)) / 2.0;
    //   calculation_state(4) = (expected_velocity(1) + current_state(4)) / 2.0;
    //   calculation_state(5) = (expected_velocity(2) + current_state(5)) / 2.0;
    calculation_state(3) = expected_velocity(0);
    calculation_state(4) = expected_velocity(1);
    calculation_state(5) = expected_velocity(2);

    Predict(delta_t, calculation_state, next_state, jacobian, logger);

    jacobian->bottomRightCorner(3, 3) = (1.0 - kAlpha_) *
                                        MatrixXd::Identity(3, 3);

    // set output_state velocity to expected value
    (*next_state)(3) = command.translation.x();
    (*next_state)(4) = command.translation.y();
    (*next_state)(5) = command.angle;
  } else {
    Predict(delta_t, current_state, next_state, jacobian, logger);
  }
}

void DefaultMotionModel::ConvertToRobotFrame(
    float delta_t,
    float current_angle,
    float current_omega,
    float angular_accel,
    const Pose2Df& world_velocity,
    Pose2Df* robot_velocity,
    Logger* logger) const {
  // World velocities
  const float dt = kConversionConstant * delta_t;

  const Eigen::Rotation2Df world_to_robot_rotation(
    -(current_angle + current_omega*dt + angular_accel * Sq(dt)/2.0));

  robot_velocity->translation = world_to_robot_rotation *
                                world_velocity.translation;

  robot_velocity->angle = world_velocity.angle;
}


}  // namespace motion_model

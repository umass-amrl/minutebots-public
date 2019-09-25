// Copyright 2018 - 2019 dbalaban@cs.umass.edu
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

#ifndef SRC_EXPERIMENTAL_SIM_SIM_MOTION_MODEL_H_
#define SRC_EXPERIMENTAL_SIM_SIM_MOTION_MODEL_H_

#include <random>

#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"
#include "obstacles/robot_obstacle.h"
#include "radio_protocol_wrapper.pb.h"
#include "experimental_sim/objects/ball.h"
#include "state/team.h"
#include "configuration_reader/reader.h"

namespace experimental_simulator {

class SimMotionModel {
 public:
  SimMotionModel();

  ~SimMotionModel() = default;

  void MoveRobot(const pose_2d::Pose2Df velocity_command,
                 const pose_2d::Pose2Df current_velocity,
                 const float time_slice,
                 pose_2d::Pose2Df* translation_pose,
                 pose_2d::Pose2Df* end_velocity);

 private:
  // minimum command velocity in mm/s to overcome static friction
  const float kMinimumVelocity_ = 0.0;

  const float kSimMaximumVelocity_ = kMaxRobotVelocity;
  const float kSimMaximumAcceleration_ = kMaxRobotAcceleration;

  // standard deviation of linear velocity
  // as proportion of commanded linear velocity magnitude
  const float kLinearActivationNoise_ =
    configuration_reader::CONFIG_linear_activation_noise;

  // standard deviation of angular velocity
  // as proportion of commanded angular velocity magnitude
  const float kAngularActivationNoise_ =
    configuration_reader::CONFIG_angular_activation_noise;

  // standard deviation of angular velocity
  // as proportion of linear velocity magnitude
  const float kAngularDeviationNoise_ =
    configuration_reader::CONFIG_angular_deviation_noise;

  // angle of bias in robot activation in rad
  const float kActivationDriftAngle_ =
    configuration_reader::CONFIG_activation_drift_angle;

  // weight for angle bias
  const float kDriftAngleWeight_ =
    configuration_reader::CONFIG_drift_angle_weight;

  const bool kUseActivationNoise_ = true;
  const bool kUseActivationDrift_ = true;

  std::default_random_engine generator;
  std::normal_distribution<float> linVelActNoise;
  std::normal_distribution<float> angVelActNoise;
  std::normal_distribution<float> angVelDevNoise;

  const unsigned int random_seed = 1;
};

}  // namespace experimental_simulator
#endif  // SRC_EXPERIMENTAL_SIM_SIM_MOTION_MODEL_H_

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

#ifndef SRC_STATE_ESTIMATION_DEFAULT_MOTION_MODEL_H_
#define SRC_STATE_ESTIMATION_DEFAULT_MOTION_MODEL_H_

#include "constants/typedefs.h"
#include "logging/logger.h"
#include "math/poses_2d.h"
#include "state_estimation/motion_model.h"

namespace motion_model {
class DefaultMotionModel : public MotionModel {
 public:
  DefaultMotionModel();
  DefaultMotionModel(SSLVisionId id, bool is_ours);
  ~DefaultMotionModel() = default;

  void UpdateInternalParams();

  void Predict(double delta_t, const Vector6d& current_state,
               Vector6d* next_state, Matrix6d* jacobian,
               logger::Logger* logger) const;

  void Predict(double delta_t, const Vector6d& current_state,
               const pose_2d::Pose2Dd command, Vector6d* next_state,
               Matrix6d* jacobian, logger::Logger* logger) const;

  // Convert from world frame to robot frame in a more intelligent manner
  void ConvertToRobotFrame(float delta_t, float current_angle,
                           float current_omega, float angular_acceleration,
                           const pose_2d::Pose2Df& world_velocity,
                           pose_2d::Pose2Df* robot_velocity,
                           logger::Logger* logger) const;

 private:
  constexpr static double kAlpha_ = 0.8;
  //   constexpr static double kAlpha_ = 1.0;

  constexpr static double kAngularVelocityThreshold = DegToRad(1.5);
  constexpr static float kConversionConstant = 0.5;
};
}  // namespace motion_model

#endif  // SRC_STATE_ESTIMATION_DEFAULT_MOTION_MODEL_H_

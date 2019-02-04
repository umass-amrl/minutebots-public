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


#ifndef SRC_STATE_ESTIMATION_MOTION_MODEL_H_
#define SRC_STATE_ESTIMATION_MOTION_MODEL_H_

#include "constants/typedefs.h"
#include "logging/logger.h"
#include "math/poses_2d.h"

namespace motion_model {
class MotionModel {
 public:
  MotionModel();
  MotionModel(SSLVisionId id, bool is_ours);
  virtual ~MotionModel() = default;

  // Sets the internal SSL Vision ID.
  // Takes as input the ID number and whether the robot is ours or not
  // Calls UpdateInternalParams if either of those values have changed
  void SetSSLVisionID(SSLVisionId id, bool is_ours);

  // This function is called to update any parameters that vary between robots
  // It is called in SetSSLVisionID after the ID and Team are set iff and
  // only if the ID or team has changed.
  virtual void UpdateInternalParams() = 0;

  // Given a delta_t > 0, and the current state, predicts the next state
  // NOTE the linear portions of current_state are in units of meters rather
  // than millimeters
  virtual void Predict(double delta_t,
                       const Vector6d& current_state,
                       Vector6d* next_state,
                       Matrix6d* jacobian,
                       logger::Logger* logger) const = 0;

  // Given a delta_t > 0, the current state, and a command, predicts the
  // next state
  // NOTE the linear portions of current_state are in units of meters rather
  // than millimeters
  virtual void Predict(double delta_t,
                       const Vector6d& current_state,
                       const pose_2d::Pose2Dd command,
                       Vector6d* next_state,
                       Matrix6d* jacobian,
                       logger::Logger* logger) const = 0;

  float GetMaxLinearVel();
  float GetMaxLinearAccel();
  float GetMaxRotVel();
  float GetMaxRotAccel();

 protected:
  // Note, when setting these, please do not exceed kMaxRobotVelocity and
  // kMaxRobotAcceleration as defined in constants.cc
  float max_linear_velocity_;
  float max_linear_acceleration_;
  // Note, when setting these, please do not exceed kMaxRobotRotVel and
  // kMaxRobotRotAccel as defined in constants.cc
  float max_angular_velocity_;
  float max_angular_acceleration_;

  SSLVisionId id_;
  bool is_ours_;
};
}  // namespace motion_model

#endif  // SRC_STATE_ESTIMATION_MOTION_MODEL_H_

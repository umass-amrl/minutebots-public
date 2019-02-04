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

#include "state_estimation/motion_model.h"

namespace motion_model {
MotionModel::MotionModel()
    : max_linear_velocity_(kMaxRobotVelocity),
      max_linear_acceleration_(kMaxRobotAcceleration),
      max_angular_velocity_(kMaxRobotRotVel),
      max_angular_acceleration_(kMaxRobotRotAccel),
      id_(0),
      is_ours_(true) {}

MotionModel::MotionModel(SSLVisionId id, bool is_ours)
  : max_linear_velocity_(kMaxRobotVelocity),
    max_linear_acceleration_(kMaxRobotAcceleration),
    max_angular_velocity_(kMaxRobotRotVel),
    max_angular_acceleration_(kMaxRobotRotAccel),
    id_(id),
    is_ours_(is_ours) {}

void MotionModel::SetSSLVisionID(SSLVisionId id, bool is_ours) {
  bool robot_changed = ((id != id_) || (is_ours_ != is_ours));

  id_ = id;
  is_ours_ = is_ours;

  if (robot_changed)
    UpdateInternalParams();
}


float MotionModel::GetMaxLinearVel() {
  return max_linear_velocity_;
}

float MotionModel::GetMaxLinearAccel() {
  return max_linear_acceleration_;
}

float MotionModel::GetMaxRotVel() {
  return max_angular_velocity_;
}

float MotionModel::GetMaxRotAccel() {
  return max_angular_acceleration_;
}
}  // namespace motion_model

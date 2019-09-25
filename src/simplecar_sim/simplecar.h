// Copyright 2019 jaholtz@cs.umass.edu
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

#ifndef SRC_SIMPLECAR_SIM_SIMPLECAR_H_
#define SRC_SIMPLECAR_SIM_SIMPLECAR_H_

#include <random>

#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"

const float kCarLength = 750.0;
const float kCarWidth = 375.0;
const float kMaxAccel = 0.001;
const float kMaxSpeed = 1.0;

namespace simplecar {

class Simplecar {
 public:
//   Simplecar() = delete;
  Simplecar(int id,
            const pose_2d::Pose2Df& initial_pose,
            const float& current_speed,
            const float& time_slice_length);

  Simplecar(const Simplecar& car);

  explicit Simplecar(const int& id);

  ~Simplecar() = default;

  void Update();

  const int GetId() const;

  const float GetMaxTranslation() const;

  const float GetMaxAcceleration() const;

  const pose_2d::Pose2Df& GetPose() const;

  const Eigen::Vector2f& GetVelocity() const;

  const float& GetSpeed() const;

  const float& GetTime() const;

  const float& GetAngleVel() const;

  void AccelToSpeed(const float& speed);
  void SteerToAngle(const float& angle);

 private:
  int id_;
  pose_2d::Pose2Df pose_;  // Location in mm, angle -PI to PI radians.
  Eigen::Vector2f velocity_;
  float rotational_velocity_;
  float speed_;
  float steer_angle_;
  float time_slice_length_;
  void UpdateVelocity();
};

}  // namespace simplecar
#endif  // SRC_SIMPLECAR_SIM_SIMPLECAR_H_

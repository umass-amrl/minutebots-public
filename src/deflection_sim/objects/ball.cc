// Copyright 2016 - 2017 kvedder@umass.edu
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

#include "deflection_sim/objects/ball.h"

#include <algorithm>
#include "shared/common_includes.h"
#include "util/colorize.h"

using colorize::ColorGreen;
using colorize::ColorBlue;
using colorize::ColorCyan;
using Eigen::Vector2f;
using pose_2d::Pose2Df;
using std::min;

namespace simulator {

Ball::Ball(const Pose2Df initial_pose, const Vector2f initial_velocity,
           const float mass, const float time_slice_length,
           const float friction_coefficient)
    : pose(initial_pose),
      velocity(initial_velocity),
      mass(mass),
      time_slice_length(time_slice_length),
      friction_coefficient(friction_coefficient) {}

Ball::~Ball() {}

const Pose2Df& Ball::GetPose2Df() const { return pose; }

const Vector2f& Ball::GetVelocity() const { return velocity; }

void Ball::SetVelocity(const Vector2f& new_velocity) {
  velocity = new_velocity;
}

void Ball::SetPose(const Vector2f& new_pose) {
  Pose2Df vector_as_pose(0, new_pose);
  pose = vector_as_pose;
}

void Ball::RegisterKick(const Kick& velocity_command) {
  kick_attempts.push_back(velocity_command);
}

void Ball::UpdatePosition() {
  // Handle friction.
  // Decel = F of friction / mass = (cof * g * mass) / mass = cof * g
  float friction_decel = (mass == 0.0f) ? 0 : 9.81f * friction_coefficient;
  // Handle the case in which the ball isn't moving (avoids '-nan')
  if (!velocity.isZero(1e-6)) {
    velocity = velocity.normalized() *
               (velocity.norm() -
                static_cast<float>(min(velocity.norm(), friction_decel)));
  }

  Vector2f translation_vector(velocity.x() * 1000 * time_slice_length,
                              velocity.y() * 1000 * time_slice_length);

  Pose2Df translation_pose(0, translation_vector);

  pose.ApplyPose(translation_pose);
}

}  // namespace simulator

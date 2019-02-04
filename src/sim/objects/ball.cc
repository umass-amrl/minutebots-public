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

#include "sim/objects/ball.h"

#include <algorithm>
#include "sim/worldstate.h"
#include "util/colorize.h"

using colorize::ColorGreen;
using colorize::ColorBlue;
using colorize::ColorCyan;
using Eigen::Vector2f;
using pose_2d::Pose2Df;
using std::min;

namespace simulator {

Ball::Ball(const WorldState& world_state, const Pose2Df initial_pose,
           const Vector2f initial_velocity, const float mass,
           const float time_slice_length, const float friction_coefficient)
    : world_state(world_state),
      pose(initial_pose),
      velocity(initial_velocity),
      mass(mass),
      time_slice_length(time_slice_length),
      friction_coefficient(friction_coefficient) {}

Ball::~Ball() {}

const Pose2Df& Ball::GetPose2Df() const { return pose; }

const Vector2f& Ball::GetVelocity() const { return velocity; }

void Ball::RegisterKick(const Kick& velocity_command) {
  kick_attempts.push_back(velocity_command);
}

void Ball::UpdatePosition() {
  // Handle kicks.
  for (const Kick& kick : kick_attempts) {
    switch (kick.kick_type) {
      case Kick::CHIP: {
        LOG(INFO) << "Chips kicks are not currently supported!";
      } break;
      case Kick::FLAT:  // Fallthrough intentional.
      default: {
        // TODO(kvedder): Handle kick scaling/dampening.
        velocity += kick.kick_addition;
      } break;
    }
  }
  kick_attempts.clear();

  // Handle friction.
  // Decel = F of friction / mass = (cof * g * mass) / mass = cof * g
  const float friction_decel =
      (mass == 0.0f) ? 0 : 9.81f * friction_coefficient;
  // Handle the case in which the ball isn't moving (avoids '-nan')
  if (!velocity.isZero(1e-6)) {
    velocity = velocity.normalized() *
               (velocity.norm() - std::min(velocity.norm(), friction_decel));
  }

  Vector2f translation_vector = velocity * time_slice_length;

  Pose2Df new_pose = pose;
  new_pose.ApplyPose({0, translation_vector});
  for (const auto& robot : world_state.GetRobots()) {
    const auto& robot_obstacle = robot.GetObstacle();
    if (robot_obstacle.LineCollision(pose.translation, new_pose.translation,
                                     0)) {
      const Vector2f initial_line = new_pose.translation - pose.translation;
      const float total_distance =
          (new_pose.translation - pose.translation).norm();
      Vector2f free_point(0, 0);
      float distance_from_start = 0;
      robot_obstacle.FurthestFreePointOnLine(pose.translation,
                                             new_pose.translation, &free_point,
                                             &distance_from_start, 0);
      const float leftover_distance = total_distance - distance_from_start;

      const Vector2f& robot_center = robot_obstacle.GetPose().translation;
      const Vector2f robot_center_to_contact =
          (free_point - robot_center).normalized();
      const Vector2f reflection_vector =
          (initial_line -
           2 * (robot_center_to_contact.dot(initial_line)) *
               robot_center_to_contact)
              .normalized() *
          leftover_distance;
      translation_vector = reflection_vector;
      velocity = velocity.norm() * reflection_vector.normalized();
      break;
    }
  }
  pose.ApplyPose({0, translation_vector});
}

}  // namespace simulator

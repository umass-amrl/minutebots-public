// Copyright 2016-2017 kvedder@umass.edu
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

#ifndef SRC_SIM_OBJECTS_BALL_H_
#define SRC_SIM_OBJECTS_BALL_H_

#include <glog/logging.h>
#include <vector>

#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"

namespace simulator {

// Forward declare WorldState so that we can get a reference to it without
// including a header.
class WorldState;

struct Kick {
  const enum KickType { CHIP, FLAT } kick_type;
  Eigen::Vector2f kick_addition;

  Kick(KickType kick_type, Eigen::Vector2f kick_addition)
      : kick_type(kick_type), kick_addition(kick_addition) {}

  ~Kick() {}
};

class Ball {
 public:
  Ball(const WorldState& world_state, const pose_2d::Pose2Df initial_pose,
       const Eigen::Vector2f initial_velocity, const float mass,
       const float time_slice_length, const float friction_coefficient);

  ~Ball();

  const pose_2d::Pose2Df& GetPose2Df() const;

  const Eigen::Vector2f& GetVelocity() const;

  void RegisterKick(const Kick& kick_attempt);

  void UpdatePosition();

 private:
  const WorldState& world_state;
  pose_2d::Pose2Df pose;
  Eigen::Vector2f velocity;
  const float mass;
  const float time_slice_length;
  const float friction_coefficient;
  std::vector<Kick> kick_attempts;
};

}  // namespace simulator
#endif  // SRC_SIM_OBJECTS_BALL_H_

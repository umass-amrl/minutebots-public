// Copyright 2016 - 2018 kvedder@umass.edu, jaholtz@cs.umass.edu
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

#ifndef SRC_SIM_OBJECTS_ROBOT_H_
#define SRC_SIM_OBJECTS_ROBOT_H_

#include <random>

#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"
#include "obstacles/robot_obstacle.h"
#include "radio_protocol_wrapper.pb.h"
#include "sim/objects/ball.h"
#include "state/team.h"

namespace simulator {

// Forward declare WorldState so that we can get a reference to it without
// including a header.
class WorldState;

class Robot {
 public:
  Robot() = delete;
  Robot(const WorldState& world_state, pose_2d::Pose2Df initial_pose,
        Eigen::Vector2f current_velocity, int id, team::Team team,
        const float time_slice_length);

  ~Robot() = default;

  bool IsInKickRectangle(const Ball& ball);

  void Update(const RadioProtocolCommand& velocity_command, Ball* ball);

  const Eigen::Vector2f GetBoundedMovement(
      const Eigen::Vector2f& input_command_velocity) const;

  const int GetId() const;

  const team::Team GetTeam() const;

  const float GetMaxTransation() const;

  const float GetMaxAcceleration() const;

  const float GetRobotRadius() const;

  const float GetExtendedRobotRadius() const;

  const pose_2d::Pose2Df& GetPose() const;

  const obstacle::RobotObstacle& GetObstacle() const;

  const Eigen::Vector2f& GetVelocity() const;

  float rotational_velocity_;

 private:
  const WorldState& world_state;
  pose_2d::Pose2Df pose;  // Location in mm, angle -PI to PI radians.
  Eigen::Vector2f current_velocity_;
  obstacle::RobotObstacle obstacle;
  const int id;
  const team::Team team;
  const float time_slice_length;

  // Sets a deadzone to zero the robot if it's moving below a certian speed.
  const float kIsMovingThreshold = 0.006;
};

}  // namespace simulator
#endif  // SRC_SIM_OBJECTS_ROBOT_H_

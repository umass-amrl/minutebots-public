// Copyright 2016 - 2019 jaholtz@cs.umass.edu
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

#ifndef SRC_EXPERIMENTAL_SIM_OBJECTS_ROBOT_H_
#define SRC_EXPERIMENTAL_SIM_OBJECTS_ROBOT_H_

#include <random>

#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"
#include "obstacles/robot_obstacle.h"
#include "radio_protocol_wrapper.pb.h"
#include "experimental_sim/objects/ball.h"
#include "experimental_sim/sim_motion_model.h"
#include "state/team.h"

namespace experimental_simulator {

// Forward declare WorldState so that we can get a reference to it without
// including a header.
class SimState;

class Robot {
 public:
  Robot() = delete;
  Robot(const SimState& world_state,
        pose_2d::Pose2Df initial_pose,
        pose_2d::Pose2Df current_velocity,
        int id,
        team::Team team,
        const float time_slice_length);

  ~Robot() = default;

  bool IsInKickRectangle(const Ball& ball);

  void Update(const RadioProtocolCommand& velocity_command,
              const float& time_slice,
              Ball* ball);

  void Update(Ball* ball, const float& time_slice);

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

  Eigen::Vector2f CircleReflection(const Vector2f& ball_pose,
                                 const Vector2f& collision,
                                 const Vector2f& relative_velocity);

  Eigen::Vector2f FlatReflection(const Vector2f& line_start,
                               const Vector2f& line_end,
                               const Vector2f& relative_velocity,
                               const bool& hit_kicker,
                               const float& robot_angle,
                               const RadioProtocolCommand& velocity_command);

  void HandleCollision(Ball* ball,
                       const pose_2d::Pose2Df& next_pose,
                       const float& angular_velocity,
                       const Eigen::Vector2f velocity_vector,
                       const RadioProtocolCommand& velocity_command);

  float rotational_velocity_;

 private:
  const SimState& world_state;
  pose_2d::Pose2Df pose;  // Location in mm, angle -PI to PI radians.
  Eigen::Vector2f current_velocity_;
  obstacle::RobotObstacle obstacle;
  SimMotionModel model;
  const int id;
  const team::Team team;
  float time_slice_length;
  const float hardware_drift_x_;
  const float hardware_drift_y_;

  // Sets a deadzone to zero the robot if it's moving below a certian speed.
  const float kIsMovingThreshold = 0.006;
};

}  // namespace experimental_simulator
#endif  // SRC_EXPERIMENTAL_SIM_OBJECTS_ROBOT_H_

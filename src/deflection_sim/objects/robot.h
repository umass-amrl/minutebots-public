// Copyright 2016 - 2018 kvedder@umass.edu
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

#ifndef SRC_DEFLECTION_SIM_OBJECTS_ROBOT_H_
#define SRC_DEFLECTION_SIM_OBJECTS_ROBOT_H_

#include <random>

#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"
#include "radio_protocol_wrapper.pb.h"
#include "deflection_sim/objects/ball.h"

namespace simulator {

class Robot {
 public:
  Robot(pose_2d::Pose2Df initial_pose, Eigen::Vector2f current_velocity, int id,
        const float time_slice_length, const float max_translation,
        const float max_accel, const float radius);

  ~Robot();

  bool IsInKickRectangle(const Ball& ball);

  void Update(const RadioProtocolCommand& velocity_command, Ball* ball);

  const Eigen::Vector2f GetBoundedMovement(
      const Eigen::Vector2f& input_command_velocity) const;

  const int GetId() const;

  const float GetMaxTransation() const;

  const float GetMaxAcceleration() const;

  const float GetRobotRadius() const;

  const float GetExtendedRobotRadius() const;

  const pose_2d::Pose2Df& GetPose() const;

  const Eigen::Vector2f& GetVelocity() const;

  void HandleCollision(Ball* ball,
                       const RadioProtocolCommand& velocity_command);

  Vector2f FlatReflection(const Vector2f& line_start,
                                 const Vector2f& line_end,
                                 const Vector2f& d_robot,
                                 const Vector2f& d_ball,
                                 const bool& hit_kicker,
                                 const float& robot_angle,
                                 const RadioProtocolCommand& velocity_command);

  Eigen::Vector2f CircleReflection(const float& ball_x,
                                   const float& ball_y,
                                   const Eigen::Vector2f& collision,
                                   const Eigen::Vector2f& d_robot,
                                   const Eigen::Vector2f& d_ball);

 private:
  pose_2d::Pose2Df pose;  // Location in mm, angle -PI to PI radians.
  Eigen::Vector2f current_velocity_;
  const int id;
  const float time_slice_length;

  const float kMaxTranslation;   // Meters per sec.
  const float kMaxAcceleration;  // Meters per sec^2.
  const float kRobotRadius;      // mm.
  // Sets a deadzone to zero the robot if it's moving below a certian speed.
  const float kIsMovingThreshold = 0.006;
};

}  // namespace simulator
#endif  // SRC_DEFLECTION_SIM_OBJECTS_ROBOT_H_

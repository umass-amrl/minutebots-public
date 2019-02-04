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

#include "sim/objects/robot.h"

#include <glog/logging.h>
#include <cmath>

#include "constants/constants.h"
#include "eigen3/Eigen/Geometry"
#include "math/math_util.h"
#include "sim/worldstate.h"
#include "util/colorize.h"

using colorize::ColorGreen;
using colorize::ColorBlue;
using colorize::ColorCyan;
using Eigen::Vector2f;
using Eigen::Matrix2f;
using Eigen::Quaternionf;
using math_util::AngleMod;
using pose_2d::Pose2Df;
using std::endl;

extern const double noise_level;
extern std::default_random_engine generator;
extern std::normal_distribution<double> distribution;
extern std::normal_distribution<double> rot_distribution;

namespace simulator {

Robot::Robot(const WorldState& world_state, Pose2Df initial_pose,
             Vector2f initial_velocity, int id, team::Team team,
             const float time_slice_length)
    : world_state(world_state),
      pose(initial_pose),
      current_velocity_(initial_velocity),
      obstacle(pose),
      id(id),
      team(team),
      time_slice_length(time_slice_length) {}

bool Robot::IsInKickRectangle(const Ball& ball) {
  // Constructs a polar rectangle that sits in front of the robot.
  // Radius from the robot the zone starts.
  const float kick_zone_depth = 50;                                   // mm
  const float kick_zone_radius = kRobotRadius - kick_zone_depth / 3;  // mm

  // If the ball is too far away to kick, don't bother with any more checks.
  const Vector2f robot_to_ball_vector =
      ball.GetPose2Df().translation - pose.translation;
  float ball_distance = fabs(robot_to_ball_vector.norm());
  if ((ball_distance < kick_zone_radius) ||
      (ball_distance > (kick_zone_radius + kick_zone_depth))) {
    return false;
  }

  // +/- half of this in the direction the robot is facing.
  const float kick_zone_radians = M_PI / 4;  // rad
  const float acceptable_min_angle =
      AngleMod(pose.angle - kick_zone_radians / 2);
  const float acceptable_max_angle =
      AngleMod(pose.angle + kick_zone_radians / 2);

  // Returns from -PI to PI.
  float robot_to_ball_angle = static_cast<float>(
      atan2(robot_to_ball_vector.y(), robot_to_ball_vector.x()));

  return (robot_to_ball_angle >= acceptable_min_angle &&
          robot_to_ball_angle <= acceptable_max_angle);
}

void Robot::Update(const RadioProtocolCommand& velocity_command, Ball* ball) {
  float velocity_x = velocity_command.velocity_x();
  if (std::isnan(velocity_x)) {
    velocity_x = 0;
  }

  float velocity_y = velocity_command.velocity_y();
  if (std::isnan(velocity_y)) {
    velocity_y = 0;
  }

  // Convert the velocity command to a Vector2f.
  const Vector2f velocity_command_vector(velocity_x, velocity_y);

  // Register any successful kick to the ball.
  if (velocity_command.has_flat_kick() && IsInKickRectangle(*ball)) {
    static constexpr float flat_kick_amount = 5000.0f;  // mm/s
    Vector2f kick_vector(flat_kick_amount * cos(pose.angle),
                         flat_kick_amount * sin(pose.angle));
    ball->RegisterKick(Kick(Kick::FLAT, kick_vector));
  } else if (velocity_command.has_chip_kick() && IsInKickRectangle(*ball)) {
    static constexpr float chip_kick_amount = 5000.0f;  // mm/s
    Vector2f kick_vector(chip_kick_amount * cos(pose.angle),
                         chip_kick_amount * sin(pose.angle));
    ball->RegisterKick(Kick(Kick::CHIP, kick_vector));
  }

  // Bound the movement based on the robot's contraints.
  // Note, this is being removed upon Joydeep's suggestion as it should
  // only be recieving resonable velocity commands. This was done at CMU
  // velocity_command_vector = GetBoundedMovement(velocity_command_vector);

  if ((velocity_command_vector - current_velocity_).norm() >
      kMaxRobotAcceleration * time_slice_length) {
    LOG(WARNING) << "Velocity change is over max accel! Commanded: ("
                 << (velocity_command_vector - current_velocity_).norm()
                 << ") vs Allowed: ("
                 << kMaxRobotAcceleration * time_slice_length << ")";
  }

  // Translate pose based on movement.
  // 1000 scales velocity command from m/s to mm/s.
  // Also simulate noise in the robot's actuation
  Pose2Df translation_pose =
      Pose2Df(velocity_command.velocity_r() * time_slice_length,
            Vector2f(
              (velocity_command_vector.x() * time_slice_length * 1000)
                * distribution(generator),
              (velocity_command_vector.y() * time_slice_length * 1000)
                * distribution(generator)));
  pose.ApplyPose(translation_pose);
  obstacle.UpdatePose(pose);
  // Update the velocity position.
  current_velocity_ = velocity_command_vector;
  rotational_velocity_ = velocity_command.velocity_r();
  static const bool kRotationNoise = false;
  if (kRotationNoise) {
    pose.angle += rot_distribution(generator);
  }
}

const Vector2f Robot::GetBoundedMovement(
    const Vector2f& input_command_velocity) const {
  Vector2f command_velocity = input_command_velocity;

  // Crank down the velocity to the upper bound.
  if (command_velocity.squaredNorm() > Sq(kMaxRobotVelocity)) {
    command_velocity = command_velocity.normalized() * kMaxRobotVelocity;
  }

  Vector2f accel = (command_velocity - current_velocity_) / time_slice_length;

  // Crank down the acceleration to the upper bound.
  if (accel.squaredNorm() > Sq(kMaxRobotAcceleration / time_slice_length)) {
    accel = accel.normalized() * (kMaxRobotAcceleration / time_slice_length);
  }

  command_velocity += accel;

  if (command_velocity.squaredNorm() < Sq(kIsMovingThreshold)) {
    return Vector2f(0, 0);
  }

  return command_velocity;
}

const int Robot::GetId() const { return id; }

const team::Team Robot::GetTeam() const { return team; }

const float Robot::GetMaxTransation() const { return kMaxRobotVelocity; }

const float Robot::GetMaxAcceleration() const { return kMaxRobotAcceleration; }

const float Robot::GetRobotRadius() const { return kRobotRadius; }

const Pose2Df& Robot::GetPose() const { return pose; }

const obstacle::RobotObstacle& Robot::GetObstacle() const { return obstacle; }

const Vector2f& Robot::GetVelocity() const { return current_velocity_; }

}  // namespace simulator

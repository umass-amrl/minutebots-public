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

#include "experimental_sim/objects/robot.h"

#include <glog/logging.h>
#include <cmath>

#include "constants/constants.h"
#include "eigen3/Eigen/Geometry"
#include "math/math_util.h"
#include "experimental_sim/sim_state.h"
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
using geometry::CheckLineLineIntersection;
using geometry::SquaredDistance;
using geometry::RayIntersect;
using std::vector;

extern const double noise_level;
extern std::default_random_engine generator;
extern std::normal_distribution<double> distribution;
// Angles of the start and end of the flat portion of the robot
// const float kicker_angle1 = 1.0708;
// const float kicker_angle2 = 1.98;
const float kicker_angle1 = DegToRad(30.0);
const float kicker_angle2 = DegToRad(330.0);
// Inset of the kicker face
const float kicker_offset = 20;
// const float kicker_offset2 = 30;

const float restitution_coefficient = 0.8;
const float dribbler_restitution = 0.1;

namespace experimental_simulator {

Robot::Robot(const SimState& world_state,
             Pose2Df initial_pose,
             Pose2Df initial_velocity,
             int id,
             team::Team team,
             const float time_slice_length)
    : world_state(world_state),
      pose(initial_pose),
      obstacle(pose),
      model(),
      id(id),
      team(team),
      time_slice_length(time_slice_length),
      hardware_drift_x_(0.0),
      hardware_drift_y_(0.00) {
  current_velocity_ = initial_velocity.translation;
  rotational_velocity_ = initial_velocity.angle;
}

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

Vector2f Robot::FlatReflection(const Vector2f& line_start,
                               const Vector2f& line_end,
                               const Vector2f& relative_velocity,
                               const bool& hit_kicker,
                               const float& robot_angle,
                               const RadioProtocolCommand& velocity_command) {
  Vector2f new_vel;
  float angle = robot_angle;
  if (angle < 0) {
    angle += 2*M_PI;
  }
  const Vector2f heading = geometry::Heading(angle);
  const Vector2f robot_perp = geometry::Perp(heading);
  new_vel = (relative_velocity / 1000);
  // Kick the ball
  if (hit_kicker && velocity_command.has_flat_kick()) {
    const Vector2f kick = velocity_command.flat_kick() * heading;
    const Eigen::Rotation2D<float> rotation(-robot_angle);
    const Vector2f tangential_velocity =
         (robot_perp.dot(dribbler_restitution * new_vel) * robot_perp);
    const Vector2f ball_reflect =
      new_vel - (2 * (new_vel.dot(heading)
      * heading));
    const Vector2f reflected_velocity =
        .5 * ball_reflect;
    Vector2f resultant_vector =
        tangential_velocity + reflected_velocity + kick;
    new_vel = resultant_vector;
    // Run into the ball without kicking it
  } else {
    new_vel = new_vel * dribbler_restitution;
  }
  return new_vel;
}

Vector2f Robot::CircleReflection(const Vector2f& ball_pose,
                                 const Vector2f& collision,
                                 const Vector2f& relative_velocity) {
  Vector2f tangent_vector;
  tangent_vector.y() = -(collision.x() - pose.translation.x());
  tangent_vector.x() = collision.y() - pose.translation.y();
  tangent_vector = tangent_vector.normalized();
  const Vector2f tangent_velocity =
      relative_velocity.dot(tangent_vector) * tangent_vector;
  const Vector2f perpendicular_velocity = relative_velocity - tangent_velocity;
  const Vector2f new_velocity = restitution_coefficient *
      ((-2 * perpendicular_velocity) /1000);
  return new_velocity;
}

Vector2f ClosestPointOnLine(double line1_x, double line1_y, double line2_x,
                            double line2_y, double point_x, double point_y) {
  double a1 = line2_y - line1_y;
  double b1 = line2_x - line1_x;
  double u = ((point_x - line1_x) * b1 + (point_y - line1_y) * a1) /
             (b1 * b1 + a1 * a1);
  Vector2f closest = {0, 0};
  if (u < 0) {
    closest[0] = line1_x;
    closest[1] = line1_y;
  } else if (u > 1) {
    closest[0] = line2_x;
    closest[1] = line2_y;
  } else {
    closest[0] = line1_x + u * b1;
    closest[1] = line1_y + u * a1;
  }
  return closest;
}

void Robot::HandleCollision(Ball* ball,
                            const Pose2Df& next_pose,
                            const float& angular_velocity,
                            const Vector2f velocity_vector,
                            const RadioProtocolCommand& velocity_command) {
  Vector2f ball_velocity = ball->GetVelocity();
  const Eigen::Rotation2D<float> rotation(pose.angle);
  const Vector2f relative_velocity =
      (ball_velocity * 1000) - (rotation * velocity_vector);
  const Vector2f ball_start = ball->GetPose2Df().translation;
  const Vector2f robot_start = pose.translation;
  // TODO(jaholtz) verify that the units are all correct here
  const Vector2f ball_end = ball_start +
      (relative_velocity * time_slice_length);
  // Check for Collision
  Vector2f free_point;
  float distance_from_start;
  if (obstacle.FurthestFreePointOnLine(ball_start,
                                       ball_end,
                                       &free_point,
                                       &distance_from_start,
                                       kBallRadius)) {
    // TODO(jaholtz) consider the edges of the ball as well as the center
    // for the collision point
    const float time_to_point = distance_from_start /
        (relative_velocity.norm());
    const float current_robot_angle = pose.angle
        + (velocity_command.velocity_r() * time_to_point);

    // Calculate angle of collision
    const Vector2f towards_collision = free_point - robot_start;
    float angle = geometry::Angle(towards_collision);
    float pose_angle = pose.angle;
    if (pose_angle < 0) {
      pose_angle += 2*M_PI;
    }
    // Rotate by the robot rotation;
    angle = angle - pose_angle;
    angle = AngleMod(angle);

    if (angle < 0) {
      angle += 2*M_PI;
    }
    // Get the vector of the ball path
    const Vector2f ball_dir = free_point - robot_start;
    // Possible collision within flat portion
    if ((angle <= kicker_angle1 && angle >= 0)
        || angle >= kicker_angle2 ) {
      // The angles for the central region of the flat face.
      const float start_angle = DegToRad(25.0);
      const float end_angle = DegToRad(335.0);

      // Build the Line for the kicker face
      const float kicker_start_x =
          robot_start.x() +
          (kRobotRadius - kicker_offset) *
          cos(start_angle - current_robot_angle);
      const float kicker_start_y =
          robot_start.y() +
          (kRobotRadius - kicker_offset) *
          sin(start_angle - current_robot_angle);
      const float kicker_end_x =
          robot_start.x() +
          (kRobotRadius - kicker_offset) *
          cos(end_angle - current_robot_angle);
      const float kicker_end_y =
          robot_start.y() +
          (kRobotRadius - kicker_offset) *
          sin(end_angle - current_robot_angle);

      // Kicker Face
      const Vector2f kicker_start(kicker_start_x, kicker_start_y);
      const Vector2f kicker_end(kicker_end_x, kicker_end_y);

      // TODO(jaholtz) implement line intersection based kicker face handling,
      // by creating the face and edge lines, rotating them in space, and
      // checking for intersection.
      // Inner Kicker
      if ((angle <= start_angle && angle >= 0)
          || angle >= end_angle ) {
          ball_velocity = FlatReflection(kicker_start,
                                      kicker_end,
                                      relative_velocity,
                                      true,
                                      pose.angle,
                                      velocity_command);
          Vector2f new_pose = free_point;
          new_pose = new_pose + ((1.5* kBallRadius)*ball_dir.normalized());
          ball->SetPose(new_pose);
      } else {
        // TODO(jaholtz) determine if the closest point here is a sufficient
        // collision point
      ball_velocity = FlatReflection(kicker_start,
                                      kicker_end,
                                      relative_velocity,
                                      false,
                                      pose.angle,
                                      velocity_command);
        Vector2f new_pose = free_point;
        new_pose = new_pose + ((kBallRadius)*ball_dir.normalized());
        ball->SetPose(new_pose);
      }
    } else {  // not in special region
      // TODO(jaholtz) determine if the closest point here is a sufficient
      // collision point
      ball_velocity +=
          CircleReflection(ball_start,
                           free_point,
                           relative_velocity);
      Vector2f new_pose = free_point;
      new_pose = new_pose + ((kBallRadius)*ball_dir.normalized());
      ball->SetPose(new_pose);
    }
  }
  ball->SetVelocity(ball_velocity);
}

void Robot::Update(Ball* ball, const float& time_slice) {
  time_slice_length = time_slice;
  Pose2Df translation_pose =
      Pose2Df(rotational_velocity_ * time_slice_length,
              Vector2f(
                (current_velocity_.x() * time_slice_length),
                      (current_velocity_.y() * time_slice_length)));
  RadioProtocolCommand velocity_command;
  HandleCollision(ball,
                  translation_pose,
                  rotational_velocity_,
                  current_velocity_,
                  velocity_command);
}

void Robot::Update(const RadioProtocolCommand& velocity_command,
                   const float& time_slice,
                   Ball* ball) {
  time_slice_length = time_slice;
  float velocity_x = velocity_command.velocity_x();
  // NaN checks, reduce to zero if nan
  if (std::isnan(velocity_x)) {
    velocity_x = 0;
  }

  float velocity_y = velocity_command.velocity_y();
  if (std::isnan(velocity_y)) {
    velocity_y = 0;
  }

  // Convert the velocity command to a Vector2f.
  const Vector2f velocity_command_vector(
      (velocity_x + hardware_drift_x_) * 1000,
      (velocity_y + hardware_drift_y_) * 1000);

  const Pose2Df velocity(velocity_command.velocity_r(),
                         velocity_command_vector);
  const Pose2Df current_velocity(rotational_velocity_, current_velocity_);

  // ---- Moves the robot regardless of collision ------
  // Translate pose based on movement.
  // 1000 scales velocity command from m/s to mm/s.
  Pose2Df translation_pose;
  Pose2Df velocity_pose;

  model.MoveRobot(velocity,
                  current_velocity,
                  time_slice,
                  &translation_pose,
                  &velocity_pose);

  HandleCollision(ball,
                  translation_pose,
                  velocity_command.velocity_r(),
                  velocity_command_vector,
                  velocity_command);

  // Move the robot
  pose.ApplyPose(translation_pose);
  obstacle.UpdatePose(pose);

  // Update the velocities.
  current_velocity_ = velocity_pose.translation;
  rotational_velocity_ = velocity_pose.angle;
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

}  // namespace experimental_simulator

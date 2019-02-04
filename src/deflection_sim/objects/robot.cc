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

#include "deflection_sim/objects/robot.h"

#include <glog/logging.h>
#include <cmath>

#include "constants/constants.h"
#include "eigen3/Eigen/Geometry"
#include "math/geometry.h"
#include "math/math_util.h"
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

// Angles of the start and end of the flat portion of the robot
// const float kicker_angle1 = 1.0708;
// const float kicker_angle2 = 1.98;
const float kicker_angle1 = DegToRad(60.0);
const float kicker_angle2 = DegToRad(120.0);
// Inset of the kicker face
const float kicker_offset = 20;
const float kicker_offset2 = 30;

const float restitution_coefficient = 0.8;

namespace simulator {

Robot::Robot(Pose2Df initial_pose, Vector2f initial_velocity, int id,
             const float time_slice_length, const float max_translation,
             const float max_accel, const float radius)
    : pose(initial_pose),
      current_velocity_(initial_velocity),
      id(id),
      time_slice_length(time_slice_length),
      kMaxTranslation(max_translation),
      kMaxAcceleration(max_accel),
      kRobotRadius(radius) {}

Robot::~Robot() {}

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

Vector2f Robot::CircleReflection(const float& ball_x, const float& ball_y,
                                 const Vector2f& collision,
                                 const Vector2f& d_robot,
                                 const Vector2f& d_ball) {
  Vector2f new_vel;
  const float collisiondist =
      sqrt(pow(ball_x - collision[0], 2) + pow(ball_y - collision[1], 2));
  const float n_x = (ball_x - collision[0]) / collisiondist;
  const float n_y = (ball_y - collision[1]) / collisiondist;
  const float p = 2 * ((d_ball[0] * n_x + d_robot[0] * n_x) +
                       (d_ball[1] * n_y + d_robot[1] * n_y)) /
                  (1 + 1);
  new_vel[0] = (d_ball[0] - p * n_x - p * n_x) / time_slice_length;
  new_vel[1] = (d_ball[1] - p * n_y - p * n_y) / time_slice_length;
  return restitution_coefficient * new_vel;
}

Vector2f Robot::FlatReflection(const Vector2f& line_start,
                               const Vector2f& line_end,
                               const Vector2f& d_robot, const Vector2f& d_ball,
                               const bool& hit_kicker, const float& robot_angle,
                               const RadioProtocolCommand& velocity_command) {
  Vector2f new_vel;
  const Vector2f heading = geometry::Heading(robot_angle);
  new_vel = restitution_coefficient * (d_ball / time_slice_length);
  const Vector2f line_direction = line_end - line_start;
  Vector2f perpendicular = Perp(line_direction);
  const float direction_comparison = perpendicular.dot(new_vel);
  if (direction_comparison > 0) {
    perpendicular = -perpendicular;
  }
  perpendicular.normalize();
  // Kick the ball
  if (hit_kicker && velocity_command.has_flat_kick()) {
    const Vector2f kick = velocity_command.flat_kick() * heading;
    //     const Vector2f kick = 3 * heading;
    new_vel = new_vel + kick;
    // Run into the ball without kicking it
  } else {
    new_vel = new_vel - (2 * (new_vel.dot(heading) * heading));
  }
  return new_vel;
}

// Currently using logic copied from deflection simulator.
// TODO(jaholtz) convert everything to robot frame and update the logic.
// Deflection simulator suffered from haste and poor planning.
void Robot::HandleCollision(Ball* ball,
                            const RadioProtocolCommand& velocity_command) {
  Vector2f ball_velocity = ball->GetVelocity();
  Vector2f ball_velocity_robot_frame = ball_velocity - current_velocity_;
  const Vector2f distance_ball_robot_frame =
      ball_velocity_robot_frame * time_slice_length;
  const Vector2f distance_ball = ball_velocity * time_slice_length;
  // Convert the velocity command to a Vector2f.
  const Vector2f velocity_command_vector(current_velocity_.x(),
                                         current_velocity_.y());
  const Vector2f distance_robot = velocity_command_vector * time_slice_length;
  const Vector2f ball_start = ball->GetPose2Df().translation;
  const Vector2f ball_end = ball_start + distance_ball_robot_frame;
  const Vector2f robot_start = pose.translation;
  // Get the closests points along the paths of the ball to the robot
  Vector2f closest_on_ball_path = ClosestPointOnLine(
      ball_start.x(), ball_start.y(), ball_end.x(), ball_end.y(),
      pose.translation.x() + distance_robot.x(),
      pose.translation.y() + distance_robot.y());
  // Get the vector of the ball path
  const Vector2f ball_dir = closest_on_ball_path - robot_start;
  // Get the distance from the robot center to the closest point
  const float dist_ball_point = (robot_start - closest_on_ball_path).norm();
  // If within the robot + ball radius (collision)

  if (dist_ball_point <= kRobotRadius + kBallRadius) {
    // Calculate angle of collision
    double angle = atan2(closest_on_ball_path[0] - robot_start.x(),
                         closest_on_ball_path[1] - robot_start.y());

    // Rotate by the robot rotation;
    angle = angle + pose.angle;
    // Collision within flat portion
    if (angle > kicker_angle1 && angle < kicker_angle2) {
      // The angles for the central region of the flat face.
      const float start_angle = DegToRad(75.0);
      const float end_angle = DegToRad(105.0);

      // Build the Line for the kicker face
      const float kicker_start_x =
          robot_start.x() +
          (kRobotRadius - kicker_offset) * cos(start_angle - pose.angle);
      const float kicker_start_y =
          robot_start.y() +
          (kRobotRadius - kicker_offset) * sin(start_angle - pose.angle);
      const float kicker_end_x =
          robot_start.x() +
          (kRobotRadius - kicker_offset) * cos(end_angle - pose.angle);
      const float kicker_end_y =
          robot_start.y() +
          (kRobotRadius - kicker_offset) * sin(end_angle - pose.angle);
      const Vector2f kicker_start(kicker_start_x, kicker_start_y);
      const Vector2f kicker_end(kicker_end_x, kicker_end_y);

      if (angle > start_angle && angle < end_angle) {
        if (dist_ball_point <= kRobotRadius + kBallRadius - kicker_offset2) {
          ball_velocity = FlatReflection(
              kicker_start, kicker_end, distance_robot,
              distance_ball_robot_frame, true, pose.angle, velocity_command);
        }
      } else {
        if (dist_ball_point <= kRobotRadius + kBallRadius - kicker_offset) {
          ball_velocity = FlatReflection(kicker_start, kicker_end,
                                         distance_robot, distance_ball, false,
                                         pose.angle, velocity_command);
          Vector2f new_pose = closest_on_ball_path;
          new_pose = new_pose + ((kBallRadius)*ball_dir.normalized());
          ball->SetPose(new_pose);
        }
      }
    } else {  // not in special region
      // TODO(jaholtz) determine if the closest point here is a sufficient
      // collision point
      ball_velocity =
          CircleReflection(pose.translation.x(), pose.translation.y(),
                           closest_on_ball_path, distance_robot, distance_ball);
      Vector2f new_pose = ball_end;
      new_pose = new_pose + ((kBallRadius)*ball_dir.normalized());
      ball->SetPose(new_pose);
    }
  }

  ball->SetVelocity(ball_velocity);
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
  Vector2f velocity_command_vector(velocity_x, velocity_y);

  HandleCollision(ball, velocity_command);

  if ((velocity_command_vector - current_velocity_).norm() >
      kMaxRobotAcceleration * time_slice_length) {
    LOG(WARNING) << "Velocity change is over max accel! Commanded: ("
                 << (velocity_command_vector - current_velocity_).norm()
                 << ") vs Allowed: (" << kMaxAcceleration * time_slice_length
                 << ")";
  }

  // Translate pose based on movement.
  // 1000 scales velocity command from m/s to mm/s.
  Pose2Df translation_pose =
      Pose2Df(velocity_command.velocity_r() * time_slice_length,
              Vector2f(velocity_command_vector.x() * time_slice_length * 1000,
                       velocity_command_vector.y() * time_slice_length * 1000));
  pose.ApplyPose(translation_pose);

  // Update the velocity position.
  current_velocity_ = velocity_command_vector;
}

const Vector2f Robot::GetBoundedMovement(
    const Vector2f& input_command_velocity) const {
  Vector2f command_velocity = input_command_velocity;
  Vector2f accel = (command_velocity - current_velocity_) / time_slice_length;

  // Crank down the velocity to the upper bound, update the accel accordingly.
  if (command_velocity.norm() > kMaxTranslation) {
    command_velocity = command_velocity.normalized() * kMaxTranslation;
    accel = (command_velocity - current_velocity_) / time_slice_length;
  }

  // Crank down the acceleration to the upper bound, update the command velocity
  // accordingly.
  if ((command_velocity - current_velocity_).norm() > kIsMovingThreshold &&
      accel.norm() > kMaxAcceleration / time_slice_length) {
    accel = accel / accel.norm() * (kMaxAcceleration / time_slice_length);
    command_velocity = current_velocity_ + accel;
  }

  if (command_velocity.norm() < kIsMovingThreshold) {
    command_velocity = Vector2f(0, 0);
  }

  return command_velocity;
}

const int Robot::GetId() const { return id; }

const float Robot::GetMaxTransation() const { return kMaxTranslation; }

const float Robot::GetMaxAcceleration() const { return kMaxAcceleration; }

const float Robot::GetRobotRadius() const { return kRobotRadius; }

const Pose2Df& Robot::GetPose() const { return pose; }

const Vector2f& Robot::GetVelocity() const { return current_velocity_; }

}  // namespace simulator

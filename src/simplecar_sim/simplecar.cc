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

#include "simplecar_sim/simplecar.h"

#include <glog/logging.h>
#include <math.h>
#include <cmath>
#include <algorithm>

#include "eigen3/Eigen/Geometry"
#include "math/math_util.h"

// using colorize::ColorGreen;
// using colorize::ColorBlue;
// using colorize::ColorCyan;
// using Eigen::Vector2f;
// using Eigen::Matrix2f;
// using Eigen::Quaternionf;
// using math_util::AngleMod;
using pose_2d::Pose2Df;
// using std::endl;
// using geometry::CheckLineLineIntersection;
// using geometry::SquaredDistance;
// using geometry::RayIntersect;
// using std::vector;

namespace simplecar {

Simplecar::Simplecar(int id,
                     const Pose2Df& initial_pose,
                     const float& initial_speed,
                     const float& time_slice_length)
:     id_(id),
      pose_(initial_pose),
      speed_(initial_speed),
      steer_angle_(0.0),
      time_slice_length_(time_slice_length) {
  UpdateVelocity();
}

Simplecar::Simplecar(const simplecar::Simplecar& car)
:     id_(car.GetId()),
      pose_(car.GetPose()),
      speed_(car.GetSpeed()),
      steer_angle_(0.0),
      time_slice_length_(car.GetTime()) {
  UpdateVelocity();
}

Simplecar::Simplecar(const int& id) :
    id_(id),
    pose_({0, 0, 0}),
    speed_(0),
    steer_angle_(0),
    time_slice_length_(0) { }


// Calculates the velocities of car from speed and heading.
void Simplecar::UpdateVelocity() {
  const float x_vel = speed_ * cos(pose_.angle);
  const float y_vel = speed_ * sin(pose_.angle);
  velocity_ = {x_vel, y_vel};
  rotational_velocity_ = (speed_ / kCarLength) * tan(steer_angle_);
}

const int Simplecar::GetId() const { return id_; }

const Pose2Df& Simplecar::GetPose() const { return pose_; }

const Vector2f& Simplecar::GetVelocity() const { return velocity_; }

const float& Simplecar::GetSpeed() const {
  return speed_;
}

const float & Simplecar::GetTime() const {
  return time_slice_length_;
}

const float& Simplecar::GetAngleVel() const { return rotational_velocity_; }

void Simplecar::AccelToSpeed(const float& speed) {
//   cout << speed_ << endl;
  float desired_speed = speed;
  if (desired_speed > kMaxSpeed) {
    desired_speed = kMaxSpeed;
  }
  if (desired_speed > speed_) {
    const float new_speed = speed_ + (kMaxAccel * time_slice_length_);

//     cout << "Speed Change: " << kMaxAccel * time_slice_length_ << endl;
//     cout << "Time Slice: " << time_slice_length_ << std::endl;
    speed_ = min(desired_speed, new_speed);
  } else {
    const float new_speed = speed_ - (kMaxAccel * time_slice_length_);
    speed_ = max(desired_speed, new_speed);
  }
  UpdateVelocity();
}

void Simplecar::SteerToAngle(const float& angle) {
  steer_angle_ = angle;
  UpdateVelocity();
}

void Simplecar::Update() {
  pose_.translation += velocity_ * time_slice_length_;
  pose_.angle += rotational_velocity_ * time_slice_length_;
}


}  // namespace simplecar

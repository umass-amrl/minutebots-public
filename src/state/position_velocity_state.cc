// Copyright 2017 - 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
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
#include "state/position_velocity_state.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "constants/constants.h"
#include "motion_control/ball_interception.h"

STANDARD_USINGS;
using datastructures::DenseArray;
using Eigen::Vector2d;

namespace state {

void InitializePositionVelocityArray(
    datastructures::DenseArray<PositionVelocityState::RobotPositionVelocity,
                               kMaxTeamRobots>* array) {
  array->GetMutableUnderlyingArray()->fill(
      PositionVelocityState::RobotPositionVelocity());
}

PositionVelocityState::BallPositionVelocity::BallPositionVelocity(
    const Eigen::Vector2f& position, const Eigen::Vector2f& velocity,
    const Eigen::Vector2f& observation, const double observed_time,
    const unsigned int last_camera_index)
    : position(position),
      velocity(velocity),
      observed_pose(observation),
      filtered_position(position),
      filtered_velocity(velocity),
      observed_time(observed_time),
      last_camera_index(last_camera_index),
      is_chip_kicked(false),
      is_chip_kick_tracking_triggered(false),
      chip_impact_point(Vector2f(0.0, 0.0)),
      chip_impact_time(0.0),
      chip_shot_time(0.0),
      chip_initial_vel(Vector3f(0.0, 0.0, 0.0)) {}


PositionVelocityState::BallPositionVelocity::BallPositionVelocity()
    : position(0, 0),
      velocity(0, 0),
      observed_pose(0, 0),
      filtered_position(0, 0),
      filtered_velocity(0, 0),
      observed_time(0),
      last_camera_index(kDefaultCameraIndex),
      is_chip_kicked(false),
      is_chip_kick_tracking_triggered(false),
      chip_impact_point(Vector2f(0.0, 0.0)),
      chip_impact_time(0.0),
      chip_shot_time(0.0),
      chip_initial_vel(Vector3f(0.0, 0.0, 0.0)) {}

PositionVelocityState::RobotPositionVelocity::RobotPositionVelocity()
    : ssl_vision_id(0),
      position(0, Vector2f(0, 0)),
      velocity(0, Vector2f(0, 0)),
      observed_pose(0, Vector2f(0, 0)),
      filtered_position(0, 0, 0),
      filtered_velocity(0, 0, 0),
      observed_velocity(0, 0, 0),
      observed_time(0),
      // Indicates that this is a default constructed object.
      confidence(0) {}

PositionVelocityState::RobotPositionVelocity::RobotPositionVelocity(
    const SSLVisionId ssl_vision_id, const pose_2d::Pose2Df& position,
    const pose_2d::Pose2Df& velocity, const pose_2d::Pose2Df& observed_pose,
    const pose_2d::Pose2Df& observed_velocity, const double observed_time,
    const float confidence)
    : ssl_vision_id(ssl_vision_id),
      position(position),
      velocity(velocity),
      observed_pose(observed_pose),
      filtered_position(position),
      filtered_velocity(velocity),
      observed_velocity(observed_velocity),
      observed_time(observed_time),
      confidence(confidence) {}

bool PositionVelocityState::RobotPositionVelocity::operator==(
    const PositionVelocityState::RobotPositionVelocity& other) const {
  return ssl_vision_id == other.ssl_vision_id && position == other.position &&
         velocity == other.velocity && observed_pose == other.observed_pose &&
         observed_time == other.observed_time && confidence == other.confidence
         && observed_velocity == other.observed_velocity;
}

bool PositionVelocityState::RobotPositionVelocity::operator<(
    const PositionVelocityState::RobotPositionVelocity& other) const {
  return ssl_vision_id < other.ssl_vision_id;
}

PositionVelocityState::PositionVelocityState()
    : ball_position_velocity_(), time_(0) {
  InitializePositionVelocityArray(&our_team_robots_);
  InitializePositionVelocityArray(&their_team_robots_);
}

PositionVelocityState::PositionVelocityState(
    const PositionVelocityState::RobotPositionVelocity& default_robot)
     : time_(0) {
  while (our_team_robots_.GetElementCount() < kMaxTeamRobots) {
    our_team_robots_.InsertBack(default_robot);
  }
  InitializePositionVelocityArray(&their_team_robots_);
}

PositionVelocityState::PositionVelocityState(const PositionVelocityState& other)
    : our_team_robots_(other.our_team_robots_),
      their_team_robots_(other.their_team_robots_),
      time_(other.time_) {}

PositionVelocityState::PositionVelocityState(PositionVelocityState&& other)
    : our_team_robots_(std::move(other.our_team_robots_)),
      their_team_robots_(std::move(other.their_team_robots_)),
      time_(std::move(other.time_)) {}

PositionVelocityState::~PositionVelocityState() {}

const DenseArray<PositionVelocityState::RobotPositionVelocity, kMaxTeamRobots>&
PositionVelocityState::GetOurTeamRobots() const {
  return our_team_robots_;
}
const DenseArray<PositionVelocityState::RobotPositionVelocity, kMaxTeamRobots>&
PositionVelocityState::GetTheirTeamRobots() const {
  return their_team_robots_;
}

DenseArray<PositionVelocityState::RobotPositionVelocity, kMaxTeamRobots>*
PositionVelocityState::GetMutableOurTeamRobots() {
  return &our_team_robots_;
}

DenseArray<PositionVelocityState::RobotPositionVelocity, kMaxTeamRobots>*
PositionVelocityState::GetMutableTheirTeamRobots() {
  return &their_team_robots_;
}

const PositionVelocityState::BallPositionVelocity&
PositionVelocityState::GetBallPositionVelocity() const {
  return ball_position_velocity_;
}

PositionVelocityState::BallPositionVelocity*
PositionVelocityState::GetMutableBallPositionVelocity() {
  return &ball_position_velocity_;
}

double PositionVelocityState::GetTime() const {
  return time_;
}

void PositionVelocityState::SetTime(double time) {
  time_ = time;
}

bool PositionVelocityState::ContainsOurRobot(SSLVisionId id) {
  bool contains_robot = false;

  for (const auto& robot : our_team_robots_) {
    if (robot.ssl_vision_id == id) {
      contains_robot = true;
      break;
    }
  }

  return contains_robot;
}


}  // namespace state

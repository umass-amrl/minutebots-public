// Copyright 2017-2018 kvedder@umass.edu
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

#include "state/world_robot.h"
#include "constants/constants.h"
#include "obstacles/circle_obstacle.h"
#include "util/timer.h"
#include "state_estimation/default_motion_model.h"

STANDARD_USINGS;
using obstacle::CircleObstacle;
using pose_2d::Pose2Df;
using motion_model::DefaultMotionModel;
using team::Team;

namespace state {

// Default constructor, sets default, temporary values.
WorldRobot::WorldRobot()
    : ssl_vision_id_(9999),
      pose_(0, Vector2f(0, 0)),
      velocity_(0, Vector2f(0, 0)),
      robot_obstacle_(pose_),
      timestamp_(0),
      observation_(0, 0, 0),
      confidence_(0) {}
//       motion_model_() {}

WorldRobot::WorldRobot(const SSLVisionId& ssl_vision_id,
                       bool is_ours,
                       const Pose2Df& pose,
                       const Vector2f& velocity,
                       const double timestamp)
    : ssl_vision_id_(ssl_vision_id),
      is_ours_(is_ours),
      pose_(pose),
      velocity_(0, velocity),
      robot_obstacle_(pose),
      timestamp_(timestamp),
      observation_(pose),
      confidence_(0) {}
//       motion_model_(ssl_vision_id) {}

WorldRobot::WorldRobot(const SSLVisionId& ssl_vision_id,
                       bool is_ours,
                       const Pose2Df& pose,
                       const Pose2Df& velocity,
                       const double timestamp)
    : ssl_vision_id_(ssl_vision_id),
      is_ours_(is_ours),
      pose_(pose),
      velocity_(velocity),
      robot_obstacle_(pose),
      timestamp_(timestamp),
      observation_(pose),
      confidence_(0) {}
//       motion_model_(ssl_vision_id, is_ours){}

WorldRobot::~WorldRobot() {}

WorldRobot& WorldRobot::operator=(const WorldRobot& other) {
  if (this != &other) {
    ssl_vision_id_ = other.ssl_vision_id_;
    is_ours_ = other.is_ours_;
    pose_ = other.pose_;
    velocity_ = other.velocity_;
    robot_obstacle_ = other.robot_obstacle_;
    timestamp_ = other.timestamp_;
    observation_ = other.observation_;
    confidence_ = other.confidence_;
    observation_time_ = other.observation_time_;
//     motion_model_.SetSSLVisionID(other.ssl_vision_id_, other.is_ours_);
  }
  return *this;
}

const Pose2Df& WorldRobot::GetPose() const { return pose_; }

const Vector2f& WorldRobot::GetLinearVelocity() const {
  return velocity_.translation;
}

const Pose2Df& WorldRobot::GetVelocity() const { return velocity_; }

const SSLVisionId WorldRobot::GetSSLVisionID() const { return ssl_vision_id_; }

const obstacle::RobotObstacle& WorldRobot::GetObstacle() const {
  return robot_obstacle_;
}

const bool WorldRobot::HasRecievedUpdate() const { return false; }

void WorldRobot::UpdateState(const Pose2Df& pose,
                             const Pose2Df& velocity,
                             double timestamp) {
  pose_ = pose;
  velocity_ = velocity;

  robot_obstacle_.UpdatePose(pose);
}

void WorldRobot::SetObservation(const Pose2Df& observation,
                                const float confidence,
                                const double observation_time) {
  observation_ = observation;
  confidence_ = confidence;
  observation_time_ = observation_time;
}

void WorldRobot::GetObservation(Pose2Df* observation, float* confidence) const {
  *observation = observation_;
  *confidence = confidence_;
}

double WorldRobot::GetObservationTime() const {
  return observation_time_;
}
}  // namespace state

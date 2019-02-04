// Copyright 2017-2018 slane@cs.umass.edu, kvedder@umass.edu
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

#ifndef SRC_STATE_WORLD_ROBOT_H_
#define SRC_STATE_WORLD_ROBOT_H_

#include <utility>

#include "constants/constants.h"
#include "datastructures/bounded_queue.h"
#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"
#include "obstacles/robot_obstacle.h"
#include "state/team.h"
// #include "state_estimation/default_motion_model.h"

namespace state {
class WorldRobot {
 public:
  WorldRobot();

  WorldRobot(const SSLVisionId& ssl_vision_id,
             bool is_ours,
             const pose_2d::Pose2Df& pose,
             const Eigen::Vector2f& velocity,
             const double timestamp);

  WorldRobot(const SSLVisionId& ssl_vision_id,
             bool is_ours,
             const pose_2d::Pose2Df& pose,
             const pose_2d::Pose2Df& velocity,
             const double timestamp);

  ~WorldRobot();

  WorldRobot& operator=(const WorldRobot& other);

  const pose_2d::Pose2Df& GetPose() const;
  const Eigen::Vector2f& GetLinearVelocity() const;
  const pose_2d::Pose2Df& GetVelocity() const;
  const SSLVisionId GetSSLVisionID() const;
  const bool HasRecievedUpdate() const;
  const obstacle::RobotObstacle& GetObstacle() const;
  void UpdateState(const pose_2d::Pose2Df& pose,
                   const pose_2d::Pose2Df& velocity,
                   double timestamp);

  void SetObservation(const pose_2d::Pose2Df& observation,
                      const float confidence, const double observation_time);
  void GetObservation(pose_2d::Pose2Df* observation, float* confidence) const;
  double GetObservationTime() const;

 private:
  SSLVisionId ssl_vision_id_;
  bool is_ours_;
  pose_2d::Pose2Df pose_;
  pose_2d::Pose2Df velocity_;
  obstacle::RobotObstacle robot_obstacle_;
  double timestamp_;

  // Most recent observation and pose of the robot
  // If it hasn't been set, the observation will be the current pose and the
  // confidence will be 0
  pose_2d::Pose2Df observation_;
  float confidence_;
  double observation_time_;
};
}  // namespace state

#endif  // SRC_STATE_WORLD_ROBOT_H_

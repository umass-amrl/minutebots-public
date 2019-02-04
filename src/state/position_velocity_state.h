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
#ifndef SRC_STATE_POSITION_VELOCITY_STATE_H_
#define SRC_STATE_POSITION_VELOCITY_STATE_H_

#include <vector>

#include "constants/constants.h"
#include "datastructures/dense_array.h"

#include "math/poses_2d.h"
#include "motion_control/motion_control_structures.h"

namespace state {

class PositionVelocityState {
 public:
  PositionVelocityState();
  PositionVelocityState(const PositionVelocityState& other);
  PositionVelocityState(PositionVelocityState&& other);
  ~PositionVelocityState();

  PositionVelocityState& operator=(const PositionVelocityState& other) =
      default;
  PositionVelocityState& operator=(PositionVelocityState&& other) = default;

  struct RobotPositionVelocity {
    SSLVisionId ssl_vision_id;
    pose_2d::Pose2Df position;
    pose_2d::Pose2Df velocity;
    pose_2d::Pose2Df observed_pose;
    pose_2d::Pose2Df filtered_position;
    pose_2d::Pose2Df filtered_velocity;
    pose_2d::Pose2Df observed_velocity;

    double observed_time;
    // Confidence of zero indicates that this is a placeholder robot.
    float confidence = 0;
    RobotPositionVelocity();

    RobotPositionVelocity(const SSLVisionId ssl_vision_id,
                          const pose_2d::Pose2Df& position,
                          const pose_2d::Pose2Df& velocity,
                          const pose_2d::Pose2Df& observed_pose,
                          const pose_2d::Pose2Df& observed_velocity,
                          const double observed_time, const float confidence);

    RobotPositionVelocity(const RobotPositionVelocity& other) = default;
    RobotPositionVelocity(RobotPositionVelocity&& other) = default;
    ~RobotPositionVelocity() = default;

    RobotPositionVelocity& operator=(const RobotPositionVelocity& other) =
        default;
    RobotPositionVelocity& operator=(RobotPositionVelocity&& other) = default;

    // For use in sorting by ssl_vision_id.
    bool operator<(const RobotPositionVelocity& other) const;

    bool operator==(const RobotPositionVelocity& other) const;
  };

  explicit PositionVelocityState(
      const PositionVelocityState::RobotPositionVelocity& default_robot);

  struct BallPositionVelocity {
   private:
    static const size_t kDefaultCameraIndex = 42;

   public:
    Eigen::Vector2f position;
    Eigen::Vector2f velocity;

    Eigen::Vector2f observed_pose;

    Eigen::Vector2f filtered_position;
    Eigen::Vector2f filtered_velocity;

    double observed_time;
    unsigned int last_camera_index;

    // This is set to true when chip kick detection has some minimum confidence
    // in a chip kick happening
    bool is_chip_kicked;

    // This is set to true when a hypothetical chip kick is being tracked. It
    // does not mean that it will necessarily reach a confident estimation
    bool is_chip_kick_tracking_triggered;
    Eigen::Vector2f chip_impact_point;
    double chip_impact_time;
    double chip_shot_time;
    Eigen::Vector3f chip_initial_vel;

    BallPositionVelocity();
    BallPositionVelocity(const Eigen::Vector2f& position,
                         const Eigen::Vector2f& velocity,
                         const Eigen::Vector2f& observation,
                         const double observed_time,
                         const unsigned int last_camera_index);
    BallPositionVelocity(const BallPositionVelocity& other) = default;
    BallPositionVelocity(BallPositionVelocity&& other) = default;
    ~BallPositionVelocity() = default;

    BallPositionVelocity& operator=(const BallPositionVelocity& other) =
        default;
    BallPositionVelocity& operator=(BallPositionVelocity&& other) = default;
  };

  // Gets the array of our team's robots.
  const datastructures::DenseArray<RobotPositionVelocity, kMaxTeamRobots>&
  GetOurTeamRobots() const;
  // Gets the array of their team's robots.
  const datastructures::DenseArray<RobotPositionVelocity, kMaxTeamRobots>&
  GetTheirTeamRobots() const;

  // Gets a mutable pointer the array of our team's robots.
  datastructures::DenseArray<RobotPositionVelocity, kMaxTeamRobots>*
  GetMutableOurTeamRobots();

  // Gets a mutable pointer the array of their team's robots.
  datastructures::DenseArray<RobotPositionVelocity, kMaxTeamRobots>*
  GetMutableTheirTeamRobots();

  // Gets a const reference to the ball position.
  const BallPositionVelocity& GetBallPositionVelocity() const;

  // Gets a mutable pointer the ball position.
  BallPositionVelocity* GetMutableBallPositionVelocity();

  double GetTime() const;
  void SetTime(double time);

  bool ContainsOurRobot(SSLVisionId id);

 private:
  datastructures::DenseArray<RobotPositionVelocity, kMaxTeamRobots>
      our_team_robots_;
  datastructures::DenseArray<RobotPositionVelocity, kMaxTeamRobots>
      their_team_robots_;

  BallPositionVelocity ball_position_velocity_;

  double time_;
};
}  // namespace state

#endif  // SRC_STATE_POSITION_VELOCITY_STATE_H_

// Copyright 2017 - 2018 slane@cs.umass.edu, kvedder@umass.edu
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

#include <memory>
#include <vector>

#ifndef SRC_TACTICS_GOALIE_H_
#define SRC_TACTICS_GOALIE_H_

#include "constants/constants.h"
#include "obstacles/obstacle_flag.h"
#include "state/shared_state.h"
#include "tactics/tactic.h"
#include "tactics/state_machine_tactic.h"

namespace tactics {
class Goalie : public StateMachineTactic {
 public:
  Goalie(const state::WorldState& world_state,
         TacticArray* tactic_list,
         state::SharedState* shared_state,
         OurRobotIndex our_robot_index,
         state::SoccerState* soccer_state);

  const char* Name() const override { return "goalie"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  bool IsComplete() override;
  float GetCost() override;
  void SetThreat(const Eigen::Vector2f& position);
  void AddDefender(OurRobotIndex index);

  static SSLVisionId goalie_ssl_id_;
  static bool goalie_set_;

 private:
  void Transition() override;
  void Guard();
  void Intercept();
  void DiveSetup();
  void Dive();
//   void MoveBall();
  void Clear();

  void Navigate(pose_2d::Pose2Df robot_goal_pose);
  void Navigate(pose_2d::Pose2Df robot_goal_pose,
                obstacle::ObstacleFlag flags);

  bool ShouldClear(const Eigen::Vector2f& ball_pose,
                   const Eigen::Vector2f& ball_velocity,
                   float ball_speed);

//   bool ShouldMove(const Eigen::Vector2f& ball_pose,
//                   const Eigen::Vector2f& ball_velocity,
//                   float ball_speed);

  bool ShouldDive();
  bool ShouldIntercept(const Eigen::Vector2f& ball_pose,
                       const Eigen::Vector2f& ball_velocity,
                       const Eigen::Vector2f& ball_observation,
                       float ball_speed);

  // Returns true if we should position at the default location
  bool CalculateGuardPosition(Eigen::Vector2f* goalie_target,
                              float* threat_angular_width);

  bool CanBlockFullAngle(float threat_distance,
                         float threat_angular_width,
                         const Vector2f& guard_position,
                         logger::Logger* logger,
                         Vector2f* goalie_position);

  State guard_;
  State intercept_;
  State dive_setup_;
  State dive_;
//   State move_ball_;
  State clear_;

  State previous_state_;

  Eigen::Vector2f threat_position_;
  Eigen::Vector2f guard_point_;
  Eigen::Vector2f previous_intercept_point_;
  Eigen::Vector2f previous_goal_point_;
  pose_2d::Pose2Df ball_placement_target_;
  bool set_ball_placement_target_;
  int dive_counter_;
  int intercept_counter_;

  std::vector<Eigen::Vector2f> defender_positions_;

  RepairableParam kMaxInterceptCount_;
  RepairableParam kMaxDiveCount_;
  RepairableParam kBallKickThreshold_;
  RepairableParam kMoveBallThresholdX_;
  RepairableParam kClearThresholdSpeed_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_GOALIE_H_

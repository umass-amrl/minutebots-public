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

#ifndef SRC_TACTICS_PRIMARY_DEFENDER_H_
#define SRC_TACTICS_PRIMARY_DEFENDER_H_

#include <memory>
#include <vector>

#include "obstacles/obstacle_flag.h"
#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"
#include "tactics/tactic.h"


namespace tactics {
// A simple defender positions itself facing the ball, on the intersection
// of the defense area perimeter and the line from the ball to the
// center of the goal.
class PrimaryDefender : public StateMachineTactic {
 public:
  PrimaryDefender(const state::WorldState& world_state,
                  TacticArray* tactic_list,
                  state::SharedState* shared_state,
                  OurRobotIndex our_robot_index,
                  state::SoccerState* soccer_state);

  const char* Name() const override { return "primary_defender"; }
  void Init() override;
  void Reset() override;
  // The goal for the primary defender is defined to be a point on the goal
  // which it is defending. The defender will situate itself on the intersection
  // between the defense area and the line between the ball and that point
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  bool IsComplete() override;
  float GetCost() override;
  void SetIntercept();

 private:
  void Transition() override;
  void Guard();
  void Intercept();
  void Navigate(const obstacle::ObstacleFlag& obstacles);

  State guard_;
  State intercept_;

  pose_2d::Pose2Df guard_pose_;

  RepairableParam kMaxInterceptCount_;
  RepairableParam kBallKickThreshold_;

  const float kIgnoreOpponentsThreshold_ = 300.0f;
  bool intercepting_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_PRIMARY_DEFENDER_H_

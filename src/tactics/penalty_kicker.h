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

#ifndef SRC_TACTICS_PENALTY_KICKER_H_
#define SRC_TACTICS_PENALTY_KICKER_H_

#include "state/referee_state.h"
#include "state/shared_state.h"
#include "tactics/tactic.h"

namespace tactics {
class PenaltyKicker : public Tactic {
 public:
  PenaltyKicker(const state::WorldState& world_state,
                TacticArray* tactic_list,
                state::SharedState* shared_state, OurRobotIndex our_robot_index,
                state::SoccerState* soccer_state);

  const char* Name() const override { return "penalty_kicker"; }
  void Init() override;
  void Run() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  bool IsComplete() override;
  float GetCost() override;

 private:
  enum KickoffKickerState { PREPERATION, PREKICK, KICKING, POSTKICK };

  float target_angle;
  Eigen::Vector2f target_position;

  KickoffKickerState execution_state;

  pose_2d::Pose2Df robot_goal_pose;

  bool set_kick_goal;
  bool is_complete_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_PENALTY_KICKER_H_

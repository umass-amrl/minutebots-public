// Copyright 2018 slane@cs.umass.edu
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

#ifndef SRC_TACTICS_TEST_ATTACKER_H_
#define SRC_TACTICS_TEST_ATTACKER_H_

#include <stdio.h>
#include <memory>
#include <vector>

#include "obstacles/obstacle_flag.h"
#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"
#include "tactics/tactic.h"


namespace tactics {
// This is a test role for testing Spencer's Role Estimation
class TestAttacker : public StateMachineTactic {
 public:
  TestAttacker(const state::WorldState& world_state,
               TacticArray* tactic_list,
               state::SharedState* shared_state,
               OurRobotIndex our_robot_index,
               state::SoccerState* soccer_state);

  const char* Name() const override { return "test_attacker"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  bool IsComplete() override;
  float GetCost() override;
  void SetIntercept();

 private:
  void Transition() override;
  void Guard();
  void Intercept();
  void Navigate(const obstacle::ObstacleFlag& obstacles,
                const Vector2f& target_point);
  void DumpData();

  State guard_;
  State intercept_;

  pose_2d::Pose2Df guard_pose_;

  bool intercepting_;

  const float kTheta_ = 500.0f;  // How far the ball it should position
  const bool kDumpData_ = false;
  static bool dump_file_opened_;
  static std::FILE* file_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_TEST_ATTACKER_H_

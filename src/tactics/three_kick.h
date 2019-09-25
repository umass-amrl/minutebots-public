// Copyright 2018 jaholtz@cs.umass.edu
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
#include <string>

#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"

#ifndef SRC_TACTICS_THREE_KICK_H_
#define SRC_TACTICS_THREE_KICK_H_

namespace tactics {

class ThreeKick : public StateMachineTactic {
 public:
  ThreeKick(const string& machine_name,
       const state::WorldState& world_state,
       TacticArray* tactic_list,
       state::SharedState* shared_state, OurRobotIndex our_robot_index,
       state::SoccerState* soccer_state);

  const char* Name() const override { return "three_kick"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  bool IsComplete() override;
  float GetCost() override;
  void SetPassOnly(const bool& pass_only);
  void SetChip(const bool& chip, const float& chip_distance);
  int GetKickCount();

  float target_angle_;

 private:
  void Transition() override;

  // Behavior Functions Used by the states
  void Kicking();

  void PostKick();

  SolutionParameters kick_solution_;

  bool is_complete_;
  bool pass_only_;
  bool chip_;
  float chip_distance_;

  // States
  State kick_;
  State post_kick_;

  // THRESHOLDS TO TUNE;
  RepairableParam thresholds_ball_velocity_;
  // Maxmimum distance between robot and ball to kick
  // (mm)
  RepairableParam thresholds_distance_;
  // Maxmimum number of timesteps to kick for
  RepairableParam thresholds_kick_timeout_;
  // Minimum portion of the velocity in the kick direction for successful kick
  RepairableParam thresholds_kick_percent_;
  RepairableParam thresholds_kick_speed_;
  RepairableParam thresholds_follow_through_;
  // END THRESHOLDS
  int kick_count_;
  bool target_set_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_THREE_KICK_H_

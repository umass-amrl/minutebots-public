// Copyright 2017 - 2018 kvedder@umass.edu
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

#ifndef SRC_TACTICS_HALT_H_
#define SRC_TACTICS_HALT_H_

#include <memory>
#include <vector>

#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "tactics/tactic.h"

namespace tactics {
class Halt : public Tactic {
 public:
  Halt(const state::WorldState& world_state,
          TacticArray* tactic_list,
          state::SharedState* shared_state, OurRobotIndex our_robot_index,
          state::SoccerState* soccer_state);

  ~Halt();

  const char* Name() const override { return "halt"; }
  void Init() override;
  void Run() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void Reset() override;
 private:
  bool translation_complete_ = false;
  bool angle_complete_ = false;
  pose_2d::Pose2Df started_velocity_;
  bool started_ = false;
};
}  // namespace tactics

#endif  // SRC_TACTICS_HALT_H_

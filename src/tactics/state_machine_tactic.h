// Copyright 2018 - 2019 jaholtz@cs.umass.edu
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
#include <string>
#include <vector>

#include "constants/constants.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/world_state.h"
#include "tactics/tactic_index.h"
#include "tactics/tactic.h"
#include "srtr/state_machine.h"
#include "util/random.h"

#include "logging/logger.h"
using srtr::StateMachine;
// using srtr::StateMachine::RepairableParam;

#ifndef SRC_TACTICS_STATE_MACHINE_TACTIC_H_
#define SRC_TACTICS_STATE_MACHINE_TACTIC_H_
// Predefining soccer state to avoid circular includes.
namespace state {
class SoccerState;
}

namespace tactics {

class StateMachineTactic : public Tactic, public srtr::StateMachine {
 public:
  StateMachineTactic(const string& machine_name,
         const state::WorldState& world_state,
         TacticArray* tactic_list,
         state::SharedState* shared_state,
         OurRobotIndex our_robot_index,
         state::SoccerState* soccer_state);
  // Overrides the Tactic Run method for some additional functionality.
  void Run() override;
  // Should handle state transitions, must be implemented for each
  // state machine.
  void Transition() override = 0;
};
}  // namespace tactics

#endif  // SRC_TACTICS_STATE_MACHINE_TACTIC_H_

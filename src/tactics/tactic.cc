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
#include "tactics/tactic.h"

#include <string>

#include "constants/constants.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"

STANDARD_USINGS;
using state::WorldState;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;
using state::SharedState;
using state::SoccerState;

namespace tactics {

Tactic::Tactic(const WorldState& world_state,
               TacticArray* tactic_list,
               SharedState* shared_state, OurRobotIndex our_robot_index,
               SoccerState* soccer_state)
    : world_state_(world_state),
      tactic_list_(tactic_list),
      shared_state_(shared_state),
      our_robot_index_(our_robot_index),
      soccer_state_(soccer_state),
      random(reinterpret_cast<size_t>(this)) {}

void Tactic::SetParameters(const vector<string>& parameters) { return; }

float Tactic::GetCost() { return 0; }

const pose_2d::Pose2Df Tactic::GetGoal() {
  pose_2d::Pose2Df goal(0, 0, 0);
  return goal;
}

bool Tactic::IsComplete() {
  return false;  // any non-ball-handling tactic should always be ongoing
}

}  // namespace tactics

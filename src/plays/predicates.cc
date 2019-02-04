// Copyright 2017 rezecib@gmail.com
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
#include "plays/predicates.h"

#include "constants/constants.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/world_state.h"

STANDARD_USINGS;
using state::SoccerState;
using state::WorldState;
using state::SoccerRobot;

namespace plays {

GameState GetGameState(const WorldState& world_state) {
  // TODO(rezecib): Actually detect these states;
  // this would require keeping track of history, ball possession, etc
  if (world_state.GetBallPosition().position.x() > 0) {
    return GameState::OFFENSE;
  } else if (world_state.GetBallPosition().position.x() < 0) {
    return GameState::DEFENSE;
  } else {
    return GameState::LOOSE_BALL;
  }
}

bool IsActiveTacticCompleted(SoccerState* soccer_state) {
  for (const SoccerRobot& robot : soccer_state->GetAllSoccerRobots()) {
    if (robot.tactic_list_[robot.current_tactic_]->IsComplete()) {
      return true;
    }
  }
  return false;
}

}  // namespace plays

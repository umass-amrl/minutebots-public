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

#include "state/soccer_state.h"
#include "state/world_state.h"

#ifndef SRC_PLAYS_PREDICATES_H_
#define SRC_PLAYS_PREDICATES_H_

namespace plays {

enum GameState {
  OFFENSE,
  DEFENSE,
  LOOSE_BALL,
  KICK_OFF,
  DIRECT_FREE_KICK,
  INDIRECT_FREE_KICK
  // Others?
};

GameState GetGameState(const state::WorldState& world_state);

bool IsActiveTacticCompleted(state::SoccerState* soccer_state);

}  // namespace plays

#endif  // SRC_PLAYS_PREDICATES_H_

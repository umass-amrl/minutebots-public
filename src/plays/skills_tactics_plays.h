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

#include <string>
#include <vector>

#include "constants/constants.h"
#include "state/soccer_state.h"
#include "state/referee_state.h"
#include "logging/logger.h"

#ifndef SRC_PLAYS_SKILLS_TACTICS_PLAYS_H_
#define SRC_PLAYS_SKILLS_TACTICS_PLAYS_H_

// I'm not sure of the best way to handle this; right now this "works",
// but I think it is polluting the global namespace
extern "C" {
  #include "lua.h"
  #include "lualib.h"
  #include "lauxlib.h"
}

// Convert Lua naming style to Google C++ style
#define LuaState lua_State

namespace plays {
class SkillsTacticsPlays {
 public:
  explicit SkillsTacticsPlays(state::SoccerState* soccer_state);

  ~SkillsTacticsPlays();

  void ExecutePlayEngine(state::RefereeState ref_state, bool test);

  logger::Logger play_logger;

 private:
  state::SoccerState* soccer_state_;
  std::vector<std::string> current_roles_;
  std::vector<OurRobotIndex> current_assignments_;

  // Holds the state of the Lua VM
  LuaState* lua_state_;
  // Lua Fast Registry Indices (fri) for Lua functions we want to call
  int fri_execute_play_engine_;
  int fri_set_predicate_;

  void SetPredicate(const char* predicate, bool value);

  // Sets a set of lua predicates given the referee based GameState
  void SetRefStatePredicates(state::RefereeState);
  void SetGameStatePredicates(state::RefereeState ref_state);
  void LogLuaLogs();
  void AssignRole(std::vector<OurRobotIndex>* unassigned_ids,
                  const std::string& tactic_name,
                  OurRobotIndex current_assignment);
  void AssignRole(std::vector<OurRobotIndex>* unassigned_ids,
                  const std::string& tactic_name,
                  const bool test_mode,
                  OurRobotIndex current_assignment);
};
}  // namespace plays

#endif  // SRC_PLAYS_SKILLS_TACTICS_PLAYS_H_

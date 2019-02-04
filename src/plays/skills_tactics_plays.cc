// Copyright 2018 rezecib@gmail.com ikhatri@umass.edu
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
#include "plays/skills_tactics_plays.h"

#include <limits>
#include <map>
#include <string>

#include "constants/constants.h"
#include "evaluators/defense_evaluation.h"
#include "plays/predicates.h"
#include "state/soccer_state.h"
#include "tactics/tactic.h"
#include "tactics/tactic_index.h"

STANDARD_USINGS;
using defense::DefenseEvaluator;
using std::unique_ptr;
using state::SoccerState;
using state::WorldState;
using state::SoccerRobot;
// using state::GameState;
using state::RefereeState;
using tactics::Tactic;
using tactics::TacticIndex;
using std::map;
using std::string;

namespace plays {

static vector<string> Split(const string& s);
vector<string> lua_logs;
// Functions for Lua to call;
// Lua requires that they take LuaState* and return int
// the return is the number of return values for Lua to take off the stack
int LuaLog(LuaState* lua_state);

SkillsTacticsPlays::SkillsTacticsPlays(SoccerState* soccer_state)
    : soccer_state_(soccer_state), lua_state_(luaL_newstate()) {
  if (!lua_state_) {
    LOG(FATAL) << "Error initializing Lua VM, exiting." << endl;
  }

  luaL_openlibs(lua_state_);  // loads the standard Lua libraries
  lua_register(lua_state_, "log", LuaLog);
  if (luaL_loadfile(lua_state_, "src/plays/play_engine.lua") != 0) {
    LOG(FATAL) << "Error loading play_engine.lua: "
               << lua_tostring(lua_state_, -1) << endl;
  }
  // play_engine.lua returns two values:
  // function execute_play_engine
  // function set_predicate
  if (lua_pcall(lua_state_, 0, 2, 0) != 0) {
    LOG(FATAL) << "Error initializing play_engine.lua: "
               << lua_tostring(lua_state_, -1) << endl;
  }
  // take the functions from the top of the stack
  // and store them in the fast registry
  // note that this needs to be done in the reverse of return order
  // (because stacks...)
  fri_set_predicate_ = luaL_ref(lua_state_, LUA_REGISTRYINDEX);
  fri_execute_play_engine_ = luaL_ref(lua_state_, LUA_REGISTRYINDEX);
}

SkillsTacticsPlays::~SkillsTacticsPlays() { lua_close(lua_state_); }

void SkillsTacticsPlays::SetRefStatePredicates(RefereeState ref_state) {
  SetPredicate("first_half_pre", ref_state.IsNormalFirstHalfPre() ||
                                     ref_state.IsExtraFirstHalfPre());
  SetPredicate("first_half",
               ref_state.IsNormalFirstHalf() || ref_state.IsExtraFirstHalf());
  SetPredicate("half_time",
               ref_state.IsNormalHalftime() || ref_state.IsExtraHalfTime());
  SetPredicate("second_half_pre", ref_state.IsNormalSecondHalfPre() ||
                                      ref_state.IsExtraSecondHalfPre());
  SetPredicate("second_half",
               ref_state.IsNormalSecondHalf() || ref_state.IsExtraSecondHalf());
  SetPredicate("break", ref_state.IsExtraTimeBreak() ||
                            ref_state.IsPenaltyShootoutBreak());
  SetPredicate("penalty_shootout_break", ref_state.IsPenaltyShootoutBreak());
  SetPredicate("penalty_shootout", ref_state.IsPenaltyShootout());
  SetPredicate("post_game", ref_state.IsPostGame());
  SetPredicate("halt", ref_state.IsHalt());
  SetPredicate("stop", ref_state.IsStop());
  SetPredicate("normal_start", ref_state.IsNormalStart());
  SetPredicate("force_start", ref_state.IsForceStart());
  SetPredicate("prepare_kickoff_us", ref_state.IsPrepareKickoffUs());
  SetPredicate("prepare_kickoff_them", ref_state.IsPrepareKickoffThem());
  SetPredicate("prepare_penalty_us", ref_state.IsPreparePenaltyUs());
  SetPredicate("prepare_penalty_them", ref_state.IsPreparePenaltyThem());
  SetPredicate("direct_free_us", ref_state.IsDirectFreeUs());
  SetPredicate("direct_free_them", ref_state.IsDirectFreeThem());
  SetPredicate("indirect_free_us", ref_state.IsIndirectFreeUs());
  SetPredicate("indirect_free_them", ref_state.IsIndirectFreeThem());
  SetPredicate("timeout_us", ref_state.IsTimeoutUs());
  SetPredicate("timeout_them", ref_state.IsTimeoutThem());
  SetPredicate("goal_us", ref_state.IsGoalUs());
  SetPredicate("goal_them", ref_state.IsGoalThem());
  SetPredicate("ball_placement_us", ref_state.IsBallPlacementUs());
  SetPredicate("ball_placement_them", ref_state.IsBallPlacementThem());
}

void SkillsTacticsPlays::SetGameStatePredicates(RefereeState ref_state) {
  // Possibly needs to be set only when certain referee states are active.
  SetPredicate("kicked", soccer_state_->IsBallKicked());
  SetPredicate("our_half", soccer_state_->BallOurHalf());
  SetPredicate("their_half", soccer_state_->BallTheirHalf());
  SetPredicate("midfield", soccer_state_->BallMidField());
  SetPredicate("possession_us", soccer_state_->BallOurPossession());
  SetPredicate("possession_them", soccer_state_->BallTheirPossession());
  SetPredicate("free_ball", soccer_state_->FreeBall());
  SetPredicate("normal_play", soccer_state_->IsNormalPlay());
  SetPredicate("kickoff", soccer_state_->IsKickoff());
}

void SkillsTacticsPlays::LogLuaLogs() {
  for (auto i : lua_logs) {
    play_logger.LogPrint(i);
  }
}

void SkillsTacticsPlays::ExecutePlayEngine(state::RefereeState ref_state,
                                           bool test) {
  play_logger.Clear();
  lua_logs.clear();
  play_logger.LogPrint("Play Manager");
  play_logger.Push();
  // Set each of the predicates
  SetRefStatePredicates(ref_state);
  SetGameStatePredicates(ref_state);
  SetPredicate("test", test);
  // put execute_play_engine on the stack
  lua_rawgeti(lua_state_, LUA_REGISTRYINDEX, fri_execute_play_engine_);
  // TODO(rezecib): figure out external outcomes (scoring, being scored on)
  lua_pushstring(lua_state_, "");
  lua_pushboolean(lua_state_, IsActiveTacticCompleted(soccer_state_));
  lua_pushboolean(lua_state_, test);
  // two arguments, variable number of return values, default error function
  if (lua_pcall(lua_state_, 3, LUA_MULTRET, 0) != 0) {
    // There could be better error handling here, like printing a stack trace
    // but I looked into it and setting it up looked like a headache
    // and could also affect runtime efficiency when no errors occur
    LOG(FATAL) << "Error while running Lua function execute_play_engine: "
               << lua_tostring(lua_state_, -1) << endl;
  }
  // If execute_play_engine returns nothing, don't do anything
  // Otherwise, assign the returned tactics in order of priority
  int num_return_values = lua_gettop(lua_state_);
  vector<OurRobotIndex> unassigned_ids;
  for (const SoccerRobot& soccer_robot : soccer_state_->GetAllSoccerRobots()) {
    if (soccer_robot.enabled_) {
      unassigned_ids.push_back(soccer_robot.our_robot_index_);
    }
  }

  if (num_return_values == 0) {
    // There were no changes to the roles, so just go through the current ones
    // to see if there's now a better assignment due to world state changes
    // There is probably a cleaner way to do this
    vector<string> current_roles = current_roles_;
    vector<OurRobotIndex> current_assignments = current_assignments_;

    DefenseEvaluator::UpdateNumDefenders(current_roles_);
    DefenseEvaluator::CalculateThreats(soccer_state_->GetWorldState(),
                                       *soccer_state_);

    current_roles_.clear();
    current_assignments_.clear();
    int assignment = -1;
    for (unsigned i = 0; i < current_roles.size(); i++) {
      if (current_assignments.size() > i) {
        assignment = current_assignments[i];
      }
      AssignRole(&unassigned_ids, current_roles[i], test, assignment);
    }
  } else {
    // Otherwise, get the new roles off of the Lua stack and assign
    vector<string> current_roles;
    vector<OurRobotIndex> current_assignments = current_assignments_;
    current_roles_.clear();
    current_assignments_.clear();

    for (int i = 1; i <= num_return_values; i++) {
      string tactic_string = lua_tostring(lua_state_, i);
      current_roles.push_back(tactic_string);
    }

    DefenseEvaluator::UpdateNumDefenders(current_roles);
    DefenseEvaluator::CalculateThreats(soccer_state_->GetWorldState(),
                                       *soccer_state_);

    for (int i = 1; i <= num_return_values; i++) {
      string tactic_string = lua_tostring(lua_state_, i);
      play_logger.LogPrint(tactic_string);
      AssignRole(&unassigned_ids, tactic_string, test, -1);
    }
    lua_pop(lua_state_, num_return_values);
  }
  LogLuaLogs();
}

void SkillsTacticsPlays::SetPredicate(const char* predicate, bool value) {
  lua_rawgeti(lua_state_, LUA_REGISTRYINDEX, fri_set_predicate_);
  lua_pushstring(lua_state_, predicate);
  lua_pushboolean(lua_state_, value);
  lua_pcall(lua_state_, 2, 0, 0);
}

void SkillsTacticsPlays::AssignRole(vector<OurRobotIndex>* unassigned_ids,
                                    const string& tactic_string,
                                    const bool test_mode,
                                    OurRobotIndex current_assignment) {
  vector<string> tactic_params = Split(tactic_string);
  CHECK_GT(tactic_params.size(), 0);
  string tactic_name = tactic_params[0];
  TacticIndex tactic_id = TacticIndex::HALT;
  float min_cost = std::numeric_limits<float>::max();
  unsigned min_index = 0;
  current_roles_.push_back(tactic_string);
  if (unassigned_ids->size() > 0) {
    if (!test_mode) {
      play_logger.LogPrint("%s Costs:", tactic_name);
      play_logger.Push();
      for (unsigned i = 0; i < unassigned_ids->size(); i++) {
        // This seems to be the only way to handle Tactics, but it's super gross
        const SoccerRobot& robot =
            soccer_state_->GetRobotByOurRobotIndex((*unassigned_ids)[i]);
        tactic_id = robot.GetTacticByName(tactic_name);
        robot.tactic_list_[tactic_id]->SetParameters(tactic_params);
        float cost = robot.tactic_list_[tactic_id]->GetCost();
        // Determine if this is the role they are currently assigned
        if (robot.our_robot_index_ == current_assignment) {
          // They already have this role; reduce the cost a bit for hysteresis
          // Maybe this should be up to the GetCost function instead?
          // May need tuning to be effective.
          play_logger.LogPrint("Robot Index: %d", robot.our_robot_index_);
          play_logger.LogPrint("Hysteresis id: %d", robot.our_robot_index_);
          cost = cost * kSTPHysteresis;
        }
        play_logger.LogPrint("Cost: %f", cost);
        if (cost < min_cost) {
          min_cost = cost;
          min_index = i;
        }
      }
      play_logger.Pop();
    } else {
      const SoccerRobot& robot =
          soccer_state_->GetRobotByOurRobotIndex((*unassigned_ids)[0]);
      tactic_id = robot.GetTacticByName(tactic_name);
      robot.tactic_list_[tactic_id]->SetParameters(tactic_params);
      min_index = 0;
    }
    OurRobotIndex our_robot_index = (*unassigned_ids)[min_index];
    //   LOG(INFO) << "Assigning to robot " << our_robot_index << " tactic "
    //             << tactic_string << endl;

    current_assignments_.push_back(our_robot_index);
    soccer_state_->SetRobotTacticByRobotIndex(our_robot_index, tactic_id);
    unassigned_ids->erase(unassigned_ids->begin() + min_index);
  }
}

static vector<string> Split(const string& s) {
  vector<string> elems;
  std::stringstream ss;
  ss.str(s);
  string item;
  while (std::getline(ss, item, ' ')) {
    elems.push_back(item);
  }
  return elems;
}

int LuaLog(LuaState* lua_state) {
  int num_args = lua_gettop(lua_state);
  for (int i = 1; i <= num_args; i++) {
    // Lua seems to automatically insert newlines at the end
    // maybe some day figure out how to remove them
    //     LOG(INFO) << lua_tostring(lua_state, i);
    lua_logs.push_back(lua_tostring(lua_state, i));
  }
  lua_pop(lua_state, num_args);
  return 0;  // the number of return values
}

}  // namespace plays

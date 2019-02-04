// Copyright 2017-2018 jaholtz@cs.umass.edu
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
#include "tactics/state_machine_tactic.h"

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
using MinuteBotsProto::StateMachineData;
using srtr::StateMachine;
using MinuteBotsProto::StateMachineData;
using MinuteBotsProto::Trace;
using std::fstream;
using std::ios;
using std::cerr;
using std::cout;
using std::endl;
namespace tactics {

StateMachineTactic::StateMachineTactic(const string& machine_name,
               const WorldState& world_state,
               TacticArray* tactic_list,
               SharedState* shared_state, OurRobotIndex our_robot_index,
               SoccerState* soccer_state)
: Tactic(world_state, tactic_list, shared_state, our_robot_index,
         soccer_state),
  StateMachine(machine_name) {}


void StateMachineTactic::Run() {
  SetupMessage();
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrintPush(machine_name_);
  Transition();
  robot_logger->LogPrint("Tactic State: %s", state_.name_);
  state_();
  robot_logger->Pop();
  robot_logger->AddStateMachineData(log_message_);
}

}  // namespace tactics

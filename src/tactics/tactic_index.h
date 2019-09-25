// Copyright 2018 - 2019 kvedder@umass.edu
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
#ifndef SRC_TACTICS_TACTIC_INDEX_H_
#define SRC_TACTICS_TACTIC_INDEX_H_

#include <memory>

namespace tactics {

enum TacticIndex {
  EIGHT_GRID,
  DEFLECTION,
  SIMPLE_DEFLECTION,
  FORWARD_BACKWARD,
  RANDOM_POINTS,
  TRIANGLE,
  CUSTOM_ROUTE,
  TRIANGLE_ID_DEPENDENT,
  JOYSTICK_CONTROLLER,
  GOALIE,
  PRIMARY_DEFENDER,
  SECONDARY_DEFENDER,
  TERTIARY_DEFENDER,
  GUARD_POINT,
  STOX_PIVOT,
  NTOC,
  TEST_NTOC,
  HALT,
  KICKOFF,
  PENALTY_KICK,
  PENALTY_RECIEVE,
  DIRECT_FREE_KICKER,
  INDIRECT_FREE_KICKER,
  BALL_INTERCEPTION,
  NAVIGATE_TO_INTERCEPTION,
  NAVIGATE_TO_CATCH,
  KICK,
  THREE_KICK,
  CATCH,
  TEST_CATCH,
  SAFE_BALL_FOLLOW,
  SECONDARY_ATTACKER,
  DIVE_CONTROLLER,
  TEST_DIVE_CONTROLLER,
  PRIMARY_ATTACKER,
  EXAMPLE_ATTACKER,
  SIMPLE_ATTACKER,
  PASS_FAIL_PRIMARY_ATTACKER,
  STATE_MACHINE_EXAMPLE,
  BALL_PLACEMENT,
  TEST_PASSING,
  RECEIVER,
  TEST_PASSBACK,
  SETUP_ATTACKER,
  PD_CONTROLLER,
  COERCIVE_ATTACKER,
  BETTER_BALL_PLACEMENT,
  TEST_DEFENDER,
  TEST_ATTACKER,
  DOCKING,
  PASS_FAIL_DOCKING,
  TSOCS,
  TEST_TSOCS,
  DEMO_TSOCS,
  TRAIN_TSOCS,
  TEST_SIM_TSOCS,
  MARIONETTE,
  TACTIC_COUNT  // This needs to stay as the last
};

class Tactic;

using TacticArray = std::array<std::unique_ptr<tactics::Tactic>,
                               tactics::TacticIndex::TACTIC_COUNT>;

}  // namespace tactics

#endif  // SRC_TACTICS_TACTIC_INDEX_H_

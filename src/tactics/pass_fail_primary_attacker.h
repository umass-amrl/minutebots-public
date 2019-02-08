// Copyright 2019 kvedder@umass.edu
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

#ifndef SRC_TACTICS_PASS_FAIL_PRIMARY_ATTACKER_H_
#define SRC_TACTICS_PASS_FAIL_PRIMARY_ATTACKER_H_

#include <memory>
#include <string>
#include <vector>

#include "state/shared_state.h"
#include "tactics/primary_attacker.h"
#include "debugging/tactic_exception.h"

namespace tactics {

class PassFailPrimaryAttacker : public PrimaryAttacker {
 public:
  PassFailPrimaryAttacker(const string& machine_name,
                          const state::WorldState& world_state,
                          TacticArray* tactic_list,
                          state::SharedState* shared_state,
                          OurRobotIndex our_robot_index,
                          state::SoccerState* soccer_state)
      : PrimaryAttacker(machine_name,
                        world_state,
                        tactic_list,
                        shared_state,
                        our_robot_index,
                        soccer_state) {}

  const char* Name() const override { return "pass_fail_primary_attacker"; }

  void DeflectionSuccess() {
    static const Vector2f goal(kHalfFieldLength, 0);
    const Vector2f& ball_position =
        world_state_.GetBallPosition().filtered_position;
    const float square_dist = (ball_position - goal).squaredNorm();
    if (square_dist < Sq(500)) {
      throw TacticException("Attacker Success");
    }
  }

  void DeflectionFailiure() {
    static constexpr int kMaxIterations = 300;
    static int count = 0;
    if (count > kMaxIterations) {
      throw TacticException("Attacker Failure");
    }
    ++count;
  }

  void Run() override {
    DeflectionFailiure();
    DeflectionSuccess();
    PrimaryAttacker::Run();
  }
};
}  // namespace tactics

#endif  // SRC_TACTICS_PASS_FAIL_PRIMARY_ATTACKER_H_

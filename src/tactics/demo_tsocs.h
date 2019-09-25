// Copyright 2018 - 2019 dbalaban@umass.edu
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
#include <vector>

#include "logging/logger.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/tactic.h"
#include "tactics/test_tsocs.h"
#include "navigation/navigation_util.h"

#ifndef SRC_TACTICS_DEMO_TSOCS_H_
#define SRC_TACTICS_DEMO_TSOCS_H_

namespace tactics {
// This class does what TestTSOCS does except instead of randomly generating
// initial/final conditions, it uses fixed examples. It also only does 1 trial
// at a time.
class DemoTSOCS : public TestTSOCS {
 public:
  DemoTSOCS(const state::WorldState& world_state,
           TacticArray* tactic_list,
           state::SharedState* shared_state, OurRobotIndex our_robot_index,
           state::SoccerState* soccer_state) :
    TestTSOCS::TestTSOCS(world_state,
                         tactic_list,
                         shared_state,
                         our_robot_index,
                         soccer_state) {}
  const char* Name() const override { return "DemoTsocs"; }
 protected:
  void GenerateNewProblem();
  int trials_per_problem = 1;
};
}  // namespace tactics

#endif  // SRC_TACTICS_DEMO_TSOCS_H_

// Copyright 2017 - 2019 slane@cs.umass.edu, kvedder@umass.edu
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
#include "util/random.h"

#include "logging/logger.h"
#include "radio_protocol_wrapper.pb.h"

#ifndef SRC_TACTICS_TACTIC_H_
#define SRC_TACTICS_TACTIC_H_

// Predefining soccer state to avoid circular includes.
namespace state {
class SoccerState;
}

namespace tactics {
class Tactic {
 public:
  Tactic(const state::WorldState& world_state,
         TacticArray* tactic_list,
         state::SharedState* shared_state,
         OurRobotIndex our_robot_index,
         state::SoccerState* soccer_state);

  virtual ~Tactic() {}

  virtual void Init() = 0;
  virtual void SetGoal(const pose_2d::Pose2Df& pose) = 0;

  virtual void Reset() = 0;
  virtual void Run() = 0;
  virtual const char* Name() const = 0;
  // These should be made pure eventually
  virtual void SetParameters(const std::vector<std::string>& parameters);
  virtual float GetCost();
  virtual const pose_2d::Pose2Df GetGoal();

  // Intended for ball-handling tactics to signal completion
  virtual bool IsComplete();

 protected:
  const state::WorldState& world_state_;
  TacticArray* tactic_list_;
  state::SharedState* shared_state_;
  OurRobotIndex our_robot_index_;
  state::SoccerState* soccer_state_;
  util_random::Random random;
};
}  // namespace tactics

#endif  // SRC_TACTICS_TACTIC_H_

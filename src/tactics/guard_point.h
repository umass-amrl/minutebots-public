// Copyright 2017 - 2018 rezecib@gmail.com
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

#ifndef SRC_TACTICS_GUARD_POINT_H_
#define SRC_TACTICS_GUARD_POINT_H_

#include <memory>
#include <string>
#include <vector>

#include "radio_protocol_wrapper.pb.h"
#include "state/shared_state.h"
#include "tactics/tactic.h"


namespace tactics {
class GuardPoint : public Tactic {
 public:
  GuardPoint(const state::WorldState& world_state,
             TacticArray* tactic_list,
             state::SharedState* shared_state, OurRobotIndex our_robot_index,
             state::SoccerState* soccer_state);

  const char* Name() const override { return "guard_point"; }
  void Init() override;
  void Run() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  float GetCost() override;
  void SetParameters(const std::vector<std::string>& parameters) override;

 private:
  Eigen::Vector2f point_;
  float leash_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_GUARD_POINT_H_

// Copyright 2017-2018 slane@cs.umass.edu, kvedder@umass.edu
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

#ifndef SRC_TACTICS_FORWARD_BACKWARD_H_
#define SRC_TACTICS_FORWARD_BACKWARD_H_

#include <memory>
#include <vector>

#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/tactic.h"

namespace tactics {
class ForwardBackward : public Tactic {
 public:
  ForwardBackward(const state::WorldState& world_state,
                  TacticArray* tactic_list, state::SharedState* shared_state,
                  OurRobotIndex our_robot_index,
                  state::SoccerState* soccer_state);

  const char* Name() const override { return "forward_backward"; }
  void Init() override;
  void Run() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;

 private:
  enum ForwardBackwardState {
    GET_TO_INIT_POSE,
    TURNING,
    WAITING,
    FORWARD,
    BACKWARD
  };

  bool IsNearGoal(logger::Logger* logger) const;

  bool IsInWaitPeriod() const;

  ForwardBackwardState execution_state_;

  pose_2d::Pose2Df pose1_;
  pose_2d::Pose2Df pose2_;
  pose_2d::Pose2Df goal_;

  static constexpr float kWaitTime = 0;  // Seconds
  double wait_start_time_;
  ForwardBackwardState last_moving_state_;

  double goal_angle_;

  OurRobotIndex our_robot_index_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_FORWARD_BACKWARD_H_

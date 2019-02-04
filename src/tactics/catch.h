// Copyright 2017-2018 dbalaban@cs.umass.edu
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

#include <algorithm>
#include <memory>
#include <vector>

#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "motion_control/optimal_control_1d.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/tactic.h"

#ifndef SRC_TACTICS_CATCH_H_
#define SRC_TACTICS_CATCH_H_

namespace tactics {
class Catch : public Tactic {
 public:
  Catch(const state::WorldState& world_state,
        TacticArray* tactic_list,
        state::SharedState* shared_state, OurRobotIndex our_robot_index,
        state::SoccerState* soccer_state);

  ~Catch();

  const char* Name() const override { return "catch"; }
  void Init() override;
  void Run() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  const pose_2d::Pose2Df GetGoal() override;
  bool IsAtTargetPosition();
  bool BadTiming();

 private:
  // the target angle of the robot cneter
  bool kDebug_ = true;
  bool bad_time_;

  Eigen::Vector2f desired_accel_world_frame_;

  // const bool kDebug_ = true;
  // const bool use_dribbler = false;
};
}  // namespace tactics

#endif  // SRC_TACTICS_CATCH_H_

// Copyright 2017 - 2018 dbalaban@cs.umass.edu
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
#include "motion_control/ball_interception.h"
#include "motion_control/motion_control_structures.h"
#include "motion_control/ntoc_2d.h"
#include "motion_control/optimal_control_1d.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/tactic.h"

#ifndef SRC_TACTICS_NAVIGATE_TO_INTERCEPT_H_
#define SRC_TACTICS_NAVIGATE_TO_INTERCEPT_H_

namespace tactics {
class NavigateToIntercept : public Tactic {
 public:
  NavigateToIntercept(const state::WorldState& world_state,
                      TacticArray* tactic_list,
                      state::SharedState* shared_state,
                      OurRobotIndex our_robot_index,
                      state::SoccerState* soccer_state);

  ~NavigateToIntercept();

  float FindInterceptNavigationPoint(pose_2d::Pose2Df* target_pose);

  const char* Name() const override { return "navigate_to_intercept"; }
  void Init() override;
  void Run() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void SetSolution(SolutionParameters solution);
  void SetAngle(float angle);

 private:
  void LogControlSequence(logger::Logger* the_log,
                          std::vector<ntoc::ControlPhase1D> control);

  SolutionParameters intercept_solution;
  float target_angle;
  // expected time addition from using NTOC over TSOCS
  const float kNavigationTimeAdjustment = -0.1;
};
}  // namespace tactics

#endif  // SRC_TACTICS_NAVIGATE_TO_INTERCEPT_H_

// Copyright 2017 - 2018 dbalaban@cs.umass.edu.
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

#ifndef SRC_TACTICS_NAVIGATE_TO_CATCH_H_
#define SRC_TACTICS_NAVIGATE_TO_CATCH_H_

namespace tactics {
class NavigateToCatch : public Tactic {
 public:
  NavigateToCatch(const state::WorldState& world_state,
                  TacticArray* tactic_list,
                  state::SharedState* shared_state,
                  OurRobotIndex our_robot_index,
                  state::SoccerState* soccer_state);

  ~NavigateToCatch();

  float FindCatchNavigationPoint(pose_2d::Pose2Df* target_pose);

  const char* Name() const override { return "navigate_to_catch"; }
  void Init() override;
  void Run() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void SetSolution(SolutionParameters solution);

 private:
  void LogControlSequence(logger::Logger* the_log,
                          std::vector<ntoc::ControlPhase1D> control);

  Eigen::Vector2f GetNavigationPoint(Eigen::Vector2f minimum,
                                     Eigen::Vector2f maximum,
                                     unsigned int iterations);

  pose_2d::Pose2Df current_velocity;
  pose_2d::Pose2Df current_pose;
  Vector2f ball_pos;
  Vector2f ball_vel;
  Vector2f ball_dir;
  Vector2f current_velocity_world;

  SolutionParameters intercept_solution;
  // expected time addition from using NTOC over TSOCS
  //   const float kNavigationTimeAdjustment = 0;
  const float kNavigationDistAdjustment = .75;
  const float kNavigationTimeAdjustment = -0.5;
  const float kTimeMargin_ = .01;
  const float kTimeBuffer_ = .1;
  const float kMaxIterations_ = 10;
};
}  // namespace tactics

#endif  // SRC_TACTICS_NAVIGATE_TO_CATCH_H_

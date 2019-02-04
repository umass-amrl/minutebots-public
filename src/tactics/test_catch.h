// Copyright 2017 - 2018 dbalaban@umass.edu
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
#include "motion_control/optimal_control_1d.h"
#include "state/shared_state.h"
#include "state/world_state.h"
#include "tactics/tactic.h"
#include "tactics/tactic_index.h"
#include "evaluators/offense_evaluation.h"

#include "logging/logger.h"
#include "radio_protocol_wrapper.pb.h"

#ifndef SRC_TACTICS_TEST_CATCH_H_
#define SRC_TACTICS_TEST_CATCH_H_

// Predefining soccer state to avoid circular includes.
namespace state {
class SoccerState;
}

namespace tactics {
class TestCatch : public Tactic {
 public:
  TestCatch(const state::WorldState& world_state,
                  TacticArray* tactic_list,
                  state::SharedState* shared_state,
                  OurRobotIndex our_robot_index,
                  state::SoccerState* soccer_state);

  const char* Name() const override { return "test_catch"; }
  void Init() override;
  void Run() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void Reset() override;
  bool ShouldCatch();

 private:
  logger::Logger* the_logger;

  SolutionParameters intercept_solution;

  pose_2d::Pose2Df current_robot_pose;
  pose_2d::Pose2Df current_robot_velocity;
  Vector2f current_ball_pose;
  Vector2f current_ball_velocity;
  Vector2f current_velocity_world;

  Vector2f robot_interception_point;
  Vector2f ball_intercept_point;
  Vector2f velocity_at_intercept;

  // const bool kDebug_ = true;

  const float kBallVelocityThreshold = 100.0;
};
}  // namespace tactics

#endif  // SRC_TACTICS_TEST_CATCH_H_

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
#include "navigation/navigation_util.h"

#ifndef SRC_TACTICS_TEST_TSOCS_H_
#define SRC_TACTICS_TEST_TSOCS_H_

namespace tactics {
class TestTSOCS : public Tactic {
 public:
  TestTSOCS(const state::WorldState& world_state,
           TacticArray* tactic_list,
           state::SharedState* shared_state, OurRobotIndex our_robot_index,
           state::SoccerState* soccer_state);
  ~TestTSOCS();

  const char* Name() const override { return "TestTsocs"; }
  void Init() override;
  void Run() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void Reset() override;

 protected:
  void Wait();
  void Start();
  void Execute();
  void Stop();
  void GotoWait();
  void NewLog();
  void ResetParams();
  void GenerateNewProblem();

  enum ExecutionState {
    WAIT,
    START,
    EXECUTE,
    STOP,
    GOTO_WAIT,
  };

  ExecutionState execution_state = GOTO_WAIT;
  ExecutionState previous_state;

  Eigen::Vector2f start_pos = Vector2f(500, 0);
  Eigen::Vector2f start_vel = Vector2f(1000.0, 0.0);
  Eigen::Vector2f goal_pos = Vector2f(1000.0, 2000.0);
  Eigen::Vector2f goal_vel = Vector2f(800.0, 800.0);
  Eigen::Vector2f wait_pos = Vector2f(1000, -1000);
  Eigen::Vector2f current_goal_pos;
  Eigen::Vector2f current_goal_vel;

  std::vector<Vector2f> planned_trajectory;

  double wait_start_time_;
  const double kWaitDuration_ = 1.0;
  bool isFirstRun;
  FILE* log_file;
  int log_number;
  double goal_orientation = 0.0;
  const unsigned int kNumTrials_ = 20;
  int trials_per_problem = 1;
  int trials_this_problem = trials_per_problem;
  unsigned int num_problems = 0;
};
}  // namespace tactics

#endif  // SRC_TACTICS_TEST_TSOCS_H_

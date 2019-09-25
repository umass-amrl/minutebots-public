// Copyright 2019 dbalaban@umass.edu
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
#include <iostream>
#include <string>

#include "logging/logger.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/tactic.h"
#include "navigation/navigation_util.h"

#ifndef SRC_TACTICS_TRAIN_TSOCS_H_
#define SRC_TACTICS_TRAIN_TSOCS_H_

namespace tactics {
class TrainTSOCS : public Tactic {
 public:
  TrainTSOCS(const state::WorldState& world_state,
            TacticArray* tactic_list,
            state::SharedState* shared_state,
            OurRobotIndex our_robot_index,
            state::SoccerState* soccer_state);
  ~TrainTSOCS();

  const char* Name() const override { return "TrainTsocs"; }
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
  void GetProblem();

  enum ExecutionState {
    WAIT,
    START,
    EXECUTE,
    STOP,
    GOTO_WAIT,
  };

  ExecutionState execution_state = GOTO_WAIT;
  ExecutionState previous_state;

  const Eigen::Vector2f start_pos = Vector2f(2000, -1000);
  const Eigen::Vector2f start_vel = Vector2f(1000.0, 0.0);
  const Eigen::Vector2f wait_pos = Vector2f(1000, -1000);
  Eigen::Vector2f prob_goal_pos;
  Eigen::Vector2f prob_goal_vel;

  std::vector<Vector2f> planned_trajectory;

  double wait_start_time_;
  const double kWaitDuration_ = 1.0;
  bool isFirstRun;
  FILE* log_file;
  int log_number;
  double goal_orientation = 0.0;
  const string problem_filename = "tsocs_problems.txt";
  std::vector<string> problems;
  const unsigned int trials_per_problem = 5;
  unsigned int trials_this_problem;
  unsigned int problem_counter;
};
}  // namespace tactics

#endif  // SRC_TACTICS_TRAIN_TSOCS_H_

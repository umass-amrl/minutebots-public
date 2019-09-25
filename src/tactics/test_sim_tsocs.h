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
#include "debugging/tactic_exception.h"
#include "configuration_reader/reader.h"

#ifndef SRC_TACTICS_TEST_SIM_TSOCS_H_
#define SRC_TACTICS_TEST_SIM_TSOCS_H_

namespace tactics {
class TestSimTSOCS : public Tactic {
 public:
  TestSimTSOCS(const state::WorldState& world_state,
           TacticArray* tactic_list,
           state::SharedState* shared_state, OurRobotIndex our_robot_index,
           state::SoccerState* soccer_state);
  ~TestSimTSOCS();

  const char* Name() const override { return "TestSimTsocs"; }
  void Init() override;
  void Run() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void Reset() override;

 protected:
  void Execute();
  void Stop();
  void NewLog();
  void ResetParams();

  Eigen::Vector2f current_goal_pos;
  Eigen::Vector2f current_goal_vel;
  Eigen::Vector2f goal_pos;
  Eigen::Vector2f goal_vel;
  std::vector<Vector2f> planned_trajectory;

  FILE* log_file;
  int log_number;
  double goal_orientation = 0.0;
};
}  // namespace tactics

#endif  // SRC_TACTICS_TEST_SIM_TSOCS_H_

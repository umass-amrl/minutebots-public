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

#include <memory>
#include <vector>

#include "math/poses_2d.h"
#include "motion_control/tsocs.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/tactic.h"

#ifndef SRC_TACTICS_TSOCS_CONTROLLER_H_
#define SRC_TACTICS_TSOCS_CONTROLLER_H_

namespace tactics {
class TSOCSController : public Tactic {
 public:
  TSOCSController(const state::WorldState& world_state,
                   TacticArray* tactic_list,
                   state::SharedState* shared_state,
                   OurRobotIndex our_robot_index,
                   state::SoccerState* soccer_state);

  const char* Name() const override { return "tsocs_controller"; }
  void Init() override;
  void Run() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void SetGoal(const pose_2d::Pose2Df& pose, const pose_2d::Pose2Df& vel);
  void SetMotionModel(motion::MotionModel motion_model);
  void GetPathPoints(std::vector<Vector2f>* points);
  SolutionParameters GetParams() { return params_; }
  bool Finished() {return finished_; }

 private:
  bool HasNans();

  tsocs::Tsocs tsocs;
  pose_2d::Pose2Df goal_pos_;
  pose_2d::Pose2Df goal_vel_;

  SolutionParameters params_;

  const bool kDebug_ = false;
  motion::MotionModel motion_model_;
  bool finished_ = false;
};
}  // namespace tactics

#endif  // SRC_TACTICS_TSOCS_CONTROLLER_H_

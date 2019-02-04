// Copyright 2018 jaholtz@cs.umass.edu
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
#include <string>

#ifndef SRC_TACTICS_BALL_PLACEMENT_H_
#define SRC_TACTICS_BALL_PLACEMENT_H_

#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"

namespace tactics {


class BallPlacement : public StateMachineTactic {
 public:
  BallPlacement(const string& machine_name,
               const state::WorldState& world_state,
               TacticArray* tactic_list,
               state::SharedState* shared_state, OurRobotIndex our_robot_index,
               state::SoccerState* soccer_state);

  const char* Name() const override { return "ball_placement"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  const pose_2d::Pose2Df GetGoal() override;
  bool IsComplete() override;
  float GetCost() override;
  float target_angle;
  pose_2d::Pose2Df robot_goal_pose;

 private:
  void Transition() override;

  // Behavior Functions Used by the states
  void Start();

  void Setup();

  void Place();

  void Finish();

  // States
  State start_;
  State setup_;
  State place_;
  State finish_;

  // THRESHOLDS TO TUNE
  RepairableParam kThresholdsX_;
  RepairableParam kThresholdsY_;
  RepairableParam kThresholdsAngle_;

  // END THRESHOLDS
  const bool complete_ = false;
  bool target_set_;

  Eigen::Vector2f target_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_BALL_PLACEMENT_H_

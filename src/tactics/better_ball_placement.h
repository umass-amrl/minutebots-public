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

#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"

#ifndef SRC_TACTICS_BETTER_BALL_PLACEMENT_H_
#define SRC_TACTICS_BETTER_BALL_PLACEMENT_H_

namespace tactics {

class BetterBallPlacement : public StateMachineTactic {
 public:
  BetterBallPlacement(const state::WorldState& world_state,
       TacticArray* tactic_list,
       state::SharedState* shared_state, OurRobotIndex our_robot_index,
       state::SoccerState* soccer_state);

  const char* Name() const override { return "better_ball_placement"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  bool IsComplete() override;
  float GetCost() override;

  pose_2d::Pose2Df robot_goal_pose;

 private:
  void Transition() override;

  void Start();

  void Setup();

  // Behavior Functions Used by the states
  void Place();

  void Finished();

  bool is_complete_;

  // States
  State start_;
  State setup_;
  State place_;
  State finished_;

  bool target_set_;
  bool direction_set_;
  Eigen::Vector2f place_start_;

  Eigen::Vector2f target_;

  // THRESHOLDS TO TUNE;
  RepairableParam thresholds_ball_velocity_;
  // Maxmimum distance between robot and ball to kick
  // (mm)
  RepairableParam thresholds_distance_;
  // THRESHOLDS TO TUNE
  RepairableParam kThresholdsX_;
  RepairableParam kThresholdsY_;
  // END THRESHOLDS
};
}  // namespace tactics

#endif  // SRC_TACTICS_BETTER_BALL_PLACEMENT_H_

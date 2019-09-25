// Copyright 2018 - 2019 jaholtz@cs.umass.edu
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

#ifndef SRC_TACTICS_DOCKING_H_
#define SRC_TACTICS_DOCKING_H_

#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"
#include "third_party/json.hpp"

namespace tactics {

class Docking : public StateMachineTactic {
 public:
  Docking(const string& machine_name,
           const state::WorldState& world_state,
           TacticArray* tactic_list, state::SharedState* shared_state,
           OurRobotIndex our_robot_index,
           state::SoccerState* soccer_state);

  const char* Name() const override { return "docking"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  float GetCost() override;
  // States
  State start_;
  State s1_left_;
  State s1_right_;
  State s1_forward_;
  State s2_left_;
  State s2_right_;
  State s2_forward_;
  State backward_;
  State docked_;

 private:
  void Transition() override;
  bool ShouldStage1Forward();
  bool ShouldStage2Forward();
  bool ShouldStage1Right();
  bool ShouldStage2Right();
  bool ShouldStage1Left();
  bool ShouldStage2Left();

  // Behavior Functions Used by the states
  void Start();
  void RotateLeft();
  void RotateRight();
  void Forward();
  void Backward();
  void Docked();
  void SendVelocity();



  Eigen::Vector2f dock_location = {0, 0};
  float dock_angle = 0;

  // THRESHOLDS TO TUNE
  // Maxmimum y offset from goal center (mm)
  RepairableParam s1_align_;
  // Minimum angular offset to the right of target
  // to trigger right rotation. (DEG)
  RepairableParam s1_right_angle_;
  // Minimum angular offset to the left of target
  // to trigger left rotation. (DEG)
  RepairableParam s1_left_angle_;
  // Maximum x offset from perpendicular point to goal (mm)
  RepairableParam perp_y_;
  // Minimum angular offset to the right of target
  // to trigger right rotation. (DEG)
  RepairableParam s2_right_angle_;
  // Minimum angular offset to the left of target
  // to trigger left rotation. (DEG)
  RepairableParam s2_left_angle_;
  // END TUNED THRESHOLDS

  // Thresholds For determining Docking (Not tuned want a definitive finish)
  const float kDockedYThreshold = 25;
  const float kDockOffsetThreshold_ = 25;
  const float kDockRotationThreshold = 10.0;
  const Eigen::Vector2f perp_point;
  // Minimum y offset from goal center.
  const float s1_align_lower_;
};

}  // namespace tactics

#endif  // SRC_TACTICS_DOCKING_H_

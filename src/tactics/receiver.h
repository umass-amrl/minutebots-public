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

#ifndef SRC_TACTICS_RECEIVER_H_
#define SRC_TACTICS_RECEIVER_H_

#include <memory>
#include <string>
#include <vector>

#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"

namespace tactics {

class Receiver : public StateMachineTactic {
 public:
  Receiver(const state::WorldState& world_state, TacticArray* tactic_list,
           state::SharedState* shared_state, OurRobotIndex our_robot_index,
           state::SoccerState* soccer_state);

  const char* Name() const override { return "receiver"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  const pose_2d::Pose2Df GetGoal() override;
  bool IsComplete() override;
  float GetCost() override;
  bool BadTiming();
  float target_angle;
  pose_2d::Pose2Df robot_goal_pose;

 private:
  void Transition() override;

  // Behavior Functions Used by the states
  void Setup();

  void Wait();

  void Catch();

  void Deflect();

  void Finish();

  void GetTarget(const Vector2f& source, float* target_angle);

  bool ShouldReceive();

  bool PassGood();

  bool ShouldDeflect();
  bool IsSetup();

  // States
  State setup_;
  State wait_;
  State catch_;
  State deflect_;
  State pass_failed_;
  State finish_;

  // THRESHOLDS TO TUNE
  const int wait_timeout_threshold_ = 50;
  const float minimum_deflection_angle_ = 90.0;
  RepairableParam thresholds_receive_velocity_;
  RepairableParam thresholds_toward_target_;
  RepairableParam thresholds_toward_robot_;
  RepairableParam thresholds_deflection_angle_;
  RepairableParam thresholds_min_deflection_angle_;
  RepairableParam thresholds_setup_time_;
  RepairableParam thresholds_setup_distance_;
  // END THRESHOLDS

  bool complete_;
  OurRobotIndex last_target_;
  int wait_count_;
  int fail_count_;
  int kFailThreshold_ = 15;
  const bool kDebug = false;
};
}  // namespace tactics

#endif  // SRC_TACTICS_RECEIVER_H_

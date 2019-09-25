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
#include <string>
#include <tuple>
#include <vector>

#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"
#include "third_party/json.hpp"

#ifndef SRC_TACTICS_SIMPLEDEFLECTION_H_
#define SRC_TACTICS_SIMPLEDEFLECTION_H_

namespace tactics {

class SimpleDeflection : public StateMachineTactic {
 public:
  SimpleDeflection(const string& machine_name, const state::WorldState& world_state,
             TacticArray* tactic_list, state::SharedState* shared_state,
             OurRobotIndex our_robot_index, state::SoccerState* soccer_state);

  const char* Name() const override { return "simple_deflection"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  float GetCost() override;
  bool IsComplete() override;
  const pose_2d::Pose2Df GetGoal() override;
  void SetValue(nlohmann::json config_json,
                               RepairableParam* param);
  void LoadConfigFile();
  void CalculateScore();
  void WriteDataFile(const string& output_file);
  float distance_score_ = 0;
  float angle_score_ = 0;

 private:

  void Transition() override;
  void Setup();
  void Kick();
  void CheckSuccess();
  void CheckFailure();

  float GetRobotKickTime(float robot_speed, float current_distance);
  float GetBallArrivalTime(float ball_velocity, float current_distance);
  Vector2f GetInterceptionPoint();
  float GetKickAngle();

  State setup_;
  State kick_;
  RepairableParam p_time_threshold_;
  RepairableParam p_setup_distance_;
  RepairableParam p_desired_accel_t_;
  RepairableParam p_desired_accel_a_;
  RepairableParam p_kick_dampening_;
  RepairableParam p_kick_speed_;
  RepairableParam p_x_damping_;
  RepairableParam p_y_damping_;

  float target_angle_;

  int kick_count_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_SIMPLEDEFLECTION_H_
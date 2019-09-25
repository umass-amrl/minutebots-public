// Copyright 2019 jaholtz@cs.umass.edu
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
#ifndef SRC_SIMPLECAR_SIM_SIMPLECAR_CONTROL_H_
#define SRC_SIMPLECAR_SIM_SIMPLECAR_CONTROL_H_

#include <string>
#include "srtr/state_machine.h"
#include "simplecar_sim/simplecar_sim.h"
#include "logging/logger.h"
#include "third_party/json.hpp"

namespace simplecar {

class SimplecarController : public srtr::StateMachine {
 public:
  SimplecarController(const string& machine_name, const int& id);
  void Transition() override;
  void UpdateState(const SimplecarState& state);
  SimplecarCommand GetControls();
  bool FreeFlying();
  void LoadConfigFile();
  void SetValue(nlohmann::json config_json, RepairableParam* param);
  MinuteBotsProto::StateMachineData GetTransitionLog();

 private:
  const int id_;
  const float switch_steering_;
  SimplecarState world_state_;
  float desired_speed_;  // mm/s
  float desired_steer_;  // angle
  float current_speed_;
  Eigen::Vector2f velocity_;
  pose_2d::Pose2Df pose_;
  float angle_vel_;
  bool  switch_right_;
  bool right_lane_;
  Simplecar f_left_;
  Simplecar f_right_;
  Simplecar b_left_;
  Simplecar b_right_;
  Simplecar same_lead_;
  Simplecar same_rear_;
  Simplecar other_lead_;
  Simplecar other_rear_;

  float other_lead_dist_;
  float other_rear_dist_;
  float same_lead_dist_;
  float same_rear_dist_;
  Eigen::Vector2f other_lead_rel_;
  Eigen::Vector2f other_rear_rel_;
  Eigen::Vector2f same_lead_rel_;
  Eigen::Vector2f same_rear_rel_;

  // State Functions
  void Cruise();
  void Accel();
  void Decel();
  void Switch();

  // States
  State cruise_;
  State accel_;
  State decel_;
  State switch_;

  // Repairable Params
  RepairableParam cruise_speed_;
  RepairableParam merge_space_front_;
  RepairableParam crash_threshold_;
  RepairableParam decel_threshold_;
  RepairableParam riding_thresh_;
//   RepairableParam crash_time_;
//   RepairableParam speed_offset_;
//   RepairableParam merge_space_rear_;
//   RepairableParam merge_speed_rear_;
//   RepairableParam merge_speed_front_;

  bool ShouldAccel();
  bool ShouldDecel();
  bool ShouldSwitch();
  bool SwitchComplete();
  void UpdateNeighbors();
  void UpdateRelatives();
};

}  // namespace simplecar

#endif  // SRC_SIMPLECAR_SIM_SIMPLECAR_CONTROL_H_

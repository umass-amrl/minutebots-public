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

#include "simplecar_sim/simplecar_control.h"
#include <math.h>
#include <algorithm>
#include <functional>
#include <string>
#include <vector>

using srtr::StateMachine;
using std::vector;

static const int kNoCarId = -1;

namespace simplecar {

SimplecarController::SimplecarController(const string& machine_name,
                                         const int& id) :
    StateMachine(machine_name),
    id_(id),
    switch_steering_(.223599),
    right_lane_(true),
    f_left_(kNoCarId),
    f_right_(kNoCarId),
    b_left_(kNoCarId),
    b_right_(kNoCarId),
    same_lead_(kNoCarId),
    same_rear_(kNoCarId),
    other_lead_(kNoCarId),
    other_rear_(kNoCarId),
    cruise_(std::bind(&SimplecarController::Cruise, this), "Cruise"),
    accel_(std::bind(&SimplecarController::Accel, this), "Accel"),
    decel_(std::bind(&SimplecarController::Decel, this), "Decel"),
    switch_(std::bind(&SimplecarController::Switch, this), "Switch"),
    cruise_speed_(0.3500000029802322, "CruiseSpeed", this),
    merge_space_front_(0.009999999776482582, "MergeSpaceFront", this),
    crash_threshold_(5000.0, "CrashThreshold", this),
    decel_threshold_(6566.48681641, "DecelThreshold", this),
    riding_thresh_(3000.0, "RidingThreshold", this) {
  state_ = cruise_;
  LoadConfigFile();
}

void SimplecarController::SetValue(nlohmann::json config_json,
                               RepairableParam* param) {
  string name = param->name_;
  auto it = config_json.find(name);
  if (it != config_json.end()) {
    std::cout << name << std::endl;
    param->SetValue(config_json[name].get<float>());
  }
}

MinuteBotsProto::StateMachineData SimplecarController::GetTransitionLog() {
  return log_message_;
}


void SimplecarController::LoadConfigFile() {
  std::ifstream json_file("src/simplecar_sim/passing_config.json");
  nlohmann::json config_json;
  json_file >> config_json;
  SetValue(config_json, &cruise_speed_);
  SetValue(config_json, &merge_space_front_);
  SetValue(config_json, &crash_threshold_);
  SetValue(config_json, &decel_threshold_);
  SetValue(config_json, &riding_thresh_);
}

SimplecarCommand SimplecarController::GetControls() {
  SimplecarCommand command;
  command.car_id = id_;
  command.desired_speed = desired_speed_;
  command.desired_steer = desired_steer_;
  return command;
}

void SimplecarController::UpdateState(const SimplecarState& state) {
  world_state_ = state;
  vector<Simplecar> cars = world_state_.GetCars();
  current_speed_ = cars[id_].GetSpeed();
  pose_ = cars[id_].GetPose();
  velocity_ = cars[id_].GetVelocity();
  angle_vel_ = cars[id_].GetAngleVel();
  UpdateNeighbors();
  UpdateRelatives();
}

bool InLeftLane(Eigen::Vector2f translation) {
  if (translation.x() < 0) {
    return true;
  }
  return false;
}

bool SimplecarController::FreeFlying() {
  if (same_lead_.GetId() == kNoCarId) {
    return true;
  } else if (same_lead_.GetSpeed() > static_cast<float>(cruise_speed_)) {
    return true;
  }
  return false;
}

void SimplecarController::UpdateNeighbors() {
  right_lane_ = !InLeftLane(pose_.translation);
  vector<Simplecar> cars = world_state_.GetCars();
  float f_left = __FLT_MAX__;
  float b_left =  -__FLT_MAX__;
  float f_right = __FLT_MAX__;
  float b_right =  -__FLT_MAX__;

  for (Simplecar car : cars) {
    const Eigen::Vector2f car_trans = car.GetPose().translation;
    const float dist = car_trans.y() - pose_.translation.y();
    if (InLeftLane(car_trans)) {
      if (dist > 0.0 && dist < f_left) {
        f_left = dist;
        f_left_ = car;
      } else if (dist < 0.0 && dist > b_left) {
        b_left = dist;
        b_left_ = car;
      }
    } else {
      if (dist > 0.0 && dist < f_right) {
        f_right = dist;
        f_right_ = car;
      } else if (dist < 0.0 && dist > b_right) {
        b_right = dist;
        b_right_ = car;
      }
    }
  }
  if (f_left == __FLT_MAX__) {
    f_left_ = Simplecar(kNoCarId);
  }
  if (f_right == __FLT_MAX__) {
    f_right_ = Simplecar(kNoCarId);
  }
  if (b_left == __FLT_MAX__) {
    b_left_ = Simplecar(kNoCarId);
  }
  if (b_right == __FLT_MAX__) {
    b_right_ = Simplecar(kNoCarId);
  }

  // Update closest cars.
  if (right_lane_) {
    same_lead_ = f_right_;
    same_rear_ = b_right_;
    other_lead_ = f_left_;
    other_rear_ = b_left_;
  } else {
    same_lead_ = f_left_;
    same_rear_ = b_left_;
    other_lead_ = f_right_;
    other_rear_ = b_right_;
  }
}

void SimplecarController::UpdateRelatives() {
  // relative positions
  other_lead_dist_ =
    other_lead_.GetPose().translation.y() - pose_.translation.y();
  other_rear_dist_ =
    other_rear_.GetPose().translation.y() - pose_.translation.y();
  same_rear_dist_ =
    same_rear_.GetPose().translation.y() - pose_.translation.y();
  same_lead_dist_ =
    same_lead_.GetPose().translation.y() - pose_.translation.y();
  if (other_lead_.GetId() == kNoCarId) {
    other_lead_dist_ = __FLT_MAX__;
  }
  if (other_rear_.GetId() == kNoCarId) {
    other_rear_dist_ = -__FLT_MAX__;
  }
  if (same_rear_.GetId() == kNoCarId) {
    same_rear_dist_ = -__FLT_MAX__;
  }
  if (same_lead_.GetId() == kNoCarId) {
    same_lead_dist_ = __FLT_MAX__;
  }
  // relative velocities
  other_lead_rel_ =
    velocity_ - other_lead_.GetVelocity();
  other_rear_rel_ =
    velocity_ - other_rear_.GetVelocity();
  other_rear_rel_ =
      velocity_ - same_rear_.GetVelocity();
  same_lead_rel_ =
    velocity_ - same_lead_.GetVelocity();
}

void SimplecarController::Cruise() {
  // Maintains speed, does nothing else.
  desired_speed_ = current_speed_;
  const float angle = pose_.angle;
  if (angle < DegToRad(90.0)) {
    desired_steer_ = switch_steering_;
  } else if (angle > DegToRad(90.0)) {
    desired_steer_ = -switch_steering_;
  } else {
    desired_steer_ = 0;
  }
}

bool SimplecarController::ShouldAccel() {
  // SET POTENTIAL TRANSITION
  potential_state_ = accel_.name_;
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;

  float lead_crash_time = same_lead_dist_ / same_lead_rel_.y();
  if (lead_crash_time < 0) { lead_crash_time = __FLT_MAX__; }
  const bool crash_free = lead_crash_time > crash_threshold_;
  // If we're riding too much.
  const bool riding_close = same_lead_dist_ > riding_thresh_;
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;

  // Want to reach cruise speed
  const bool too_slow = current_speed_ < cruise_speed_;

  const bool match_merge = false;
  // Smaller crash free
  const bool can_match = false;

  const bool should_accel = (crash_free && riding_close) &&
                            (too_slow || (match_merge && can_match));

  SetTransition(should_accel);
  return should_accel;
}

void SimplecarController::Accel() {
  desired_speed_ = kMaxSpeed;
  const float angle = pose_.angle;
  if (angle < DegToRad(90.0)) {
    desired_steer_ = switch_steering_;
  } else if (angle > DegToRad(90.0)) {
    desired_steer_ = -switch_steering_;
  } else {
    desired_steer_ = 0;
  }
}

bool SimplecarController::ShouldDecel() {
  // SET POTENTIAL TRANSITION
  potential_state_ = decel_.name_;
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;

  // If we're above our cruise speed slow down.
  const bool too_fast = current_speed_ > cruise_speed_;

  // If we're going to hit something slow down.
  float lead_crash_time = same_lead_dist_ / same_lead_rel_.y();
  if (lead_crash_time < 0) { lead_crash_time = __FLT_MAX__; }
  const bool crash_free = lead_crash_time < decel_threshold_;
  //   std::cout << lead_crash_time << std::endl;

  // If we're riding too much.
  const bool riding_close = same_lead_dist_ < riding_thresh_;

  // If we need to match speed to merge.

  const bool should_decel = too_fast || crash_free || riding_close;

  SetTransition(should_decel);
  return should_decel;
}

void SimplecarController::Decel() {
  desired_speed_ = 0.0;
  const float angle = pose_.angle;
  if (angle < DegToRad(90.0)) {
    desired_steer_ = switch_steering_;
  } else if (angle > DegToRad(90.0)) {
    desired_steer_ = -switch_steering_;
  } else {
    desired_steer_ = 0;
  }
}

bool SimplecarController::ShouldSwitch() {
  // We will never change lanes if there isn't another car in this lane.
  if (same_lead_.GetId() != kNoCarId) {
//     std::cout << "Checking switch" << std::endl;
    // SET POTENTIAL TRANSITION
    potential_state_ = switch_.name_;
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;

    // If the car in front is going too slow.
    const bool blocked = same_lead_.GetSpeed() < cruise_speed_;
    const bool more_blocked = other_lead_dist_ < same_lead_dist_;
    const bool front_clear = other_lead_dist_ > merge_space_front_;
    const bool rear_clear = fabs(other_rear_dist_) > merge_space_front_;
    float lead_crash_time = other_lead_dist_ / other_lead_rel_.y();
    float rear_crash_time = other_rear_dist_ / other_rear_rel_.y();
    if (lead_crash_time < 0) { lead_crash_time = __FLT_MAX__; }
    if (rear_crash_time < 0) { rear_crash_time = __FLT_MAX__; }
    lead_crash_time = std::min(lead_crash_time, __FLT_MAX__);
    rear_crash_time = std::min(rear_crash_time, __FLT_MAX__);
    const bool lead_safe = lead_crash_time > decel_threshold_;
    const bool rear_safe = rear_crash_time > decel_threshold_;
    const bool should_switch = blocked && !more_blocked &&
                               front_clear &&
                               rear_clear &&
                               lead_safe &&
                               rear_safe;
    SetTransition(should_switch);
    return should_switch;
  }
  return false;
}

bool SimplecarController::SwitchComplete() {
  // SET POTENTIAL TRANSITION
  potential_state_ = cruise_.name_;
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;

  if (switch_right_) {
  const bool moved = pose_.translation.x() > 700;

  const bool complete = moved;
  SetTransition(complete);

  return complete;
  } else {
    const bool moved = pose_.translation.x() < -700;

    const bool complete = moved;
    SetTransition(complete);

    return complete;
  }
  return false;
}

void SimplecarController::Switch() {
  desired_speed_ = current_speed_;
  if (switch_right_) {
    const float distance = pose_.translation.x();
    if (distance < 500) {
      if (pose_.angle >= DegToRad(70.0)) {
        desired_steer_ = -switch_steering_;
      } else {
        desired_steer_ = 0;
      }
    } else {
      if (pose_.angle < DegToRad(90.0)) {
        desired_steer_ = switch_steering_;
      } else {
        desired_steer_ = 0;
      }
    }
  } else {
    const float distance = pose_.translation.x();
    if (distance > -500) {
      if (pose_.angle <= DegToRad(110.0)) {
        desired_steer_ = switch_steering_;
      } else {
        desired_steer_ = 0;
      }
    } else {
      if (pose_.angle > DegToRad(90.0)) {
        desired_steer_ = -switch_steering_;
      } else {
        desired_steer_ = 0;
      }
    }
  }
}

void SimplecarController::Transition() {
//   std::cout << state_.name_ << std::endl;
//   std::cout << "Speed: " << current_speed_ << std::endl;
//   std::cout << "other_lead_dist_: " << other_lead_dist_ << std::endl;
//   std::cout << "other_rear_dist_: " << other_rear_dist_ << std::endl;
  desired_speed_ = current_speed_;
  desired_steer_ = 0.0;
  if (state_ != switch_) {
    if (ShouldAccel()) {
      state_ = accel_;
    } else if (ShouldSwitch()) {
      switch_right_ = !right_lane_;
      state_ = switch_;
    } else if (ShouldDecel()) {
      state_ = decel_;
    } else {
      state_ = cruise_;
    }
  } else {
    if (SwitchComplete()) {
      state_ = cruise_;
    }
  }
}

}  // namespace simplecar

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

#include "simplecar_sim/simplecar_merge.h"
#include <math.h>
#include <algorithm>
#include <functional>
#include <string>
#include <vector>

using srtr::StateMachine;
using std::vector;

static const int kNoCarId = -1;

namespace simplecar {

SimplecarMerge::SimplecarMerge(const string& machine_name,
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
    wait_(std::bind(&SimplecarMerge::Wait, this), "Wait"),
    cruise_(std::bind(&SimplecarMerge::Cruise, this), "Cruise"),
    merge_(std::bind(&SimplecarMerge::Merge, this), "Merge"),
    cruise_speed_(0.3500000029802322, "CruiseSpeed", this),
    merge_space_front_(3000.0, "MergeSpaceFront", this),
    merge_space_back_(1000.0, "MergeSpaceBack", this),
    crash_threshold_(5000.0, "CrashThreshold", this),
    decel_threshold_(6566.48681641, "DecelThreshold", this),
    wait_dist_threshold_(2000, "WaitDistThreshold", this) {
  state_ = cruise_;
//   LoadConfigFile();
}

void SimplecarMerge::SetValue(nlohmann::json config_json,
                               RepairableParam* param) {
  string name = param->name_;
  auto it = config_json.find(name);
  if (it != config_json.end()) {
    param->SetValue(config_json[name].get<float>());
  }
}

void SimplecarMerge::LoadConfigFile() {
  std::ifstream json_file("src/simplecar_sim/merg_config.json");
  nlohmann::json config_json;
  json_file >> config_json;
  SetValue(config_json, &cruise_speed_);
  SetValue(config_json, &merge_space_front_);
  SetValue(config_json, &crash_threshold_);
  SetValue(config_json, &decel_threshold_);
}

MinuteBotsProto::StateMachineData SimplecarMerge::GetTransitionLog() {
  return log_message_;
}


SimplecarCommand SimplecarMerge::GetControls() {
  SimplecarCommand command;
  command.car_id = id_;
  command.desired_speed = desired_speed_;
  command.desired_steer = desired_steer_;
  return command;
}

void SimplecarMerge::UpdateState(const SimplecarState& state) {
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

bool SimplecarMerge::FreeFlying() {
  const float speed_offset =
    fabs(current_speed_ - static_cast<float>(cruise_speed_));
  return InLeftLane(pose_.translation) &&
         state_ == cruise_ &&
         speed_offset < 20 &&
         fabs(pose_.angle - DegToRad(90.0)) < 0.001;
}

void SimplecarMerge::UpdateNeighbors() {
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
      if (dist >= 0.0 && dist < f_left) {
        f_left = dist;
        f_left_ = car;
      } else if (dist <= 0.0 && dist > b_left) {
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

void SimplecarMerge::UpdateRelatives() {
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

void SimplecarMerge::Cruise() {
  // Maintains speed, does nothing else.
  desired_speed_ = static_cast<float>(cruise_speed_);
  const float angle = pose_.angle;
  if (angle < DegToRad(90.0)) {
    desired_steer_ = switch_steering_;
  } else if (angle > DegToRad(90.0)) {
    desired_steer_ = -switch_steering_;
  } else {
    desired_steer_ = 0;
  }
}

bool SimplecarMerge::ShouldWait() {
  // SET POTENTIAL TRANSITION
  potential_state_ = wait_.name_;
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;

  // If we are too close to the end of our lane
  const float end_dist = kLaneEnd - pose_.translation.y();
  const bool out_of_lane = end_dist > 0 && end_dist < wait_dist_threshold_;
  SetTransition(out_of_lane);
  return out_of_lane;
}

void SimplecarMerge::Wait() {
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

bool SimplecarMerge::SwitchComplete() {
  // SET POTENTIAL TRANSITION
  potential_state_ = cruise_.name_;
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  const bool moved = pose_.translation.x() < -700;

  const bool complete = moved;
  SetTransition(complete);

  return complete;
}

void SimplecarMerge::Switch() {
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

bool SimplecarMerge::ShouldMerge() {
  // Never try to merge from the left lane.
  if (!InLeftLane(pose_.translation)) {
    // SET POTENTIAL TRANSITION
    potential_state_ = merge_.name_;
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;


    const bool front_clear = fabs(other_lead_dist_) > merge_space_front_;
    const bool rear_clear = fabs(other_rear_dist_) > merge_space_back_;
    float lead_crash_time = other_lead_dist_ / other_lead_rel_.y();
    float rear_crash_time = other_rear_dist_ / other_rear_rel_.y();
    if (lead_crash_time < 0) { lead_crash_time = __FLT_MAX__; }
    if (rear_crash_time < 0) { rear_crash_time = __FLT_MAX__; }
    lead_crash_time = std::min(lead_crash_time, __FLT_MAX__);
    rear_crash_time = std::min(rear_crash_time, __FLT_MAX__);
    const bool lead_safe = lead_crash_time > decel_threshold_;
    const bool rear_safe = rear_crash_time > decel_threshold_;
    const bool should_switch = front_clear &&
        rear_clear &&
        lead_safe &&
        rear_safe;
    SetTransition(should_switch);
    return should_switch;
  }
  return false;
}

void SimplecarMerge::Merge() {
  desired_speed_ = static_cast<float>(cruise_speed_);
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

void SimplecarMerge::Transition() {
  desired_speed_ = current_speed_;
  desired_steer_ = 0.0;
  if (state_ != merge_) {
    if (ShouldMerge()) {
      state_ = merge_;
    } else if (ShouldWait()) {
      state_ = wait_;
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

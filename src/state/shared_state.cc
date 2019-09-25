// Copyright 2017-2019 slane@cs.umass.edu, kvedder@umass.edu
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

#include "state/shared_state.h"

#include <iomanip>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "logging/logger.h"
#include "radio_protocol_wrapper.pb.h"
#include "state/world_state.h"
#include "state_estimation/default_motion_model.h"

STANDARD_USINGS;
using datastructures::OptionalValue;
using datastructures::OptionalValueMutable;
using Eigen::Rotation2Df;
using geometry::SafeVectorNorm;
using logger::Logger;
using math_util::Sign;
using pose_2d::Pose2Df;

namespace state {

static std::fstream commanded_velocity_file_;

SharedRobotState::SharedRobotState(const OurRobotIndex our_robot_index,
                                   const SSLVisionId ssl_vision_id)
    : enabled(false),
      our_robot_index(our_robot_index),
      ssl_vision_id(ssl_vision_id),
      velocity_x(0),
      velocity_y(0),
      velocity_r(0),
      acceleration_command(0, 0, 0),
      max_velocity(kMaxRobotRotVel, kMaxRobotVelocity, kMaxRobotVelocity),
      dss_changed_command(false),
      flat_kick(0),
      flat_kick_set(false),
      chip_kick(0),
      chip_kick_set(false),
      dribbler_spin(0),
      dribbler_set(false),
      should_stop_linear(false),
      should_stop_angular(false),
      cmd_time(0) {}

SharedState::SharedState()
    : pass_set_(false),
      pass_target_(42),
      pass_location_({0, 0}),
      pass_shot_(false) {}

SharedState::~SharedState() {}

void SharedState::Init() {
  shared_state_list.clear();
  for (size_t i = 0; i < kMaxTeamRobots; ++i) {
    shared_state_list.push_back(SharedRobotState(i, 0));
  }
}

void SharedState::ResetCommands(const state::PositionVelocityState& pvs) {
  size_t pvs_index = 0;
  for (SharedRobotState& shared_state : shared_state_list) {
    const auto& robot_pv = pvs.GetOurTeamRobots().Get(pvs_index);
    shared_state.enabled = (robot_pv.confidence > 0);
    shared_state.ssl_vision_id = robot_pv.ssl_vision_id;
    shared_state.our_robot_index = pvs_index;
    shared_state.velocity_x = 0;
    shared_state.velocity_y = 0;
    shared_state.velocity_r = 0;
    shared_state.acceleration_command = {0, 0, 0};
    shared_state.max_velocity = {
        kMaxRobotRotVel, kMaxRobotVelocity, kMaxRobotVelocity};
    shared_state.dss_changed_command = false;
    shared_state.flat_kick = 0;
    shared_state.flat_kick_set = false;
    shared_state.chip_kick = 0;
    shared_state.chip_kick_set = false;
    shared_state.dribbler_spin = 0;
    shared_state.dribbler_set = false;
    shared_state.should_stop_angular = false;
    shared_state.should_stop_linear = false;
    ++pvs_index;
  }

  // Sanitize those out of index with the current position velocity set.
  for (size_t i = pvs_index; i < shared_state_list.size(); ++i) {
    auto& shared_state = shared_state_list[i];
    shared_state.enabled = false;
    shared_state.ssl_vision_id = 0;
    shared_state.our_robot_index = pvs_index;
    shared_state.velocity_x = 0;
    shared_state.velocity_y = 0;
    shared_state.velocity_r = 0;
    shared_state.acceleration_command = {0, 0, 0};
    shared_state.max_velocity = {
        kMaxRobotRotVel, kMaxRobotVelocity, kMaxRobotVelocity};
    shared_state.dss_changed_command = false;
    shared_state.flat_kick = 0;
    shared_state.flat_kick_set = false;
    shared_state.chip_kick = 0;
    shared_state.chip_kick_set = false;
    shared_state.dribbler_spin = 0;
    shared_state.dribbler_set = false;
    shared_state.should_stop_angular = false;
    shared_state.should_stop_linear = false;
  }
}

RadioProtocolWrapper SharedState::ConvertToRadioWrapper(
    const WorldState& world_state, Logger* logger) {
  RadioProtocolWrapper message;

  logger->LogPrintPush("Planned Velocities");

  for (SharedRobotState& shared_state : shared_state_list) {
    if (shared_state.enabled) {
      RadioProtocolCommand* command = message.add_command();

      Pose2Df max_velocity(shared_state.max_velocity);

      if (shared_state.dss_changed_command) {
        max_velocity = {kMaxRobotRotVel, kMaxRobotVelocity, kMaxRobotVelocity};
      }

      if (shared_state.acceleration_command.translation.norm() >
          kMaxRobotAcceleration) {
      }
      
      Pose2Df planned_robot_velocity =
          GetNextCommand(world_state,
                         shared_state.acceleration_command,
                         max_velocity,
                         shared_state.our_robot_index,
                         kTransmitPeriodSeconds,
                         kTransmitPeriodSeconds,
                         logger);

      Pose2Df commanded_robot_velocity =
          GetNextCommand(world_state,
                         shared_state.acceleration_command,
                         max_velocity,
                         shared_state.our_robot_index,
                         kControlPeriodTranslation,
                         kControlPeriodRotation,
                         logger);

      if (shared_state.should_stop_linear) {
        shared_state.velocity_x = 0.0;
        shared_state.velocity_y = 0.0;
        command->set_velocity_x(0.0);
        command->set_velocity_y(0.0);
      } else {
        shared_state.velocity_x = planned_robot_velocity.translation.x();
        shared_state.velocity_y = planned_robot_velocity.translation.y();
        command->set_velocity_x(commanded_robot_velocity.translation.x() /
                                1000.0f);
        command->set_velocity_y(commanded_robot_velocity.translation.y() /
                                1000.0f);
      }

      if (shared_state.should_stop_angular) {
        shared_state.velocity_r = 0.0;
        command->set_velocity_r(0.0);
      } else {
        shared_state.velocity_r = planned_robot_velocity.angle;
        command->set_velocity_r(commanded_robot_velocity.angle);
      }
      command->set_robot_id(shared_state.ssl_vision_id);

      if (shared_state.dribbler_set) {
        command->set_dribbler_spin(shared_state.dribbler_spin);
      }
      if (shared_state.flat_kick_set) {
        command->set_flat_kick(shared_state.flat_kick);
      } else if (shared_state.chip_kick_set) {
        command->set_chip_kick(shared_state.chip_kick);
      }

      logger->LogPrint("ID: %d to (%.5f, %.5f, %.5f deg/s)\n",
                       command->robot_id(),
                       shared_state.velocity_x,
                       shared_state.velocity_y,
                       RadToDeg(shared_state.velocity_r));
    }
  }

  logger->Pop();

  return message;
}

Pose2Df SharedState::GetNextCommand(const WorldState& world_state,
                                    const Pose2Df& acceleration,
                                    const Pose2Df& max_velocity,
                                    OurRobotIndex our_robot_index,
                                    float delta_t_translation,
                                    float delta_t_rotation,
                                    Logger* logger) {
  static const bool kDebug = false;
  Pose2Df next_command;

  Pose2Df current_robot_velocity =
      world_state.GetOurRobotPosition(our_robot_index).velocity;
  Pose2Df current_pose =
      world_state.GetOurRobotPosition(our_robot_index).position;

  Pose2Df current_world_velocity(current_robot_velocity);
  current_world_velocity.translation =
      Rotation2Df(current_pose.angle) * current_robot_velocity.translation;

  if (kDebug) {
    logger->LogPrint("Max Vel: %f, %f, %f",
                     max_velocity.translation.x(),
                     max_velocity.translation.y(),
                     max_velocity.angle);
  }

  Pose2Df next_world_velocity;
  next_world_velocity.translation =
      acceleration.translation * delta_t_translation +
      current_world_velocity.translation;

  if (fabs(next_world_velocity.translation.x()) >
      max_velocity.translation.x()) {
    next_world_velocity.translation.x() =
        Sign(next_world_velocity.translation.x()) *
        max_velocity.translation.x();
  }

  if (fabs(next_world_velocity.translation.y()) >
      max_velocity.translation.y()) {
    next_world_velocity.translation.y() =
        Sign(next_world_velocity.translation.y()) *
        max_velocity.translation.y();
  }

  next_world_velocity.angle =
      acceleration.angle * delta_t_rotation + current_world_velocity.angle;

  //   if (fabs(next_world_velocity.angle) > max_velocity.angle) {
  //     next_world_velocity.angle =
  //         Sign(next_world_velocity.angle) * kMaxRobotRotVel;
  //   }

  if (kDebug) {
    logger->LogPrint("Current Velocity World Frame: %f, %f, %f",
                     current_world_velocity.translation.x(),
                     current_world_velocity.translation.y(),
                     current_world_velocity.angle);
    logger->LogPrint("Next Velocity World Frame: %f, %f, %f",
                     next_world_velocity.translation.x(),
                     next_world_velocity.translation.y(),
                     next_world_velocity.angle);
  }

  LogCommandedVelocityToFile(
      world_state,
      world_state.GetOurRobotPosition(our_robot_index).ssl_vision_id,
      next_world_velocity);

  world_state.GetOurMotionModel(our_robot_index)
      .ConvertToRobotFrame(delta_t_translation,
                           current_pose.angle,
                           current_world_velocity.angle,
                           acceleration.angle,
                           next_world_velocity,
                           &next_command,
                           logger);
  return next_command;
}

SharedRobotState* SharedState::GetSharedState(OurRobotIndex our_robot_index) {
  if (our_robot_index < shared_state_list.size()) {
    return &(shared_state_list[our_robot_index]);
  } else {
    return nullptr;
  }
}

SharedRobotState* SharedState::GetSharedStateByID(SSLVisionId ssl_vision_id) {
  for (auto& robot : shared_state_list) {
    if (robot.ssl_vision_id == ssl_vision_id) {
      return &robot;
    }
  }
  return nullptr;
}

int SharedState::GetNumRobots() { return shared_state_list.size(); }

vector<SharedRobotState>* SharedState::GetMutableSharedStates() {
  return &shared_state_list;
}

const vector<SharedRobotState>& SharedState::GetSharedStatesRef() const {
  return shared_state_list;
}

void SharedState::SetCommandTime(double time) {
  for (SharedRobotState& robot : shared_state_list) {
    robot.cmd_time = time;
  }
  cmd_time_ = time;
}

double SharedState::GetCommandTime() const { return cmd_time_; }

bool SharedState::IsPass() { return pass_set_; }

OurRobotIndex SharedState::GetPassTarget() { return pass_target_; }

Eigen::Vector2f SharedState::GetPassLocation() { return pass_location_; }

void SharedState::SetPass(const OurRobotIndex& pass_target,
                          const Eigen::Vector2f& pass_location) {
  pass_set_ = true;
  NP_CHECK(pass_target < kMaxTeamRobots);
  pass_target_ = pass_target;
  pass_location_ = pass_location;
  pass_shot_ = false;
}

void SharedState::ClearPass() {
  pass_set_ = false;
  pass_target_ = 42;
  pass_shot_ = false;
}

void SharedState::SetPassShot() { pass_shot_ = true; }

bool SharedState::GetPassShot() const { return pass_shot_; }

pose_2d::Pose2Dd SharedState::GetCommandByID(SSLVisionId ssl_vision_id) const {
  for (auto& robot : shared_state_list) {
    if (robot.ssl_vision_id == ssl_vision_id) {
      return pose_2d::Pose2Dd(static_cast<double>(robot.velocity_r),
                              static_cast<double>(robot.velocity_x),
                              static_cast<double>(robot.velocity_y));
    }
  }

  return pose_2d::Pose2Df(0, 0, 0);
}

void SharedState::LogCommandedVelocityToFile(
    const WorldState& world_state,
    const SSLVisionId& ssl_vision_id,
    const Pose2Df& commanded_velocity) {
  if (!kLogCommandedVelocity) {
    return;
  }

  if (!commanded_velocity_file_.is_open()) {
    commanded_velocity_file_.open("shared_state_commanded_velocities.txt",
                                  std::ios::out);
  }

  if (!commanded_velocity_file_) {
    return;
  }

  commanded_velocity_file_ << std::setprecision(20) << world_state.world_time_
                           << ", " << ssl_vision_id << ", "
                           << commanded_velocity.translation.x() << ", "
                           << commanded_velocity.translation.y() << ", "
                           << commanded_velocity.angle << '\n';
}

}  // namespace state

// Copyright 2018 slane@cs.umass.edu
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

#include "open_loop_executors/open_loop_joystick.h"

#include <atomic>
#include <thread>

#include "constants/constants.h"
#include "constants/typedefs.h"
#include "joystick/joystick.h"
#include "logging/logger.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "state/position_velocity_state.h"
#include "state/soccer_state.h"
#include "state/team.h"
#include "state/world_state.h"
#include "thread_safe/thread_safe_actor.h"
#include "thread_safe/thread_safe_queue.h"

using joystick::Joystick;
using logger::Logger;
using math_util::Sign;
using net::UDPMulticastServer;
using pose_2d::Pose2Df;
using threadsafe::ThreadSafeActor;
using threadsafe::ThreadSafeQueue;
using team::Team;
using state::PositionVelocityState;
using state::SharedRobotState;
using state::SharedState;
using state::SoccerState;
using state::WorldState;
using std::atomic_bool;
using std::thread;

namespace open_loop {

OpenLoopJoystick::OpenLoopJoystick(
    SSLVisionId id,
    ThreadSafeActor<PositionVelocityState>* thread_safe_position_velocity_state,
    ThreadSafeActor<Logger>* thread_safe_kalman_logger,
    ThreadSafeQueue<SharedState>* thread_safe_shared_state_queue,
    const Team& team)
    : joystick_(new Joystick()),
      id_(id),
      thread_safe_position_velocity_state_(thread_safe_position_velocity_state),
      thread_safe_kalman_logger_(thread_safe_kalman_logger),
      thread_safe_shared_state_queue_(thread_safe_shared_state_queue),
      team_(team),
      logger_(logger::NetLogger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT)) {
  is_running_ = true;
  local_position_velocity_state_.SetTime(GetWallTime());
}
void OpenLoopJoystick::RunJoystick() {
  UDPMulticastServer udp_server;
  if (!udp_server.Open(DATA_STREAM_CMD_IP, DATA_STREAM_CMD_PORT, false)) {
    is_running_ = false;
    LOG(FATAL) << "Error opening UDP port for commands. Exiting";
  }

  WorldState world_state(&local_position_velocity_state_, team_, false);
  SoccerState soccer_state(world_state, team_);

  joystick_->Open(kJoystickID);
  RateLoop loop(kTransmitFrequency);

  while (is_running_) {
    loop.Sleep();
    const state::PositionVelocityState last_state =
        local_position_velocity_state_;
    if (thread_safe_position_velocity_state_->ReadOrDefault(
            &local_position_velocity_state_)) {
      world_state.UpdateLastState(last_state);
    }
    world_state.UpdateState(&logger_);
    soccer_state.GetMutableSharedState()->ResetCommands(
        local_position_velocity_state_);
    soccer_state.UpdateExistances();

    if (local_position_velocity_state_.ContainsOurRobot(id_)) {
      RadioProtocolWrapper message;
      OurRobotIndex our_robot_index = world_state.GetOurRobotIndex(id_);

      UpdateJoystick(
          soccer_state.GetMutableSharedState()->GetSharedState(our_robot_index),
          &message);

      soccer_state.GetMutableSharedState()->SetCommandTime(
          world_state.world_time_);

      thread_safe_shared_state_queue_->Add(soccer_state.GetSharedState());
      world_state.AddSharedState(soccer_state.GetSharedState());

      udp_server.SendProtobuf(message);
    }

    Logger kalman_logger;
    thread_safe_kalman_logger_->ReadOrDefault(&kalman_logger);
    logger_.MergeLoggers(kalman_logger);
    logger_.SetMessageTime(GetWallTime());
    if (team_ == Team::YELLOW) {
      logger_.SetTeam(true);
    } else {
      logger_.SetTeam(false);
    }
    logger_.SendData();
    logger_.Clear();
  }
  joystick_->Close();
}
void OpenLoopJoystick::Start() {
  execution_thread_ = thread(&OpenLoopJoystick::RunJoystick, this);
}

void OpenLoopJoystick::Stop() {
  is_running_ = false;
  execution_thread_.join();
}

float ApplyDeadZone(float axis, float dead_zone, float scale) {
  if (axis < dead_zone && axis > -dead_zone) {
    return 0;
  }
  if (axis > dead_zone) {
    return (scale * (axis - dead_zone));
  }
  if (axis < -dead_zone) {
    return (scale * (axis + dead_zone));
  }
  return 0;
}

void OpenLoopJoystick::UpdateJoystick(SharedRobotState* robot_state,
                                      RadioProtocolWrapper* wrapper) {
  RadioProtocolCommand* command = wrapper->add_command();

  static const int kDeadZone = 8000;
  static const int kKickTriggerThreshold = 30000;

  if (joystick_->ProcessEvents(0) < 0) {
    return;
  }

  if (joystick_->axes[5] > kKickTriggerThreshold) {
    // Issue a flat kick command.
    robot_state->flat_kick = 6.5;
    robot_state->flat_kick_set = true;
    command->set_flat_kick(6.5);
  } else if (joystick_->axes[2] > kKickTriggerThreshold) {
    // Issue a flat kick command.
    robot_state->chip_kick = 2.0;
    robot_state->chip_kick_set = true;
    command->set_chip_kick(2.0);
  }

  if (joystick_->buttons[5]) {
    // The sensorless BLDC driver board firmware is currently hard-coded to a
    // specific speed. The main board will just check if this is non-zero.
    robot_state->dribbler_spin = -1;
    robot_state->dribbler_set = true;
    command->set_dribbler_spin(-1);
  }

  Vector2f translational_velocity(0, 0);
  translational_velocity.x() =
      ApplyDeadZone(-static_cast<float>(joystick_->axes[4]), kDeadZone,
                    kMaxRobotVelocity / (32768.0f - kDeadZone));

  translational_velocity.y() =
      ApplyDeadZone(-static_cast<float>(joystick_->axes[3]), kDeadZone,
                    kMaxRobotVelocity / (32768.0f - kDeadZone));

  float r_velocity =
      ApplyDeadZone(-static_cast<float>(joystick_->axes[0]), kDeadZone,
                    kMaxRobotRotVel / (32768.0f - kDeadZone));

  if ((translational_velocity - last_translational_velocity_).norm() /
          kTransmitPeriodSeconds >
      kMaxRobotAcceleration) {
    translational_velocity =
        (translational_velocity - last_translational_velocity_).normalized() *
        kMaxRobotAcceleration / kTransmitPeriodSeconds;
  }

  if (fabs(r_velocity - last_rotational_velocity_) > kMaxRobotRotAccel) {
    r_velocity =
        Sign(r_velocity - last_rotational_velocity_) * kMaxRobotRotAccel;
  }

  robot_state->velocity_x = translational_velocity.x();
  robot_state->velocity_y = translational_velocity.y();
  robot_state->velocity_r = r_velocity;

  command->set_robot_id(id_);
  command->set_velocity_x(translational_velocity.x() / 1000.0f);
  command->set_velocity_y(translational_velocity.y() / 1000.0f);
  command->set_velocity_r(r_velocity);

  last_translational_velocity_ = translational_velocity;
  last_rotational_velocity_ = r_velocity;
}

}  // namespace open_loop

// Copyright 2018 - 2019 slane@cs.umass.edu
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

#include "open_loop_executors/ramp_controller.h"

#include <atomic>
#include <thread>

#include "constants/constants.h"
#include "constants/includes.h"
#include "constants/typedefs.h"
#include "math/geometry.h"
#include "math/poses_2d.h"
#include "net/netraw.h"
#include "radio_protocol_wrapper.pb.h"
#include "state/team.h"
#include "thread_safe/thread_safe_actor.h"

STANDARD_USINGS;
using geometry::Heading;
using geometry::SafeVectorNorm;
using geometry::ScalarProjection;
using pose_2d::Pose2Df;
using threadsafe::ThreadSafeActor;
using std::fprintf;
using std::fopen;
using std::thread;

namespace open_loop {
RampController::RampController(
    SSLVisionId robot_id, bool is_translation,
    ThreadSafeActor<Observation>* thread_safe_observation)
    : robot_id_(robot_id),
      is_translation_(is_translation),
      thread_safe_observation_(thread_safe_observation),
      is_running_(true),
      execution_count_(0),
      current_velocity_(0),
      is_instantiated_(false),
      prev_obs_vel_(0) {
  execution_state_ = ACCEL_FORWARD;
  previous_observation_.time = GetWallTime();
  previous_observation_.obs = Pose2Df(0, 0, 0);

  if (is_translation_) {
    desired_velocity_ = 2500;
    configuration_acceleration_ = 3000 * kTransmitPeriodSeconds;
  } else {
    desired_velocity_ = M_PI;
    configuration_acceleration_ = 6.0 * M_PI * kTransmitPeriodSeconds;
  }
}

void RampController::Run() {
  RateLoop loop(kTransmitFrequency);
  Observation observation;

  Pose2Df command(0, 0, 0);

  while (is_running_) {
    thread_safe_observation_->ReadOrDefault(&observation);

    if (!is_instantiated_) {
      if (observation.time > 1.0 || true) {
        previous_observation_ = observation;
        is_instantiated_ = true;
        loop.Sleep();
      } else {
        // Sleep 5 milliseconds
        Sleep(0.05);
        LOG(INFO) << "No observation, sleeping;";
      }
      continue;
    }

    GetNextVelocity();
    double current_time = GetWallTime();
    if (is_translation_) {
      command.translation.x() = current_velocity_;
      PrintTranslational(observation, command, current_time);
    } else {
      command.angle = current_velocity_;
      PrintRotational(observation, command, current_time);
    }

    SendCommand(command);
    previous_observation_ = observation;
    loop.Sleep();
  }
}

void RampController::Start() {
  command_server_.Open(DATA_STREAM_CMD_IP, DATA_STREAM_CMD_PORT, false);
  file_ = fopen("ramp_controller.csv", "w");
  control_thread_ = thread(&RampController::Run, this);
}

void RampController::Stop() { is_running_ = false; }

void RampController::SendCommand(Pose2Df velocity) {
  RadioProtocolWrapper message;

  RadioProtocolCommand* command = message.add_command();
  command->set_robot_id(robot_id_);
  command->set_velocity_x(velocity.translation.x() / 1000.0);
  command->set_velocity_y(velocity.translation.y() / 1000.0);
  command->set_velocity_r(velocity.angle);
  LOG(INFO) << "Vel: " << velocity.translation.x() / 1000.0;
  command_server_.SendProtobuf(message);
}

void RampController::GetNextVelocity() {
  if (fabs(current_velocity_) < desired_velocity_) {
    if (execution_state_ == WAIT_BACKWARD) {
      if (execution_count_ >= 5) {
        execution_state_ = ACCEL_FORWARD;
        execution_count_ = 0;
      } else {
        execution_count_++;
        current_velocity_ = 0;
      }
    } else if (execution_state_ == WAIT_FORWARD) {
      if (execution_count_ >= 5) {
        execution_state_ = ACCEL_BACKWARD;
        execution_count_ = 0;
      } else {
        execution_count_++;
        current_velocity_ = 0;
      }
    }

    if (execution_state_ == ACCEL_FORWARD) {
      current_velocity_ += configuration_acceleration_;
    } else if (execution_state_ == DECEL_BACKWARD) {
      current_velocity_ += configuration_acceleration_;

      if (current_velocity_ >= 0.0f) {
        current_velocity_ = 0.0f;
        execution_state_ = WAIT_BACKWARD;
      }
    } else if (execution_state_ == ACCEL_BACKWARD) {
      current_velocity_ -= configuration_acceleration_;
    } else if (execution_state_ == DECEL_FORWARD) {
      current_velocity_ -= configuration_acceleration_;

      if (current_velocity_ <= 0.0f) {
        current_velocity_ = 0.0f;
        execution_state_ = WAIT_FORWARD;
      }
    }
  } else if (fabs(current_velocity_) >= desired_velocity_) {
    if (execution_count_ >= 32) {
      if (execution_state_ == CRUISE_FORWARD) {
        execution_state_ = DECEL_FORWARD;
        current_velocity_ -= configuration_acceleration_;
        execution_count_ = 0;
      } else if (execution_state_ == CRUISE_BACKWARD) {
        execution_state_ = DECEL_BACKWARD;
        current_velocity_ += configuration_acceleration_;
        execution_count_ = 0;
      }
    } else {
      if (execution_state_ == ACCEL_FORWARD) {
        execution_state_ = CRUISE_FORWARD;
        current_velocity_ = desired_velocity_;
      } else if (execution_state_ == ACCEL_BACKWARD) {
        execution_state_ = CRUISE_BACKWARD;
        current_velocity_ = -desired_velocity_;
      }
      execution_count_++;
    }
  }
}

void RampController::PrintRotational(Observation obs, Pose2Df command,
                                     double current_time) {
  float obs_velocity;
  if (obs.time - previous_observation_.time > kEpsilon) {
    obs_velocity = AngleDiff(obs.obs.angle, previous_observation_.obs.angle) /
                   (obs.time - previous_observation_.time);
    prev_obs_vel_ = obs_velocity;
  } else {
    obs_velocity = prev_obs_vel_;
  }
  fprintf(file_, "%f, %f, %f, %f\n", obs.time, obs_velocity, current_time,
          current_velocity_);
}

void RampController::PrintTranslational(Observation obs, Pose2Df command,
                                        double current_time) {
  float obs_velocity;
  if (obs.time - previous_observation_.time > kEpsilon) {
    Vector2f velocity =
        obs.obs.translation - previous_observation_.obs.translation;
    Vector2f robot_dir = Heading(obs.obs.angle);

    obs_velocity = ScalarProjection(velocity, robot_dir) /
                   (obs.time - previous_observation_.time);
    prev_obs_vel_ = obs_velocity;
  } else {
    obs_velocity = prev_obs_vel_;
  }

  fprintf(file_, "%f, %f, %f, %f\n", obs.time, obs_velocity, current_time,
          current_velocity_);
}
}  // namespace open_loop

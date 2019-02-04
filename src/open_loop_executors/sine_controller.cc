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

#include "open_loop_executors/sine_controller.h"

#include <atomic>
#include <thread>

#include "constants/constants.h"
#include "constants/typedefs.h"
#include "constants/includes.h"
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
SineController::SineController(SSLVisionId robot_id,
                               bool is_translation,
                               ThreadSafeActor<Observation>*
                                   thread_safe_observation)
  : robot_id_(robot_id),
    is_translation_(is_translation),
    thread_safe_observation_(thread_safe_observation),
    is_running_(true),
    current_velocity_(0),
    start_time_(0),
    is_instantiated_(false),
    prev_obs_vel_(0) {
  previous_observation_.time = GetWallTime();
  previous_observation_.obs = Pose2Df(0, 0, 0);

  if (is_translation_) {
    desired_velocity_ = 1000;
  } else {
    desired_velocity_ = 2*M_PI;
  }
}

void SineController::Run() {
  RateLoop loop(kTransmitFrequency);
  Observation observation;

  Pose2Df command(0, 0, 0);

  while (is_running_) {
    thread_safe_observation_->ReadOrDefault(&observation);

    if (!is_instantiated_) {
      if (observation.time > 1.0) {
        previous_observation_ = observation;
        is_instantiated_ = true;
        // Sleep 5 milliseconds
        start_time_ = GetWallTime();
        loop.Sleep();
      } else {
        Sleep(0.005);
      }

      continue;
    }

    double current_time = GetWallTime();
    GetNextVelocity(GetWallTime());
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

void SineController::Start() {
  command_server_.Open(DATA_STREAM_CMD_IP, DATA_STREAM_CMD_PORT, false);
  file_ = fopen("sine_controller.csv", "w");
  control_thread_ = thread(&SineController::Run, this);
}

void SineController::Stop() {
  is_running_ = false;
}

void SineController::SendCommand(Pose2Df velocity) {
  RadioProtocolWrapper message;

  RadioProtocolCommand* command = message.add_command();
  command->set_robot_id(robot_id_);
  command->set_velocity_x(velocity.translation.x()/1000.0f);
  command->set_velocity_y(velocity.translation.y()/1000.0f);
  command->set_velocity_r(velocity.angle);
  command_server_.SendProtobuf(message);
}

void SineController::GetNextVelocity(double current_time) {
  current_velocity_ = desired_velocity_ * sin(current_time - start_time_);
}

void SineController::PrintRotational(Observation obs,
                                     Pose2Df command,
                                     double current_time) {
  float obs_velocity;
  if (obs.time - previous_observation_.time > kEpsilon) {
    obs_velocity = AngleDiff(obs.obs.angle,
                             previous_observation_.obs.angle)/
                             (obs.time - previous_observation_.time);
    prev_obs_vel_ = obs_velocity;
  } else {
    obs_velocity = prev_obs_vel_;
  }
  fprintf(file_,
          "%f, %f, %f, %f\n",
          obs.time,
          obs_velocity,
          current_time,
          current_velocity_);
}

void SineController::PrintTranslational(Observation obs,
                                        Pose2Df command,
                                        double current_time) {
  float obs_velocity;
  if (obs.time - previous_observation_.time) {
    Vector2f velocity = obs.obs.translation -
                        previous_observation_.obs.translation;
    Vector2f robot_dir = Heading(obs.obs.angle);

    obs_velocity = ScalarProjection(velocity, robot_dir) /
                   (obs.time - previous_observation_.time);
    prev_obs_vel_ = obs_velocity;
  } else {
    obs_velocity = prev_obs_vel_;
  }
  fprintf(file_,
          "%f, %f, %f, %f\n",
          obs.time,
          obs_velocity,
          current_time,
          current_velocity_);
}
}  // namespace open_loop

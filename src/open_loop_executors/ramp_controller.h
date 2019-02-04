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

#ifndef SRC_OPEN_LOOP_EXECUTORS_RAMP_CONTROLLER_H_
#define SRC_OPEN_LOOP_EXECUTORS_RAMP_CONTROLLER_H_

#include <stdio.h>

#include <atomic>
#include <thread>

#include "constants/typedefs.h"
#include "open_loop_executors/camera_reader.h"
#include "thread_safe/thread_safe_actor.h"

namespace open_loop {
class RampController {
 public:
  RampController(SSLVisionId robot_id,
                 bool is_translation,
                 threadsafe::ThreadSafeActor<Observation>*
                     thread_safe_observation);
  void Start();
  void Stop();
  void Run();

 private:
  void SendCommand(pose_2d::Pose2Df velocity);

  void GetNextVelocity();

  void PrintTranslational(Observation obs,
                          pose_2d::Pose2Df command,
                          double current_time);
  void PrintRotational(Observation obs,
                       pose_2d::Pose2Df command,
                       double current_time);

  enum ConfigState {ACCEL_FORWARD,
                    CRUISE_FORWARD,
                    DECEL_FORWARD,
                    WAIT_FORWARD,
                    ACCEL_BACKWARD,
                    CRUISE_BACKWARD,
                    DECEL_BACKWARD,
                    WAIT_BACKWARD};

  net::UDPMulticastServer command_server_;
  SSLVisionId robot_id_;
  bool is_translation_;
  threadsafe::ThreadSafeActor<Observation>* thread_safe_observation_;

  std::atomic_bool is_running_;
  std::thread control_thread_;

  Observation previous_observation_;

  int execution_count_;
  ConfigState execution_state_;

  std::FILE* file_;

  float current_velocity_;
  float desired_velocity_;
  float configuration_acceleration_;

  bool is_instantiated_;
  float prev_obs_vel_;
};
}  // namespace open_loop

#endif  // SRC_OPEN_LOOP_EXECUTORS_RAMP_CONTROLLER_H_

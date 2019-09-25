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

#ifndef SRC_OPEN_LOOP_EXECUTORS_OPEN_LOOP_JOYSTICK_H_
#define SRC_OPEN_LOOP_EXECUTORS_OPEN_LOOP_JOYSTICK_H_

#include <atomic>
#include <memory>
#include <thread>

#include "constants/constants.h"
#include "constants/typedefs.h"
#include "joystick/joystick.h"
#include "logging/logger.h"
#include "math/poses_2d.h"
#include "state/position_velocity_state.h"
#include "state/team.h"
#include "thread_safe/thread_safe_actor.h"
#include "thread_safe/thread_safe_queue.h"

namespace open_loop {
class OpenLoopJoystick {
 public:
  OpenLoopJoystick(
      SSLVisionId id,
      threadsafe::ThreadSafeActor<state::PositionVelocityState>*
          thread_safe_position_velocity_state,
      threadsafe::ThreadSafeActor<logger::Logger>* thread_safe_kalman_logger,
      threadsafe::ThreadSafeQueue<state::SharedState>*
          thread_safe_shared_state_queue,
      const team::Team& team);
  void Start();
  void Stop();
  void UpdateJoystick(state::SharedRobotState* robot_state,
                      RadioProtocolWrapper* wrapper);
  void RunJoystick();

 private:
  std::unique_ptr<joystick::Joystick> joystick_;
  SSLVisionId id_;

  std::atomic_bool is_running_;
  std::thread execution_thread_;

  state::PositionVelocityState local_position_velocity_state_;

  threadsafe::ThreadSafeActor<state::PositionVelocityState>*
      thread_safe_position_velocity_state_;
  threadsafe::ThreadSafeActor<logger::Logger>* thread_safe_kalman_logger_;
  threadsafe::ThreadSafeQueue<state::SharedState>*
      thread_safe_shared_state_queue_;

  Eigen::Vector2f last_translational_velocity_;
  float last_rotational_velocity_;

  team::Team team_;
  logger::NetLogger logger_;
};
}  // namespace open_loop

#endif  // SRC_OPEN_LOOP_EXECUTORS_OPEN_LOOP_JOYSTICK_H_

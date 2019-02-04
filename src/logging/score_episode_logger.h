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

#ifndef SRC_LOGGING_SCORE_EPISODE_LOGGER_H_
#define SRC_LOGGING_SCORE_EPISODE_LOGGER_H_

#include "datastructures/bounded_queue.h"
#include "state/position_velocity_state.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "thread_safe/thread_safe_actor.h"

namespace logger {
class ScoreEpisodeLogger {
 public:
  explicit ScoreEpisodeLogger(
      threadsafe::ThreadSafeActor<state::PositionVelocityState>*
          thread_safe_position_velocity_state);
  ~ScoreEpisodeLogger();
  void Start();
  void Stop();

 private:
  void HandleUpdate();
  void WriteBuffer();
  void ClearBuffer();

  bool IsGoal();

  // Input parameters.
  threadsafe::ThreadSafeActor<state::PositionVelocityState>*
      thread_safe_position_velocity_state_;
  std::atomic_bool is_running_;
  std::thread update_thread_;

  datastructures::BoundedQueue<state::PositionVelocityState>
      state_buffer_;

  state::PositionVelocityState current_state_;
  state::WorldState world_state_;
  state::SoccerState soccer_state_;

  double previous_time_;
  int num_episodes_;
  bool kicked_off_;

  static constexpr unsigned int kBufferLength = 300;  // 5 Seconds
};
}  // namespace logger

#endif  // SRC_LOGGING_SCORE_EPISODE_LOGGER_H_

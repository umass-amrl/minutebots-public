// Copyright 2017 - 2019 kvedder@umass.edu
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

#include <atomic>
#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "logging/logger.h"
#include "plays/skills_tactics_plays.h"
#include "state/direction.h"
#include "state/position_velocity_state.h"
#include "state/referee_state.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/team.h"
#include "state/world_state.h"
#include "tactics/tactic.h"
#include "tactics/tactic_index.h"
#include "thread_safe/thread_safe_actor.h"
#include "thread_safe/thread_safe_queue.h"
#include "experimental_sim/experimental_sim.h"

#ifndef SRC_SOCCER_EXECUTOR_H_
#define SRC_SOCCER_EXECUTOR_H_

using Eigen::Matrix;
using Eigen::Vector3d;
using std::vector;
using state::SharedRobotState;
using Eigen::Transpose;
using Eigen::Inverse;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 10, 1> Vector10d;
// using Eigen::Vector6f;

namespace app {
class Executor {
 public:
  Executor(
      const std::string& input_udp_address,
      const int input_udp_port,
      const int refbox_port,
      const bool& test_plays,
      const std::vector<float>& attacker_params,
      const string& trace_file_name,
      threadsafe::ThreadSafeActor<state::PositionVelocityState>*
          thread_safe_position_velocity_state,
      threadsafe::ThreadSafeActor<logger::Logger>* thread_safe_kalman_logger,
      threadsafe::ThreadSafeQueue<state::SharedState>*
          thread_safe_shared_state_queue,
      const std::vector<std::pair<SSLVisionId, tactics::TacticIndex>>&
          default_tactics,
      const team::Team& team,
      const direction::Direction& direction,
      const bool& simulating,
      std::exception_ptr* exception);

  ~Executor();

  void Start();

  void SimStart(
    threadsafe::ThreadSafeActor
        <experimental_simulator::Simulator*>* thread_safe_sim,
    experimental_simulator::Simulator* simulator);

  void Stop();

 private:
  void HandleExecution();
  void MergeRobotMessages();
  void ClearRobotLogs();
  state::PositionVelocityState local_position_velocity_state_;
  bool simulating_;
  state::WorldState local_world_state_;

  // UDP Info for writing out command data.
  const std::string udp_address_;
  const int udp_port_;
  const int refbox_port_;

  // boolean for determining which play set to use
  const bool test_plays_;
  const std::vector<float> attacker_params_;
  std::string trace_file_name_;
  MinuteBotsProto::Trace tactic_trace;
  // UDP Info for reading from referee.
  //   const std::string referee_address_ = "224.5.23.1";
  //   const int referee_port_ = 10003;

  std::atomic_bool is_running_;
  std::thread execution_thread_;

  threadsafe::ThreadSafeActor<state::PositionVelocityState>*
      thread_safe_position_velocity_state_;
  threadsafe::ThreadSafeActor<logger::Logger>* thread_safe_kalman_logger_;
  threadsafe::ThreadSafeQueue<state::SharedState>*
      thread_safe_shared_state_queue_;

  state::SoccerState soccer_state_;
  state::RefereeState referee_state_;
  logger::NetLogger logger_;
  plays::SkillsTacticsPlays skills_tactics_plays_;
  const bool use_stp_;
  const std::vector<std::pair<SSLVisionId, tactics::TacticIndex>>&
      default_tactics_;
  threadsafe::ThreadSafeActor<experimental_simulator::Simulator*>*
      thread_safe_simulator_;
  experimental_simulator::Simulator* local_simulator_;
  std::exception_ptr* exception_;
};
}  // namespace app

#endif  // SRC_SOCCER_EXECUTOR_H_

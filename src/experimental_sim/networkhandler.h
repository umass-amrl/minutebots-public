// Copyright 2016 - 2019 jaholtz@cs.umass.edu
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
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "messages_robocup_ssl_wrapper.pb.h"
#include "net/netraw.h"
#include "radio_protocol_wrapper.pb.h"
#include "experimental_sim/sim_state.h"
#include "util/random.h"
#include "thread_safe/thread_safe_queue.h"

#ifndef SRC_EXPERIMENTAL_SIM_NETWORKHANDLER_H_
#define SRC_EXPERIMENTAL_SIM_NETWORKHANDLER_H_

namespace experimental_simulator {
class NetworkHandler {
 public:
  NetworkHandler(SimState* world_state,
                 const std::string& input_udp_address,
                 const int input_udp_port,
                 const std::string& output_udp_address,
                 const int output_udp_port,
                 const double output_loop_rate);
  ~NetworkHandler();
  void Start();

 private:
  void HandleInput(const std::string udpAddress, const int udpPort);
  void HandleOutput(const std::string udpAddress, const int udpPort);

  bool ValidateVelocityCommandIntegrity(
      const RadioProtocolWrapper& velocity_command);

  void MakeSSLWrapperPackets(
      int frame_index,
      util_random::Random* random,
      std::vector<SSLVisionProto::SSL_WrapperPacket>* packets);

  SimState* world_state;

  const std::string input_udp_address;
  const int input_udp_port;
  const std::string output_udp_address;
  const int output_udp_port;
  const double output_loop_rate;  // Hz.

  std::atomic_bool is_running;

  std::thread input_thread;
  std::thread output_thread;

  threadsafe::ThreadSafeQueue<RadioProtocolWrapper> threadsafe_input_queue;
};
}  // namespace experimental_simulator

#endif  // SRC_EXPERIMENTAL_SIM_NETWORKHANDLER_H_

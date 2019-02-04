// Copyright 2016 - 2018 kvedder@umass.edu
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
#include "deflection_sim/worldstate.h"
#include "util/random.h"
#include "thread_safe/thread_safe_queue.h"

#ifndef SRC_DEFLECTION_SIM_NETWORKHANDLER_H_
#define SRC_DEFLECTION_SIM_NETWORKHANDLER_H_

namespace simulator {
class NetworkHandler {
 public:
  NetworkHandler(WorldState* world_state, const std::string& input_udp_address,
                 const int input_udp_port,
                 const std::string& output_udp_address,
                 const int output_udp_port,
                 const double worker_loop_rate,
                 const double output_loop_rate,
                 const double input_loop_rate);
  ~NetworkHandler();
  void Start();

  bool Success();
  bool DockingSuccess();
  bool DeflectionSuccess();
  bool Failure();

 private:
  void HandleInput(const std::string udpAddress, const int udpPort);
  void Work();
  void HandleOutput(const std::string udpAddress, const int udpPort);

  bool ValidateVelocityCommandIntegrity(
      const RadioProtocolWrapper& velocity_command);

  void SetupSSLWrapperPacket(
      std::vector<SSLVisionProto::SSL_WrapperPacket*>* wrapper_packets,
      int frame_index, util_random::Random* random);

  WorldState* world_state;

  const std::string input_udp_address;
  const int input_udp_port;
  const std::string output_udp_address;
  const int output_udp_port;
  const double worker_loop_rate;  // Hz.
  const double output_loop_rate;  // Hz
  const double input_loop_rate;   // Hz.
  bool success_;
  bool failure_;
  std::atomic_bool is_running;

  std::thread input_thread;
  std::thread output_thread;
  std::thread work_thread;

  threadsafe::ThreadSafeQueue<RadioProtocolWrapper> threadsafe_input_queue;

  threadsafe::ThreadSafeQueue<SSLVisionProto::SSL_WrapperPacket>
      threadsafe_output_queue;
};
}  // namespace simulator

#endif  // SRC_DEFLECTION_SIM_NETWORKHANDLER_H_

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
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "net/netraw.h"
#include "sim/worldstate.h"
#include "thread_safe/thread_safe_priority_queue.h"

#include "messages_robocup_ssl_wrapper.pb.h"
#include "radio_protocol_wrapper.pb.h"

#ifndef SRC_SOCCER_SSLVISIONINPUTHANDLER_H_
#define SRC_SOCCER_SSLVISIONINPUTHANDLER_H_

namespace app {
class SSLVisionInputHandler {
 public:
  SSLVisionInputHandler(
      const string& input_udp_address,
      const int input_udp_port,
      const std::vector<unsigned int >& usable_camera_ids,
      threadsafe::ThreadSafePriorityQueue<SSLVisionProto::SSL_WrapperPacket,
          double>* ssl_vision_thread_safe_queue);

  ~SSLVisionInputHandler();

  void Start();

  void Stop();

 private:
  void HandleInput(const std::string udpAddress, const int udpPort);

  const std::string input_udp_address_;
  const int input_udp_port_;

  const std::vector<unsigned int>& usable_camera_ids_;

  std::atomic_bool is_running_;
  std::thread input_thread_;

  threadsafe::ThreadSafePriorityQueue<SSLVisionProto::SSL_WrapperPacket,
                                      double>* ssl_vision_thread_safe_queue_;
};
}  // namespace app

#endif  // SRC_SOCCER_SSLVISIONINPUTHANDLER_H_

// Copyright 2017 kvedder@umass.edu
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

#ifndef SRC_YISIBOT_RADIO_NETWORKHANDLER_H_
#define SRC_YISIBOT_RADIO_NETWORKHANDLER_H_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "messages_robocup_ssl_wrapper.pb.h"
#include "net/netraw.h"
#include "radio_protocol_wrapper.pb.h"
#include "sim/worldstate.h"
#include "yisibot_radio/yisibot_radio.h"

namespace yisibot_radio {
class NetworkHandler {
 public:
  NetworkHandler(const std::string& input_udp_address, const int input_udp_port,
                 const float loop_rate_hz);
  ~NetworkHandler();
  void Start();

 private:
  void HandleInput(const std::string udpAddress, const int udpPort);

  const std::string input_udp_address;
  const int input_udp_port;
  const float loop_rate_hz_;

  std::atomic_bool is_running;

  std::thread input_thread;
  YisibotRadio radio;
};
}  // namespace yisibot_radio

#endif  // SRC_YISIBOT_RADIO_NETWORKHANDLER_H__

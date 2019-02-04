// Copyright 2017 - 2018 kvedder@umass.edu, jaholtz@umass.edu
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

#include "radio/minutebot_networkhandler.h"

#include <glog/logging.h>
#include <condition_variable>
#include <iostream>
#include <thread>
#include <vector>

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "radio/minutebot_radio.h"
#include "util/timer.h"

using pose_2d::Pose2Df;
using net::UDPMulticastServer;
using std::string;
using std::cout;
using std::endl;
using std::cerr;
using std::thread;
using std::vector;
using std::atomic_bool;
using std::string;
using std::vector;
using std::lock_guard;
using std::unique_lock;
using std::mutex;
using std::condition_variable;

namespace radio {

NetworkHandler::NetworkHandler(const string& input_udp_address,
                               const int input_udp_port)
    : input_udp_address(input_udp_address),
      input_udp_port(input_udp_port),
      is_running(true) {
  if (!radio.Init("/dev/ttyUSB2", 0)) {
    LOG(FATAL) << "ERROR: Unable to open serial port device(s)!\n";
    return;
  }
}

NetworkHandler::~NetworkHandler() {
  // Signal loops of the worker threads to terminate.
  is_running = false;
  input_thread.join();
}

void NetworkHandler::Start() {
  input_thread = thread(&NetworkHandler::HandleInput, this, input_udp_address,
                        input_udp_port);
}

void NetworkHandler::HandleInput(const string udp_address, const int udp_port) {
  UDPMulticastServer udp_server;
  if (!udp_server.Open(udp_address, udp_port, true)) {
    LOG(ERROR) << "Error opening UDP port of HandleInput thread, exiting.";
  }
  // Set timeout to 50 milliseconds.
  CHECK(udp_server.SetReceiveTimeout(50000));
  // Ensure that we are ready to receive SSL vision data.
  CHECK(udp_server.IsOpen());

  RadioProtocolWrapper command;
  while (is_running) {
    if (udp_server.TryReceiveProtobuf(&command)) {
      radio.Send(command);
    }
  }

  radio.Close();
  udp_server.Close();
}
}  // namespace radio

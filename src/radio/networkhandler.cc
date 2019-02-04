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

#include "radio/networkhandler.h"

#include <glog/logging.h>
#include <condition_variable>
#include <iostream>
#include <thread>
#include <vector>

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
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
    : input_udp_address(input_udp_address), input_udp_port(input_udp_port) {
  input_queue_lock_read_ = false;
  is_running = true;
}

NetworkHandler::~NetworkHandler() {
  // Signal loops of the worker threads to terminate.
  is_running = false;

  // Clean out potential blocks in the pipeline.
  input_queue_lock_read_ = true;
  input_queue_access_cv.notify_all();

  // Join all threads.
  output_thread.join();
  input_thread.join();
}

void NetworkHandler::Start() {
  input_thread = thread(&NetworkHandler::HandleInput, this, input_udp_address,
                        input_udp_port);
  output_thread = thread(&NetworkHandler::HandleOutput, this);
}

void NetworkHandler::HandleInput(const string udpAddress, const int udpPort) {
  UDPMulticastServer udp_server;
  if (!udp_server.Open(udpAddress, udpPort)) {
    LOG(ERROR) << "Error opening UDP port of HandleInput thread, exiting."
               << endl;
  } else {
  }
  CHECK(udp_server.IsOpen());

  RadioProtocolWrapper command;

  while (is_running.load()) {
    if (udp_server.TryReceiveProtobuf(&command)) {
      {
        lock_guard<mutex> guard(input_queue_mutex);
        // Push back the future into the queue.
        input_queue.push_back(command);
        input_queue_lock_read_ = true;
        input_queue_access_cv.notify_one();
      }
    } else {
    }
  }

  udp_server.Close();
}

void NetworkHandler::HandleOutput() {
  vector<RadioProtocolWrapper> local_queue;

  while (is_running.load()) {
    {  // Transfer from the output queue to the local queue.
      unique_lock<mutex> guard(input_queue_mutex);

      // Block until woken up.
      // Check to make sure that the wakeup isn't spurrious.
      while (!input_queue_lock_read_) {
        input_queue_access_cv.wait(guard);
      }

      // Push all of the queued data into a local queue.
      for (const RadioProtocolWrapper& input : input_queue) {
        local_queue.push_back(input);
      }
      input_queue.clear();

      // Reset the read flag.
      input_queue_lock_read_ = false;
    }  // Lock loses scope here.

    // TODO(slane): Use local queue to do radio server stuff.
    //     for (const RadioProtocolWrapper& output : local_queue) {
    //
    //     }

    // Reset read flag.
    local_queue.clear();
  }
}

}  // namespace radio

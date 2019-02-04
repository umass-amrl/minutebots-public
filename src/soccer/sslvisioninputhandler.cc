// Copyright 2017 - 2018 kvedder@umass.edu
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
#include "soccer/sslvisioninputhandler.h"

#include <fstream>
#include <iomanip>

#include "util/colorize.h"
#include "util/timer.h"

using colorize::ColorGreen;
using net::UDPMulticastServer;
using SSLVisionProto::SSL_WrapperPacket;
using std::atomic_bool;
using std::condition_variable;
using std::endl;
using std::lock_guard;
using std::unique_lock;
using std::mutex;
using std::string;
using std::thread;
using std::vector;

namespace app {

SSLVisionInputHandler::SSLVisionInputHandler(
      const std::string& input_udp_address,
      const int input_udp_port,
      const std::vector<unsigned int>& usable_camera_ids,
      threadsafe::ThreadSafePriorityQueue<SSLVisionProto::SSL_WrapperPacket,
          double>* ssl_vision_thread_safe_queue) :
    input_udp_address_(input_udp_address),
    input_udp_port_(input_udp_port),
    usable_camera_ids_(usable_camera_ids),
    ssl_vision_thread_safe_queue_(ssl_vision_thread_safe_queue) {
  is_running_ = true;
}

SSLVisionInputHandler::~SSLVisionInputHandler() {
  // Signal loops of the worker threads to terminate.
  is_running_ = false;

  ssl_vision_thread_safe_queue_->Shutdown();

  // Blocking wait for input thread to terminate.
  if (input_thread_.joinable()) {
    input_thread_.join();
  }
}

void SSLVisionInputHandler::Start() {
  input_thread_ = thread(&SSLVisionInputHandler::HandleInput, this,
                         input_udp_address_, input_udp_port_);
}

void SSLVisionInputHandler::HandleInput(const string udpAddress,
                                        const int udpPort) {
  std::ofstream ofs;
  if (!kProduction) {
    ofs.open("vision_timing.txt", std::ofstream::out);
  }

  UDPMulticastServer udp_server;
  if (!udp_server.Open(udpAddress, udpPort, true)) {
    LOG(FATAL) << "Error opening UDP port of SSLVisionInputHandler's "
               << "HandleInput thread, exiting." << endl;
  }
  // Set timeout to 50 milliseconds.
  CHECK(udp_server.SetReceiveTimeout(50000));
  // Ensure that we are ready to receive SSL vision data.
  CHECK(udp_server.IsOpen());

  // Reads SSL_WrapperPacket from the SSL_Vision system as input.
  SSL_WrapperPacket command;

  while (is_running_) {
    if (udp_server.TryReceiveProtobuf(&command)) {
      const auto processing_time =
          command.detection().t_sent() - command.detection().t_capture();
      command.mutable_detection()->set_t_capture(GetWallTime() -
                                                 processing_time);
      if (command.has_detection()) {
        if (usable_camera_ids_.size() == 0) {
          ssl_vision_thread_safe_queue_->Add(command,
                                             command.detection().t_capture());
        } else {
          const auto& detection = command.detection();
          for (const auto& usable_camera_id : usable_camera_ids_) {
            if (detection.camera_id() == usable_camera_id) {
              ssl_vision_thread_safe_queue_->Add(
                  command, command.detection().t_capture());
              break;
            }
          }
        }
      }
    }
    if (!kProduction) {
      ofs << std::setprecision(15) << GetMonotonicTime() << "\n";
    }
  }
  if (!kProduction) {
    ofs.close();
  }
  udp_server.Close();
}

void SSLVisionInputHandler::Stop() {
  is_running_ = false;
}

}  // namespace app

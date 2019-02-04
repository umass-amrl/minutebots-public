// Copyright 2017 - 2018 kvedder@umass.edu, slane@cs.umass.edu,
// jaholtz@umass.edu
//
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Radio server main executable for UMass RoboCup SSL robots.
//
//========================================================================
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
//========================================================================
#include <glog/logging.h>
#include <signal.h>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "constants/constants.h"
#include "net/netraw.h"
#include "radio/minutebot_networkhandler.h"
#include "radio_protocol_wrapper.pb.h"
#include "util/colorize.h"

using colorize::ColorGreen;
using colorize::ColorBlue;
using colorize::ColorCyan;
using net::UDPMulticastServer;
using radio::NetworkHandler;
using std::atomic_bool;
using std::condition_variable;
using std::mutex;
using std::unique_lock;
using std::vector;
using std::endl;

static atomic_bool shutdown_flag(false);
static mutex shutdown_mutex;
static condition_variable shutdown_access_cv;

void SigHandler(int signo) {
  if (signo == SIGINT) {
    shutdown_flag = true;
    shutdown_access_cv.notify_all();
  }
}

int main(int argc, char** argv) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;   // INFO level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.
  FLAGS_logtostderr = true;    // Don't log to disk.

  if (signal(SIGINT, SigHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGINT\n";
  }

  NetworkHandler network_handler(DATA_STREAM_CMD_IP, DATA_STREAM_CMD_PORT);
  network_handler.Start();

  // Begin the blocking process which awakes when SIG_INT is handled.
  {
    unique_lock<mutex> guard(shutdown_mutex);

    // Block until woken up.
    // Check to make sure that the wakeup isn't spurrious.
    while (!shutdown_flag) {
      shutdown_access_cv.wait(guard);
    }
  }  // Lock loses scope here.

  google::FlushLogFiles(google::GLOG_INFO);
  return 0;
}

// Copyright 2016 - 2018 jaholtz@cs.umass.edu
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
#include <glog/logging.h>
#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <atomic>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "constants/constants.h"
#include "eigen3/Eigen/Core"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "net/netraw.h"
#include "netraw_test_message.pb.h"
#include "radio_protocol_wrapper.pb.h"
#include "experimental_sim/networkhandler.h"
#include "experimental_sim/objects/ball.h"
#include "experimental_sim/objects/robot.h"
#include "experimental_sim/sim_state.h"
#include "util/colorize.h"
#include "util/timer.h"

using colorize::ColorGreen;
using colorize::ColorBlue;
using colorize::ColorCyan;
using Eigen::Vector2f;
using google::protobuf::RepeatedPtrField;
using net::UDPMulticastServer;
using pose_2d::Pose2Df;
using experimental_simulator::Ball;
using experimental_simulator::NetworkHandler;
using experimental_simulator::Robot;
using experimental_simulator::SimState;
using std::cerr;
using std::cout;
using std::endl;
using std::ofstream;
using std::string;
using std::thread;
using std::vector;

// static constexpr float kUpdateRate = 360.0f;
static constexpr float kOutputRate = 60.0f;
static constexpr float kTimeSlice = 1.0f / kOutputRate;
static constexpr int kNumRobots = 12;
// static constexpr double kReceiveRate = 360.0;  // Hz.

const double noise_level = 0.00;
std::default_random_engine generator;
std::normal_distribution<double> distribution(1.0, noise_level);

static std::atomic_bool shutdown_flag(false);
static std::mutex shutdown_mutex;
static std::condition_variable shutdown_access_cv;

void SigHandler(int signo) {
  if (signo == SIGINT) {
    shutdown_flag = true;
    shutdown_access_cv.notify_all();
  }
}

std::vector<Pose2Df> ReadStartPositions(const std::string& file_path) {
  std::vector<Pose2Df> positions;
  std::ifstream infile(file_path);

  if (!infile) {
    LOG(FATAL) << "Cannot open file " << file_path;
  }

  std::string line;
  while (std::getline(infile, line)) {
    LOG(INFO) << "Line: " << line;
    std::istringstream iss(line);
    float x = 0, y = 0, theta = 0;
    if (!(iss >> x >> y >> theta)) {
      break;
    }

    positions.push_back({theta, x, y});
  }
  return positions;
}

std::vector<Pose2Df> ReadStartVelocities(const std::string& file_path) {
  std::vector<Pose2Df> velocities;
  std::ifstream infile(file_path);

  if (!infile) {
    LOG(FATAL) << "Cannot open file " << file_path;
  }

  std::string line;
  while (std::getline(infile, line)) {
    LOG(INFO) << "Line: " << line;
    std::istringstream iss(line);
    float x = 0, y = 0, theta = 0;
    if (!(iss >> x >> y >> theta)) {
      break;
    }

    velocities.push_back({theta, x, y});
  }
  return velocities;
}

SimState GenerateWorldState(int argc, char** argv) {
  if (argc > 1) {
    // Grab positions file.
    const std::vector<Pose2Df> positions = ReadStartPositions(argv[1]);
    std::vector<Pose2Df> velocities;
    if (argc > 2) {
      velocities = ReadStartVelocities(argv[2]);
    } else {
      for (unsigned int i = 0; i < positions.size(); i++) {
        velocities.push_back({0.0, 0.0, 0.0});
      }
    }
//     CHECK_EQ(positions.size(), kNumRobots);
    if (positions.size() != velocities.size()) {
      printf("WARNING: there are %lu positions listed but %lu velocities"
        ", these must be equal", positions.size(), velocities.size());
    }
    return {positions, velocities, 0.0005f, kTimeSlice};
  } else {
    return {kNumRobots, 0.0005f, kTimeSlice};
  }
}

int main(int argc, char** argv) {
  generator.seed(1);
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 1;   // WARNING level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.
  FLAGS_logtostderr = true;    // Don't log to disk

  if (signal(SIGINT, SigHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGINT\n";
  }

  SimState world_state = GenerateWorldState(argc, argv);
  NetworkHandler receiver(&world_state,
                          DATA_STREAM_CMD_IP,
                          DATA_STREAM_CMD_PORT,
                          DATA_STREAM_VISION_IP,
                          DATA_STREAM_VISION_PORT,
                          kOutputRate);
  receiver.Start();

  // Begin the blocking process which awakes when SIG_INT is handled.
  {
    std::unique_lock<std::mutex> guard(shutdown_mutex);

    // Block until woken up.
    // Check to make sure that the wakeup isn't spurrious.
    while (!shutdown_flag) {
      shutdown_access_cv.wait(guard);
    }
  }  // Lock loses scope here.

  google::protobuf::ShutdownProtobufLibrary();
  google::FlushLogFiles(google::GLOG_INFO);
}

// Copyright 2016 - 2018 kvedder@umass.edu, jaholtz@cs.umass.edu
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
#include "deflection_sim/networkhandler.h"
#include "deflection_sim/objects/ball.h"
#include "deflection_sim/objects/robot.h"
#include "deflection_sim/worldstate.h"
#include "eigen3/Eigen/Core"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "net/netraw.h"
#include "netraw_test_message.pb.h"
#include "radio_protocol_wrapper.pb.h"
#include "state/shared_state.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "util/colorize.h"
#include "util/timer.h"

#include "evaluators/offense_evaluation.h"

#include "obstacles/obstacle_flag.h"

#include "motion_control/ntoc_2d.h"
#include "tactics/ball_interception.h"

using colorize::ColorGreen;
using colorize::ColorBlue;
using colorize::ColorCyan;
using Eigen::Vector2f;
using google::protobuf::RepeatedPtrField;
using net::UDPMulticastServer;
using pose_2d::Pose2Df;
using simulator::Ball;
using simulator::NetworkHandler;
using simulator::Robot;
using simulator::WorldState;
using std::cerr;
using std::cout;
using std::endl;
using std::ofstream;
using std::string;
using std::thread;
using std::vector;
using Eigen::Vector2d;
using Eigen::Vector2f;
// using tactics::BallInterception;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using offense::AimOption;
using offense::CalculateAimOptions;

static const float kUpdateRate = 60.0f;
static const float kOutputRate = 60.0f;
static const float kTimeSlice = 1.0 / kUpdateRate;
static const int kNumRobots = 1;
static const double kReceiveRate = 120.0;  // Hz.

static std::atomic_bool shutdown_flag(false);
// static std::mutex shutdown_mutex;
static std::condition_variable shutdown_access_cv;

void SigHandler(int signo) {
  if (signo == SIGINT) {
    shutdown_flag = true;
    shutdown_access_cv.notify_all();
  }
}

// Sets up robots in initial positions.
// TODO(jaholtz): Make this take an input
vector<Robot> CreateRobots(const float& pos_x, const float& pos_y,
                           const float& pos_theta) {
  vector<Robot> robots;
  for (int i = 0; i < kNumRobots; ++i) {
    const Pose2Df pose(pos_theta,
                       Vector2f(pos_x + ((kRobotRadius * 3) * i), pos_y));
    const Vector2f velocity(0, 0);
    const int id = i;
    const float max_translation = 10;  // meters/s
    const float max_accel = 0.001;     // m/s^2
    const float radius = 90;           // mm
    robots.push_back(Robot(pose, velocity, id, kTimeSlice, max_translation,
                           max_accel, radius));
  }
  return robots;
}

Ball CreateBall(const float& pos_x, const float& pos_y, const float& vel_x,
                const float& vel_y) {
  const float ball_mass = 0.08f;
  const float friction_coefficient = 0.0005f;
  return Ball(Pose2Df(0, Vector2f(pos_x, pos_y)), Vector2f(vel_x, vel_y),
              ball_mass, kTimeSlice, friction_coefficient);
}

int main(int argc, char** argv) {
  std::ofstream data_file;
  data_file.open("large_kick_experiment.txt");
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 1;   // WARNING level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.
  FLAGS_logtostderr = true;    // Don't log to disk.

  if (argc != 10) {
    std::cout << "ERROR: 9 arguments required" << std::endl;
    std::cout
        << "robot x, robot y, robot theta, ball x, ball y, ball v_x, ball v_y, "
        << "command_port, vision port" << std::endl;
    return 1;
  }

  if (signal(SIGINT, SigHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGINT\n";
  }

  vector<Robot> robots =
      CreateRobots(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  Ball ball =
      CreateBall(atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]));

  //   // Begin the blocking process which awakes when SIG_INT is handled.
  //   {
  //     std::unique_lock<std::mutex> guard(shutdown_mutex);
  //
  //     // Block until woken up.
  //     // Check to make sure that the wakeup isn't spurrious.
  //     while (!shutdown_flag) {
  //       shutdown_access_cv.wait(guard);
  //     }
  //   }  // Lock loses scope here.
  for (int x = -4000; x < -kRobotRadius * 3; x += (kRobotRadius * 3)) {
    for (int y = 2500; y > -2500; y -= (kRobotRadius * 3)) {
      int count = 0;
      int total = 0;
      for (float r = 0; r < 360; r += 20) {
        std::cout << x << "," << y << "," << r << std::endl;
        vector<Robot> robots = CreateRobots(x, y, DegToRad(r));
        Ball ball = CreateBall(-9999, -9999, 0, 0);
        WorldState world_state(robots, ball);

        NetworkHandler receiver(&world_state, DATA_STREAM_CMD_IP, atoi(argv[8]),
                                DATA_STREAM_VISION_IP, atoi(argv[9]),
                                kUpdateRate * 5, kOutputRate * 5,
                                kReceiveRate * 5);
        //           system("./bin/soccer -tb -py&");
        receiver.Start();
        while (true) {
          if (receiver.DockingSuccess()) {
            count += 1;
            total += 1;
            break;
          } else if (receiver.Failure()) {
            total += 1;
            break;
          }
        }
        //           system("pkill -KILL soccer");
      }
      data_file << x << "\t";
      data_file << y << "\t" << count << "\t" << total << std::endl;
    }
  }

  google::protobuf::ShutdownProtobufLibrary();
  google::FlushLogFiles(google::GLOG_INFO);
}

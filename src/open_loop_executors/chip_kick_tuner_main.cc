// Copyright 2018 - 2019 kvedder@umass.edu
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

#include <signal.h>
#include <bitset>
#include <iomanip>

#include "constants/constants.h"
#include "joystick/joystick.h"
#include "math/poses_2d.h"
#include "open_loop_executors/open_loop_joystick.h"
#include "state/direction.h"
#include "state/position_velocity_state.h"
#include "state/team.h"
#include "thread_safe/thread_safe_actor.h"
#include "thread_safe/thread_safe_priority_queue.h"
#include "thread_safe/thread_safe_queue.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using state::PositionVelocityState;
using state::SharedState;
using open_loop::OpenLoopJoystick;
using threadsafe::ThreadSafePriorityQueue;
using threadsafe::ThreadSafeQueue;
using threadsafe::ThreadSafeActor;
using logger::Logger;
using net::UDPMulticastServer;

static std::atomic_bool shutdown_flag(false);

void SigHandler(const int signo) {
  if (signo == SIGINT) {
    shutdown_flag = true;
  }
}

void Setup(int argc, char** argv) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;   // INFO level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.
  FLAGS_logtostderr = true;    // Don't log to disk
  if (signal(SIGINT, SigHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGINT";
  }
}

SSLVisionId GetIds(int argc, char** argv) {
  // Set Robot ID
  if (argc <= 2 || argc > 3) {
    std::cout << "Usage: " << argv[0] << " robot_id camera_id" << std::endl;
    exit(-1);
  }
  return static_cast<SSLVisionId>(std::atoi(argv[1]));
}

UDPMulticastServer SetupUDPServer() {
  UDPMulticastServer udp_server;
  if (!udp_server.Open(DATA_STREAM_CMD_IP, DATA_STREAM_CMD_PORT, false)) {
    LOG(FATAL) << "Error opening UDP port for commands. Exiting...";
  }
  return udp_server;
}

static constexpr float kMinPower = 5.0f;
// static constexpr float kMaxPower = 50.0f;
static constexpr float kPowerStep = 1.0f;
static constexpr size_t kIterationsPerStep = 1;
static constexpr float kWaitAfterKickTime = 1.0f;
static float shot_power = kMinPower;
static size_t shot_iteration = 0;
static bool prev_tick_sent_message = false;
static bool new_command = false;
static double last_kick_time = 0;
static bool ready_to_record_result = false;

void TickShotPowerInfo() {
  ++shot_iteration;
  if (shot_iteration >= kIterationsPerStep) {
    shot_iteration = 0;
    shot_power += kPowerStep;
  }
}

RadioProtocolWrapper BuildRadioCommand(const joystick::Joystick& joystick,
                                       const SSLVisionId& ssl_vision_id) {
  static constexpr int kKickTriggerThreshold = 30000;
  RadioProtocolWrapper message;
  RadioProtocolCommand* command = message.add_command();
  command->set_robot_id(ssl_vision_id);
  command->set_velocity_x(0);
  command->set_velocity_y(0);
  command->set_velocity_r(0);
  if (joystick.axes[5] > kKickTriggerThreshold) {
    command->set_flat_kick(shot_power);
    new_command = !prev_tick_sent_message;
    prev_tick_sent_message = true;
    if (new_command) {
      ready_to_record_result = false;
      last_kick_time = GetMonotonicTime();
      LOG(INFO) << "Recharging kick, please wait!";
    }
  } else if (prev_tick_sent_message) {
    TickShotPowerInfo();
    prev_tick_sent_message = false;
    new_command = false;
  }
  return message;
}

void DumpData(const float& distance,
              const SSLVisionId& ssl_vision_id) {
  const std::string file_path =
      "kick_tuning_ssl_" + std::to_string(ssl_vision_id) + "_power_" +
      std::to_string(shot_power) + "_itr_" + std::to_string(shot_iteration);
  std::fstream file(file_path, std::ios::out);
  file << std::setprecision(20) << distance << '\n';
  file.close();
  LOG(INFO) << "Wrote to " << file_path;
}

std::pair<bool, float> HandleRecordingResult() {
  if (!ready_to_record_result &&
      (GetMonotonicTime() - last_kick_time) > kWaitAfterKickTime) {
    ready_to_record_result = true;
    LOG(INFO) << "Enter chip distance in meters:!";
    float dist = 0;
    std::cin >> dist;
    return {true, dist};
  }
  return {false, 0.0f};
}

int main(int argc, char** argv) {
  Setup(argc, argv);
  const SSLVisionId ssl_vision_id = GetIds(argc, argv);

  UDPMulticastServer udp_server = SetupUDPServer();

  joystick::Joystick joystick;
  joystick.Open(kJoystickID);

  RateLoop loop(kTransmitFrequency);
  while (!shutdown_flag) {
    loop.Sleep();
    joystick.ProcessEvents(0);
    udp_server.SendProtobuf(BuildRadioCommand(joystick, ssl_vision_id));
    const auto recording_result = HandleRecordingResult();
    if (recording_result.first) {
      DumpData(recording_result.second, ssl_vision_id);
    }
  }

  joystick.Close();
  udp_server.Close();

  return 0;
}

// Copyright 2018 kvedder@umass.edu
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
#include "open_loop_executors/ball_reader.h"
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

using CameraId = unsigned int;

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

std::pair<SSLVisionId, CameraId> GetIds(int argc, char** argv) {
  // Set Robot ID
  if (argc <= 2 || argc > 3) {
    std::cout << "Usage: " << argv[0] << " robot_id camera_id" << std::endl;
    exit(-1);
  }
  return {static_cast<SSLVisionId>(std::atoi(argv[1])),
          static_cast<CameraId>(std::atoi(argv[2]))};
}

UDPMulticastServer SetupUDPServer() {
  UDPMulticastServer udp_server;
  if (!udp_server.Open(DATA_STREAM_CMD_IP, DATA_STREAM_CMD_PORT, false)) {
    LOG(FATAL) << "Error opening UDP port for commands. Exiting...";
  }
  return udp_server;
}

struct BallPosition {
  Eigen::Vector2f position;
  double time;
};

static constexpr float kMinPower = 25.0f;
// static constexpr float kMaxPower = 50.0f;
static constexpr float kPowerStep = 3.0f;
static constexpr size_t kIterationsPerStep = 1;
static float shot_power = kMinPower;
static size_t shot_iteration = 0;
static bool prev_tick_sent_message = false;
static bool new_command = false;
static double last_kick_time = 0;
static bool ready_to_kick = false;

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
      ready_to_kick = false;
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

bool LastFewObservationsSimilar(
    const std::vector<BallPosition>& saved_observations) {
  if (saved_observations.size() < 10) {
    return false;
  }

  if (saved_observations[saved_observations.size() - 1].time - GetWallTime() >
      1.0f) {
    return true;
  }

  Eigen::Vector2f first_half(0, 0);
  for (size_t i = saved_observations.size() - 10;
       i < saved_observations.size() - 5; ++i) {
    first_half += saved_observations[i].position;
  }
  first_half /= 5.0f;

  Eigen::Vector2f second_half(0, 0);
  for (size_t i = saved_observations.size() - 5; i < saved_observations.size();
       ++i) {
    second_half += saved_observations[i].position;
  }
  second_half /= 5.0f;

  return ((first_half - second_half).squaredNorm() < Sq(10));
}

bool ShouldRecordData(const std::vector<BallPosition>& saved_observations) {
  static bool should_record_data = false;
  // Start recording conditions.
  if (new_command && saved_observations.empty()) {
    should_record_data = true;
  }

  // Stop recording conditions.
  if (LastFewObservationsSimilar(saved_observations)) {
    should_record_data = false;
  }

  return should_record_data;
}

void ConditionallyAddData(std::vector<BallPosition>* saved_positions,
                          const open_loop::BallObservation& observation) {
  if (observation.time == 0.0f || observation.observations.empty()) {
    return;
  }

  if (saved_positions->empty()) {
    saved_positions->push_back(
        {observation.observations[0].first, observation.time});
    return;
  }

  const BallPosition& last_position =
      (*saved_positions)[saved_positions->size()];
  Eigen::Vector2f closest_observation = observation.observations[0].first;
  for (const auto& e : observation.observations) {
    if ((last_position.position - e.first).squaredNorm() <
        (last_position.position - closest_observation).squaredNorm()) {
      closest_observation = e.first;
    }
  }
  saved_positions->push_back({closest_observation, observation.time});
}

void DumpData(std::vector<BallPosition>* saved_observations,
              const SSLVisionId& ssl_vision_id) {
  if (saved_observations->empty()) {
    return;
  }
  const std::string file_path =
      "kick_tuning_ssl_" + std::to_string(ssl_vision_id) + "_power_" +
      std::to_string(shot_power) + "_itr_" + std::to_string(shot_iteration);
  std::fstream file(file_path, std::ios::out);
  for (const BallPosition& e : *saved_observations) {
    const auto& position = e.position;
    file << std::setprecision(20) << position.x() << ", " << position.y()
         << " @ " << e.time << '\n';
  }
  file.close();
  LOG(INFO) << "Wrote to " << file_path;
  saved_observations->clear();
}

void HandleAlertReady() {
  if (!ready_to_kick && GetMonotonicTime() - last_kick_time > 30.0f) {
    ready_to_kick = true;
    LOG(INFO) << "Ready to kick!";
  }
}

int main(int argc, char** argv) {
  Setup(argc, argv);
  const std::pair<SSLVisionId, CameraId> ids = GetIds(argc, argv);
  const SSLVisionId& ssl_vision_id = ids.first;
  const CameraId& camera_id = ids.second;

  // The most vexing parse!
  ThreadSafeActor<open_loop::BallObservation> thread_safe_observation(
      (open_loop::BallObservation()));
  open_loop::BallReader ball_reader(camera_id, &thread_safe_observation);
  ball_reader.Start();
  UDPMulticastServer udp_server = SetupUDPServer();

  joystick::Joystick joystick;
  joystick.Open(kJoystickID);

  RateLoop loop(kTransmitFrequency);
  std::vector<BallPosition> saved_positions;
  while (!shutdown_flag) {
    loop.Sleep();
    HandleAlertReady();
    joystick.ProcessEvents(0);
    udp_server.SendProtobuf(BuildRadioCommand(joystick, ssl_vision_id));
    open_loop::BallObservation ball_observation;
    thread_safe_observation.ReadOrDefault(&ball_observation);
    if (ShouldRecordData(saved_positions)) {
      ConditionallyAddData(&saved_positions, ball_observation);
    } else if (!saved_positions.empty()) {
      DumpData(&saved_positions, ssl_vision_id);
    }
  }

  joystick.Close();
  udp_server.Close();
  ball_reader.Stop();

  return 0;
}

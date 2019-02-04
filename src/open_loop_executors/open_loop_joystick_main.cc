// Copyright 2018 slane@cs.umass.edu
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

#include "constants/constants.h"
#include "constants/typedefs.h"
#include "constants/includes.h"
#include "math/poses_2d.h"
#include "open_loop_executors/open_loop_joystick.h"
#include "soccer/kalmanupdate.h"
#include "state/direction.h"
#include "state/position_velocity_state.h"
#include "state/shared_state.h"
#include "state/team.h"

STANDARD_USINGS;
using app::KalmanUpdate;
using direction::Direction;
using logger::Logger;
using open_loop::OpenLoopJoystick;
using pose_2d::Pose2Df;
using state::PositionVelocityState;
using state::SharedState;
using std::atomic_bool;
using std::bitset;
using std::stoi;
using team::Team;
using threadsafe::ThreadSafePriorityQueue;
using threadsafe::ThreadSafeQueue;
using threadsafe::ThreadSafeActor;

static atomic_bool shutdown_flag(false);

void SigHandler(int signo) {
  if (signo == SIGINT) {
    shutdown_flag = true;
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
  FLAGS_logtostderr = true;    // Don't log to disk

  if (signal(SIGINT, SigHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGINT";
  }

  // Set Robot ID
  SSLVisionId id = 0;
  if (argc < 2 || argc >= 3) {
    cout << "Usage: " << argv[0] << " robot_id" << endl;
    return 0;
  } else {
    string arg = argv[1];
    id = static_cast<SSLVisionId>(stoi(arg));
  }

  // Set Team
  Team team = Team::BLUE;
  // Team team = Team::YELLOW;

  Direction direction = Direction::POSITIVE;
  // Direction direction = Direction::NEGATIVE;

  // Set camera mask if there is one
  bitset<kNumCameras> camera_mask;
  for (unsigned int c = 0; c < kNumCameras; ++c) {
    camera_mask.set(c);
  }

  PositionVelocityState global_position_velocity_state(
      PositionVelocityState::RobotPositionVelocity(
          0, Pose2Df(0, Vector2f(0, 0)), Pose2Df(0, Vector2f(0, 0)),
          Pose2Df(0, Vector2f(0, 0)), {0, 0, 0}, 0, 0));

  // Global logging for logging on kalman update thread
  Logger global_logger;

  ThreadSafeQueue<SharedState> thread_safe_shared_state_queue;
  ThreadSafeActor<PositionVelocityState> thread_safe_position_velocity_state(
      global_position_velocity_state);
  ThreadSafeActor<Logger> thread_safe_kalman_logger(global_logger);

  KalmanUpdate kalman_update(camera_mask,
                             DATA_STREAM_VISION_IP,
                             DATA_STREAM_VISION_PORT,
                             &thread_safe_position_velocity_state,
                             &thread_safe_kalman_logger,
                             &thread_safe_shared_state_queue,
                             direction,
                             team,
                             false);

  OpenLoopJoystick joystick(id,
                            &thread_safe_position_velocity_state,
                            &thread_safe_kalman_logger,
                            &thread_safe_shared_state_queue,
                            team);


  kalman_update.Start();
  joystick.Start();

  // Begin waiting for SIGINT to proceed to shutdown.
  while (!shutdown_flag) {
    // Sleep of 50 ms.
    Sleep(0.05);
  }

  // Kill all threads.
  kalman_update.Stop();
  joystick.Stop();

  thread_safe_position_velocity_state.Shutdown();
  thread_safe_kalman_logger.Shutdown();
  thread_safe_shared_state_queue.Shutdown();

  return 1;
}

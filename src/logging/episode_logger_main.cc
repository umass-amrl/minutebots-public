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

#include <glog/logging.h>
#include <signal.h>

#include "logging/logger.h"
#include "logging/score_episode_logger.h"
#include "soccer/kalmanupdate.h"
#include "state/direction.h"
#include "state/team.h"

using app::KalmanUpdate;
using direction::Direction;
using logger::Logger;
using logger::ScoreEpisodeLogger;
using pose_2d::Pose2Df;
using state::SharedState;
using std::atomic_bool;
using std::bitset;
using team::Team;
using threadsafe::ThreadSafeActor;
using threadsafe::ThreadSafeQueue;

// Used for handling the lifetime and eventual shutdown of the main RoboCup
// stack.
static atomic_bool shutdown_flag(false);

void SigHandler(int signo) {
  if (signo == SIGINT) {
    shutdown_flag = true;
  }
}

int main(int argc, char** argv) {
  PositionVelocityState global_position_velocity_state(
    PositionVelocityState::RobotPositionVelocity(
      0, Pose2Df(0, Vector2f(0, 0)), Pose2Df(0, Vector2f(0, 0)),
                                                 Pose2Df(0, Vector2f(0, 0)), {0,
                                                   0, 0}, 0, 0));

  // Global logging for logging on kalman update thread
  Logger global_logger;

  // Scope intended to destroy threading primitives.
  {
    ThreadSafeQueue<SharedState> thread_safe_shared_state_queue;
    ThreadSafeActor<PositionVelocityState> thread_safe_position_velocity_state(
      global_position_velocity_state);
    ThreadSafeActor<Logger> thread_safe_kalman_logger(global_logger);

    // Sets up the threads for Kalman update.
    // RAII takes care of ensuring everything shutsdown appropriately.
    bitset<kNumCameras> camera_mask;
    for (unsigned int c = 0; c < kNumCameras; c++) {
      camera_mask.set(c);
    }
    KalmanUpdate kalman_update(camera_mask,
                               DATA_STREAM_VISION_IP,
                               DATA_STREAM_VISION_PORT,
                               &thread_safe_position_velocity_state,
                               &thread_safe_kalman_logger,
                               &thread_safe_shared_state_queue,
                               Direction::POSITIVE,
                               Team::BLUE,
                               false);

    ScoreEpisodeLogger episode_logger(&thread_safe_position_velocity_state);

    kalman_update.Start();
    episode_logger.Start();

    while (!shutdown_flag) {
      // Sleep of 50 ms.
      Sleep(0.05);
    }
    kalman_update.Stop();

    thread_safe_kalman_logger.Shutdown();
    thread_safe_shared_state_queue.Shutdown();
  }
  // Cleanly exit the protobuf library.
  google::protobuf::ShutdownProtobufLibrary();
  // Ensures that all log files are cleanly flushed.
  google::FlushLogFiles(google::GLOG_INFO);
}

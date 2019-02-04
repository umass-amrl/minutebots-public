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

#include <string>

#include "constants/constants.h"
#include "logging/logger.h"
#include "logging/score_episode_logger.h"
#include "net/netraw.h"
#include "state/direction.h"
#include "state/referee_state.h"
#include "state/team.h"

#include "referee.pb.h"

STANDARD_USINGS;

using direction::Direction;
using logger::WriteLogger;
using MinuteBotsProto::SSL_Referee;
using net::UDPMulticastServer;
using state::PositionVelocityState;
using state::RefereeState;
using std::thread;
using team::Team;
using threadsafe::ThreadSafeActor;

namespace logger {
ScoreEpisodeLogger::ScoreEpisodeLogger(
    ThreadSafeActor<PositionVelocityState>* thread_safe_position_velocity_state)
  : thread_safe_position_velocity_state_(thread_safe_position_velocity_state),
    is_running_(false),
    state_buffer_(kBufferLength),
    world_state_(&current_state_, Team::BLUE, true),
    soccer_state_(world_state_, Team::BLUE, Direction::POSITIVE),
    previous_time_(0),
    num_episodes_(0),
    kicked_off_(false) {}

ScoreEpisodeLogger::~ScoreEpisodeLogger() {
  is_running_ = false;
  thread_safe_position_velocity_state_->Shutdown();
  if (update_thread_.joinable()) {
    update_thread_.join();
  }
}

void ScoreEpisodeLogger::Start() {
  update_thread_ = thread(&ScoreEpisodeLogger::HandleUpdate, this);
}

void ScoreEpisodeLogger::Stop() {
  is_running_ = false;
}

void ScoreEpisodeLogger::HandleUpdate() {
  // Used for reading in messages from the referee.
  UDPMulticastServer referee_server;
  if (!referee_server.Open((DATA_STREAM_REF_IP), DATA_STREAM_REF_PORT, true)) {
    LOG(FATAL) << "Error opening UDP for referee "
    << "HandleExecution thread exiting.";
  }
  CHECK(referee_server.IsOpen());

  RefereeState ref_state(Team::BLUE);
  SSL_Referee referee_message;
  current_state_.SetTime(0);

  Logger logger;

  // Used to maintain a constant transmit rate to the robots.
  RateLoop loop(kTransmitFrequency);
  while (is_running_) {
    state::PositionVelocityState last_state = current_state_;
    if (thread_safe_position_velocity_state_->ReadOrDefault(
      &current_state_)) {
      world_state_.UpdateLastState(last_state);
    }
    world_state_.UpdateState(&logger);
    soccer_state_.GetMutableSharedState()->ResetCommands(current_state_);
    soccer_state_.UpdateExistances();

    if (referee_server.TryReceiveProtobuf(&referee_message)) {
      ref_state.SetRefereeMessage(referee_message);
      soccer_state_.SetRefereeState(ref_state);
    }
    soccer_state_.UpdateGameState();

    if (ref_state.IsPrepareKickoffThem() || ref_state.IsPrepareKickoffUs()) {
      kicked_off_ = true;
      std::cout << "Preparing Kickoff" << std::endl;
    }

    if (world_state_.world_time_ > previous_time_) {
      state_buffer_.Add(current_state_);
      // Check for episode
      if (soccer_state_.IsNormalPlay() && IsGoal()) {
        last_state = current_state_;
        num_episodes_++;
        WriteBuffer();
        current_state_ = last_state;
        world_state_.UpdateLastState(current_state_);
        kicked_off_ = false;
      } else if (!soccer_state_.IsNormalPlay()) {
        // Clear log if necessary
        ClearBuffer();
      }
    }

    logger.Clear();

    // Sleep
    loop.Sleep();
  }
}

void ScoreEpisodeLogger::WriteBuffer() {
  std::cout << "Writing Buffer" << std::endl;
  double start_time = state_buffer_.First().GetTime();

  char c_filename[50];
  std::snprintf(c_filename,
                sizeof(c_filename),
                "episode_%d.log",
                num_episodes_);
  string filename(c_filename);
  WriteLogger logger(filename);
  Logger junk_logger;

  world_state_.world_time_ = start_time;
  world_state_.UpdateLastState(state_buffer_.First());

  for (unsigned int i = 1; i < state_buffer_.Size(); i++) {
    current_state_ = state_buffer_[i];
    world_state_.UpdateState(&junk_logger);

    // Log the current world state
    logger.SetWorldState(world_state_);
    logger.SetMessageTime(state_buffer_[i].GetTime());
    logger.LogPrint("Time: %f", state_buffer_[i].GetTime());
    logger.WriteData();
    logger.Clear();
    junk_logger.Clear();
  }

  ClearBuffer();
  std::cout << "Finished" << std::endl;
}

void ScoreEpisodeLogger::ClearBuffer() {
  state_buffer_.Clear();
}

bool ScoreEpisodeLogger::IsGoal() {
  Vector2f ball_position = world_state_.GetBallPosition().filtered_position;
  if (ball_position.x() < kFieldXMax && ball_position.x() > -kFieldXMax) {
    return false;
  } else if (ball_position.y() < -field_dimensions::kGoalWidth / 2.0f ||
        ball_position.y() > field_dimensions::kGoalWidth / 2.0f) {
    return false;
  } else {
    return true;
  }
}


}  // namespace logger

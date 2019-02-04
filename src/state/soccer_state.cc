// Copyright 2017-2018 kvedder@umass.edu, slane@cs.umass.edu
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
#include "state/soccer_state.h"

#include <memory>
#include <utility>

#include "constants/constants.h"
#include "math/geometry.h"
#include "state/soccer_robot.h"
#include "state/world_robot.h"
#include "state_estimation/default_motion_model.h"

using geometry::EuclideanDistance;
using direction::Direction;
using motion_model::DefaultMotionModel;
using tactics::TacticIndex;
using state::WorldState;
using state::RefereeState;
using tactics::Tactic;
using team::Team;
using std::unique_ptr;
using pose_2d::Pose2Df;
STANDARD_USINGS;

namespace state {

SoccerState::SoccerState(const WorldState& world_state)
    : SoccerState(world_state, Team::BLUE, Direction::POSITIVE) {}

SoccerState::SoccerState(const WorldState& world_state,
                         const Team& team)
    : SoccerState(world_state, team, Direction::POSITIVE) {}

SoccerState::SoccerState(const WorldState& world_state,
                         const Team& team,
                         const Direction& direction)
    : direction_(direction),
      world_state_(world_state),
      referee_state_(team),
      static_collision_grid_(kEightGridSquareSize),
      dynamic_collision_grid_(kEightGridSquareSize),
      ball_was_passed_(false),
      receiving_robot_id_(42),
      pass_time_(0),
      ball_start_(0, 0),
      ball_start_update_time_(0),
      eight_grid_timing_file_() {
  this->our_team_ = team;

  for (size_t i = 0; i < kMaxTeamRobots; ++i) {
    robots_.emplace_back(world_state_, TacticIndex::HALT, this, &shared_state_,
                         i);
  }
  shared_state_.Init();
  //   referee_state_ = referee_state_(team);

  for (auto& robot : robots_) {
    robot.InitTactics();
  }

  static_collision_grid_.RebuildStatic(obstacle::SafetyMargin());
  setup_positions_.fill({{0, 0}, false});
  setup_scores_.fill(5.0);
  aggressive_setup_positions_.fill({0, 0});
  aggressive_setup_scores_.fill(5.0);
  best_pass_angle_ = 0;
}

SoccerState::~SoccerState() {}

void SoccerState::UpdateExistances() {
  static const bool kDebug = false;
  // Intentionally signed, for use in comparison with the signed index to copy
  // right.
  int world_robot_index = 0;
  for (const auto& world_robot : world_state_.GetOurRobots()) {
    // Confidence of zero indicates placeholder robot.
    if (world_robot.confidence > 0) {
      SoccerRobot& soccer_robot = robots_[world_robot_index];
      if (soccer_robot.enabled_ &&
          world_robot.ssl_vision_id == soccer_robot.ssl_vision_id_) {
        soccer_robot.SetRobotData(world_robot.ssl_vision_id, world_robot_index);
      } else {
        int matching_index = -1;
        // Check right to see if any of the other robots are already set to the
        // current SSL ID. If so, copy left.
        for (int i = world_robot_index; i < static_cast<int>(robots_.size());
             ++i) {
          if (robots_[i].enabled_ &&
              robots_[i].ssl_vision_id_ == world_robot.ssl_vision_id) {
            matching_index = i;
            break;
          }
        }

        if (matching_index >= 0) {
          // Shift all of the robots to the left 1 slot.
          // Repeat this shift the correct number of times.
          for (int i = world_robot_index; i < matching_index; ++i) {
            for (int j = world_robot_index;
                 j < static_cast<int>(robots_.size()) - 1; ++j) {
              robots_[j] = robots_[j + 1];
            }
            robots_[robots_.size() - 1].ClearRobotData();
            robots_[world_robot_index].SetRobotData(world_robot.ssl_vision_id,
                                                    world_robot_index);
          }
        } else {
          // Copy data to the right to make a new slot, reset and then set the
          // data
          // for the fresh slot.
          if (robots_.size() > 1) {
            for (int j = static_cast<int>(robots_.size()) - 2;
                 j >= world_robot_index; --j) {
              robots_[j + 1] = robots_[j];
            }
          }
          robots_[world_robot_index].ClearRobotData();
          robots_[world_robot_index].SetRobotData(world_robot.ssl_vision_id,
                                                  world_robot_index);
        }

        if (kDebug) {
          size_t soccer_bot_index = 0;
          for (const auto& soccer_bot : robots_) {
            auto* shared_state = shared_state_.GetSharedState(soccer_bot_index);
            const auto& our_ws_bot =
                world_state_.GetOurRobots().Get(soccer_bot_index);
            LOG(WARNING) << "Index: " << soccer_bot_index
                         << " SSL ID: " << soccer_bot.ssl_vision_id_
                         << " enabled: "
                         << (soccer_bot.enabled_ ? "TRUE" : "FALSE")
                         << " Shared state SSL ID: "
                         << shared_state->ssl_vision_id << " enabled: "
                         << (shared_state->enabled ? "TRUE" : "FALSE")
                         << " WS SSL ID: " << our_ws_bot.ssl_vision_id
                         << " confidence: " << our_ws_bot.confidence;
            ++soccer_bot_index;
          }

          soccer_bot_index = 0;
          for (const auto& our_ws_bot : world_state_.GetOurRobots()) {
            LOG(WARNING) << "Index: " << soccer_bot_index
                         << " SSL ID:  " << our_ws_bot.ssl_vision_id
                         << " confidence " << our_ws_bot.confidence;
            ++soccer_bot_index;
          }
        }
      }
    } else {
      robots_[world_robot_index].ClearRobotData();
    }
    ++world_robot_index;
  }

  // Reset robots that are no longer included in the worl state listing.
  for (size_t i = world_robot_index; i < kMaxTeamRobots; ++i) {
    robots_[world_robot_index].ClearRobotData();
  }
}

const SoccerRobot& SoccerState::GetRobotByOurRobotIndex(
    OurRobotIndex robot_index) const {
  NP_CHECK(robot_index < robots_.size());
  return robots_[robot_index];
}

SoccerRobot* SoccerState::GetMutableRobot(OurRobotIndex robot_index) {
  NP_CHECK(robot_index < robots_.size());
  return &robots_[robot_index];
}

logger::Logger* SoccerState::GetMutableRobotLoggerByOurRobotIndex(
    OurRobotIndex robot_index) {
  NP_CHECK(robot_index < robots_.size());
  return &(robots_[robot_index].bot_logger);
}

void SoccerState::SetRobotTacticByRobotIndex(OurRobotIndex robot_index,
                                             TacticIndex tactic) {
  NP_CHECK(robot_index < robots_.size());
  robots_[robot_index].current_tactic_ = tactic;
}

const Pose2Df SoccerState::GetGoalByRobotIndex(OurRobotIndex robot_index) {
  NP_CHECK(robot_index < robots_.size());
  pose_2d::Pose2Df goal = robots_[robot_index].GetGoal();
  return goal;
}

void SoccerState::UpdateNavigationState(logger::Logger* logger) {
  const auto before_rebuild = GetMonotonicTime();
  dynamic_collision_grid_.RebuildDynamic(obstacle::SafetyMargin());
  const auto after_rebuild = GetMonotonicTime();
  const auto time_delta_millis = (after_rebuild - before_rebuild) * 1000.0;
  if (kDumpCollisionRebuild) {
    if (!eight_grid_timing_file_.is_open()) {
      eight_grid_timing_file_.open("collision_grid_rebuild_times.txt");
    }
    eight_grid_timing_file_ << time_delta_millis << '\n';
  }
  GetDynamicCollisionGrid().DrawVertices(logger, {0, 1, 0, 1},
                                         obstacle::ObstacleFlag::GetEmpty());
}

void SoccerState::RunAllTactics() {
  size_t index = 0;
  for (SoccerRobot& soccer_robot : robots_) {
    if (soccer_robot.enabled_) {
      soccer_robot.RunTactic();
    }
    ++index;
  }
}

const Team& SoccerState::GetTeam() const { return our_team_; }

const vector<SoccerRobot>& SoccerState::GetAllSoccerRobots() const {
  return robots_;
}

vector<SoccerRobot>* SoccerState::GetAllMutableSoccerRobots() {
  return &robots_;
}

SharedState* SoccerState::GetMutableSharedState() { return &shared_state_; }

const SharedState& SoccerState::GetSharedState() const { return shared_state_; }

const WorldState& SoccerState::GetWorldState() { return world_state_; }

const RefereeState& SoccerState::GetRefereeState() const {
  return referee_state_;
}

void SoccerState::SetRefereeState(const RefereeState& ref_state) {
  bool normal = normal_play_;
  RefereeState last_state = referee_state_;
  referee_state_ = ref_state;
  //   // Handling the kickoff transition.
  //   if (last_state.IsPrepareKickoffThem() ||
  //       last_state.IsNormalStart() ||
  //       last_state.IsPrepareKickoffUs()) {
  //     if (referee_state_.IsNormalStart()) {
  //       kickoff_ = true;
  //     }
  //   } else if (referee_state_.IsPrepareKickoffThem() ||
  //       referee_state_.IsPrepareKickoffUs()) {
  //     kickoff_ = true;
  //   } else {
  //     kickoff_ = false;
  //   }
  if ((referee_state_.IsDirectFreeThem() ||
       referee_state_.IsIndirectFreeThem()) &&
      (last_state.IsIndirectFreeThem() || last_state.IsDirectFreeThem())) {
    // do nothing
  } else if ((referee_state_.IsDirectFreeUs() ||
       referee_state_.IsIndirectFreeUs()) &&
      (last_state.IsIndirectFreeUs() || last_state.IsDirectFreeUs())) {
    // do nothing
  } else {
    if (referee_state_.IsPrepareKickoffThem() ||
        referee_state_.IsPrepareKickoffUs()) {
      kickoff_ = true;
    } else if (!referee_state_.IsNormalStart()) {
      kickoff_ = false;
    }

    if (referee_state_.IsPreparePenaltyUs()) {
      our_penalty_kick_ = true;
    } else if (!referee_state_.IsNormalStart()) {
      our_penalty_kick_ = false;
    }

    if (referee_state_.IsPreparePenaltyThem()) {
      their_penalty_kick_ = true;
    } else if (!referee_state_.IsNormalStart()) {
      their_penalty_kick_ = false;
    }

    if (kickoff_ || their_penalty_kick_ || our_penalty_kick_) {
      normal = false;
      if (referee_state_.IsForceStart()) {
        normal = true;
      }
    } else {
      normal =
          (referee_state_.IsNormalStart() || referee_state_.IsForceStart()) &&
          (!referee_state_.IsNormalHalftime() || !referee_state_.IsHalt() ||
           !referee_state_.IsStop() ||
           !referee_state_.IsNormalSecondHalfPre() ||
           !referee_state_.IsNormalFirstHalfPre() ||
           !referee_state_.IsExtraFirstHalfPre() ||
           !referee_state_.IsExtraSecondHalfPre() ||
           !referee_state_.IsPenaltyShootout() ||
           !referee_state_.IsPenaltyShootoutBreak() ||
           !referee_state_.IsExtraHalfTime() ||
           !referee_state_.IsExtraTimeBreak() ||
           !referee_state_.IsBallPlacementThem() ||
           !referee_state_.IsBallPlacementUs() ||
           !referee_state_.IsDirectFreeThem() ||
           !referee_state_.IsDirectFreeUs() ||
           !referee_state_.IsIndirectFreeThem() ||
           !referee_state_.IsIndirectFreeUs() ||
           !referee_state_.IsPrepareKickoffThem() ||
           !referee_state_.IsPrepareKickoffUs() ||
           !referee_state_.IsPreparePenaltyUs() ||
           !referee_state_.IsPreparePenaltyThem() ||
           !referee_state_.IsBallPlacementThem() ||
           !referee_state_.IsBallPlacementUs());
    }
  }
  normal_play_ = normal;
  referee_state_ = ref_state;
}

bool SoccerState::IsNormalPlay() const { return normal_play_; }

bool SoccerState::IsKickoff() const { return kickoff_; }

bool SoccerState::IsOurPenaltyKick() const { return our_penalty_kick_; }

bool SoccerState::IsTheirPenaltyKick() const { return their_penalty_kick_; }

void SoccerState::UpdateGameState() {
  if (referee_state_.IsPrepareKickoffThem() ||
      referee_state_.IsPreparePenaltyUs() ||
      referee_state_.IsPreparePenaltyThem() ||
      referee_state_.IsDirectFreeThem() ||
      referee_state_.IsIndirectFreeThem() ||
      referee_state_.IsPrepareKickoffUs() || referee_state_.IsDirectFreeUs() ||
      referee_state_.IsIndirectFreeUs()) {
    double current_time = GetMonotonicTime();
    if (current_time - ball_start_update_time_ > kBallMovedTimeThreshold) {
      ball_start_ = world_state_.GetBallPosition().position;
      ball_start_update_time_ = current_time;
    }
  }

  if (IsBallMoved()) {
    // if (IsBallKicked() || IsBallMoved()) {
    // Set the game mode to normal play if we are in any of these modes and
    // the ball is kicked.
    if (referee_state_.IsNormalStart() || referee_state_.IsForceStart() ||
        referee_state_.IsDirectFreeThem() || referee_state_.IsDirectFreeUs() ||
        referee_state_.IsIndirectFreeThem() ||
        referee_state_.IsIndirectFreeUs() ||
        referee_state_.IsPrepareKickoffUs() ||
        referee_state_.IsPrepareKickoffThem() ||
        referee_state_.IsPreparePenaltyThem() ||
        referee_state_.IsPreparePenaltyUs()) {
      normal_play_ = true;
    }
    // Unset the kickoff and penalty kick flags if we are in normal start and
    // the ball has been kicked,
    if (referee_state_.IsNormalStart()) {
      if (kickoff_) {
        kickoff_ = false;
      }

      if (our_penalty_kick_) {
        our_penalty_kick_ = false;
      }

      if (their_penalty_kick_) {
        their_penalty_kick_ = false;
      }
    }
  }

  UpdateBallWasPassed();
}

bool SoccerState::IsBallKicked() {
  const Vector2f& velocity = world_state_.GetBallPosition().velocity;
  return fabs(velocity.norm()) > kKickDetectionThreshold;
}

bool SoccerState::IsBallMoved() const {
  const Vector2f& ball_position =
      world_state_.GetBallPosition().filtered_position;
  return EuclideanDistance(ball_position, ball_start_) > kBallMovedDistance;
}

bool SoccerState::BallOurHalf() {
  const Vector2f& position = world_state_.GetBallPosition().position;
  return position.x() < 0;
}

bool SoccerState::BallTheirHalf() {
  const Vector2f& position = world_state_.GetBallPosition().position;
  return position.x() > 0;
}

bool SoccerState::BallMidField() {
  const Vector2f& position = world_state_.GetBallPosition().position;
  if ((position.x() > -kFieldXMax / 2) && (position.x() < kFieldXMax / 2)) {
    return true;
  }
  return false;
}

bool SoccerState::BallOurPossession() {
  auto ball = world_state_.GetBallPosition().position;
  for (auto robot : world_state_.GetOurRobots()) {
    auto position = robot.position.translation;
    float distance = (ball - position).norm();
    if (distance < kRobotFaceRadius) {
      return true;
    }
  }
  return false;
}

bool SoccerState::BallTheirPossession() {
  auto ball = world_state_.GetBallPosition().position;
  for (auto robot : world_state_.GetTheirRobots()) {
    auto position = robot.position.translation;
    float distance = (ball - position).norm();
    if (distance < kRobotRadius * 2) {
      return true;
    }
  }
  return false;
}

bool SoccerState::FreeBall() {
  auto ball = world_state_.GetBallPosition().position;
  for (auto robot : world_state_.GetOurRobots()) {
    auto position = robot.position.translation;
    float distance = (ball - position).norm();
    if (distance < kRobotFaceRadius) {
      return false;
    }
  }
  for (auto robot : world_state_.GetTheirRobots()) {
    auto position = robot.position.translation;
    float distance = (ball - position).norm();
    if (distance < kRobotFaceRadius) {
      return false;
    }
  }
  return true;
}

bool SoccerState::BallWasPassed() const { return ball_was_passed_; }

SSLVisionId SoccerState::GetReceivingRobot() const {
  return receiving_robot_id_;
}

void SoccerState::SetBallWasPassed(SSLVisionId receiving_robot_id,
                                   double pass_time) {
  ball_was_passed_ = true;
  receiving_robot_id_ = receiving_robot_id;
  pass_time_ = pass_time;
}

void SoccerState::UpdateBallWasPassed() {
  if (!IsNormalPlay()) {
    ball_was_passed_ = false;
    receiving_robot_id_ = 42;
    return;
  }

  if (ball_was_passed_) {
    double current_time = world_state_.world_time_;

    logger::Logger* local_logger = &robots_[receiving_robot_id_].bot_logger;
    local_logger->LogPrint("Updating Ball Passing");
    local_logger->Push();
    local_logger->LogPrint("Current time: %0.3f", current_time);
    local_logger->LogPrint("Pass time: %0.3f", pass_time_);
    local_logger->LogPrint("Threshold: %0.3f",
                           kWasPassedToThreshold / kTransmitFrequency);

    if (current_time - pass_time_ >
        kWasPassedToThreshold / kTransmitFrequency) {
      ball_was_passed_ = false;
      receiving_robot_id_ = 42;
    }
  }
}

const navigation::production::eight_grid::CollisionGrid&
SoccerState::GetDynamicCollisionGrid() const {
  return dynamic_collision_grid_;
}

const navigation::production::eight_grid::CollisionGrid&
SoccerState::GetStaticCollisionGrid() const {
  return static_collision_grid_;
}

}  // namespace state

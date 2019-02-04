// Copyright 2018 - 2019 kvedder@umass.edu, slane@cs.umass.edu
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
#include "obstacles/obstacle_flag.h"

#include <utility>

#include "constants/constants.h"
#include "state/referee_state.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "tactics/tactic_index.h"

STANDARD_USINGS;
using geometry::SafeVectorNorm;
using state::RefereeState;
using state::SoccerRobot;
using state::SoccerState;
using state::WorldState;
using std::bitset;
using tactics::TacticIndex;

namespace obstacle {

ObstacleFlag::ObstacleFlagIter::ObstacleFlagIter(const ObstacleFlag& p_vec,
                                                 const size_t pos)
    : pos_(pos), p_vec_(p_vec) {}

bool ObstacleFlag::ObstacleFlagIter::operator!=(
    const ObstacleFlagIter& other) const {
  return pos_ != other.pos_;
}

const Obstacle* const ObstacleFlag::ObstacleFlagIter::operator*() const {
  return state::WorldState::master_obstacle_buffer[pos_].get();
}

const ObstacleFlag::ObstacleFlagIter& ObstacleFlag::ObstacleFlagIter::
operator++() {
  do {
    ++pos_;
    // Not at the end of the obstacle list and the robot flag is enabled.
  } while (pos_ < state::WorldState::master_obstacle_buffer.size() &&
           !(p_vec_.flags_[pos_] &&
             state::WorldState::master_obstacle_buffer[pos_]->IsEnabled()));
  return *this;
}
//
ObstacleFlag::IndexedObstacleFlagIter::IndexedObstacleFlagIter(
    const ObstacleFlag& p_vec, const size_t pos)
    : pos_(pos), p_vec_(p_vec) {}

bool ObstacleFlag::IndexedObstacleFlagIter::operator!=(
    const IndexedObstacleFlagIter& other) const {
  return pos_ != other.pos_;
}

const std::pair<size_t, Obstacle*> ObstacleFlag::IndexedObstacleFlagIter::
operator*() const {
  return {pos_, state::WorldState::master_obstacle_buffer[pos_].get()};
}

const ObstacleFlag::IndexedObstacleFlagIter&
ObstacleFlag::IndexedObstacleFlagIter::operator++() {
  do {
    ++pos_;
    // Not at the end of the obstacle list and the robot flag is enabled.
  } while (pos_ < state::WorldState::master_obstacle_buffer.size() &&
           !(p_vec_.flags_[pos_] &&
             state::WorldState::master_obstacle_buffer[pos_]->IsEnabled()));
  return *this;
}

ObstacleFlag::ObstacleFlag(const bitset<kNumObstacles>& flags)
    : flags_(flags) {}

ObstacleFlag::~ObstacleFlag() {}

ObstacleFlag ObstacleFlag::operator|(const ObstacleFlag& other) const {
  return ObstacleFlag(this->flags_ | other.flags_);
}

ObstacleFlag ObstacleFlag::operator&(const ObstacleFlag& other) const {
  return ObstacleFlag(this->flags_ & other.flags_);
}

ObstacleFlag ObstacleFlag::operator~() const {
  return ObstacleFlag(~this->flags_);
}

bitset<kNumObstacles> ObstacleFlag::GetFlags() const { return flags_; }

ObstacleFlag::ObstacleFlagIter ObstacleFlag::begin() const {
  size_t pos = 0;
  while (pos < state::WorldState::master_obstacle_buffer.size() &&
         (!this->flags_[pos] ||
          !state::WorldState::master_obstacle_buffer[pos]->IsEnabled()))
    pos++;

  return ObstacleFlagIter(*this, pos);
}

ObstacleFlag::ObstacleFlagIter ObstacleFlag::end() const {
  return ObstacleFlagIter(*this,
                          state::WorldState::master_obstacle_buffer.size());
}

ObstacleFlag::IndexedObstacleFlagIter ObstacleFlag::IndexedBegin() const {
  size_t pos = 0;
  while (pos < state::WorldState::master_obstacle_buffer.size() &&
         (!this->flags_[pos] ||
          !state::WorldState::master_obstacle_buffer[pos]->IsEnabled()))
    pos++;

  return IndexedObstacleFlagIter(*this, pos);
}

ObstacleFlag::IndexedObstacleFlagIter ObstacleFlag::IndexedEnd() const {
  return IndexedObstacleFlagIter(
      *this, state::WorldState::master_obstacle_buffer.size());
}

string ObstacleFlag::GetString() const { return flags_.to_string(); }

// Static methods.

static constexpr size_t kOurRobotsMinIndex = 0;
static constexpr size_t kOurRobotsMaxIndex =
    kOurRobotsMinIndex + kMaxTeamRobots - 1;
static constexpr size_t kTheirRobotsMinIndex = kOurRobotsMaxIndex + 1;
static constexpr size_t kTheirRobotsMaxIndex =
    kTheirRobotsMinIndex + kMaxTeamRobots * (kNumTeams - 1) - 1;
static constexpr size_t kBallMinIndex = kTheirRobotsMaxIndex + 1;
static constexpr size_t kBallMaxIndex = kBallMinIndex + kNumBallObstacles - 1;
static constexpr size_t kBallSmallIndex = kBallMinIndex;
static constexpr size_t kBallMediumIndex = kBallSmallIndex + 1;
static constexpr size_t kBallLargeIndex = kBallMediumIndex + 1;
static_assert(kBallLargeIndex == kBallMaxIndex, "Not all balls accounted for");
static_assert(kNumBallObstacles > 1 || kBallMinIndex == kBallMaxIndex,
              "If the number of ball obstacles is 1, the index of the max and "
              "min index should be the same.");
static constexpr size_t kRulesObstaclesMinIndex = kBallMaxIndex + 1;
static constexpr size_t kRulesObstaclesMaxIndex =
    kRulesObstaclesMinIndex + kNumRulesObstacles - 1;
static constexpr size_t kStaticObstaclesMinIndex = kRulesObstaclesMaxIndex + 1;
static constexpr size_t kStaticObstaclesMaxIndex =
    kStaticObstaclesMinIndex + kNumStaticObstacles - 1;
static_assert(kStaticObstaclesMaxIndex + 1 == kNumObstacles,
              "Max index plus one should equal num obstacles");

ObstacleFlag ObstacleFlag::GetAll(const WorldState& world_state,
                                  const SoccerState& soccer_state,
                                  OurRobotIndex calling_robot_index) {
  // Set all robots, ball and static obstacles to true, or this with
  // GetRulesObstacles
  // TODO(slane): Do this but better
  return GetAllRobots() | GetBall() | GetStaticObstacles() |
         GetRulesObstacles(world_state, soccer_state, calling_robot_index);
}

ObstacleFlag ObstacleFlag::GetStaticObstacles() {
  auto bits = bitset<kNumObstacles>();
  for (size_t i = kStaticObstaclesMinIndex; i <= kStaticObstaclesMaxIndex;
       ++i) {
    bits.set(i);
  }
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetAllExceptTeam(
    const WorldState& world_state,
    const SoccerState& soccer_state,
    OurRobotIndex calling_robot_index,
    OurRobotIndex excluded_robot_index) {
  NP_CHECK_MSG(calling_robot_index < kMaxTeamRobots,
               "Calling Robot index (" << calling_robot_index
                                       << ") larger than max num robots ("
                                       << kMaxTeamRobots << ")");
  NP_CHECK_MSG(excluded_robot_index < kMaxTeamRobots,
               "Excluded Robot index (" << excluded_robot_index
                                        << ") larger than max num robots ("
                                        << kMaxTeamRobots << ")");
  // TODO(slane): Do this but better
  return GetAllRobotsExceptTeam(excluded_robot_index) | GetBall() |
         GetStaticObstacles() |
         GetRulesObstacles(world_state, soccer_state, calling_robot_index);
}

ObstacleFlag ObstacleFlag::GetAllBalls() {
  auto bits = bitset<kNumObstacles>().reset();
  for (size_t i = kBallMinIndex; i <= kBallMaxIndex; ++i) {
    bits.set(i);
  }
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetBall() {
  auto bits = bitset<kNumObstacles>().reset();
  bits.set(kBallSmallIndex);
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetMediumBall() {
  auto bits = bitset<kNumObstacles>().reset();
  bits.set(kBallMediumIndex);
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetLargeBall() {
  auto bits = bitset<kNumObstacles>().reset();
  bits.set(kBallLargeIndex);
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetEmpty() {
  auto bits = bitset<kNumObstacles>().reset();
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetFull() {
  auto bits = bitset<kNumObstacles>().set();
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetAllRobots() {
  auto bits = bitset<kNumObstacles>().set() >>
              (kNumObstacles - kMaxTeamRobots * kNumTeams);
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetAllRobotsExceptTeam(
    const OurRobotIndex our_robot_index) {
  NP_CHECK_MSG(our_robot_index < kMaxTeamRobots,
               "Robot index (" << our_robot_index
                               << ") larger than max num robots ("
                               << kMaxTeamRobots << ")");
  auto bits = bitset<kNumObstacles>().set() >>
              (kNumObstacles - kMaxTeamRobots * kNumTeams);
  if (!kProduction) {
    for (size_t i = kOurRobotsMinIndex; i <= kTheirRobotsMaxIndex; ++i) {
      NP_CHECK_MSG(bits[i] == true, "Index " << i << " should be enabled");
    }
    for (size_t i = kTheirRobotsMaxIndex + 1; i < kNumObstacles; ++i) {
      NP_CHECK_MSG(bits[i] == false, "Index " << i << " should be disabled");
    }
  }
  bits.set(our_robot_index, 0);

  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetOpponentRobots() {
  auto bits = bitset<kNumObstacles>().reset();
  for (size_t i = kTheirRobotsMinIndex; i <= kTheirRobotsMaxIndex; ++i) {
    bits.set(i);
  }
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetRulesObstacles(
    const WorldState& world_state,
    const SoccerState& soccer_state,
    OurRobotIndex calling_robot_index) {
  NP_CHECK_MSG(calling_robot_index < kMaxTeamRobots,
               "Robot index (" << calling_robot_index
                               << ") larger than max num robots ("
                               << kMaxTeamRobots << ")");
  auto bits = bitset<kNumObstacles>().reset();
  const RefereeState& ref_state = soccer_state.GetRefereeState();

  const SoccerRobot& soccer_robot =
      soccer_state.GetRobotByOurRobotIndex(calling_robot_index);
  const TacticIndex& current_tactic = soccer_robot.current_tactic_;

  const pose_2d::Pose2Df robot_position =
      world_state.GetOurRobotPosition(calling_robot_index).position;

  // Our Defense Area
  if (current_tactic != TacticIndex::GOALIE) {
    bits.set(kRulesObstaclesMinIndex);
  }

  // Their Defense Area
  if (current_tactic != TacticIndex::BALL_PLACEMENT &&
      current_tactic != TacticIndex::KICKOFF) {
    bits.set(kRulesObstaclesMinIndex + 1);
  }

  // Ball Padding
  if (!soccer_state.IsNormalPlay()) {
    if ((ref_state.IsStop() || ref_state.IsDirectFreeThem() ||
         ref_state.IsIndirectFreeThem()) &&
        (current_tactic != TacticIndex::GOALIE)) {
      bits.set(kRulesObstaclesMinIndex + 2);
    }
  }

  if (ref_state.IsBallPlacementUs() &&
      current_tactic != tactics::BALL_PLACEMENT &&
      current_tactic != TacticIndex::GOALIE) {
    bits.set(kRulesObstaclesMinIndex + 2);
  }

  if (ref_state.IsBallPlacementThem() &&
      current_tactic != TacticIndex::GOALIE) {
    bits.set(kRulesObstaclesMinIndex + 2);
  }

  // Center Circle
  // On during kickoff when not the kicker
  if (soccer_state.IsKickoff() && current_tactic != TacticIndex::KICKOFF) {
    bits.set(kRulesObstaclesMinIndex + 3);
  }

  //

  // Their side of field
  // During kickoff when not the kicker
  if (soccer_state.IsKickoff() && current_tactic != TacticIndex::KICKOFF) {
    bits.set(kRulesObstaclesMinIndex + 4);
  }

  // Their Defense Area Padded
  if (!soccer_state.IsNormalPlay() && current_tactic != TacticIndex::KICKOFF &&
      current_tactic != tactics::BALL_PLACEMENT) {
    bits.set(kRulesObstaclesMinIndex + 5);
  }

  // Our Penalty Kick Obstacle
  if (soccer_state.IsOurPenaltyKick() &&
      current_tactic != TacticIndex::KICKOFF) {
    bits.set(kRulesObstaclesMinIndex + 6);
  }

  // Their Penalty Kick Obstacle
  if (soccer_state.IsTheirPenaltyKick() &&
      current_tactic != TacticIndex::GOALIE) {
    bits.set(kRulesObstaclesMinIndex + 7);
  }

  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetDefenseAreas() {
  auto bits = bitset<kNumObstacles>().reset();
  bits.set(kRulesObstaclesMinIndex);
  bits.set(kRulesObstaclesMinIndex + 1);
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetOurDefenseArea() {
  auto bits = bitset<kNumObstacles>().reset();
  bits.set(kRulesObstaclesMinIndex);
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetTheirDefenseArea() {
  auto bits = bitset<kNumObstacles>().reset();
  bits.set(kRulesObstaclesMinIndex + 1);
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetOurRobots() {
  auto bits = bitset<kNumObstacles>().reset();
  for (size_t i = kOurRobotsMinIndex; i <= kOurRobotsMaxIndex; ++i) {
    bits.set(i);
  }
  return ObstacleFlag(bits);
}

ObstacleFlag ObstacleFlag::GetKickoffOtherHalfObstacle() {
  auto bits = bitset<kNumObstacles>().reset();
  bits.set(kRulesObstaclesMinIndex + 3);
  return ObstacleFlag(bits);
}

}  // namespace obstacle
